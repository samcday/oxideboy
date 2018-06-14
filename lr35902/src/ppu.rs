#![allow(unused_variables)]
#![allow(dead_code)]

use std::collections::VecDeque;

const LCDC_LCD_ENABLED: u8 = 0b1000_0000;
const LCDC_BG_ENABLED:  u8 = 0b0000_0001;
const LCDC_BG_CODE_HI:  u8 = 0b0000_1000;
const LCDC_BG_DATA_HI:  u8 = 0b0001_0000;

#[derive(Debug, PartialEq)]
enum PPUState {
    Inactive,           // LCD is currently switched off.
    OAMSearch(u8),      // OAM search phase, with the number of cycles remaining.
    PixelTransfer(PixelTransferState),  // 
    HBlank(u8),         // HBlank phase, with number of cycles remaining.
    VBlank(u8),
}
use self::PPUState::{*};

#[derive(Copy, Clone, Debug, PartialEq)]
struct PixelTransferState {
    cycles: u8,     // Counts the number of cycles we've consumed. We use this to calculate how
                    // many cycles HBlank should subsequently consume.
    x: u8,          // Tracks which pixel in the scanline we're up to fetching.
    next: [u8; 8],  // The next set of pixels being worked on by fetcher.
    flush: bool,    // When true, the next 8 pixels are written into FIFO.
}

struct OAMEntry {
    y: u8,
    x: u8,
    code: u8,
    palette: u8,
    horz_flip: bool,
    vert_flip: bool,
    priority: bool,
}

pub struct PPU {
    state: PPUState,
    pub scy: u8,
    pub scx: u8,
    pub ly: u8,
    pub lyc: u8,
    pub wx: u8,
    pub wy: u8,

    bg_enabled: bool,   // Toggles BG tile display on/off.
    bg_code_hi: bool,   // Toggles where we look for BG code data. true: 0x9C00-0x9FFF, false: 0x9800-0x9BFF
    bg_data_hi: bool,   // Toggles where we look for BG tile data. true: 0x8800-0x97FF, false: 0x8000-0x8FFF

    vram: [u8; 0x2000], // 0x8000 - 0x9FFF 
    oam:  [u8; 0xA0],   // 0xFE00 - 0xFDFF

    scanline_objs: Vec<OAMEntry>,
    pixel_fifo: VecDeque<u8>,

    pub framebuffer: [u32; ::SCREEN_SIZE],
    fb_pos: usize,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            state: Inactive,
            scy: 0, scx: 0,
            ly: 0, lyc: 0,
            wx: 0, wy: 0,
            bg_enabled: false, bg_code_hi: false, bg_data_hi: false,
            vram: [0; 0x2000], oam: [0; 0xA0],
            scanline_objs: Vec::new(), pixel_fifo: VecDeque::new(),
            framebuffer: [0; 160*144], fb_pos: 0,
        }
    }

    pub fn is_vblank(&self) -> bool {
        if self.ly == 144 {
            if let VBlank(0) = self.state {
                return true;
            }
        }

        false
    }

    pub fn oam_read(&self, addr: u16) -> u8 {
        match self.state {
            Inactive | HBlank(_) | VBlank(_) => self.oam[addr as usize],
            OAMSearch(_) | PixelTransfer(_) => 0xFF,
        }
    }

    pub fn oam_write(&mut self, addr: u16, v: u8) {
        match self.state {
            Inactive | HBlank(_) | VBlank(_) => { self.oam[addr as usize] = v },
            OAMSearch(_) | PixelTransfer(_) => { },
        }
    }

    pub fn vram_read(&self, addr: u16) -> u8 {
        match self.state {
            Inactive | OAMSearch(_) | HBlank(_) | VBlank(_) => self.vram[addr as usize],
            PixelTransfer(_) => 0xFF,
        }
    }

    pub fn vram_write(&mut self, addr: u16, v: u8) {
        match self.state {
            Inactive | OAMSearch(_) | HBlank(_) | VBlank(_) => { self.vram[addr as usize] = v },
            PixelTransfer(_) => { },
        }
    }

    // Advances PPU display by a single clock cycle. Basically, all the video magic happens in here.
    pub fn advance(&mut self) -> Option<::Interrupt> {
        let mut intr = None;

        // Each scanline is 114 clock cycles. 20 for OAM search, 43+ for pixel transfer, remaining for hblank.
        let new_state = match self.state {
            Inactive => { Inactive },
            OAMSearch(n) => {
                self.oam_search(n)
            },
            PixelTransfer(pt_state) => {
                self.pixel_transfer(pt_state)
            },
            HBlank(n) => {
                if n > 0 {
                    HBlank(n - 1)
                } else {
                    self.ly += 1;
                    if self.ly == 143 {
                        intr = Some(::Interrupt::VBlank);
                        VBlank(0)
                    } else {
                        OAMSearch(0)
                    }
                }
            },
            VBlank(n) => {
                if n < 113 {
                    VBlank(n + 1)
                } else if self.ly == 154 {
                    self.ly = 0;
                    self.scanline_objs.clear();
                    self.fb_pos = 0;
                    OAMSearch(0)
                } else {
                    self.ly += 1;
                    VBlank(0)
                }
            }
        };
        self.state = new_state;

        intr
    }

    // The first stage of a scanline. Takes 20 cycles.
    // In this stage, we search the OAM table for OBJs that overlap the current line (LY).
    // I'm sure it's totally different in hardware, but we simulate this process by searching 2 bytes
    // of OAM each cycle. OAM is 40 bytes and OAM Search takes 20 cycles, so this divides nicely.
    // TODO: need to resolve sprite conflicts based on x pos here.
    fn oam_search(&mut self, n: u8) -> PPUState {
        let mut idx = 0;
        // TODO: this should be 16 in big sprite mode
        let h = 8;
        while self.scanline_objs.len() < 10 && idx < 2 {
            let obj = self.read_oam_entry(n * 2 + idx);
            if obj.x > 0 && self.ly + 16 >= obj.y && self.ly + 16 < obj.y + h {
                self.scanline_objs.push(obj);
            }
            idx += 1;
        }

        if n < 19 {
            OAMSearch(n + 1)
        } else {
            PixelTransfer(PixelTransferState{cycles: 0, x: 0, next: [0; 8], flush: false})
        }
    }

    // The second stage of the scanline. Takes 43+ cycles.
    // Here, we actually rasterize pixels out to the LCD.
    // There's two concurrent processes here, the pixel FIFO and the fetcher.
    // The numbers get a little wonky here because the PPU runs at 4mhz, connected to VRAM at 2mhz.
    // But because the CPU runs at 4mhz connected to 1mhz system RAM, we actually model everything
    // based on 1mhz.
    // The pixel FIFO is fairly straightforward, we push out 4 pixels every clock cycle, so long
    // as doing so does not cause the FIFO to fall below 8 pixels. In the case the FIFO does not have
    // enough elements (or is empty, as is the case at the beginning of pixel transfer stage) then we
    // simply don't push out any pixels that clock cycle.
    // The fetcher is a little more complicated. The first cycle is spent reading the data from VRAM.
    // the second cycle is when we push 8 more pixels into FIFO.
    // This approximation is close enough to the real PPU that even complex scanline effects should
    // emulate ok.
    // Note that the first 4 cycles are spent waiting for the fetcher to fill the FIFO, which is why
    // this stage takes a minimum of 43 cycles.
    fn pixel_transfer(&mut self, mut pt_state: PixelTransferState) -> PPUState {
        if self.pixel_fifo.len() >= 12 || pt_state.x == 160 {
            // Send out 4 pixels.
            for _ in 0..4 {
                let pix = self.pixel_fifo.pop_front().unwrap() as u32;
                self.framebuffer[self.fb_pos] = match pix {
                    0 => { 255 },
                    1 => { 100 },
                    2 => { 50 },
                    3 => { 0 },
                    _ => panic!("lol: {}", pix)
                };
                self.fb_pos += 1;
            }
        }

        if pt_state.x < 160 && pt_state.flush {
            for pix in pt_state.next.iter() {
                self.pixel_fifo.push_back(*pix);
            }
            pt_state.x += 8;
            pt_state.flush = false;
        } else if pt_state.x < 160 {
            if self.bg_enabled {
                let code_base_addr = if self.bg_code_hi { 0x1C00 } else { 0x1800 };
                let data_base_addr = if self.bg_data_hi { 0x0800 } else { 0 };

                let scanline_x = pt_state.x as u16;
                let scy = self.scy as u16;
                let scx = self.scx as u16;
                let ly = self.ly as u16;

                let tile_y = ((scy + ly) % 8) as usize;
                let bg_y = ((ly + scy) / 8) % 32;

                for i in 0..8 {
                    // TODO: document this madness and extract out some constants.

                    // Figure out which BG tile we're rendering.
                    let bg_x = ((scanline_x + scx + i) / 8) % 32;
                    let tile_num = bg_y * 32 + bg_x;
                    let tile = self.vram[(code_base_addr + tile_num) as usize] as usize;

                    let tile_addr = data_base_addr + (tile * 16);

                    // Now figure out which pixels in this particular tile we need.
                    let tile_x = (i + scx + scanline_x) % 8;

                    let lo = self.vram[tile_addr + (tile_y * 2) + 0] >> (7 - tile_x);
                    let hi = self.vram[tile_addr + (tile_y * 2) + 1] >> (7 - tile_x) << 1;

                    pt_state.next[i as usize] = (lo & 1) | (hi & 2);
                }
            } else {
                for i in 0..8 {
                    pt_state.next[i] = 0;
                }
            }

            pt_state.flush = true;
        }
        pt_state.cycles += 1;

        if pt_state.x == 160 && self.pixel_fifo.len() == 0 {
            HBlank(51+43 - pt_state.cycles)
        } else {
            PixelTransfer(pt_state)
        }
    }

    fn read_oam_entry(&self, n: u8) -> OAMEntry {
        let addr = (n * 4) as usize;
        OAMEntry{
            y: self.oam[addr],
            x: self.oam[addr + 1],
            code: self.oam[addr + 2],
            palette: (self.oam[addr + 3] & 0x10) >> 4,
            horz_flip: self.oam[addr + 3] & 0x20 == 0x20,
            vert_flip: self.oam[addr + 3] & 0x40 == 0x40,
            priority: self.oam[addr + 3] & 0x80 == 0x80,
        }
    }

    // Compute value of the LCDC register.
    pub fn get_lcdc(&self) -> u8 {
        let mut lcdc = 0;

        if self.state != Inactive {
            lcdc |= LCDC_LCD_ENABLED;
        }

        lcdc |= if self.bg_enabled { LCDC_BG_ENABLED } else { 0 };
        lcdc |= if self.bg_code_hi { LCDC_BG_CODE_HI } else { 0 };
        lcdc |= if self.bg_data_hi { 0 } else { LCDC_BG_DATA_HI };

        lcdc
    }

    // Update PPU state based on new LCDC value.
    pub fn set_lcdc(&mut self, v: u8) {
        // Are we enabling LCD from a previously disabled state?
        if v & LCDC_LCD_ENABLED > 0 && self.state == Inactive {
            self.state = OAMSearch(0);
            self.fb_pos = 0;
            self.ly = 0;
            self.scanline_objs.clear();
        } else if v & LCDC_LCD_ENABLED == 0 && self.state != Inactive {
            // TODO: check current state here, can't switch LCD off in the middle of pixel transfer.
            self.state = Inactive;
        }
        self.bg_enabled = v & LCDC_BG_ENABLED > 0;
        self.bg_code_hi = v & LCDC_BG_CODE_HI > 0;
        self.bg_data_hi = v & LCDC_BG_DATA_HI == 0;
    }

    // Compute value of the STAT register.
    pub fn get_stat(&self) -> u8 {
        let stat = match self.state {
            Inactive => 0,
            HBlank(_) => 0,
            VBlank(_) => 1,
            OAMSearch(_) => 2,
            PixelTransfer(_) => 3,
        };
        stat
    }

    // Update PPU state based on new STAT value.
    pub fn set_stat(&mut self, _v: u8) {

    }
}
