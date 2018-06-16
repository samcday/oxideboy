#![allow(unused_variables)]
#![allow(dead_code)]

use std::collections::VecDeque;

const LCDC_LCD_ENABLED: u8 = 0b1000_0000;
const LCDC_WIN_ENABLED: u8 = 0b0100_0000;
const LCDC_WIN_CODE_HI: u8 = 0b0010_0000;
const LCDC_BG_DATA_HI:  u8 = 0b0001_0000;
const LCDC_BG_CODE_HI:  u8 = 0b0000_1000;
const LCDC_OBJ_COMP_HI: u8 = 0b0000_0100;
const LCDC_OBJ_ENABLED: u8 = 0b0000_0010;
const LCDC_BG_ENABLED:  u8 = 0b0000_0001;

#[derive(Debug, PartialEq)]
pub enum PPUState {
    OAMSearch,          // OAM search phase
    PixelTransfer,  // 
    HBlank(u8),         // HBlank phase for given number of cycles.
    VBlank,             // VBlank for 114 cycles.
}
use self::PPUState::{*};

#[derive(Debug, PartialEq)]
struct PixelTransferState {
    pixel_fifo: VecDeque<u8>,
    scratch: [u8; 8], // The next set of pixels being worked on by fetcher.
    x: u8,          // Tracks which pixel in the scanline we're up to fetching.
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
    pub enabled: bool,      // Master switch to turn LCD on/off.
    pub state: PPUState,
    cycles: u8,         // Counts how many CPU cycles have elapsed in the current PPU stage.

    pub scy: u8,
    pub scx: u8,
    pub ly: u8,
    pub lyc: u8,
    pub wx: u8,
    pub wy: u8,

    pub bgp: u8,
    pub obp0: u8,
    pub obp1: u8,

    win_enabled: bool,  // Toggles window display on/off.
    win_code_hi: bool,

    bg_enabled: bool,   // Toggles BG tile display on/off.
    bg_code_hi: bool,   // Toggles where we look for BG code data. true: 0x9C00-0x9FFF, false: 0x9800-0x9BFF
    bg_data_hi: bool,   // Toggles where we look for BG tile data. true: 0x8800-0x97FF, false: 0x8000-0x8FFF

    obj_enabled: bool,  // Toggles display of sprites on/off.
    obj_tall: bool,     // If true, we're rendering 8x16 sprites.


    pub vram: [u8; 0x2000], // 0x8000 - 0x9FFF
    pub oam: [u8; 0xA0],// 0xFE00 - 0xFDFF

    scanline_objs: Vec<OAMEntry>,

    pt_state: PixelTransferState,

    pub framebuffer: [u32; ::SCREEN_SIZE],
    fb_pos: usize,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            enabled: false, state: OAMSearch, cycles: 0,
            scy: 0, scx: 0,
            ly: 0, lyc: 0,
            wx: 0, wy: 0,
            bgp: 0, obp0: 0, obp1: 0,
            win_enabled: false, win_code_hi: false,
            bg_enabled: false, bg_code_hi: false, bg_data_hi: true,
            obj_enabled: false, obj_tall: false,
            vram: [0; 0x2000], oam: [0; 0xA0],
            pt_state: PixelTransferState{scratch: [0; 8], pixel_fifo: VecDeque::new(), flush: false, x: 0},
            scanline_objs: Vec::new(),
            framebuffer: [0; 160*144], fb_pos: 0,
        }
    }

    pub fn is_vblank(&self) -> bool {
        if self.ly == 144 {
            if self.cycles == 0 && self.state == VBlank {
                return true;
            }
        }

        false
    }

    pub fn oam_read(&self, addr: u16) -> u8 {
        match self.state {
            HBlank(_) | VBlank => self.oam[addr as usize],
            OAMSearch | PixelTransfer => 0xFF,
        }
    }

    pub fn oam_write(&mut self, addr: u16, v: u8) {
        match self.state {
            HBlank(_) | VBlank => { self.oam[addr as usize] = v },
            OAMSearch | PixelTransfer => { },
        }
    }

    pub fn vram_read(&self, addr: u16) -> u8 {
        match self.state {
            OAMSearch | HBlank(_) | VBlank => self.vram[addr as usize],
            PixelTransfer => 0xFF,
        }
    }

    pub fn vram_write(&mut self, addr: u16, v: u8) {
        match self.state {
            OAMSearch | HBlank(_) | VBlank => { self.vram[addr as usize] = v },
            PixelTransfer => { },
        }
    }

    fn next_state(&mut self, new: PPUState) {
        self.cycles = 0;
        self.state = new;
    }

    // Advances PPU display by a single clock cycle. Basically, all the video magic happens in here.
    pub fn advance(&mut self) -> Option<::Interrupt> {
        if !self.enabled {
            return None
        }

        // Each scanline is 114 clock cycles. 20 for OAM search, 43+ for pixel transfer, remaining for hblank.
        match self.state {
            OAMSearch => {
                self.cycles += 1;
                self.oam_search();
                None
            },
            PixelTransfer => {
                self.cycles += 1;
                self.pixel_transfer();
                None
            },
            HBlank(n) => {
                self.cycles += 1;
                if self.cycles == n {
                    self.ly += 1;

                    if self.ly == 144 {
                        self.next_state(VBlank);
                        Some(::Interrupt::VBlank)
                    } else {
                        self.next_state(OAMSearch);
                        None
                    }
                } else {
                    None
                }
            },
            VBlank => {
                self.cycles += 1;
                if self.cycles == 114 {
                    if self.ly == 154 {
                        self.ly = 0;
                        self.scanline_objs.clear();
                        self.fb_pos = 0;
                        self.next_state(OAMSearch);
                    } else {
                        self.ly += 1;
                        self.next_state(VBlank);
                    }
                }

                None
            }
        }
    }

    // The first stage of a scanline. Takes 20 cycles.
    // In this stage, we search the OAM table for OBJs that overlap the current line (LY).
    // I'm sure it's totally different in hardware, but we simulate this process by searching 2 bytes
    // of OAM each cycle. OAM is 40 bytes and OAM Search takes 20 cycles, so this divides nicely.
    // TODO: need to resolve sprite conflicts based on x pos here.
    fn oam_search(&mut self) {
        let mut idx = 0;
        // TODO: this should be 16 in big sprite mode
        let h = 8;
        while self.scanline_objs.len() < 10 && idx < 2 {
            let obj = self.read_oam_entry((self.cycles-1) * 2 + idx);
            if obj.x > 0 && self.ly + 16 >= obj.y && self.ly + 16 < obj.y + h {
                self.scanline_objs.push(obj);
            }
            idx += 1;
        }

        if self.cycles == 20 {
            self.pt_state.x = 0;
            self.pt_state.flush = false;
            self.next_state(PixelTransfer);
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
    fn pixel_transfer(&mut self) {
        if self.pt_state.x < 160 && self.pt_state.flush {
            for pix in self.pt_state.scratch.iter() {
                self.pt_state.pixel_fifo.push_back(*pix);
            }
            self.pt_state.x += 8;
            self.pt_state.flush = false;
        } else if self.pt_state.x < 160 {
            if self.bg_enabled {
                let code_base_addr = if self.bg_code_hi { 0x1C00 } else { 0x1800 };
                let data_base_addr = if self.bg_data_hi { 0x0800 } else { 0 };

                let scanline_x = self.pt_state.x as u16;
                let scy = self.scy as u16;
                let scx = self.scx as u16;
                let ly = self.ly as u16;

                let tile_y = ((scy + ly) % 8) as usize;
                let bg_y = ((ly + scy) / 8) % 32;

                for i in 0..8 {
                    // TODO: document this madness and extract out some constants.

                    // Figure out which BG tile we're rendering.
                    let bg_x = ((scanline_x + scx + i) / 8) % 32;
                    let tile_num = (bg_y * 32 + bg_x) as usize;
                    let tile = self.vram[(code_base_addr + tile_num) as usize] as usize;

                    let tile_addr = data_base_addr + (tile * 16);

                    // Now figure out which pixels in this particular tile we need.
                    let tile_x = (i + scx + scanline_x) % 8;

                    let lo = self.vram[tile_addr + (tile_y * 2) + 0] >> (7 - tile_x);
                    let hi = (self.vram[tile_addr + (tile_y * 2) + 1] >> (7 - tile_x)) << 1;
                    let result = (lo & 1) | (hi & 2);

                    self.pt_state.scratch[i as usize] = result;
                }
            } else {
                for i in 0..8 {
                    self.pt_state.scratch[i] = 0;
                }
            }

            self.pt_state.flush = true;
        }

        if self.pt_state.pixel_fifo.len() >= 12 || self.pt_state.x == 160 {
            // Send out 4 pixels.
            for _ in 0..4 {
                let pix = self.pt_state.pixel_fifo.pop_front().unwrap() as u32;
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

        if self.pt_state.x == 160 && self.pt_state.pixel_fifo.len() == 0 {
            let hblank_cycles = 51+43 - self.cycles;
            self.next_state(HBlank(hblank_cycles));
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

        lcdc |= if self.enabled     { LCDC_LCD_ENABLED } else { 0 };
        lcdc |= if self.win_enabled { LCDC_WIN_ENABLED } else { 0 };
        lcdc |= if self.win_code_hi { LCDC_WIN_CODE_HI } else { 0 };
        lcdc |= if self.bg_data_hi  { 0 } else { LCDC_BG_DATA_HI };
        lcdc |= if self.bg_code_hi  { LCDC_BG_CODE_HI } else { 0 };
        lcdc |= if self.obj_tall    { LCDC_OBJ_COMP_HI } else { 0 };
        lcdc |= if self.obj_enabled { LCDC_OBJ_ENABLED } else { 0 };
        lcdc |= if self.bg_enabled  { LCDC_BG_ENABLED } else { 0 };

        lcdc
    }

    // Update PPU state based on new LCDC value.
    pub fn set_lcdc(&mut self, v: u8) {
        // println!("Updating LCDC! {:b}", v);

        // Are we enabling LCD from a previously disabled state?
        if v & LCDC_LCD_ENABLED > 0 && !self.enabled {
            self.enabled = true;
            self.next_state(OAMSearch);
            self.fb_pos = 0;
            self.ly = 0;
            self.scanline_objs.clear();
        } else if v & LCDC_LCD_ENABLED == 0 && self.enabled {
            // TODO: check current state here, can't switch LCD off in the middle of pixel transfer.
            if self.state != VBlank {
                panic!("Tried to disable LCD in incorrect state: {:?}", self.state);
            }
            self.enabled = false;
        }

        self.win_enabled = v & LCDC_WIN_ENABLED > 0;
        self.win_code_hi = v & LCDC_WIN_CODE_HI > 0;
        self.bg_data_hi  = v & LCDC_BG_DATA_HI == 0;
        self.bg_code_hi  = v & LCDC_BG_CODE_HI > 0;
        self.obj_tall    = v & LCDC_OBJ_COMP_HI > 0;
        self.obj_enabled = v & LCDC_OBJ_ENABLED > 0;
        self.bg_enabled  = v & LCDC_BG_ENABLED > 0;
    } 

    // Compute value of the STAT register.
    pub fn get_stat(&self) -> u8 {
        panic!("fuck");
        let stat = match self.state {
            HBlank(_) => 0,
            VBlank => 1,
            OAMSearch => 2,
            PixelTransfer => 3,
        };
        stat
    }

    // Update PPU state based on new STAT value.
    pub fn set_stat(&mut self, _v: u8) {
    }

    pub fn dma_ok(&self) -> bool {
        match self.state {
            HBlank(_) | VBlank => true,
            _ => false,
        }
    }
}
