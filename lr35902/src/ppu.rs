// Models the 4 states the PPU can be in when it is active.
#[derive(Debug, PartialEq)]
pub enum PPUState {
    OAMSearch,
    PixelTransfer,
    HBlank,
    VBlank,
}
use self::PPUState::{*};

#[derive(Debug, PartialEq)]
struct PixelTransferState {
    fifo_stalled: bool,     // If true, we're stalled on flushing out pixels until a sprite has been drawn.
    fetch_obj: u8,    // If fifo_stalled is true, this will be set to the sprite index (in OAM table).
    fifo: u64,
    fifo_size: u8,
    scratch: u16,
    x: u8,              // Tracks which pixel in the scanline we're up to fetching.
    flush: bool,        // When true, the next 8 pixels are written into FIFO.
    obj_flush: bool,

    tile_y: usize,
    bg_y: u16,
}

#[derive(Debug)]
struct OAMEntry {
    num: u8,
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
    sleep_cycles: u8,   // If >0, the current cycle will be swallowed and sleep_cycles will be decremented.

    pub scy: u8,
    pub scx: u8,
    pub ly: u8,
    pub lyc: u8,
    pub wx: u8,
    pub wy: u8,

    pub bgp: u8,
    pub obp0: u8,
    pub obp1: u8,

    interrupt_oam: bool,
    interrupt_hblank: bool,
    interrupt_vblank: bool,
    interrupt_lyc: bool,

    win_enabled: bool,  // Toggles window display on/off.
    win_code_hi: bool,

    bg_enabled: bool,   // Toggles BG tile display on/off.
    bg_code_hi: bool,   // Toggles where we look for BG code data. true: 0x9C00-0x9FFF, false: 0x9800-0x9BFF
    bg_data_lo: bool,   // Toggles where we look for BG tile data. true: 0x8000-0x8FFF, false: 0x8800-0x97FF

    obj_enabled: bool,  // Toggles display of sprites on/off.
    obj_tall: bool,     // If true, we're rendering 8x16 sprites.

    pub vram: [u8; 0x2000], // 0x8000 - 0x9FFF
    pub oam: [u8; 0xA0],// 0xFE00 - 0xFDFF

    scanline_objs: Vec<(u8, u8)>,

    pt_state: PixelTransferState,

    pub framebuffer: [u32; ::SCREEN_SIZE],
    fb_pos: usize,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            enabled: false, state: OAMSearch, cycles: 0, sleep_cycles: 0,
            scy: 0, scx: 0,
            ly: 0, lyc: 0,
            wx: 0, wy: 0,
            bgp: 0, obp0: 0, obp1: 0,
            win_enabled: false, win_code_hi: false,
            bg_enabled: false, bg_code_hi: false, bg_data_lo: false,
            obj_enabled: false, obj_tall: false,
            interrupt_oam: false, interrupt_hblank: false, interrupt_vblank: false, interrupt_lyc: false,
            vram: [0; 0x2000], oam: [0; 0xA0],
            pt_state: PixelTransferState{
                scratch: 0, fifo_stalled: false, fetch_obj: 0, fifo: 0, fifo_size: 0, flush: false,
                obj_flush: false, x: 0, tile_y: 0, bg_y: 0},
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

    fn next_state(&mut self, new: PPUState) {
        self.cycles = 0;
        self.state = new;
    }

    // Advances PPU display by a single clock cycle. Basically, all the video magic happens in here.
    // PPU can cause two different interrupts: STAT and VBLANK. What we return here is ORd with the main IF register.
    pub fn advance(&mut self) -> u8 {
        if !self.enabled {
            return 0;
        }

        let mut if_ = 0;

        self.cycles += 1;

        if self.sleep_cycles > 0 {
            self.sleep_cycles -= 1;
        }
        if self.sleep_cycles > 0 {
            return 0;
        }

        match self.state {
            OAMSearch => {
                if self.interrupt_oam && self.cycles == 1 {
                    if_ |= 0x2;
                }
                self.oam_search();
            },
            PixelTransfer => {
                self.pixel_transfer();
            },
            HBlank => {
                if self.interrupt_hblank && self.cycles == 1 {
                    if_ |= 0x2;
                }
                self.ly += 1;

                if self.interrupt_lyc && self.ly == self.lyc {
                    if_ |= 0x2;
                }

                if self.ly == 144 {
                    if_ |= 0x1;
                    if self.interrupt_vblank {
                        if_ |= 0x2;
                    }

                    self.next_state(VBlank);
                } else {
                    self.next_state(OAMSearch);
                }
            },
            VBlank => {
                if self.cycles == 114 {
                    if self.ly < 153 {
                        self.ly += 1;
                        self.next_state(VBlank);
                    } else {
                        self.ly = 0;
                        self.fb_pos = 0;
                        self.next_state(OAMSearch);
                    }
                }
            }
        }

        if_
    }

    // The first stage of a scanline. Takes 20 cycles.
    // In this stage, we search the OAM table for OBJs that overlap the current line (LY).
    // The actual hardware performs this process over 20 clock cycles. However, since OAM memory
    // is inaccessible during this stage, there's no need to emulate it in a cycle accurate manner.
    // Instead we just perform all the work on the first cycle and spin for the remaining 19.
    fn oam_search(&mut self) {
        if self.cycles == 1 {
            self.scanline_objs.clear();

            let mut idx = 0;
            let h = if self.obj_tall { 16 } else { 8 };
            let ly_bound = self.ly + 16;

            while self.scanline_objs.len() < 10 && idx < 40 {
                let x = self.read_obj_x(idx);
                let y = self.read_obj_y(idx);

                if x > 0 && ly_bound >= y && ly_bound < y + h {
                    self.scanline_objs.push((idx, x));
                }
                idx += 1;
            }

            // Once the list is built, we ensure the elements are ordered by the OBJ x coord.
            // If two OBJs have the same x coord, then we prioritise the OBJ by its position in OAM table.
            // This ordering is important, we build the list here once, but then access it many times during
            // pixel transfer, so this makes things more efficient.
            // Also, in the name of effiency, the list is ordered with the lowest x co-ord coming last.
            // This way, as we encounter OBJs, we just pop them off the list.
            self.scanline_objs.sort_unstable_by(|(a_idx, a_x), (b_idx, b_x)| a_x.cmp(b_x).then(a_idx.cmp(b_idx)));
            self.scanline_objs.reverse();
            self.sleep_cycles = 19;
        } else if self.cycles == 20 {
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
    // TODO: window rendering
    fn pixel_transfer(&mut self) {
        // On the first cycle of this stage we ensure our state info is clean.
        if self.cycles == 1 {
            let scy = self.scy as u16;
            let ly = self.ly as u16;
            self.pt_state.tile_y = ((scy + ly) % 8) as usize;
            self.pt_state.bg_y = ((ly + scy) / 8) % 32;
            self.pt_state.x = 0;
            self.pt_state.flush = false;
        }

        self.pixel_transfer_flush_pixels();
        self.pixel_transfer_fetch_pixels();

        if self.pt_state.x == 160 && self.pt_state.fifo_size == 0 {
            let hblank_cycles = 51+43 - self.cycles;
            self.next_state(HBlank);
            self.sleep_cycles = hblank_cycles;
        }
    }

    fn pixel_transfer_flush_pixels(&mut self) {
        // The FIFO must always have at least 8 pixels in it.
        if self.pt_state.fifo_stalled || (self.pt_state.fifo_size < 12 && self.pt_state.x < 160) {
            return;
        }

        // Send out 4 pixels.
        let fifo_x = self.pt_state.x - self.pt_state.fifo_size;

        // First, we need to make sure there isn't a sprite starting somewhere in the next 4 pixels.
        // If there is, we note where it is, and mark ourselves as stalled, then flush the pixels up to the
        // beginning of the sprite.
        // TODO: need to resolve x conflicts here.
        let mut max = 4;

        if self.scanline_objs.len() > 0 {
            let (obj_idx, obj_x) = self.scanline_objs.last().unwrap();
            if fifo_x + 4 > obj_x - 8 {
                max = obj_x - 8 - fifo_x;
                self.pt_state.fifo_stalled = true;
                self.pt_state.fetch_obj = *obj_idx;
            }
        }

        for _ in 0..max {
            let pix = (self.pt_state.fifo & (0b11 << (self.pt_state.fifo_size * 2))) >> (self.pt_state.fifo_size * 2);
            self.pt_state.fifo_size -= 1;
            // TODO: better palette choice here :)
            self.framebuffer[self.fb_pos] = match pix {
                0 => { 255 },
                1 => { 100 },
                2 => { 50 },
                3 => { 0 },
                _ => unreachable!("Pixels are 2bpp")
            };
            self.fb_pos += 1;
        }
    }

    fn pixel_transfer_fetch_pixels(&mut self) {
        if self.pt_state.fifo_stalled {
            if self.pt_state.obj_flush {
                self.scanline_objs.pop();
                self.pt_state.fifo_stalled = false;
                self.pt_state.obj_flush = false;
            } else {
                let obj = self.read_oam_entry(self.pt_state.fetch_obj);
                // TODO: vert flip.
                let obj_pixels = self.get_tile_row((obj.code) as usize, (self.ly + 16 - obj.y) as usize, false, obj.horz_flip);
                // TODO: need to be tracking where pixel came from (BG/sprite num) to resolve conflicts.
                // and ensure empty pixels are preserved on target.
                // self.pt_state.fifo &= !(0b11 << i*2);
                self.pt_state.fifo |= (obj_pixels as u64) << 16;
                self.pt_state.obj_flush = true;
            }
            return;
        }

        if self.pt_state.x < 160 && self.pt_state.flush {
            self.pt_state.fifo <<= 16;
            self.pt_state.fifo |= self.pt_state.scratch as u64;
            self.pt_state.fifo_size += 8;
            self.pt_state.x += 8;
            self.pt_state.flush = false;
            return;
        }

        if self.pt_state.x < 160 {
            self.pt_state.scratch = 0;
            self.pt_state.flush = true;

            if !self.bg_enabled {
                return;
            }

            let code_base_addr = if self.bg_code_hi { 0x1C00 } else { 0x1800 };

            let scanline_x = self.pt_state.x as u16;
            let scx = self.scx as u16;

            let i = 0;
            // TODO: handle SCX offsets properly again.
            let _tile_x = (i + scx + scanline_x) % 8;

            // Figure out which BG tile we're rendering.
            let bg_x = ((scanline_x + scx + i) / 8) % 32;
            let tile_num = (self.pt_state.bg_y * 32 + bg_x) as usize;
            let tile = self.vram[(code_base_addr + tile_num) as usize] as usize;

            // Now figure out which pixels in this particular tile we need.

            let tile_data = self.get_tile_row(tile, self.pt_state.tile_y, !self.bg_data_lo, false);

            self.pt_state.scratch = tile_data;
        }
    }

    // Decodes a row of pixels for a tile.
    // Can fetch tiles from either the low range (0x8000-0x8FFF in VRAM)
    // Or from the high range (0x8800) in which case the tile num is interpreted as a signed offset.
    fn get_tile_row(&self, tile_num: usize, y: usize, hi: bool, flip: bool) -> u16 {
        let tile_addr: usize = if hi {
            0x800 + (((tile_num as u8 as i8) as usize) * 16)
        } else {
            tile_num * 16
        } + y * 2;

        let mut lo = self.vram[tile_addr] as u16;
        let mut hi = self.vram[tile_addr + 1] as u16;

        if flip {
            // This insanity flips the order of the bits in each byte using dark sorcery.
            // http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith64BitsDiv
            lo = (((lo as u64) * 0x0202020202 & 0x010884422010) % 1023) as u16;
            hi = (((hi as u64) * 0x0202020202 & 0x010884422010) % 1023) as u16;
        }

        let mut mask = 0b1111111100000000;

        for _ in 0..8 {
            lo = ((lo & mask) << 1) | (lo & !mask);
            hi = ((hi & mask) << 1) | (hi & !mask);
            mask |= mask >> 1;
        }

        lo | (hi << 1)
    }

    fn read_oam_entry(&self, n: u8) -> OAMEntry {
        let addr = (n * 4) as usize;
        OAMEntry{
            num: n,
            y: self.oam[addr],
            x: self.oam[addr + 1],
            code: self.oam[addr + 2],
            palette: (self.oam[addr + 3] & 0x10) >> 4,
            horz_flip: self.oam[addr + 3] & 0x20 == 0x20,
            vert_flip: self.oam[addr + 3] & 0x40 == 0x40,
            priority: self.oam[addr + 3] & 0x80 == 0x80,
        }
    }

    fn read_obj_x(&self, n: u8) -> u8 {
        self.oam[(n * 4) as usize + 1]
    }

    fn read_obj_y(&self, n: u8) -> u8 {
        self.oam[(n * 4) as usize]
    }


    pub fn read_oam(&self, addr: u16) -> u8 {
        match self.state {
            HBlank | VBlank => self.oam[addr as usize],
            OAMSearch | PixelTransfer => 0xFF,
        }
    }

    pub fn write_oam(&mut self, addr: u16, v: u8) {
        match self.state {
            HBlank | VBlank => { self.oam[addr as usize] = v },
            OAMSearch | PixelTransfer => { },
        }
    }

    pub fn read_vram(&self, addr: u16) -> u8 {
        match self.state {
            OAMSearch | HBlank | VBlank => self.vram[addr as usize],
            PixelTransfer => 0xFF,
        }
    }

    pub fn write_vram(&mut self, addr: u16, v: u8) {
        match self.state {
            OAMSearch | HBlank | VBlank => { self.vram[addr as usize] = v },
            PixelTransfer => { },
        }
    }

    // Compute value of the LCDC register.
    pub fn read_lcdc(&self) -> u8 {
        let mut lcdc = 0;

        lcdc |= if self.enabled     { 0b1000_0000 } else { 0 };
        lcdc |= if self.win_enabled { 0b0100_0000 } else { 0 };
        lcdc |= if self.win_code_hi { 0b0010_0000 } else { 0 };
        lcdc |= if self.bg_data_lo  { 0b0001_0000 } else { 0 };
        lcdc |= if self.bg_code_hi  { 0b0000_1000 } else { 0 };
        lcdc |= if self.obj_tall    { 0b0000_0100 } else { 0 };
        lcdc |= if self.obj_enabled { 0b0000_0010 } else { 0 };
        lcdc |= if self.bg_enabled  { 0b0000_0001 } else { 0 };

        lcdc
    }

    // Update PPU state based on new LCDC value.
    pub fn write_lcdc(&mut self, v: u8) {
        let enabled = v & 0b1000_0000 > 0;
        // Are we enabling LCD from a previously disabled state?
        if enabled && !self.enabled {
            self.enabled = true;
            self.next_state(OAMSearch);
            self.fb_pos = 0;
            self.ly = 0;
            self.sleep_cycles = 0;
        } else if !enabled && self.enabled {
            // TODO: check current state here, can't switch LCD off in the middle of pixel transfer.
            if self.state != VBlank {
                panic!("Tried to disable LCD in incorrect state: {:?}", self.state);
            }
            self.enabled = false;
        }

        self.win_enabled = v & 0b0100_0000 > 0;
        self.win_code_hi = v & 0b0010_0000 > 0;
        self.bg_data_lo  = v & 0b0001_0000 > 0;
        self.bg_code_hi  = v & 0b0000_1000 > 0;
        self.obj_tall    = v & 0b0000_0100 > 0;
        self.obj_enabled = v & 0b0000_0010 > 0;
        self.bg_enabled  = v & 0b0000_0001 > 0;
    }

    // Compute value of the STAT register.
    pub fn read_stat(&self) -> u8 {
        0
            | match self.state {
                HBlank        => 0b00,
                VBlank        => 0b01,
                OAMSearch     => 0b10,
                PixelTransfer => 0b11}
            | if self.ly == self.lyc   { 0b0000_0100 } else { 0 }
            | if self.interrupt_hblank { 0b0000_1000 } else { 0 }
            | if self.interrupt_vblank { 0b0001_0000 } else { 0 }
            | if self.interrupt_oam    { 0b0010_0000 } else { 0 }
            | if self.interrupt_lyc    { 0b0100_0000 } else { 0 }
    }

    // Update PPU state based on new STAT value.
    pub fn write_stat(&mut self, v: u8) {
        self.interrupt_hblank = v & 0b0000_1000 > 0;
        self.interrupt_vblank = v & 0b0001_0000 > 0;
        self.interrupt_oam    = v & 0b0010_0000 > 0;
        self.interrupt_lyc    = v & 0b0100_0000 > 0;
    }

    pub fn dma_ok(&self) -> bool {
        match self.state {
            HBlank | VBlank => true,
            _ => false,
        }
    }
}
