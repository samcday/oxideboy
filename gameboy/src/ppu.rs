const DEFAULT_PALETTE: [u8; 4] = [0, 1, 2, 3];

// Models the 4 states the PPU can be in when it is active.
#[derive(Debug)]
pub enum PPUState {
    OAMSearch,
    PixelTransfer,
    HBlank(u16),     // Number of cycles to remain in HBlank for.
    VBlank,
}
use self::PPUState::{*};

struct PixelFlusher {
    fifo: PixelFifo,    // The pixels waiting to go out to the LCD.
    stalled: bool,      // Set to true if we're waiting on pixel fetcher to fetch OBJ.
    x: u8,              // The x position on screen we're up to flushing. Offset by 8 pixels.
    skip: u8,           // Tracks how many pixels to throw away at the beginning of a scanline.
}

enum PixelFetchState {
    ReadTile,
    ReadData0,
    ReadData1,
    Stalled((u16, u16)),
}
struct PixelFetcher {
    state: PixelFetchState,
    obj: bool,                  // Set to true if we're currently fetching OBJ data (instead of BG/window)
    obj_idx: u8,                // If obj=true, this is the index in the OAM table.
    data_loc: usize,            // Memory addr in VRAM where the two tile data bytes will be read from.
    data0: u8,
    map_base: usize,            // The memory addr in VRAM for the start of the 32 byte map row we're in.
    map_x: usize,               // The x position (0-32) for the tile offset past map_base.
    tile_y: usize,
}

struct PixelTransferState {
    flusher: PixelFlusher,
    fetcher: PixelFetcher,
    in_win: bool,
}

// While drawing a scanline, we keep a small number of pixels in a FIFO queue before writing them to framebuffer.
// We operate on this FIFO a lot, so we want it to be as efficient as possible. Furthermore, we also need it
// to track a bit of extra information about which pixels are "writable".
struct PixelFifo {
    fifo: u32,
    obj_mask: u32,
    bg_mask: u32,
    size: u8,
}

impl PixelFifo {
    fn new() -> PixelFifo {
        PixelFifo { fifo: 0, obj_mask: 0, bg_mask: 0, size: 0 }
    }

    fn clear(&mut self) {
        self.size = 0;
    }

    fn len(&self) -> u8 {
        self.size / 2
    }

    // Pushes a BG tile row into the FIFO.
    fn push_bg(&mut self, row: u16, mask: u16) {
        // Shift the bits in the FIFO left by 16. 8 pixels, each pixel has 2 bits.
        self.fifo <<= 16;
        self.bg_mask <<= 16;
        self.obj_mask <<= 16;
        self.fifo |= row as u32;
        self.bg_mask |= mask as u32;
        self.size += 16;
    }

    // Blends OBJ data with the front 8 pixels of the FIFO.
    fn blend_obj(&mut self, row: u16, mask: u16, priority: bool) {
        // Shift the incoming row and mask data across to line up with the 16 bits (8 pixels) we're operating on
        // in the FIFO.
        let mut row: u32 = (row as u32) << (self.size - 16);
        let mut mask: u32 = (mask as u32) << (self.size - 16);

        if !priority {
            // If the OBJ does *not* have priority over the BG, then we can immediately apply the BG
            // mask to the OBJ pixels.
            row &= !(self.bg_mask);
            mask &= !(self.bg_mask);
        }

        // Next, we mask out the new mask based on previous OBJ data in this FIFO.
        // This is so that we correctly handle multiple OBJs overlapping each other. However here we are assuming
        // that the higher priority OBJ was blended first (which is true due to how we priority-sort OAM entries).
        mask &= !(self.obj_mask);

        // Now, we're ready to mask out any pixels in the FIFO based on our processed OBJ mask.
        self.fifo &= !mask;

        row &= mask;

        // And now all that's left to do is merge the final pixel data + mask data into the FIFO. Easy, right?
        self.fifo |= row;
        self.obj_mask |= mask;
    }

    // Pops a single pixel from the FIFO.
    fn pop(&mut self) -> u8 {
        self.size -= 2;
        (((self.fifo & (0b11 << self.size)) >> self.size) & 0b11) as u8
    }
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

pub struct PPU<'cb> {
    pub enabled: bool,      // Master switch to turn LCD on/off.
    pub state: PPUState,
    cycles: u16,         // Counts how many CPU cycles have elapsed in the current PPU stage.

    pub scy: u8,
    pub scx: u8,
    pub ly: u8,
    pub lyc: u8,
    pub wx: u8,
    pub wy: u8,
    bgp: [u8; 4],
    obp0: [u8; 4],
    obp1: [u8; 4],

    interrupt_oam: bool,
    interrupt_hblank: bool,
    interrupt_vblank: bool,
    interrupt_lyc: bool,

    win_enabled: bool,  // Toggles window display on/off.
    win_code_hi: bool,

    bg_enabled: bool,   // Toggles BG tile display on/off.
    bg_code_hi: bool,   // Toggles where we look for BG code data. true: 0x9C00-0x9FFF, false: 0x9800-0x9BFF
    bg_data_lo: bool,   // Toggles where we look for BG tile data. true: 0x8000-0x8FFF, false: 0x8800-0x97FF

    obj_enabled: bool,  // Toggles display of OBJs on/off.
    obj_tall: bool,     // If true, we're rendering 8x16 OBJs.

    pub vram: [u8; 0x2000], // 0x8000 - 0x9FFF
    pub oam: [u8; 0xA0],    // 0xFE00 - 0xFDFF

    scanline_objs: Vec<(u8, u8)>,

    pt_state: PixelTransferState,

    pub framebuffer: [u32; ::SCREEN_SIZE],
    fb_pos: usize,

    cb: &'cb mut FnMut(&[u32])
}

impl <'cb> PPU<'cb> {
    pub fn new(cb: &'cb mut FnMut(&[u32])) -> PPU<'cb> {
        PPU {
            enabled: false, state: OAMSearch, cycles: 0,
            scy: 0, scx: 0,
            ly: 0, lyc: 0,
            wx: 0, wy: 0,
            bgp: DEFAULT_PALETTE, obp0: DEFAULT_PALETTE, obp1: DEFAULT_PALETTE,
            win_enabled: false, win_code_hi: false,
            bg_enabled: false, bg_code_hi: false, bg_data_lo: false,
            obj_enabled: false, obj_tall: false,
            interrupt_oam: false, interrupt_hblank: false, interrupt_vblank: false, interrupt_lyc: false,
            vram: [0; 0x2000], oam: [0; 0xA0],
            pt_state: PixelTransferState {
                flusher: PixelFlusher {
                    fifo: PixelFifo::new(),
                    stalled: false,
                    x: 0,
                    skip: 0,
                },
                fetcher: PixelFetcher {
                    state: PixelFetchState::ReadTile,
                    obj: false, obj_idx: 0,
                    data_loc: 0,
                    data0: 0,
                    map_x: 0, map_base: 0,
                    tile_y: 0,
                },
                in_win: false, },
            scanline_objs: Vec::new(),
            framebuffer: [0; 160*144], fb_pos: 0,
            cb
        }
    }

    pub fn is_vblank(&self) -> bool {
        if let VBlank = self.state {
            self.ly == 144 && self.cycles == 0
        } else {
            false
        }
    }

    fn next_state(&mut self, new: PPUState) {
        self.cycles = 0;
        self.state = new;
    }

    // Advances PPU display by a single clock cycle. Basically, all the video magic happens in here.
    // Before you read this code, go and watch this video: https://www.youtube.com/watch?v=HyzD8pNlpwI&t=29m12s
    // Any interrupt codes we return here (STAT / VBlank) are ORd with the main CPU IF register.
    pub fn advance(&mut self) -> u8 {
        if !self.enabled {
            return 0;
        }

        self.cycles += 1;

        let mut if_ = 0;
        match self.state {
            // The first stage of a scanline.
            // In this stage, we search the OAM table for OBJs that overlap the current scanline (LY).
            // The actual hardware performs this process over 20 clock cycles. However, since OAM memory
            // is inaccessible during this stage, there's no need to emulate it in a cycle accurate manner.
            // Instead we just perform all the work on the first cycle and spin for the remaining 19.
            OAMSearch => {
                if self.cycles == 1 {
                    if self.interrupt_oam {
                        if_ |= 0x2;
                    }
                    if self.interrupt_lyc && self.ly == self.lyc {
                        if_ |= 0x2;
                    }

                    self.oam_search();
                }

                if self.cycles == 20 {
                    self.next_state(PixelTransfer);
                    return 0;
                }
            }
            // The second stage of a scanline, and the most important one. This is where we rasterize
            // the background map, window map, and OBJs into actual pixels that go into the framebuffer.
            PixelTransfer => {
                self.pixel_transfer();
            },
            // HBlank stage is a variable number of cycles pause between drawing a line and moving on
            // to the next line. The amount of cycles varies depending on the amount of work that was
            // done in Pixel Transfer. The more OBJs in the scanline, the less time we pause here.
            HBlank(n) => {
                // On the first cycle we check if STAT register has enabled HBlank interrupts.
                if self.cycles == 1 && self.interrupt_hblank {
                    if_ |= 0x2;
                }

                // Otherwise, we wait until we've counted the required number of cycles.
                if self.cycles == n {
                    // Alright, time to move on to the next line.
                    self.ly += 1;

                    // Is the next line off screen? If so, we've reached VBlank.
                    if self.ly == 144 {
                        // Set the VBlank interrupt request.
                        if_ |= 0x1;
                        // And also set the STAT interrupt request if VBlank interrupts are enabled in there.
                        if self.interrupt_vblank {
                            if_ |= 0x2;
                        }
                        self.next_state(VBlank);
                    } else {
                        self.next_state(OAMSearch);
                    }
                }
            },
            // The VBlank stage is when we've finished rendering all lines for the current frame.
            // In this stage we snooze for 10 invisible scanlines.
            // A whole scanline is ordinarily 114 cycles, so this is essentially 1140 cycles of quiet
            // time in which the game can update OAM, prepare for the next frame, etc.
            VBlank => {
                // Every 114 cycles we advance LY.
                if self.cycles % 114 == 0 {
                    self.ly += 1;
                }
                if self.cycles == 1140 {
                    (self.cb)(&self.framebuffer);
                    self.ly = 0;
                    self.fb_pos = 0;
                    self.next_state(OAMSearch);
                }
            }
        }

        if_
    }

    // Searches through the OAM table to find any OBJs that overlap the line we're currently drawing.
    fn oam_search(&mut self) {
        self.scanline_objs.clear();

        let mut idx = 0;
        let h = if self.obj_tall { 16 } else { 8 };
        let ly_bound = self.ly + 16;

        while self.scanline_objs.len() < 10 && idx < 40 {
            let y = self.read_obj_y(idx);
            if ly_bound >= y && ly_bound < y + h {
                let x = self.read_obj_x(idx);
                if x > 0 {
                    self.scanline_objs.push((idx, x));
                }
            }
            idx += 1;
        }

        // Once the list is built, we ensure the elements are ordered by the OBJ x coord.
        // If two OBJs have the same x coord, then we prioritise the OBJ by its position in OAM table.
        // This ordering is important, we build the list here once, but then access it many times during
        // pixel transfer, so this makes things more efficient.
        // Also, in the name of effiency, the list is ordered with the lowest x co-ord coming last.
        // This way, once we've drawn an OBJ, we just pop it off the Vec (which simply decrements len).
        self.scanline_objs.sort_unstable_by(|(a_idx, a_x), (b_idx, b_x)| a_x.cmp(b_x).then(a_idx.cmp(b_idx)));
        self.scanline_objs.reverse();
    }

    /// The second stage of the scanline. Takes 43+ cycles.
    /// This is where we fetch BG/window/OBJ tiles and flush pixels out to the LCD.
    /// Our PPU code is run in lockstep with the CPU at 1Mhz. However the real PPU runs at 4Mhz/2Mhz, so we simulate that
    /// here by running the flusher steps at 4Mhz and the pixel fetching process at 2Mhz.
    fn pixel_transfer(&mut self) {
        // On the first cycle of this stage we ensure our state info is clean.
        if self.cycles == 1 {
            let scy = self.scy as u16;
            let scx = self.scx as u16;
            let ly = self.ly as u16;
            self.pt_state.in_win = false;

            self.pt_state.flusher.fifo.clear();
            self.pt_state.flusher.stalled = false;
            self.pt_state.flusher.x = 0;
            self.pt_state.flusher.skip = (scx % 8) as u8;

            let code_base_addr = if self.bg_code_hi { 0x1C00 } else { 0x1800 };
            self.pt_state.fetcher.tile_y = ((scy + ly) % 8) as usize;
            self.pt_state.fetcher.map_base = (code_base_addr + (((ly + scy) / 8) % 32) * 32) as usize;
            self.pt_state.fetcher.map_x = ((31 + (scx / 8)) % 32) as usize;
            self.pt_state.fetcher.state = PixelFetchState::ReadTile;
            self.pt_state.fetcher.obj = false;

            // A bit of a cheat here. To simplify our code around OBJ blending, we generate 8 pixels off-screen
            // to the left. We want to be cycle accurate though, doing this extra work consumes more cycles than the
            // real PPU does, so we just pump the extra fetch cycles we need at the beginning.
            self.pt_fetch();
            self.pt_fetch();
            self.pt_fetch();
        }

        self.pt_fetch();
        self.pt_flush();
        self.pt_flush();
        self.pt_fetch();
        self.pt_flush();
        self.pt_flush();

        // Once we've flushed all pixels for this scanline, we can move on to the HBlank stage.
        if self.pt_state.flusher.x == 168 {
            // HBlank runs for a variable number of cycles, depending on how long this pixel transfer took.
            // If there was no OBJs or window on the current line, then pixel transfer takes 43 cycles and HBlank
            // takes 51 cycles.
            let hblank_cycles = 51+43 - self.cycles;
            self.next_state(HBlank(hblank_cycles));
            return;
        }
    }

    /// The pixel transfer FIFO flush runs at 4Mhz. Every cycle it pushes a single pixel from the FIFO to the LCD.
    /// If there's 8 or less pixels in the FIFO, we do nothing for the cycle.
    fn pt_flush(&mut self) {
        let state = &mut self.pt_state.flusher;
        if state.stalled || state.fifo.len() <= 8 {
            // If we're stalled waiting for fetcher to get some OBJ data, or the FIFO doesn't have enough data, then
            // there's nothing to do here.
            return;
        }

        if state.x == 168 {
            // We've pushed out enough pixels.
            return;
        }

        // At the very beginning of the scanline, if SCX modulo 8 is nonzero, we need to throw away that many pixels.
        if state.skip > 0 {
            state.fifo.pop();
            state.skip -= 1;
            return;
        }

        // If window is enabled and we've reached the location where it begins, then we dump the FIFO
        // and switch the fetcher over to window mode.
        if self.win_enabled && !self.pt_state.in_win && state.x == self.wx && self.ly >= self.wy {
            self.pt_state.in_win = true;
            self.pt_state.fetcher.state = PixelFetchState::ReadTile;
            state.fifo.clear();

            let ly = self.ly as u16;
            let wy = self.wy as u16;
            let code_base_addr = if self.win_code_hi { 0x1C00 } else { 0x1800 };
            let win_y = ((ly - wy) / 8) % 32;

            self.pt_state.fetcher.tile_y = ((ly - wy) % 8) as usize;
            self.pt_state.fetcher.map_base = (code_base_addr + win_y * 32) as usize;
            self.pt_state.fetcher.map_x = 0 as usize;

            return;
        }

        // If OBJs are enabled, we need to make sure the next pixel to go out isn't where a sprite starts.
        if self.obj_enabled && !self.scanline_objs.is_empty() {
            let (obj_idx, obj_x) = self.scanline_objs[self.scanline_objs.len() - 1];
            if state.x == obj_x {
                self.pt_state.fetcher.obj = true;
                self.pt_state.fetcher.obj_idx = obj_idx;
                self.pt_state.fetcher.state = PixelFetchState::ReadTile;
                state.stalled = true;
                return;
            }
        }

        let pix = state.fifo.pop();
        let x = state.x;
        state.x += 1;

        // The first 8 pixels are off-screen, so we throw them away.
        // We only generate them so we have something to blend with if there's a sprite partially off screen.
        if x < 8 {
            return;
        }

        self.framebuffer[self.fb_pos] = match pix {
            0 => { 255 },
            1 => { 100 },
            2 => { 50 },
            3 => { 0 },
            _ => unreachable!("Pixels are 2bpp")
        };
        self.fb_pos += 1;
    }

    fn pt_fetch(&mut self) {
        if self.pt_state.fetcher.obj {
            self.pt_fetch_obj();
            return;
        }

        self.pt_fetch_bg();
    }

    fn pt_fetch_obj(&mut self) {
        match self.pt_state.fetcher.state {
            PixelFetchState::ReadTile => {
                let obj = self.read_oam_entry(self.pt_state.fetcher.obj_idx);

                let obj_y = if obj.vert_flip {
                    (if self.obj_tall { 16 } else { 8 }) - (self.ly + 16 - obj.y) as usize
                } else {
                    (self.ly + 16 - obj.y) as usize
                };

                self.pt_state.fetcher.data_loc = ((obj.code as usize) * 16) + obj_y * 2;
                self.pt_state.fetcher.state = PixelFetchState::ReadData0;
            }
            PixelFetchState::ReadData0 => {
                self.pt_state.fetcher.data0 = self.vram[self.pt_state.fetcher.data_loc];
                self.pt_state.fetcher.data_loc += 1;
                self.pt_state.fetcher.state = PixelFetchState::ReadData1;
            }
            PixelFetchState::ReadData1 => {
                let obj = self.read_oam_entry(self.pt_state.fetcher.obj_idx);

                let mut lo = self.pt_state.fetcher.data0;
                let mut hi = self.vram[self.pt_state.fetcher.data_loc];

                if obj.horz_flip {
                    // This insanity flips the order of the bits in each byte using dark sorcery.
                    // http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith64BitsDiv
                    lo = (((lo as u64) * 0x0202020202 & 0x010884422010) % 1023) as u8;
                    hi = (((hi as u64) * 0x0202020202 & 0x010884422010) % 1023) as u8;
                }

                let palette = if obj.palette == 1 { &self.obp1 } else { &self.obp0 };
                let (pixels, mask) = PPU::build_tile(lo, hi, palette);
                self.pt_state.flusher.fifo.blend_obj(pixels, mask, !obj.priority);

                self.scanline_objs.pop();
                self.pt_state.flusher.stalled = false;
                self.pt_state.fetcher.obj = false;
                self.pt_state.fetcher.state = PixelFetchState::ReadTile;
            }
            PixelFetchState::Stalled(_) => {
                panic!("pt_fetch_obj should never stall");
            }
        }
    }

    fn pt_fetch_bg(&mut self) {
        match self.pt_state.fetcher.state {
            PixelFetchState::ReadTile => {
                let tile = self.vram[self.pt_state.fetcher.map_base + self.pt_state.fetcher.map_x] as usize;
                self.pt_state.fetcher.data_loc = if !self.bg_data_lo {
                    (0x1000 + ((tile as u8 as i8 as i16) * 16)) as usize
                } else {
                    (tile * 16)
                } + self.pt_state.fetcher.tile_y * 2;

                self.pt_state.fetcher.state = PixelFetchState::ReadData0;
            }
            PixelFetchState::ReadData0 => {
                self.pt_state.fetcher.data0 = self.vram[self.pt_state.fetcher.data_loc];
                self.pt_state.fetcher.data_loc += 1;
                self.pt_state.fetcher.state = PixelFetchState::ReadData1;
            }
            PixelFetchState::ReadData1 => {
                let lo = self.pt_state.fetcher.data0;
                let hi = self.vram[self.pt_state.fetcher.data_loc];
                let (pixels, mask) = PPU::build_tile(lo, hi, &self.bgp);
                if self.pt_state.flusher.fifo.len() <= 8 {
                    self.pt_state.flusher.fifo.push_bg(pixels, mask);
                    self.pt_state.fetcher.map_x = (self.pt_state.fetcher.map_x + 1) % 32;
                    self.pt_state.fetcher.state = PixelFetchState::ReadTile;
                } else {
                    // We can't push pixels into FIFO just yet, it's too full. We'll try again next cycle.
                    self.pt_state.fetcher.state = PixelFetchState::Stalled((pixels, mask));
                }
            }
            PixelFetchState::Stalled((pixels, mask)) => {
                if self.pt_state.flusher.fifo.len() > 8 {
                    return;
                }
                self.pt_state.flusher.fifo.push_bg(pixels, mask);
                self.pt_state.fetcher.map_x = (self.pt_state.fetcher.map_x + 1) % 32;
                self.pt_state.fetcher.state = PixelFetchState::ReadTile;
                // The stalled state is a bit of a kludge, it doesn't actually consume a cycle. So, we pump
                // the fetcher again.
                return self.pt_fetch_bg();
            }
        }
    }

    // Tile pixels are stored as 2 bytes which must be interleaved. Each pixel is 2 bit. The first byte
    // contains the high bit for each pixel, and the second byte contains the low bits.
    // Both the pixels and a mask are returned. The mask specifies which bits contain active pixels.
    // The mask information is used by PixelFifo when blending OBJs.
    // Finally, we apply the relevant palette transformation to each pixel. The palette can be either
    // the BGP, OBP0 or OBP1 tables. An interesting thing to note here is that we specify the mask based
    // on the original value. This means a pixel with value 00 is considered "transparent" or "not present", even
    // if that pixel then maps to some other value via the palette.
    fn build_tile(lo: u8, hi: u8, pal: &[u8; 4]) -> (u16, u16) {
        let mut lo = lo as u16;
        let mut hi = hi as u16;
        let mut mask: u16 = 0;
        let mut row: u16 = 0;

        // Loop through each bit pair in lo+hi and interleave them. If the result is anything but zero
        // we also set the mask.
        hi <<= 1;
        for i in 0..8 {
            let pix = (lo & 1) | (hi & 2);
            lo >>= 1; hi >>= 1;
            if pix > 0 {
                mask |= 0b11 << (i*2);
            }
            row |= (pal[pix as usize] as u16) << (i * 2);
        }

        (row, mask)
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
            HBlank(_) | VBlank => self.oam[addr as usize],
            OAMSearch | PixelTransfer => 0xFF,
        }
    }

    pub fn write_oam(&mut self, addr: u16, v: u8) {
        match self.state {
            HBlank(_) | VBlank => { self.oam[addr as usize] = v },
            OAMSearch | PixelTransfer => { },
        }
    }

    pub fn read_vram(&self, addr: u16) -> u8 {
        match self.state {
            OAMSearch | HBlank(_) | VBlank => self.vram[addr as usize],
            PixelTransfer => 0xFF,
        }
    }

    pub fn write_vram(&mut self, addr: u16, v: u8) {
        match self.state {
            OAMSearch | HBlank(_) | VBlank => { self.vram[addr as usize] = v },
            PixelTransfer => { },
        }
    }

    // Compute value of the LCDC register.
    pub fn read_lcdc(&self) -> u8 {
        let mut lcdc = 0;

        lcdc |= if self.enabled     { 0b1000_0000 } else { 0 };
        lcdc |= if self.win_code_hi { 0b0100_0000 } else { 0 };
        lcdc |= if self.win_enabled { 0b0010_0000 } else { 0 };
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
        } else if !enabled && self.enabled {
            if let VBlank = self.state {
                self.enabled = false;
            } else {
                panic!("Tried to disable LCD outside of VBlank");
            }
        }

        self.win_code_hi = v & 0b0100_0000 > 0;
        self.win_enabled = v & 0b0010_0000 > 0;
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
                HBlank(_)     => 0b00,
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

    pub fn read_bgp(&self) -> u8 {
        self.bgp[0] | self.bgp[1] << 2 | self.bgp[2] << 4 | self.bgp[3] << 6
    }

    pub fn write_bgp(&mut self, v: u8) {
        self.bgp[0] = v & 0b11;
        self.bgp[1] = (v & 0b1100) >> 2;
        self.bgp[2] = (v & 0b110000) >> 4;
        self.bgp[3] = (v & 0b11000000) >> 6;
    }

    pub fn read_obp0(&self) -> u8 {
        self.obp0[0] | self.obp0[1] << 2 | self.obp0[2] << 4 | self.obp0[3] << 6
    }

    pub fn write_obp0(&mut self, v: u8) {
        self.obp0[0] = v & 0b11;
        self.obp0[1] = (v & 0b1100) >> 2;
        self.obp0[2] = (v & 0b110000) >> 4;
        self.obp0[3] = (v & 0b11000000) >> 6;
    }

    pub fn read_obp1(&self) -> u8 {
        self.obp1[0] | self.obp1[1] << 2 | self.obp1[2] << 4 | self.obp1[3] << 6
    }

    pub fn write_obp1(&mut self, v: u8) {
        self.obp1[0] = v & 0b11;
        self.obp1[1] = (v & 0b1100) >> 2;
        self.obp1[2] = (v & 0b110000) >> 4;
        self.obp1[3] = (v & 0b11000000) >> 6;
    }

    pub fn dma_ok(&self) -> bool {
        match self.state {
            HBlank(_) | VBlank => true,
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_tile() {
        // Check that row and mask are built correctly.
        let (row, mask) = PPU::build_tile(0b10101010, 0b10000001, &[0b00, 0b01, 0b10, 0b11]);
        assert_eq!(row,  0b1100010001000110);
        assert_eq!(mask, 0b1100110011001111);

        // Check that palette transformation is done with correct mask.
        let (row, mask) = PPU::build_tile(0b10101010, 0b10000001, &[0b11, 0b10, 0b01, 0b00]);
        assert_eq!(row,  0b0011101110111001);
        assert_eq!(mask, 0b1100110011001111);
    }

    // Test that basic push / pop is working correctly.
    #[test]
    fn fifo_push_pop() {
        let mut fifo: PixelFifo = PixelFifo::new();

        fifo.push_bg(0b00110100_00000000, 0);
        assert_eq!(fifo.pop(), 0b00);
        assert_eq!(fifo.pop(), 0b11);
        assert_eq!(fifo.pop(), 0b01);
        assert_eq!(fifo.pop(), 0b00);
        // Actual fifo u32 shouldn't be needlessly shifted around when popping.
        assert_eq!(fifo.fifo, 0b0011010000000000);
        assert_eq!(fifo.size, 8);
    }

    // Test the behaviour when overlaying an OBJ onto some BG data.
    #[test]
    fn fifo_obj_blend() {
        let mut fifo: PixelFifo = PixelFifo::new();

        // Test how we blend an OBJ that has priority OVER the background.
        //                      ↓↓    ↓↓ these BG pixels should be overridden by OBJ.
        fifo.push_bg         (0b00_11_01_00_00000000, 0b00111100_00000000);
        fifo.blend_obj       (0b10_00_11_00_00000000, 0b11001100_00000000, true);
        assert_eq!(fifo.fifo, 0b10_11_11_00_00000000);

        // Confirm that a subsequent OBJ with lower priority does not overwrite existing OBJ pixels.
        //                      ↓↓ this pixel *should not* be overridden since a OBJ already drew there.
        //                               ↓↓ this pixel *should* be overridden since an OBJ did not yet draw there.
        fifo.blend_obj       (0b01_00_00_11_00000000, 0b11000011_00000000, true);
        assert_eq!(fifo.fifo, 0b10_11_11_11_00000000);

        // And now test blending an OBJ that does *not* have priority over the background.
        let mut fifo: PixelFifo = PixelFifo::new();

        //                            ↓↓ these BG pixels should be preserved.
        fifo.push_bg         (0b00_11_01_00_00000000, 0b00111100_00000000);
        fifo.blend_obj       (0b10_00_11_00_00000000, 0b11001100_00000000, false);
        assert_eq!(fifo.fifo, 0b10_11_01_00_00000000);
    }
}
