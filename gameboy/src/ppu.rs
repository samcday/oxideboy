// TODO: restore bg_enabled support
// TODO: investigate sprite vs BG priority issues
const DEFAULT_PALETTE: [u8; 4] = [0, 1, 2, 3];

// Models the 4 states the PPU can be in when it is active.
#[derive(Clone, Debug, PartialEq)]
pub enum PPUState {
    OAMSearch,
    PixelTransfer,
    HBlank(u16),     // Number of cycles to remain in HBlank for.
    VBlank,
}
use self::PPUState::{*};

struct PixelTransferState {
    ppu_cycles: u16,
    cycle_budget: u16,
    cycle_countdown: u8,
    scanline: [u8; 176],
    scanline_prio: [bool; 176],
    x: u8,
    code_addr: usize,
    fetch_obj: bool,
    tile_y: usize,
    in_win: bool,
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
    pub enabled: bool,          // Master switch to turn LCD on/off.
    pub state: PPUState,
    pub prev_state: PPUState,   // STAT reports the current mode perpetually 1 cycle late
    cycles: u16,                // Counts how many CPU cycles have elapsed in the current PPU stage.

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

    if_: u8,

    cb: &'cb mut FnMut(&[u32]),
}

impl <'cb> PPU<'cb> {
    pub fn new(cb: &'cb mut FnMut(&[u32])) -> PPU<'cb> {
        PPU {
            enabled: false, state: OAMSearch, prev_state: OAMSearch, cycles: 0,
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
                cycle_budget: 0,
                ppu_cycles: 0,
                cycle_countdown: 0,
                fetch_obj: false,
                code_addr: 0, tile_y: 0, x: 0,
                scanline: [0; 176], scanline_prio: [false; 176],
                in_win: false,
            },
            scanline_objs: Vec::new(),
            framebuffer: [0; 160*144],
            if_: 0,
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

        self.prev_state = self.state.clone();
        self.cycles += 1;
        self.if_ = 0;

        match self.state {
            // The first stage of a scanline.
            // In this stage, we search the OAM table for OBJs that overlap the current scanline (LY).
            // The actual hardware performs this process over 20 clock cycles. However, since OAM memory
            // is inaccessible during this stage, there's no need to emulate it in a cycle accurate manner.
            // Instead we just perform all the work on the first cycle and spin for the remaining 19.
            OAMSearch => {
                if self.cycles == 1 {
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
                // Otherwise, we wait until we've counted the required number of cycles.
                if self.cycles == n {
                    // Alright, time to move on to the next line.
                    self.ly += 1;

                    // Is the next line off screen? If so, we've reached VBlank.
                    if self.ly == 144 {
                        // Set the VBlank interrupt request.
                        self.if_ |= 0x1;
                        // And also set the STAT interrupt request if VBlank interrupts are enabled in there.
                        if self.interrupt_vblank {
                            self.if_ |= 0x2;
                        }
                        self.next_state(VBlank);
                    } else {
                        if self.interrupt_oam {
                            self.if_ |= 0x2;
                        }
                        if self.interrupt_lyc && self.ly == self.lyc {
                            self.if_ |= 0x2;
                        }
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

                    if self.interrupt_oam {
                        self.if_ |= 0x2;
                    }
                    if self.interrupt_lyc && self.ly == self.lyc {
                        self.if_ |= 0x2;
                    }
                    self.next_state(OAMSearch);
                }
            }
        }

        self.if_
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
                if x < 168 {
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
            // TODO: explain this.
            self.pt_state.cycle_budget = (172 + (self.scx % 8)).into();

            self.pt_state.ppu_cycles = 0;
            self.pt_state.cycle_countdown = 12;
            self.pt_state.x = 8 - (self.scx % 8);
            self.pt_state.fetch_obj = false;
            self.pt_state.in_win = false;

            // TODO: we're latching the memory addr for BG code map reads here.
            // If SCX changes mid scanline what is supposed to happen?
            let code_base_addr = if self.bg_code_hi { 0x1C00 } else { 0x1800 };
            let ly = self.ly as usize;
            let scy = self.scy as usize;
            let scx = self.scx as usize;
            self.pt_state.code_addr = code_base_addr + (((((ly + scy) / 8) % 32) * 32) + ((scx / 8) % 32));
            self.pt_state.tile_y = (scy + ly) % 8;

            let mut stall_buckets = [0; 21];
            let mut extra = 0;
            for (_, obj_x) in &self.scanline_objs {
                let idx = (obj_x / 8) as usize;
                stall_buckets[idx] = (5u16.saturating_sub((*obj_x as u16) % 8)).max(stall_buckets[idx]);
                extra += 6;
            }
            extra += stall_buckets.iter().sum::<u16>();
            self.pt_state.cycle_budget += extra & !3;
        }

        for _ in 0..4 {
            self.pt_state.ppu_cycles += 1;
            if self.pt_state.ppu_cycles >= self.pt_state.cycle_budget {
                break;
            }

            self.pt_state.cycle_countdown -= 1;
            if self.pt_state.cycle_countdown > 0 {
                continue;
            }

            if self.pt_state.fetch_obj {
                let (obj_idx, obj_x) = self.scanline_objs[self.scanline_objs.len() - 1];
                let obj = self.read_oam_entry(obj_idx);
                let obj_y = if obj.vert_flip {
                    (if self.obj_tall { 16 } else { 8 }) - (self.ly + 16 - obj.y) as usize
                } else {
                    (self.ly + 16 - obj.y) as usize
                };
                let tile_addr = ((obj.code as usize) * 16) + obj_y * 2;
                let palette = if obj.palette == 1 { &self.obp1 } else { &self.obp0 };
                let mut lo = self.vram[tile_addr];
                let mut hi = self.vram[tile_addr + 1];
                if obj.horz_flip {
                    // This insanity flips the order of the bits in each byte using dark sorcery.
                    // http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith64BitsDiv
                    lo = (((lo as u64) * 0x0202020202 & 0x010884422010) % 1023) as u8;
                    hi = (((hi as u64) * 0x0202020202 & 0x010884422010) % 1023) as u8;
                }
                let obj_x = obj_x as usize;
                Self::write_tile(&mut self.pt_state.scanline[obj_x..], &mut self.pt_state.scanline_prio[obj_x..], lo, hi, true, !obj.priority, palette);
                self.scanline_objs.pop();
                self.pt_state.fetch_obj = false;
            } else if self.pt_state.x < 176 {
                // Fetch BG tile and write it into FIFO.
                let tile = self.vram[self.pt_state.code_addr];
                let tile_addr = (if !self.bg_data_lo {
                    (0x1000 + ((tile as u8 as i8 as i16) * 16)) as usize
                } else {
                    ((tile as usize) * 16)
                }) + self.pt_state.tile_y * 2;
                let mut lo = self.vram[tile_addr];
                let mut hi = self.vram[tile_addr + 1];
                let x = self.pt_state.x as usize;
                Self::write_tile(&mut self.pt_state.scanline[x..], &mut self.pt_state.scanline_prio[x..], lo, hi, false, false, &self.bgp);
                self.pt_state.x = (self.pt_state.x + 8).min(176);
                self.pt_state.code_addr = (self.pt_state.code_addr&!0x1F)|(((self.pt_state.code_addr+1)&0x1F));
            }

            let mut pending_objs = self.obj_enabled && !self.scanline_objs.is_empty();
            let mut next_obj_x = if pending_objs {
                (self.scanline_objs[self.scanline_objs.len() - 1]).1
            } else { 0 };

            // Did we just cross over window boundary?
            self.pt_state.cycle_countdown = 8;
            if self.win_enabled && self.ly >= self.wy && !self.pt_state.in_win && self.pt_state.x >= self.wx + 1 {
                self.pt_state.cycle_budget += 6;
                self.pt_state.in_win = true;
                self.pt_state.x = self.wx + 1;
                let code_base_addr = if self.win_code_hi { 0x1C00 } else { 0x1800 };
                // TODO: gotta handle that weird window wrapping thing at 0xa6
                let ly = self.ly as usize;
                let wy = self.wy as usize;
                self.pt_state.code_addr = code_base_addr + (((((ly-wy) / 8) % 32) * 32));
                self.pt_state.tile_y = ((self.ly-self.wy) % 8) as usize;
                self.pt_state.cycle_countdown = 6;
            } else if pending_objs && self.pt_state.x >= next_obj_x + 8 {
                self.pt_state.fetch_obj = true;
                self.pt_state.cycle_countdown = 6;
            }
        }


        if self.pt_state.ppu_cycles >= self.pt_state.cycle_budget {
            for (idx, pix) in self.pt_state.scanline[8..168].iter().enumerate() {
                self.framebuffer[(self.ly as usize) * 160 + idx] = match pix {
                    0 => { 0xE0F8D0FF },
                    1 => { 0x88C070FF },
                    2 => { 0x346856FF },
                    3 => { 0x081820FF },
                    _ => unreachable!("Pixels are 2bpp")
                };
            }

            let hblank_cycles = 51+43 - self.cycles;
            self.next_state(HBlank(hblank_cycles));

            // If HBlank STAT interrupt is enabled, we send it now.
            if self.interrupt_hblank {
                self.if_ |= 0x2;
            }
        }
    }

    // TODO: document
    fn write_tile(dst: &mut [u8], dst_prio: &mut[bool], mut lo: u8, mut hi: u8, sprite: bool, prio: bool, pal: &[u8; 4]) {
        for i in 0..8 {
            let pix = (lo & 1) | ((hi & 1) << 1);
            lo >>= 1; hi >>= 1;
            if 7 - i >= dst.len() {
                continue;
            }
            if sprite {
                if pix == 0 || dst_prio[7-i] || (!prio && dst[7-i] > 0) {
                    continue;
                }
                dst_prio[7-i] = true;
            } else {
                dst_prio[7-i] = false;
            }
            dst[7-i] = pal[pix as usize];
        }
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
        match self.prev_state {
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
        0
            | if self.enabled     { 0b1000_0000 } else { 0 }
            | if self.win_code_hi { 0b0100_0000 } else { 0 }
            | if self.win_enabled { 0b0010_0000 } else { 0 }
            | if self.bg_data_lo  { 0b0001_0000 } else { 0 }
            | if self.bg_code_hi  { 0b0000_1000 } else { 0 }
            | if self.obj_tall    { 0b0000_0100 } else { 0 }
            | if self.obj_enabled { 0b0000_0010 } else { 0 }
            | if self.bg_enabled  { 0b0000_0001 } else { 0 }
    }

    // Update PPU state based on new LCDC value.
    pub fn write_lcdc(&mut self, v: u8) {
        let enabled = v & 0b1000_0000 > 0;
        // Are we enabling LCD from a previously disabled state?
        if enabled && !self.enabled {
            self.enabled = true;
            self.next_state(OAMSearch);
            self.ly = 0;
        } else if !enabled && self.enabled {
            if let VBlank = self.state {
                self.enabled = false;
                // Clear the framebuffer to white.
                unsafe { ::std::ptr::write_bytes(self.framebuffer.as_mut_ptr(), 255, self.framebuffer.len()); }
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
            | match self.prev_state {
                HBlank(_)             => 0b0000_0000,
                VBlank                => 0b0000_0001,
                OAMSearch             => 0b0000_0010,
                PixelTransfer         => 0b0000_0011}
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
    fn test_scanline_pt_cycles_bg() {
        let test = |scx| {
            let cb = &mut |_: &[u32]| {};
            let mut ppu = PPU::new(cb);
            ppu.ly = 66;
            ppu.scx = scx;
            ppu.enabled = true;
            for _ in 0..20 { ppu.advance(); }
            assert_eq!(ppu.state, PPUState::PixelTransfer);

            let mut cycles = 0;
            while ppu.state == PPUState::PixelTransfer {
                cycles += 1;
                ppu.advance();
            }
            cycles
        };

        assert_eq!(43, test(0));
        assert_eq!(44, test(1));
        assert_eq!(45, test(7));
        assert_eq!(43, test(8));
    }

    #[test]
    fn test_scanline_pt_cycles_window() {
        let test = |scx, wx| {
            let cb = &mut |_: &[u32]| {};
            let mut ppu = PPU::new(cb);
            ppu.wy = 66;
            ppu.wx = wx;
            ppu.ly = 66;
            ppu.scx = scx;
            ppu.enabled = true;
            ppu.win_enabled = true;
            for _ in 0..20 { ppu.advance(); }
            assert_eq!(ppu.state, PPUState::PixelTransfer);

            let mut cycles = 0;
            while ppu.state == PPUState::PixelTransfer {
                cycles += 1;
                ppu.advance();
            }
            cycles
        };

        assert_eq!(45, test(0, 7));
        assert_eq!(47, test(7, 7));
        assert_eq!(45, test(8, 7));
    }

    #[test]
    fn test_scanline_pt_cycles_obj() {
        let test = |sprites: Vec<u8>| {
            let cb = &mut |_: &[u32]| {};
            let mut ppu = PPU::new(cb);
            ppu.ly = 66;
            ppu.enabled = true;
            ppu.obj_enabled = true;
            for (n, x) in sprites.iter().enumerate() {
                ppu.oam[(n * 4) as usize + 1] = *x;
                ppu.oam[(n * 4) as usize] = 66 + 16;
            }
            for _ in 0..20 { ppu.advance(); }
            assert_eq!(ppu.scanline_objs.len(), sprites.len());
            assert_eq!(ppu.state, PPUState::PixelTransfer);
 
            let mut cycles = 0;
            while ppu.state == PPUState::PixelTransfer {
                cycles += 1;
                ppu.advance();
            }
            cycles
        };

        assert_eq!(45, test(vec![0]));
        assert_eq!(45, test(vec![1]));
        assert_eq!(45, test(vec![2]));
        assert_eq!(45, test(vec![3]));
        assert_eq!(44, test(vec![4]));
        assert_eq!(44, test(vec![5]));
        assert_eq!(44, test(vec![6]));
        assert_eq!(44, test(vec![7]));
        assert_eq!(45, test(vec![8]));
        assert_eq!(45, test(vec![9]));
        assert_eq!(45, test(vec![10]));
        assert_eq!(45, test(vec![11]));
        assert_eq!(44, test(vec![12]));
        assert_eq!(44, test(vec![13]));
        assert_eq!(44, test(vec![14]));
        assert_eq!(44, test(vec![15]));
        assert_eq!(45, test(vec![16]));

        assert_eq!(45, test(vec![0]));
        assert_eq!(47, test(vec![0, 0]));
        assert_eq!(48, test(vec![0, 0, 0]));
        assert_eq!(50, test(vec![0, 0, 0, 0]));
        assert_eq!(51, test(vec![0, 0, 0, 0, 0]));

        assert_eq!(48, test(vec![0, 8]));
    }
}
