use std::slice;
use ::interrupt::{InterruptState, Interrupt};

use self::Mode::{*};

// The default palette colors, in ARGB8888 format.
const COLOR_MAPPING: [u32; 4] = [
    0xFFE0F8D0,
    0xFF88C070,
    0xFF346856,
    0xFF081820,
];
const DEFAULT_PALETTE: Palette = Palette{entries: [0, 3, 3, 3], bus_conflict: 0, old_entries: [0; 4]};

#[derive(Serialize, Deserialize)]
pub struct PPUState {
    pub enabled: bool,          // Master switch to turn LCD on/off.
    pub mode: Mode,
    pub cycles: u16,            // Counts how many CPU cycles have elapsed in the current PPU stage.

    pub scy: u8,
    pub scx: u8,
    pub ly: u8,
    pub lyc: u8,
    pub wx: u8,
    pub wy: u8,
    pub bgp: Palette,
    pub obp0: Palette,
    pub obp1: Palette,

    interrupt_oam: bool,
    interrupt_hblank: bool,
    interrupt_vblank: bool,
    interrupt_lyc: bool,

    win_enabled: bool,    // Toggles window display on/off.
    bg_enabled: bool,     // Toggles BG tile display on/off.
    win_code_addr: usize, // The base address where we look for window tile codes.
    bg_code_addr: usize,  // The base address where we look for background tile codes.
    tile_data_hi: bool,   // If true, tile code is interpreted as signed and based from 0x8800.

    pub obj_enabled: bool,  // Toggles display of OBJs on/off.
    obj_tall: bool,         // If true, we're rendering 8x16 OBJs.

    tiles: Vec<u8>,     // 0x8000 - 0x97FF
    tilemap: Vec<u8>,   // 0x9800 - 0x9FFF
    oam: Vec<OAMEntry>, // 0xFE00 - 0xFDFF

    scanline_objs: Vec<(usize, u8)>,

    pt_state: PixelTransferState,

    framebuffers: Vec<u32>,
    active_fb: bool,
}

// Advances PPU by a single clock cycle. Basically, all the video magic happens in here.
// Before you read this code, go and watch this video: https://www.youtube.com/watch?v=HyzD8pNlpwI&t=29m12s
// Any interrupt codes we return here (STAT / VBlank) are ORd with the main CPU IF register.
pub fn clock(state: &mut PPUState, interrupts: &mut InterruptState) {
    if !state.enabled {
        return;
    }

    state.cycles += 1;

    match state.mode {
        // The first stage of a scanline.
        // In this stage, we search the OAM table for OBJs that overlap the current scanline (LY).
        // The actual hardware performs this process over 20 clock cycles. However, since OAM memory
        // is inaccessible during this stage, there's no need to emulate it in a cycle accurate manner.
        // Instead we just perform all the work on the first cycle and spin for the remaining 19.
        OAMSearch if state.cycles == 1 => {
            oam_search(state);
        }
        OAMSearch if state.cycles == 20 => {
            state.next_mode(PixelTransfer);
        }
        // The second stage of a scanline, where we rasterize the background/window maps and any visible OBJs into
        // actual pixels that go to the LCD.
        PixelTransfer => {
            pixel_transfer(state, interrupts);
        },
        // HBlank stage is a variable number of cycles pause between drawing a line and moving on to the next line. The
        // amount of cycles varies depending on the amount of work that was done during Pixel Transfer. The more OBJs in
        // the scanline, the less time we pause here.
        HBlank(n) if state.cycles == n - 2 => {
            // OAM STAT interrupts are one cycle early, except on the first line (but that interrupt is handled during
            // VBlank state, see below).
            if state.interrupt_oam {
                interrupts.request(Interrupt::Stat);
            }
        }
        HBlank(n) if state.cycles == n => {
            // Alright, time to move on to the next line.
            state.ly += 1;

            // Is the next line off screen? If so, we've reached VBlank.
            if state.ly == 144 {
                // Set the VBlank interrupt request.
                interrupts.request(Interrupt::VBlank);
                // And also set the STAT interrupt request if VBlank interrupts are enabled in there.
                if state.interrupt_vblank {
                    interrupts.request(Interrupt::Stat);
                }
                state.next_mode(VBlank);
            } else {
                if state.interrupt_lyc && state.ly == state.lyc {
                    interrupts.request(Interrupt::Stat);
                }
                state.next_mode(OAMSearch);
            }
        },
        // The VBlank stage is when we've finished rendering all lines for the current frame.
        // In this stage we snooze for 10 invisible scanlines.
        // A whole scanline is ordinarily 114 cycles, so this is essentially 1140 cycles of quiet
        // time in which the game can update OAM, prepare for the next frame, etc.
        VBlank => {
            // Every 114 cycles we advance LY.
            if state.cycles % 114 == 0 {
                state.ly += 1;
            }
            if state.cycles == 1139 && state.interrupt_oam {
                interrupts.request(Interrupt::Stat);
            }
            if state.cycles == 1140 {
                state.ly = 0;

                if state.interrupt_lyc && state.ly == state.lyc {
                    interrupts.request(Interrupt::Stat);
                }
                state.active_fb = !state.active_fb;
                state.next_mode(OAMSearch);
            }
        }
        _ => {}
    }

    state.bgp.bus_conflict = 0;
    state.obp0.bus_conflict = 0;
    state.obp1.bus_conflict = 0;
}

// Searches through the OAM table to find any OBJs that overlap the line we're currently drawing.
pub fn oam_search(state: &mut PPUState) {
    state.scanline_objs.clear();

    let h = if state.obj_tall { 16 } else { 8 };
    let ly_bound = state.ly + (h + 8);

    for (idx, obj) in state.oam.iter().enumerate() {
        if ly_bound >= obj.y && ly_bound < obj.y + h && obj.x < 168 {
            state.scanline_objs.push((idx, obj.x));
        }

        // Each scanline can display a maximum of 10 OBJs.
        if state.scanline_objs.len() == 10 {
            break;
        }
    }

    // Once the list is built, we ensure the elements are ordered by the OBJ x coord.
    // If two OBJs have the same x coord, then we prioritise the OBJ by its position in OAM table.
    // This ordering is important, we build the list here once, but then access it many times during
    // pixel transfer, so this makes things more efficient.
    // Also, in the name of effiency, the list is ordered with the lowest x co-ord coming last.
    // This way, once we've drawn an OBJ, we just pop it off the Vec (which simply decrements len).
    state.scanline_objs.sort_unstable_by(|(a_idx, a_x), (b_idx, b_x)| a_x.cmp(b_x).then(a_idx.cmp(b_idx)));
    state.scanline_objs.reverse();
}

/// The second stage of the scanline. Takes 43+ cycles.
/// This is where we fetch BG/window/OBJ tiles and flush pixels out to the LCD.
/// We emulate the main CPU at a 1Mhz instruction cycle granularity, but the PPU runs at 4Mhz. So the implementation
/// here is modelled on 4Mhz steps.
fn pixel_transfer(state: &mut PPUState, interrupts: &mut InterruptState) {
    if state.cycles == 1 {
        // We're just starting a new Mode 3, reset all the state data.
        state.pt_state = Default::default();
        state.pt_state.fb_pos = ((state.ly as usize) * 160) + (if state.active_fb { ::SCREEN_SIZE } else { 0 });
        state.pt_state.fifo.push_tile(0, 0, false);
        state.pt_state.idle_cycles = 4;
    }

    for _ in 0..4 {
        if state.pt_state.idle_cycles > 0 {
            state.pt_state.idle_cycles -= 1;
            continue;
        }

        if state.pt_state.line_x == 168 {
            break;
        }

        if state.pt_state.fifo.len > 0 {
            let pixel = state.bgp.entry(state.pt_state.fifo.pop_pixel());

            // The first 8 pixels are shifted from the FIFO but are not clocked to the LCD.
            if state.pt_state.line_x >= 8 {
                state.framebuffers[state.pt_state.fb_pos] = COLOR_MAPPING[pixel as usize];
                state.pt_state.fb_pos += 1;
            }

            state.pt_state.line_x += 1;
        }

        // Now we run the fetcher state machine. This is the process that fetches tile data from VRAM and pushes
        // processed pixels into the FIFO.
        match state.pt_state.fetch_state {
            TileFetcherState::SleepRead => {
                state.pt_state.fetch_state = TileFetcherState::Read;
            }
            // Step 1. Figure out which tile we're rendering next by reading from window/BG tile map.
            TileFetcherState::Read => {
                state.pt_state.tile_code = state.tilemap[if state.pt_state.in_win {
                    let map_y = ((state.scy as usize) + (state.ly as usize)) / 8 * 32;
                    state.win_code_addr + (state.pt_state.tile_x) + map_y
                } else {
                    let map_y = ((state.scy as usize) + (state.ly as usize)) / 8 * 32;
                    state.bg_code_addr + (state.pt_state.tile_x) + map_y
                }] as usize;
                state.pt_state.fetch_state = TileFetcherState::SleepFetchLo;
            }
            TileFetcherState::SleepFetchLo => {
                state.pt_state.fetch_state = TileFetcherState::FetchLo;
            }
            // Step 2. We know which tile we're reading now. The row of pixels we need is in two bytes in tile data
            // section of VRAM. Read the first byte, which contains the LSBs of each pixel.
            TileFetcherState::FetchLo => {
                let offset = (((state.scy as usize) + (state.ly as usize)) % 8) * 2;
                state.pt_state.tile_lo = state.tiles[if state.tile_data_hi {
                    let tile_code = state.pt_state.tile_code as i8 as i16;
                    (0x800 + (tile_code * 0x10)) as usize
                } else {
                    state.pt_state.tile_code * 0x10
                } + offset];
                state.pt_state.fetch_state = TileFetcherState::SleepFetchHi;
            }
            TileFetcherState::SleepFetchHi => {
                state.pt_state.fetch_state = TileFetcherState::FetchHi;
            }
            // Step 3. Follow up from step 2 above, now we read the high byte containing MSBs of each pixel.
            TileFetcherState::FetchHi => {
                let offset = (((state.scy as usize) + (state.ly as usize)) % 8) * 2;
                state.pt_state.tile_hi = state.tiles[if state.tile_data_hi {
                    let tile_code = state.pt_state.tile_code as i8 as i16;
                    (0x800 + (tile_code * 0x10)) as usize
                } else {
                    state.pt_state.tile_code * 0x10
                } + offset + 1];
                state.pt_state.fetch_state = TileFetcherState::SleepPush;

                // At the very beginning of the scanline, the first 6 clocks are just throwaway work.
                // if state.pt_state.line_x == 0 {
                //     state.pt_state.fetch_state = TileFetcherState::Read;
                // }
            }
            TileFetcherState::SleepPush => {
                state.pt_state.fetch_state = TileFetcherState::Push;
            }
            // Step 4. We have the tile data we need, now we interleave the bits from both bytes and write the pixel
            // data into the FIFO.
            TileFetcherState::Push => {
                state.pt_state.fifo.push_tile(state.pt_state.tile_hi, state.pt_state.tile_lo, false);
                state.pt_state.tile_x = (state.pt_state.tile_x + 1) % 32;
                state.pt_state.fetch_state = TileFetcherState::SleepRead;
            }
        }

        if state.bgp.bus_conflict > 0 {
            state.bgp.bus_conflict -= 1;
        }
        if state.obp0.bus_conflict > 0 {
            state.obp0.bus_conflict -= 1;
        }
        if state.obp1.bus_conflict > 0 {
            state.obp1.bus_conflict -= 1;
        }
    }

    if state.pt_state.line_x == 168 {
        // Thje HBlank phase runs for 51 cycles *or less*, depending on how much work we did during this Mode 3 (which
        // takes a minimum of 43 cycles).
        let hblank_cycles = 51+43 - state.cycles;
        state.next_mode(HBlank(hblank_cycles));

        // If HBlank STAT interrupt is enabled, we send it now.
        if state.interrupt_hblank {
            interrupts.request(Interrupt::Stat);
        }
    }
}

impl PPUState {
    pub fn new() -> PPUState {
        let mut state = PPUState {
            enabled: false, mode: OAMSearch, cycles: 0,
            scy: 0, scx: 0,
            ly: 0, lyc: 0,
            wx: 0, wy: 0,
            bgp: DEFAULT_PALETTE, obp0: DEFAULT_PALETTE, obp1: DEFAULT_PALETTE,
            win_enabled: false, win_code_addr: 0,
            bg_enabled: false, bg_code_addr: 0, tile_data_hi: true,
            obj_enabled: false, obj_tall: false,
            interrupt_oam: false, interrupt_hblank: false, interrupt_vblank: false, interrupt_lyc: false,
            tiles: vec![0; 0x1800],
            tilemap: vec![0; 0x800],
            oam: vec![Default::default(); 40],
            pt_state: Default::default(),
            scanline_objs: Vec::new(),
            framebuffers: vec![0; ::SCREEN_SIZE * 2],
            active_fb: false,
        };
        state.clear_framebuffers();
        state
    }

    // Gets the last drawn framebuffer (i.e the currently *inactive* one).
    pub fn framebuffer(&self) -> &[u32] {
        &self.framebuffers[if self.active_fb { 0..::SCREEN_SIZE } else { ::SCREEN_SIZE..::SCREEN_SIZE*2 }]
    }

    fn clear_framebuffers(&mut self) {
        for pix in self.framebuffers.iter_mut() {
            *pix = COLOR_MAPPING[0];
        }
    }

    fn next_mode(&mut self, new: Mode) {
        self.cycles = 0;
        self.mode = new;
    }

    pub fn oam_read(&self, addr: usize) -> u8 {
        if self.mode == OAMSearch || self.mode == PixelTransfer {
            return 0xFF; // Reading OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts(self.oam.as_ptr() as *const u8, 160) })[addr]
    }

    pub fn oam_write(&mut self, addr: usize, v: u8) {
        if self.mode == OAMSearch || self.mode == PixelTransfer {
            return; // Writing OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts_mut(self.oam.as_ptr() as *mut u8, 160) })[addr] = v
    }

    pub fn vram_read(&self, addr: u16) -> u8 {
        if self.mode == PixelTransfer {
            return 0xFF; // Reading VRAM during Mode3 is not permitted.
        }

        if addr < 0x1800 {
            self.tiles[addr as usize]
        } else {
            self.tilemap[(addr - 0x1800) as usize]
        }
    }

    pub fn vram_write(&mut self, addr: u16, v: u8) {
        if self.mode == PixelTransfer {
            return; // Writing VRAM during Mode3 is not permitted.
        }

        if addr < 0x1800 {
            self.tiles[addr as usize] = v;
        } else {
            self.tilemap[(addr - 0x1800) as usize] = v;
        }
    }

    // Compute value of the LCDC register.
    pub fn reg_lcdc_read(&self) -> u8 {
        0
            | if self.enabled                { 0b1000_0000 } else { 0 }
            | if self.win_code_addr == 0x400 { 0b0100_0000 } else { 0 }
            | if self.win_enabled            { 0b0010_0000 } else { 0 }
            | if !self.tile_data_hi          { 0b0001_0000 } else { 0 }
            | if self.bg_code_addr == 0x400  { 0b0000_1000 } else { 0 }
            | if self.obj_tall               { 0b0000_0100 } else { 0 }
            | if self.obj_enabled            { 0b0000_0010 } else { 0 }
            | if self.bg_enabled             { 0b0000_0001 } else { 0 }
    }

    // Update PPU state based on new LCDC value.
    pub fn reg_lcdc_write(&mut self, v: u8) {
        let enabled = v & 0b1000_0000 > 0;

        // Are we enabling LCD from a previously disabled state?
        if enabled && !self.enabled {
            self.enabled = true;
            self.next_mode(OAMSearch);
            self.ly = 0;
        } else if !enabled && self.enabled {
            if self.mode != VBlank {
                // TODO: games are not supposed to disable LCD outside of VBlank.
                // When this happens I think we're supposed to draw a single line in the middle of the screen.
            }

            self.enabled = false;
            // Clear the framebuffers.
            self.clear_framebuffers();
        }

        self.win_code_addr = if v & 0b0100_0000 > 0 { 0x400 } else { 0 };
        self.win_enabled   =    v & 0b0010_0000 > 0;
        self.tile_data_hi  = if v & 0b0001_0000 > 0 { false } else { true };
        self.bg_code_addr  = if v & 0b0000_1000 > 0 { 0x400 } else { 0 };
        self.obj_tall      =    v & 0b0000_0100 > 0;
        self.obj_enabled   =    v & 0b0000_0010 > 0;
        self.bg_enabled    =    v & 0b0000_0001 > 0;
    }

    // Compute value of the STAT register.
    pub fn reg_stat_read(&self) -> u8 {
        0b1000_0000 // Unused bits
            | match self.mode {
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
    pub fn reg_stat_write(&mut self, v: u8) {
        self.interrupt_hblank = v & 0b0000_1000 > 0;
        self.interrupt_vblank = v & 0b0001_0000 > 0;
        self.interrupt_oam    = v & 0b0010_0000 > 0;
        self.interrupt_lyc    = v & 0b0100_0000 > 0;
    }

    pub fn dma_ok(&self) -> bool {
        match self.mode {
            HBlank(_) | VBlank => true,
            _ => false,
        }
    }

    pub fn maybe_trash_oam(&mut self) {
        if self.mode == Mode::OAMSearch && self.cycles < 19 {
            // Corruption of OAM is duplicating the current 8 bytes being searched into the next 8 bytes.
            let pos = (self.cycles as usize) * 2;
            self.oam[pos + 2] = self.oam[pos];
            self.oam[pos + 3] = self.oam[pos + 1];
        }
    }
}

// Models the 4 states the PPU can be in when it is active.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub enum Mode {
    HBlank(u16),   // Mode 0
    VBlank,        // Mode 1
    OAMSearch,     // Mode 2
    PixelTransfer, // Mode 3
}

#[derive(Serialize, Deserialize)]
enum TileFetcherState {
    SleepRead,
    Read,
    SleepFetchHi,
    FetchHi,
    SleepFetchLo,
    FetchLo,
    SleepPush,
    Push
}
impl Default for TileFetcherState { fn default() -> Self { TileFetcherState::SleepRead } }

/// The PixelFifo holds pending pixel data that has not yet been flushed to the LCD.
/// As tiles are fetched from VRAM and processed, they're held in the FIFO and pushed to the LCD at a rate of 1 pixel
/// per 4Mhz clock cycle. The FIFO is necessary because OBJ pixels needs to be blended with BG/window pixels before
/// being sent to the LCD.
/// The FIFO is implemented as a very basic bounded (16 entry) ring buffer.
#[derive(Serialize, Deserialize, Default)]
struct PixelFifo {
    pixels: [usize; 16],
    prio: [bool; 16],
    read_pos: usize,
    write_pos: usize,
    len: usize,
}
impl PixelFifo {
    /// Pushes 8 pixels into the FIFO from provided tile data (raw hi and lo bytes read from VRAM).
    /// Each pixel is translated through the provided Palette. The pixels can optionally be flipped horizontally, which
    /// is used by OBJs.
    fn push_tile(&mut self, mut hi: u8, mut lo: u8, flip: bool) {
        if self.len > 0 {
            panic!("push_tile on a FIFO with {} pixels", self.len);
        }

        if !flip {
            // Flip the hi and lo bytes using bit hackery.
            // http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith64BitsDiv
            hi = (((hi as u64) * 0x0202020202u64 & 0x010884422010u64) % 1023) as u8;
            lo = (((lo as u64) * 0x0202020202u64 & 0x010884422010u64) % 1023) as u8;
        }
        let mut lo = lo as usize;
        let mut hi = (hi as usize) << 1;

        self.len += 8;
        for _ in 0..8 {
            self.pixels[self.write_pos] = (lo & 0b01) | (hi & 0b10);
            lo >>= 1; hi >>= 1;
            self.write_pos = (self.write_pos + 1) % 16;
        }
    }

    /// Pops a single pixel out of the FIFO.
    fn pop_pixel(&mut self) -> usize {
        if self.len == 0 {
            panic!("pop_pixel() on an empty FIFO");
        }
        let pixel = self.pixels[self.read_pos];
        self.len -= 1;
        self.read_pos = (self.read_pos + 1) % 16;
        pixel
    }
}

/// Mode 3 is somewhat complex, with a lot of information that needs to be tracked. All of that info is kept in here.
/// It's worth noting that very little data is "cached" here, as the operation of Mode3 can be manipulated while it's
/// in progress by writing to LCDC / SCX / palette registers.
#[derive(Serialize, Deserialize, Default)]
struct PixelTransferState {
    tile_code: usize,   // Code of the current tile being read, used to index into the relevant (BG/window) tile map.
    tile_hi: u8,        // The 1st byte of the current tile.
    tile_lo: u8,        // The 2nd byte of the current tile.
    tile_x: usize,      // X position of current tile in the tile map, wraps at 32 (the tile width of tile map).
    in_win: bool,       // Denotes whether we're rendering window or BG.
    fetch_state: TileFetcherState,
    fifo: PixelFifo,
    line_x: usize,
    fb_pos: usize,      // Location in the framebuffer to write the next pixel.
    idle_cycles: u8,    // At the beginning of Mode 3 we idle for a few cycles, we track that here.
}

/// The OAMEntry struct is packed in C representation (no padding) so that we can view it as a contiguous byte array
/// when performing DMA copies and OAM memory writes from the CPU.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct OAMEntry {
    y: u8,
    x: u8,
    code: u8,
    attrs: u8,
}
impl OAMEntry {
    fn priority(&self)  -> bool { self.attrs & 0x80 > 0 }
    fn vert_flip(&self) -> bool { self.attrs & 0x40 > 0 }
    fn horz_flip(&self) -> bool { self.attrs & 0x20 > 0 }
    fn palette(&self)   -> bool { self.attrs & 0x10 > 0 }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct Palette {
    entries: [u8; 4],
    bus_conflict: u8,
    old_entries: [u8; 4],
}
impl Palette {
    pub fn pack(&self) -> u8 {
        self.entries[0] | (self.entries[1] << 2) | (self.entries[2] << 4) | (self.entries[3] << 6)
    }

    pub fn update(&mut self, v: u8) {
        self.old_entries = self.entries;
        self.bus_conflict = 2;
        self.entries[0] =  v & 0b00000011;
        self.entries[1] = (v & 0b00001100) >> 2;
        self.entries[2] = (v & 0b00110000) >> 4;
        self.entries[3] = (v & 0b11000000) >> 6;
    }

    /// Gets the appropriate entry in the palette, accounting for the current bus conflict if it exists.
    /// When updating any of the BGP/OBPx DMG registers, a bus conflict can occur. Because the CPU takes 4 T-cycles to
    /// update the register, during those cycles the register can be read in different states. In the first cycle, the
    /// value is still unchanged, but in the second cycle the value will be equivalent to the previous value OR'd with
    /// the new value. Even though these effects are unobservable to CPU instructions, they matter for the PPU because
    /// we're calculating pixels at 4Mhz. See the mattcurrie/mealybug-tearoom-tests/m3_bgp_change for a visual example
    /// of this phenomenon.
    pub fn entry(&self, idx: usize) -> u8 {
        if self.bus_conflict == 0 {
            return self.entries[idx];
        }
        if self.bus_conflict == 2 {
            return self.old_entries[idx];
        }
        return self.entries[idx] | self.old_entries[idx];
    }
}
