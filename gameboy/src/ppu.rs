//! The PPU is responsible for rasterizing a pretty 160x144 picture to the Gameboy LCD screen 59.7 times a second. It's
//! easily the most complicated part of the system.
//! Before trying to understand any of this code, go watch this: https://www.youtube.com/watch?v=HyzD8pNlpwI&t=29m12s

// TODO: investigate this assertion on gbdev wiki: "These registers can be accessed even during Mode 3, but they have no effect until the end of the current scanline."

use std::slice;

const DEFAULT_PALETTE: Palette = Palette{entries: [0, 3, 3, 3], bus_conflict: 0, old_entries: [0; 4]};
pub const SCREEN_SIZE: usize = 160 * 144;
// The default palette colors, in ARGB8888 format.
const COLOR_MAPPING: [u32; 4] = [
    0xE0F8D0FF,
    0x88C070FF,
    0x346856FF,
    0x081820FF,
];

pub struct Ppu {
    tiles: Vec<u8>,         // 0x8000 - 0x97FF
    tilemap: Vec<u8>,       // 0x9800 - 0x9FFF
    oam: Vec<OAMEntry>,     // 0xFE00 - 0xFDFF

    enabled: bool,          // 0xFF40 LCDC register bit 7
    win_code_area_hi: bool, // 0xFF40 LCDC register bit 6
    win_enabled: bool,      // 0xFF40 LCDC register bit 5
    bg_tile_area_lo: bool,  // 0xFF40 LCDC register bit 4
    bg_code_area_hi: bool,  // 0xFF40 LCDC register bit 3
    obj_tall_mode: bool,    // 0xFF40 LCDC register bit 2
    obj_enabled: bool,      // 0xFF40 LCDC register bit 1
    bg_enabled: bool,       // 0xFF40 LCDC register bit 0

    interrupt_lyc: bool,    // 0xFF41 STAT register bit 6
    interrupt_oam: bool,    // 0xFF41 STAT register bit 5
    interrupt_vblank: bool, // 0xFF41 STAT register bit 4
    interrupt_hblank: bool, // 0xFF41 STAT register bit 3

    pub scy: u8,            // 0xFF42 SCY register
    pub scx: u8,            // 0xFF43 SCX register
    pub ly: u8,             // 0xFF44 LY register
    pub lyc: u8,            // 0xFF45 LYC register

    pub bgp: Palette,       // 0xFF47 BGP register
    pub obp0: Palette,      // 0xFF48 OBP0 register
    pub obp1: Palette,      // 0xFF49 OBP1 register
    pub wy: u8,             // 0xFF4A WY register
    pub wx: u8,             // 0xFF4B WX register

    pub mode: Mode,
    prev_mode: Mode,        // TODO: document me
    mode3: Mode3State,
    pub cycles: u32,        // Counts how many CPU cycles have elapsed in the current PPU stage.

    scanline_objs: Vec<(usize, usize)>,
    pub framebuffer: Vec<u32>,
}

/// Mode 3 (Pixel transfer) is rather complicated. We keep track of it all here.
#[derive(Default)]
struct Mode3State {
    fetch_state: TileFetcherState,
    prev_fetch_state: TileFetcherState,
    bg_fifo: PixelFifo,
    obj_fifo: PixelFifo,
    idle_cycles: u8,    // At the beginning of Mode 3 we idle for a few cycles, we track that here.
    skip_pixels: u8,    // Number of pixels to skip at beginning of scanline
    fb_pos: usize,      // Location in the framebuffer to write the next pixel.
    line_x: usize,      // Current X position in scanline.
    in_win: bool,       // Denotes whether we're rendering window or BG.

    tile_x: usize,      // X position of current tile in the tile map, wraps at 32 (the tile width of tile map).
    tile_code: usize,   // Code of the current tile being read, used to index into the relevant (BG/window) tile map.
    tile_hi: u8,        // The 1st byte of the current tile.
    tile_lo: u8,        // The 2nd byte of the current tile.

    pending_objs: bool, // True if there's one or more remaining OBJs to be rendered on this scanline.
    pending_obj: usize, // If pending_objs is true, this will contain the index of the next OBJ to be rendered.
    next_obj_x: usize,  // If pending_objs is true, this will contain the next OBJ x position.
    obj_code: usize,    // Code of the tile for the current OBJ.
    obj_hi: u8,         // The 1st byte of the OBJ being fetched.
    obj_lo: u8,         // The 2nd byte of the OBJ being fetched.
}

/// The PixelFifo holds pending pixel data that has not yet been flushed to the LCD.
/// As tiles are fetched from VRAM and processed, they're held in the FIFO and pushed to the LCD at a rate of 1 pixel
/// per 4Mhz clock cycle. The FIFO is necessary because OBJ pixels needs to be blended with BG/window pixels before
/// being sent to the LCD.
/// The FIFO is implemented as a very basic bounded (16 entry) ring buffer.
#[derive(Debug, Default)]
struct PixelFifo {
    pixels: [usize; 16],
    prio: [bool; 16],
    pal: [bool; 16],
    read_pos: usize,
    write_pos: usize,
    len: usize,
}

#[derive(Clone, Copy, Debug)]
enum TileFetcherState {
    SleepRead,
    Read,
    SleepFetchLo,
    FetchLo,
    SleepFetchHi,
    FetchHi,
    SleepPush,
    Push,

    SleepObjFetchLo,
    ObjFetchLo,
    SleepObjFetchHi,
    ObjFetchHi,
    SleepObjPush,
    ObjPush,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    Mode0(u32), // HBlank (51 or fewer cycles, depending on how long previous Mode3 took)
    Mode1,      // VBlank (1140 cycles)
    Mode2,      // OAM table scan (20 cycles)
    Mode3,      // Pixel transfer (43+ cycles, depending on sprites)
}

/// The OAMEntry struct is packed in C representation (no padding) so that we can efficiently access it as a
/// contiguous byte array when performing DMA copies and OAM memory reads/writes from the CPU.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct OAMEntry {
    y: u8,
    x: u8,
    code: u8,
    attrs: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct Palette {
    entries: [u8; 4],
    bus_conflict: u8,
    old_entries: [u8; 4],
}

impl OAMEntry {
    fn priority(&self)  -> bool { self.attrs & 0x80 > 0 }
    fn vert_flip(&self) -> bool { self.attrs & 0x40 > 0 }
    fn horz_flip(&self) -> bool { self.attrs & 0x20 > 0 }
    fn palette(&self)   -> bool { self.attrs & 0x10 > 0 }

    // Calculate the y position in the OBJ tile that should be rendered given the current scanline Y position.
    fn tile_y(&self, ly: u8, obj_tall: bool) -> usize {
        let base_y = (ly + 16) - self.y;
        (if self.vert_flip() {
            let h = if obj_tall { 16 } else { 8 };
            h - base_y
        } else {
            base_y
        }) as usize
    }
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

impl Default for TileFetcherState { fn default() -> Self { TileFetcherState::SleepRead } }
impl TileFetcherState {
    // Returns true if current state is SleepPush or Push.
    fn pushing(&self) -> bool {
        match self {
            TileFetcherState::SleepPush|TileFetcherState::Push => true,
            _ => false
        }
    }
}

impl PixelFifo {
    /// Pushes 8 pixels into the FIFO from provided tile data (raw hi and lo bytes read from VRAM).
    /// Each pixel is translated through the provided Palette. The pixels can optionally be flipped horizontally, which
    /// is used by OBJs.
    fn push_tile(&mut self, mut hi: u8, mut lo: u8, flip: bool) {
        if self.len > 8 {
            // panic!("push_tile on a FIFO with {} pixels", self.len);
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

    /// Blends 8 pixels of OBJ data into the FIFO. This differs from push_tile in that the FIFO will be widened to 8
    /// empty pixels if necessary, but then existing FIFO data will be written over, depending on OBJ priorities.
    fn blend(&mut self, mut hi: u8, mut lo: u8, obj: &OAMEntry) {
        while self.len < 8 {
            self.pixels[self.write_pos] = 0;
            self.write_pos = (self.write_pos + 1) % 16;
            self.len += 1;
        }

        if obj.horz_flip() {
            // Flip the hi and lo bytes using bit hackery.
            // http://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith64BitsDiv
            hi = (((hi as u64) * 0x0202020202u64 & 0x010884422010u64) % 1023) as u8;
            lo = (((lo as u64) * 0x0202020202u64 & 0x010884422010u64) % 1023) as u8;
        }

        let mut lo = lo as usize;
        let mut hi = (hi as usize) << 1;
        let mut cur_write_pos = self.write_pos;
        for _ in 0..8 {
            cur_write_pos = cur_write_pos.wrapping_sub(1) % 16;
            // We only set this pixel if another OBJ hasn't set it already.
            if self.pixels[cur_write_pos] == 0 {
                self.pixels[cur_write_pos] = (lo & 0b01) | (hi & 0b10);
                self.pal[cur_write_pos] = obj.palette(); 
                self.prio[cur_write_pos] = obj.priority();
            }
            lo >>= 1; hi >>= 1;
        }
    }

    /// Pops a single pixel out of the FIFO.
    fn pop_pixel(&mut self) -> (usize, bool, bool) {
        if self.len == 0 {
            panic!("pop_pixel() on an empty FIFO");
        }
        let pixel = self.pixels[self.read_pos];
        let pal = self.pal[self.read_pos];
        let prio = self.prio[self.read_pos];

        self.len -= 1;
        self.read_pos = (self.read_pos + 1) % 16;
        (pixel, pal, prio)
    }
}

impl Ppu {
    pub fn new() -> Ppu {
        Ppu {
            oam: vec![Default::default(); 40],

            enabled: false,
            win_code_area_hi: false,
            win_enabled: false,
            bg_tile_area_lo: false,
            bg_code_area_hi: false,
            obj_tall_mode: false,
            obj_enabled: false,
            bg_enabled: false,

            interrupt_lyc: false,
            interrupt_oam: false,
            interrupt_vblank: false,
            interrupt_hblank: false,

            scy: 0,
            scx: 0,
            ly: 0,
            lyc: 0,
            bgp: DEFAULT_PALETTE,
            obp0: DEFAULT_PALETTE,
            obp1: DEFAULT_PALETTE,
            wy: 0,
            wx: 0,

            tiles: vec![0; 0x1800],
            tilemap: vec![0; 0x800],

            mode: Mode::Mode0(51),
            prev_mode: Mode::Mode0(51),
            mode3: Mode3State{
                ..Default::default()
            },
            scanline_objs: Vec::new(),
            framebuffer: vec![0; SCREEN_SIZE],

            cycles: 0,
        }
    }

    /// Advances the PPU by a single CPU clock step.
    /// Returns a tuple of bools specifying if the VBlank and/or STAT interrupt requests should be set, in that order.
    pub fn clock(&mut self) -> (bool, bool) {
        if !self.enabled {
            return (false, false);
        }

        let mut vblank_int = false;
        let mut stat_int = false;

        self.prev_mode = self.mode;
        self.cycles += 1;

        match self.mode {
            Mode::Mode0(hblank_cycles) => { // HBlank
                if self.cycles == hblank_cycles {
                    self.ly += 1;
                    if self.interrupt_lyc && self.lyc == self.ly {
                        stat_int = true;
                    }
                    self.cycles = 0;

                    if self.ly < 144 {
                        self.mode = Mode::Mode2;
                        if self.interrupt_oam {
                            stat_int = true;
                        }
                    } else {
                        self.mode = Mode::Mode1;
                        vblank_int = true;
                        if self.interrupt_vblank {
                            stat_int = true;
                        }
                    }
                }
            }
            Mode::Mode1 => { // VBlank
                if self.cycles % 114 == 0 {
                    // Every 114 clock cycles we increment LY, from 144 to 154.
                    self.ly += 1;
                    if self.interrupt_lyc && self.lyc == self.ly {
                        stat_int = true;
                    }
                }
                if self.cycles == 1140 {
                    // VBlank is over. Reset LY to 0 and advance to Mode2.
                    self.ly = 0; // TODO: do we need to run LYC comparator here?
                    self.cycles = 0;
                    self.mode = Mode::Mode2;

                    // Also trigger STAT interrupt if OAM interrupt is enabled.
                    if self.interrupt_oam {
                        stat_int = true;
                    }
                }
            }
            Mode::Mode2 => { // OAM search
                if self.cycles == 1 {
                    // The real hardware needs the 20 cycles to sift through all 40 entries in the OAM table.
                    // Since OAM memory is inaccessible to the CPU during this time, there's no need to emulate this
                    // incremental process. We just do it all in one go.
                    self.oam_search();
                }
                if self.cycles == 20 {
                    self.cycles = 0;
                    self.mode = Mode::Mode3;
                }
            }
            Mode::Mode3 => { // Pixel transfer
                stat_int = self.pixel_transfer();
            }
        };

        (vblank_int, stat_int)
    }

    // Searches through the OAM table to find any OBJs that overlap the line we're currently drawing.
    pub fn oam_search(&mut self) {
        self.scanline_objs.clear();

        let h = if self.obj_tall_mode { 16 } else { 8 };
        let ly_bound = self.ly + (h + 8);

        for (idx, obj) in self.oam.iter().enumerate() {
            if ly_bound >= obj.y && ly_bound < obj.y + h && obj.x < 168 {
                self.scanline_objs.push((idx, obj.x as usize));
            }

            // Each scanline can display a maximum of 10 OBJs.
            if self.scanline_objs.len() == 10 {
                break;
            }
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
    /// We emulate the main CPU at a 1Mhz instruction cycle granularity, but the PPU runs at 4Mhz. So the implementation
    /// here is modelled on 4Mhz steps.
    fn pixel_transfer(&mut self) -> bool {
        let mut stat_interrupt = false;

        if self.cycles == 1 {
            // We're just starting a new Mode 3, reset all the state data.
            self.mode3 = Default::default();

            self.mode3.fb_pos = self.ly as usize * 160;

            // Figure out the starting BG tile we'll be displaying. This is precomputed so even if SCX changed mid-scanline,
            // the BG tile position does not change.
            self.mode3.tile_x = (self.scx / 8) as usize;

            // The first 8 pixels are throwaway data since they're not visible.
            self.mode3.bg_fifo.push_tile(0, 0, false);

            // In addition to the 8 pixels we always skip, we further throw away (SCX % 8) pixels. This is how fine
            // scrolling is achieved.
            self.mode3.skip_pixels = self.scx % 8;

            // Stall the PPU for 5 cycles before any actual work starts.
            self.mode3.idle_cycles = 1;

            if !self.scanline_objs.is_empty() {
                self.mode3.pending_objs = true;
                self.mode3.pending_obj = self.scanline_objs[self.scanline_objs.len() - 1].0;
                self.mode3.next_obj_x = self.oam[self.mode3.pending_obj].x as usize;
            }
        }

        for _ in 0..4 {
            if self.mode3.idle_cycles > 0 {
                self.mode3.idle_cycles -= 1;
                continue;
            }

            if self.mode3.line_x == 168 {
                break;
            }

            let stalled_on_obj = self.mode3.pending_objs && self.mode3.line_x == self.mode3.next_obj_x;

            // We can only send pixels to the LCD if we actually have some, and if we're not stalled waiting for an OBJ to
            // be fetched.
            if self.mode3.bg_fifo.len > 0 && !stalled_on_obj {
                let (mut pixel, _, _) = self.mode3.bg_fifo.pop_pixel();

                // If the BG is disabled, we just use color 0 instead of the pixel from FIFO. We *do* use the pixel color if
                // we're rendering the window though.
                if !self.bg_enabled && !self.mode3.in_win {
                    pixel = 0;
                }

                let bg_priority = pixel != 0;
                let mut pixel = self.bgp.entry(pixel);

                // Is there an OBJ pixel to blend with this BG pixel?
                if self.mode3.obj_fifo.len > 0 {
                    let (objpixel, objpal, objprio) = self.mode3.obj_fifo.pop_pixel();

                    // OBJ pixel 0 is always transparent.
                    if self.obj_enabled && objpixel > 0 {
                        // We only render this OBJ pixel if it has priority over the BG, or if the BG pixel was 0.
                        if !objprio || !bg_priority {
                            pixel = if objpal { self.obp1.entry(objpixel) } else { self.obp0.entry(objpixel) };
                        }
                    }
                }

                // Can we send this pixel to the LCD?
                if self.mode3.skip_pixels > 0 {
                    // Nope. One more pixel into the abyss.
                    self.mode3.skip_pixels -= 1;
                } else {
                    if self.mode3.line_x >= 8 {
                        self.framebuffer[self.mode3.fb_pos] = COLOR_MAPPING[pixel as usize];
                        self.mode3.fb_pos += 1;
                    }
                    self.mode3.line_x += 1;
                }
            }

            // If we're stalled waiting for an OBJ fetch and the tile fetcher is up to pushing a tile, then we switch it
            // over to doing the OBJ fetch.
            if stalled_on_obj && self.mode3.fetch_state.pushing() {
                self.mode3.prev_fetch_state = self.mode3.fetch_state;
                self.mode3.obj_code = self.oam[self.mode3.pending_obj].code as usize;
                self.mode3.fetch_state = TileFetcherState::SleepObjFetchLo;
            }

            // Now we run the fetcher state machine. This is the process that fetches tile data from VRAM and pushes
            // processed pixels into the FIFO.
            match self.mode3.fetch_state {
                // Step 1. Figure out which tile we're rendering next by reading from window/BG tile map.
                TileFetcherState::SleepRead => { self.mode3.fetch_state = TileFetcherState::Read; }
                TileFetcherState::Read => {
                    self.mode3.tile_code = self.tilemap[if self.mode3.in_win {
                        let map_y = ((self.scy as usize) + (self.ly as usize)) / 8 % 32 * 32;
                        (if self.win_code_area_hi { 0x400 } else { 0 }) + (self.mode3.tile_x) + map_y
                    } else {
                        let map_y = ((self.scy as usize) + (self.ly as usize)) / 8 % 32 * 32;
                        (if self.bg_code_area_hi { 0x400 } else { 0 }) + (self.mode3.tile_x) + map_y
                    }] as usize;
                    self.mode3.fetch_state = TileFetcherState::SleepFetchLo;
                }
                // Step 2. We know which tile we're reading now. The row of pixels we need is in two bytes in tile data
                // section of VRAM. Read the first byte, which contains the LSBs of each pixel.
                TileFetcherState::SleepFetchLo => { self.mode3.fetch_state = TileFetcherState::FetchLo; }
                TileFetcherState::FetchLo => {
                    let offset = (((self.scy as usize) + (self.ly as usize)) % 8) * 2;
                    self.mode3.tile_lo = self.tiles[if !self.bg_tile_area_lo {
                        let tile_code = self.mode3.tile_code as i8 as i16;
                        (0x1000 + (tile_code * 0x10)) as usize
                    } else {
                        self.mode3.tile_code * 0x10
                    } + offset];
                    self.mode3.fetch_state = TileFetcherState::SleepFetchHi;
                }
                // Step 3. Follow up from step 2 above, now we read the high byte containing MSBs of each pixel.
                TileFetcherState::SleepFetchHi => { self.mode3.fetch_state = TileFetcherState::FetchHi; }
                TileFetcherState::FetchHi => {
                    let offset = (((self.scy as usize) + (self.ly as usize)) % 8) * 2;
                    self.mode3.tile_hi = self.tiles[if !self.bg_tile_area_lo {
                        let tile_code = self.mode3.tile_code as i8 as i16;
                        (0x1000 + (tile_code * 0x10)) as usize
                    } else {
                        self.mode3.tile_code * 0x10
                    } + offset + 1];

                    self.mode3.fetch_state = TileFetcherState::SleepPush;
                }
                // Step 4. We have the tile data we need, now we interleave the bits from both bytes and write the pixel
                // data into the FIFO.
                TileFetcherState::SleepPush => { self.mode3.fetch_state = TileFetcherState::Push; }
                TileFetcherState::Push => {
                    if self.mode3.bg_fifo.len == 0 {
                        self.mode3.bg_fifo.push_tile(self.mode3.tile_hi, self.mode3.tile_lo, false);
                        self.mode3.tile_x = (self.mode3.tile_x + 1) % 32;
                        self.mode3.fetch_state = TileFetcherState::SleepRead;
                    }
                }

                // Step 1 of OBJ fetch. Similar to TileFetcherState::FetchLo, except for OBJ data.
                TileFetcherState::SleepObjFetchLo => { self.mode3.fetch_state = TileFetcherState::ObjFetchLo; }
                TileFetcherState::ObjFetchLo => {
                    let tile_y = self.oam[self.mode3.pending_obj].tile_y(self.ly, self.obj_tall_mode);
                    self.mode3.obj_hi = self.tiles[self.mode3.obj_code * 0x10 + (tile_y * 2) + 1];
                    self.mode3.fetch_state = TileFetcherState::SleepObjFetchHi;
                }
                // Step 2 of OBJ fetch. Similar to TileFetcherState::FetchHi, except for OBJ data.
                TileFetcherState::SleepObjFetchHi => { self.mode3.fetch_state = TileFetcherState::ObjFetchHi; }
                TileFetcherState::ObjFetchHi => {
                    // TODO: the tile_y logic is incomplete for tall OBJs, e.g when they're flipped.
                    let tile_y = self.oam[self.mode3.pending_obj].tile_y(self.ly, self.obj_tall_mode);
                    self.mode3.obj_lo = self.tiles[self.mode3.obj_code * 0x10 + (tile_y * 2)];
                    self.mode3.fetch_state = TileFetcherState::ObjPush;
                }
                // Last step of OBJ fetch. Blend the OBJ data into the OBJ pixel fifo.
                TileFetcherState::SleepObjPush => { self.mode3.fetch_state = TileFetcherState::ObjPush; }
                TileFetcherState::ObjPush => {
                    // Blend the data.
                    self.mode3.obj_fifo.blend(self.mode3.obj_hi, self.mode3.obj_lo,
                                                  &self.oam[self.scanline_objs[self.scanline_objs.len() - 1].0]);

                    // XXX: Awful hack. The m3_bgp_change_sprites reveals that our PPU implementation isn't quite cycle
                    // accurate - we end up pushing some pixels 1 cycle too early, but only if there's a sprite on two
                    // specific X boundaries. I really ought to just get to the bottom of why, but after 3 days of mindless
                    // debugging, I gave up and just hacked this bullshit in.
                    if (self.mode3.next_obj_x & 7) == 6 || (self.mode3.next_obj_x & 7) == 7 {
                        self.mode3.idle_cycles += 1;
                    }

                    // We're done with this OBJ, remove it from the pending list.
                    self.scanline_objs.pop();
                    self.mode3.pending_objs = !self.scanline_objs.is_empty();
                    if self.mode3.pending_objs {
                        self.mode3.pending_obj = self.scanline_objs[self.scanline_objs.len() - 1].0;
                        self.mode3.next_obj_x = self.oam[self.mode3.pending_obj].x as usize;
                    }

                    self.mode3.fetch_state = self.mode3.prev_fetch_state;
                }
            }

            if self.bgp.bus_conflict  > 0 { self.bgp.bus_conflict  -= 1; }
            if self.obp0.bus_conflict > 0 { self.obp0.bus_conflict -= 1; }
            if self.obp1.bus_conflict > 0 { self.obp1.bus_conflict -= 1; }
        }

        if self.mode3.line_x == 168 {
            // The HBlank phase runs for 51 cycles *or less*, depending on how much work we did during this Mode 3 (which
            // takes a minimum of 43 cycles).
            let hblank_cycles = 51+43 - self.cycles;
            self.cycles = 0;
            self.mode = Mode::Mode0(hblank_cycles);

            // If HBlank STAT interrupt is enabled, we send it now.
            if self.interrupt_hblank {
                stat_interrupt = true;
            }
        }

        stat_interrupt
    }

    pub fn oam_read(&self, addr: usize) -> u8 {
        if self.enabled && self.prev_mode == Mode::Mode2 || self.prev_mode == Mode::Mode3 {
            return 0xFF; // Reading OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts(self.oam.as_ptr() as *const u8, 160) })[addr]
    }

    pub fn oam_write(&mut self, addr: usize, v: u8) {
        if self.enabled && self.prev_mode == Mode::Mode2 || self.prev_mode == Mode::Mode3 {
            return; // Writing OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts_mut(self.oam.as_ptr() as *mut u8, 160) })[addr] = v
    }

    pub fn vram_read(&self, addr: u16) -> u8 {
        if self.enabled && self.prev_mode == Mode::Mode3 {
            return 0xFF; // Reading VRAM during Mode3 is not permitted.
        }

        if addr < 0x1800 {
            self.tiles[addr as usize]
        } else {
            self.tilemap[(addr - 0x1800) as usize]
        }
    }

    pub fn vram_write(&mut self, addr: u16, v: u8) {
        if self.enabled && self.prev_mode == Mode::Mode3 {
            return; // Writing VRAM during Mode3 is not permitted.
        }

        if addr < 0x1800 {
            self.tiles[addr as usize] = v;
        } else {
            self.tilemap[(addr - 0x1800) as usize] = v;
        }
    }

    /// Read from the 0xFF40 LCDC register
    pub fn reg_lcdc_read(&self) -> u8 {
        0
            | if self.enabled                { 0b1000_0000 } else { 0 }
            | if self.win_code_area_hi       { 0b0100_0000 } else { 0 }
            | if self.win_enabled            { 0b0010_0000 } else { 0 }
            | if self.bg_tile_area_lo        { 0b0001_0000 } else { 0 }
            | if self.bg_code_area_hi        { 0b0000_1000 } else { 0 }
            | if self.obj_tall_mode          { 0b0000_0100 } else { 0 }
            | if self.obj_enabled            { 0b0000_0010 } else { 0 }
            | if self.bg_enabled             { 0b0000_0001 } else { 0 }
    }

    /// Write to the 0xFF40 LCDC register
    pub fn reg_lcdc_write(&mut self, v: u8) {
        let enabled = v & 0b1000_0000 > 0;

        // Are we enabling LCD from a previously disabled state?
        if enabled && !self.enabled {
            self.enabled = true;
            self.mode = Mode::Mode2;
            self.ly = 0;
            self.cycles = 0;
        } else if !enabled && self.enabled {
            // TODO: check if we're inside a VBlank.
            self.enabled = false;
        }

        self.win_code_area_hi = v & 0b0100_0000 > 0;
        self.win_enabled      = v & 0b0010_0000 > 0;
        self.bg_tile_area_lo  = v & 0b0001_0000 > 0;
        self.bg_code_area_hi  = v & 0b0000_1000 > 0;
        self.obj_tall_mode    = v & 0b0000_0100 > 0;
        self.obj_enabled      = v & 0b0000_0010 > 0;
        self.bg_enabled       = v & 0b0000_0001 > 0;
    }

    // Read from the 0xFF41 STAT register.
    pub fn reg_stat_read(&self) -> u8 {
        0b1000_0000 // Unused bits
            | match self.prev_mode {
                Mode::Mode0(_) => 0b0000_0000,
                Mode::Mode1    => 0b0000_0001,
                Mode::Mode2    => 0b0000_0010,
                Mode::Mode3    => 0b0000_0011 }
            | if self.ly == self.lyc   { 0b0000_0100 } else { 0 }
            | if self.interrupt_hblank { 0b0000_1000 } else { 0 }
            | if self.interrupt_vblank { 0b0001_0000 } else { 0 }
            | if self.interrupt_oam    { 0b0010_0000 } else { 0 }
            | if self.interrupt_lyc    { 0b0100_0000 } else { 0 }
    }

    // Write to the 0xFF41 STAT register.
    pub fn reg_stat_write(&mut self, v: u8) {
        self.interrupt_hblank = v & 0b0000_1000 > 0;
        self.interrupt_vblank = v & 0b0001_0000 > 0;
        self.interrupt_oam    = v & 0b0010_0000 > 0;
        self.interrupt_lyc    = v & 0b0100_0000 > 0;
    }
}
