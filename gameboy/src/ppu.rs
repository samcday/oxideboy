use std::slice;

const DEFAULT_PALETTE: Palette = Palette{entries: [0, 3, 3, 3], bus_conflict: 0, old_entries: [0; 4]};

pub struct Ppu {
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

    tiles: Vec<u8>,         // 0x8000 - 0x97FF
    tilemap: Vec<u8>,       // 0x9800 - 0x9FFF

    // Temporary.
    prev_mode: Mode,
    pub mode: Mode,
    pub cycle_counter: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    Mode0,      // HBlank (51 or fewer cycles, depending on how long previous Mode3 took)
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

            prev_mode: Mode::Mode0,
            mode: Mode::Mode0,
            cycle_counter: 0,
        }
    }

    /// Advances the PPU by a single CPU clock step.
    pub fn clock(&mut self) {
        if !self.enabled {
            return;
        }

        // println!("Okay. Mode={:?} LY={} cycle={}", self.mode, self.ly, self.cycle_counter);
        self.prev_mode = self.mode;
        self.cycle_counter += 1;

        match self.mode {
            Mode::Mode0 => {
                if self.cycle_counter == 51 {
                    self.ly += 1;
                    self.cycle_counter = 0;

                    if self.ly < 144 {
                        self.mode = Mode::Mode2;
                    } else {
                        self.mode = Mode::Mode1;
                    }
                }
            }
            Mode::Mode1 => {
                if self.cycle_counter % 114 == 0 {
                    self.ly += 1;
                }
                if self.cycle_counter == 1140 {
                    self.cycle_counter = 0;
                    self.ly = 0;
                    self.mode = Mode::Mode2;
                }
            }
            Mode::Mode2 => {
                if self.cycle_counter == 20 {
                    self.cycle_counter = 0;
                    self.mode = Mode::Mode3;
                }
            }
            Mode::Mode3 => {
                if self.cycle_counter == 43 {
                    self.cycle_counter = 0;
                    self.mode = Mode::Mode0;
                }
            }
        }
    }

    pub fn oam_read(&self, addr: usize) -> u8 {
        if self.enabled && self.prev_mode == Mode::Mode2 || self.prev_mode == Mode::Mode3 {
            return 0xFF; // Reading OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts(self.oam.as_ptr() as *const u8, 160) })[addr]
    }

    pub fn oam_write(&mut self, addr: usize, v: u8) {
        if self.enabled && self.prev_mode == Mode::Mode2 || self.prev_mode == Mode::Mode3 {
            println!("ah-ha!");
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
                Mode::Mode0 => 0b0000_0000,
                Mode::Mode1 => 0b0000_0001,
                Mode::Mode2 => 0b0000_0010,
                Mode::Mode3 => 0b0000_0011 }
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
