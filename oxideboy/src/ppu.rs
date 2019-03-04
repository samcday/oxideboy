//! The PPU is responsible for rasterizing a pretty 160x144 picture to the Gameboy LCD screen 59.7 times a second. It's
//! easily the most complicated part of the Gameboy system.
//! Right now, this implementation is *not* cycle accurate during "Mode 3" (pixel transfer). As a result, advanced
//! graphics tricks will not render correctly. First we're focussing on getting perfect timing accuracy, then we'll
//! focus on Mode 3 accuracy.

use crate::interrupt::{Interrupt, InterruptController};
use crate::util::BIT_REVERSE_TABLE;
use std::slice;

type Color = (u32, u32, u32);
pub const SCREEN_SIZE: usize = 160 * 144;
const DEFAULT_PALETTE: Palette = Palette { entries: [0, 3, 3, 3] };
const COLOR_MAPPING: [Color; 4] = [
    (0xE0, 0xF8, 0xD0),
    (0x88, 0xC0, 0x70),
    (0x34, 0x68, 0x56),
    (0x08, 0x18, 0x20),
];

#[derive(Debug, Default)]
pub struct Ppu {
    tiles: Vec<u8>,         // 0x8000 - 0x97FF
    tilemap: Vec<u8>,       // 0x9800 - 0x9FFF
    pub oam: Vec<OAMEntry>, // 0xFE00 - 0xFDFF

    pub enabled: bool,         // 0xFF40 LCDC register bit 7
    win_code_area_hi: bool,    // 0xFF40 LCDC register bit 6
    win_enabled: bool,         // 0xFF40 LCDC register bit 5
    pub bg_tile_area_lo: bool, // 0xFF40 LCDC register bit 4
    bg_code_area_hi: bool,     // 0xFF40 LCDC register bit 3
    obj_tall_mode: bool,       // 0xFF40 LCDC register bit 2
    pub obj_enabled: bool,     // 0xFF40 LCDC register bit 1
    pub bg_enabled: bool,      // 0xFF40 LCDC register bit 0

    interrupt_lyc: bool,    // 0xFF41 STAT register bit 6
    interrupt_oam: bool,    // 0xFF41 STAT register bit 5
    interrupt_vblank: bool, // 0xFF41 STAT register bit 4
    interrupt_hblank: bool, // 0xFF41 STAT register bit 3

    pub scy: u8, // 0xFF42 SCY register
    pub scx: u8, // 0xFF43 SCX register
    pub ly: u8,  // 0xFF44 LY register
    pub lyc: u8, // 0xFF45 LYC register

    pub bgp: Palette,  // 0xFF47 BGP register
    pub obp0: Palette, // 0xFF48 OBP0 register
    pub obp1: Palette, // 0xFF49 OBP1 register
    pub wy: u8,        // 0xFF4A WY register
    pub wx: u8,        // 0xFF4B WX register

    reported_mode: u8,
    oam_accessible: bool,
    vram_accessible: bool,
    stat_lyc_match: bool,

    pub mode: Mode,
    pub scanline_objs: Vec<(usize, usize)>,
    pub mode_cycles: u8,
    pub framebuffer: Vec<u32>,
    framebuf_colors: [u32; 4],

    mode3_extra_cycles: u8,
}

/// The OAMEntry struct is packed in C representation (no padding) so that we can efficiently access it as a
/// contiguous byte array when performing DMA copies and OAM memory reads/writes from the CPU.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct OAMEntry {
    pub y: u8,
    pub x: u8,
    pub code: u8,
    pub attrs: u8,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Palette {
    entries: [u8; 4],
}

#[derive(Debug, PartialEq)]
pub enum Mode {
    Mode0(u8),  // HBlank (51 or fewer cycles, depending on how long previous Mode3 took)
    FirstMode0, // When enabling the LCD, the first line starts in a shortened Mode0 that then goes straight to Mode0.
    Mode1,      // VBlank (1140 cycles)
    Mode2,      // OAM table scan (20 cycles)
    Mode3,      // Pixel transfer (43+ cycles, depending on sprites)
}

pub enum PixelFormat {
    RGBA,
    ABGR,
}

impl Ppu {
    pub fn new() -> Ppu {
        let mut ppu = Ppu {
            tiles: vec![0; 0x1800],
            tilemap: vec![0; 0x800],
            oam: vec![Default::default(); 40],

            bgp: DEFAULT_PALETTE,
            obp0: DEFAULT_PALETTE,
            obp1: DEFAULT_PALETTE,

            scanline_objs: Vec::new(),
            framebuffer: vec![0; SCREEN_SIZE],
            framebuf_colors: [0; 4],

            ..Default::default()
        };

        ppu.set_pixel_format(PixelFormat::RGBA);

        ppu
    }

    pub fn set_pixel_format(&mut self, fmt: PixelFormat) {
        for (i, mapping) in COLOR_MAPPING.iter().enumerate() {
            self.framebuf_colors[i] = fmt.to_u32(*mapping);
        }
    }

    pub fn clock(&mut self, interrupts: &mut InterruptController) -> bool {
        if !self.enabled {
            return false;
        }

        let mut new_frame = false;

        self.mode_cycles += 1;

        // PPU interrupts are weird. In most cases they're delivered one cycle too late.
        // For example when we hit VBlank at line 144, we don't set the VBlank interrupt until the next cycle. Same with
        // the STAT interrupts. Except on line 153, the last line before we start a new frame. In that line, we deliver
        // the interrupts on time (i.e on the first cycle they happened).
        if self.mode_cycles == 1 {
            match self.mode {
                Mode::Mode0(_) => {
                    if self.interrupt_hblank {
                        interrupts.request(Interrupt::Stat);
                    }
                }
                Mode::Mode1 => {
                    if self.ly == 144 {
                        interrupts.request(Interrupt::VBlank);
                        if self.interrupt_vblank {
                            interrupts.request(Interrupt::Stat);
                        }
                        // Line 144 still triggers a STAT interrupt if OAM interrupts are enabled, even though we're not
                        // in a Mode2.
                        if self.interrupt_oam {
                            interrupts.request(Interrupt::Stat);
                        }
                        new_frame = true;
                    }
                    if self.interrupt_lyc && self.lyc > 0 && self.lyc == self.ly {
                        interrupts.request(Interrupt::Stat);
                    }
                }
                Mode::Mode2 => {}
                _ => {}
            }
        }

        match self.mode {
            Mode::Mode0(n) => self.mode_0_hblank(n, false),
            Mode::FirstMode0 => self.mode_0_hblank(18, true),
            Mode::Mode1 => self.mode_1_vblank(),
            Mode::Mode2 => self.mode_2_oam_search(interrupts),
            Mode::Mode3 => self.mode_3_pixel_transfer(),
        }

        new_frame
    }

    pub fn mode_0_hblank(&mut self, hblank_cycles: u8, is_first_mode0: bool) {
        if self.mode_cycles == 1 {
            self.reported_mode = 0;
            self.oam_accessible = true;
            self.vram_accessible = true;
        }

        if self.mode_cycles == hblank_cycles {
            self.mode_cycles = 0;

            if is_first_mode0 {
                self.mode_cycles = 0;
                self.mode = Mode::Mode3;
                return;
            }

            self.ly += 1;
            if self.ly < 144 {
                self.mode = Mode::Mode2;
            } else {
                self.mode = Mode::Mode1;
            }
        }
    }

    pub fn mode_1_vblank(&mut self) {
        if self.mode_cycles == 1 {
            self.reported_mode = 1;
        }

        if self.mode_cycles == 114 {
            self.ly += 1;
            self.mode_cycles = 0;

            if self.ly == 154 {
                self.ly = 0;
                self.mode = Mode::Mode2;
            }
        }
    }

    pub fn mode_2_oam_search(&mut self, interrupts: &mut InterruptController) {
        if self.mode_cycles == 1 {
            self.reported_mode = 2;
            self.oam_accessible = false;
            self.stat_lyc_match = self.ly == self.lyc;

            if self.interrupt_lyc && self.stat_lyc_match {
                interrupts.request(Interrupt::Stat);
            }

            if self.interrupt_oam {
                interrupts.request(Interrupt::Stat);
            }
        }

        // The real hardware needs the 20 cycles to sift through all 40 entries in the OAM table.
        // Since OAM memory is inaccessible to the CPU during this time, there's no need to emulate this
        // incremental process. We just do it all in one go.
        if self.mode_cycles == 20 {
            self.mode_cycles = 0;
            self.mode = Mode::Mode3;

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
            self.scanline_objs
                .sort_unstable_by(|(a_idx, a_x), (b_idx, b_x)| a_x.cmp(b_x).then(a_idx.cmp(b_idx)));
        }
    }

    pub fn mode_3_pixel_transfer(&mut self) {
        if self.mode_cycles == 1 {
            self.reported_mode = 3;
            self.vram_accessible = false;
            self.mode3_extra_cycles = 0;
            self.draw_line();
        }
        if self.mode_cycles == 43 + self.mode3_extra_cycles {
            self.mode = Mode::Mode0(51 - self.mode3_extra_cycles);
            self.mode_cycles = 0;
        }
    }

    /// Draws an entire line of graphical data as efficiently as possible.
    /// This method is used when we can "get away with it". That is, if the CPU isn't trying to modify any PPU-related
    /// registers in the middle of a Mode 3, then we don't need to run the complex (and slower) state machine, we can
    /// just rasterize the whole line in one go.
    pub fn draw_line(&mut self) {
        let mut skip = self.scx % 8; // Throw away the first self.scx % 8 pixels. This is how fine scrolling is done.
        let mut in_win = false;

        // When the PPU clocks pixels out to the LCD, it actually shifts out the first skip number of pixels without the
        // display clock. This has the effect of basically throwing away those pixels, which is how scrolling is
        // achieved. This approach means that the Mode3 takes either 1 or 2 extra cycles, depending on how many pixels
        // were thrown away.
        if skip > 0 {
            self.mode3_extra_cycles += 1;
        }
        if skip > 4 {
            self.mode3_extra_cycles += 1;
        }

        if self.obj_enabled {
            // The PPU stalls sending out pixels while it fetches sprites. Each sprite takes an extra 6 PPU clocks to
            // fetch and draw. Depending on the X position of the sprite, it may also stall the PPU for longer, up to
            // another 5 PPU clocks. This is because while the PPU is fetching background tiles it cannot fetch sprite
            // tiles. So for example if there's a sprite at position 0, the PPU has only just started fetching the next
            // background tile and will need another 5 clocks before it gets around to starting on the 6 clock sprite
            // fetch process. This extra overhead of up to 5 clocks is only paid once per sprite X location.
            let mut sprite_overhead = 0;
            let mut penalty = (0, false);
            for (_, sprite_x) in &self.scanline_objs {
                // Have we paid the sprite penalty for this position yet?
                if penalty.0 != *sprite_x || !penalty.1 {
                    // Nope, calculate it now.
                    penalty.0 = *sprite_x;
                    penalty.1 = true;
                    sprite_overhead += 5 - std::cmp::min(5, (sprite_x + usize::from(self.scx)) % 8);
                }
                sprite_overhead += 6;
            }
            // We calculated the sprite overhead in 4Mhz PPU clock terms, now we divide it back down to the CPU M-cycle
            // speed that we're dealing with in the rest of the emulator.
            self.mode3_extra_cycles += (sprite_overhead / 4) as u8;
        }

        // Determine the x,y co-ordinates in the tilemap.
        let mut map_x = usize::from(self.scx / 8);
        let map_y = ((self.scy as usize) + (self.ly as usize)) / 8 % 32 * 32;

        // Alias self.wx into a usize now since we'll be checking it constantly in the main loop below.
        let wx = usize::from(self.wx).saturating_sub(7);

        // We build the whole line of pixels here first.
        let mut x_pos: usize = 0;
        let mut pixels = [0; 160];
        let mut pixel_prio = [false; 160];

        // Calculate memory offsets into map / tile data areas now, we'll be using these frequently in the loop below.
        let mut map_base = (if self.bg_code_area_hi { 0x400 } else { 0 }) + map_y;
        let mut tile_offset = (((self.scy as usize) + (self.ly as usize)) % 8) * 2;

        // First we build the BG/window pixel data.
        while x_pos < 160 {
            if self.win_enabled && !in_win && self.ly >= self.wy && x_pos >= wx {
                // We're entering window mode. Change where we're looking in the tile map and update some state.
                map_x = 0;
                tile_offset = (((self.ly - self.wy) % 8) * 2) as usize;
                let map_y = ((self.ly as usize) - (self.wy as usize)) / 8 % 32 * 32;
                map_base = if self.win_code_area_hi { 0x400 } else { 0 } + map_y;
                x_pos = wx;
                in_win = true;
            }

            let tile_code = self.tilemap[map_base + map_x] as usize;
            map_x = (map_x + 1) % 32;

            let tile_addr = if !self.bg_tile_area_lo {
                let tile_code = i16::from(tile_code as i8);
                (0x1000 + (tile_code * 0x10)) as usize
            } else {
                tile_code * 0x10
            } + tile_offset;

            let mut tile_lo = BIT_REVERSE_TABLE[self.tiles[tile_addr] as usize] as usize;
            let mut tile_hi = BIT_REVERSE_TABLE[self.tiles[tile_addr + 1] as usize] as usize;

            tile_hi <<= 1;
            for _ in 0..8 {
                if x_pos == 160 {
                    break;
                }

                if skip > 0 {
                    skip -= 1;
                } else {
                    pixels[x_pos] = self.bgp.entry((tile_lo & 0b01) | (tile_hi & 0b10)) as usize;
                    x_pos += 1;
                }

                tile_lo >>= 1;
                tile_hi >>= 1;
            }
        }

        // Next we overlay any sprites.
        if self.obj_enabled {
            for (sprite_idx, sprite_x) in &self.scanline_objs {
                let sprite = &self.oam[*sprite_idx];
                let pal = if sprite.palette() { &self.obp1 } else { &self.obp0 };
                let tile_y = sprite.tile_y(self.ly, self.obj_tall_mode);
                let mut sprite_lo = self.tiles[usize::from(sprite.code) * 0x10 + (tile_y * 2)] as usize;
                let mut sprite_hi = self.tiles[usize::from(sprite.code) * 0x10 + (tile_y * 2) + 1] as usize;

                if !sprite.horz_flip() {
                    sprite_hi = BIT_REVERSE_TABLE[sprite_hi] as usize;
                    sprite_lo = BIT_REVERSE_TABLE[sprite_lo] as usize;
                }

                sprite_hi <<= 1;

                for i in 0..8 {
                    if (sprite_x + i).saturating_sub(8) >= 160 {
                        break;
                    }

                    let sprite_pix = (sprite_lo & 0b01) | (sprite_hi & 0b10);
                    sprite_lo >>= 1;
                    sprite_hi >>= 1;

                    if sprite_x + i < 8 {
                        continue;
                    }
                    if pixel_prio[sprite_x + i - 8] {
                        continue;
                    }

                    if sprite_pix == 0 {
                        continue;
                    }

                    pixel_prio[sprite_x + i - 8] = true;

                    if sprite.priority() && pixels[sprite_x + i - 8] > 0 {
                        continue;
                    }

                    pixels[sprite_x + i - 8] = pal.entry(sprite_pix) as usize;
                }
            }
        }

        // Now we rasterize the pixels into actual colors in the framebuffer.
        let framebuffer_base = usize::from(self.ly) * 160;
        #[allow(clippy::needless_range_loop)]
        for i in 0..160 {
            let pixel = self.framebuf_colors[pixels[i]];
            self.framebuffer[framebuffer_base + i] = pixel;
        }
    }

    pub fn oam_read(&self, addr: usize) -> u8 {
        if !self.oam_accessible {
            return 0xFF; // Reading OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts(self.oam.as_ptr() as *const u8, 160) })[addr]
    }

    pub fn oam_write(&mut self, addr: usize, v: u8) {
        if !self.oam_accessible {
            return; // Writing OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts_mut(self.oam.as_ptr() as *mut u8, 160) })[addr] = v
    }

    pub fn vram_read(&self, addr: u16) -> u8 {
        if !self.vram_accessible {
            return 0xFF; // Reading VRAM during Mode3 is not permitted.
        }

        if addr < 0x1800 {
            self.tiles[addr as usize]
        } else {
            self.tilemap[(addr - 0x1800) as usize]
        }
    }

    pub fn vram_write(&mut self, addr: u16, v: u8) {
        if !self.vram_accessible {
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
        (if self.enabled { 0b1000_0000 } else { 0 })
            | if self.win_code_area_hi { 0b0100_0000 } else { 0 }
            | if self.win_enabled { 0b0010_0000 } else { 0 }
            | if self.bg_tile_area_lo { 0b0001_0000 } else { 0 }
            | if self.bg_code_area_hi { 0b0000_1000 } else { 0 }
            | if self.obj_tall_mode { 0b0000_0100 } else { 0 }
            | if self.obj_enabled { 0b0000_0010 } else { 0 }
            | if self.bg_enabled { 0b0000_0001 } else { 0 }
    }

    /// Write to the 0xFF40 LCDC register
    pub fn reg_lcdc_write(&mut self, v: u8) {
        let enabled = v & 0b1000_0000 > 0;

        if enabled {
            self.enabled = true;

            // When the LCD is first enabled, line 0 starts in a HBlank for 19 cycles, then goes straight to Mode 3
            // (skips mode 2).
            self.mode = Mode::FirstMode0;
            self.reported_mode = 0;
            self.ly = 0;
            self.mode_cycles = 0;
            self.stat_lyc_match = self.ly == self.lyc;
        } else {
            // TODO: check if we're inside a VBlank.
            self.enabled = false;
            self.ly = 0;
            self.mode = Default::default();
            self.oam_accessible = true;
            self.vram_accessible = true;
            self.stat_lyc_match = false;
        }

        self.win_code_area_hi = v & 0b0100_0000 > 0;
        self.win_enabled = v & 0b0010_0000 > 0;
        self.bg_tile_area_lo = v & 0b0001_0000 > 0;
        self.bg_code_area_hi = v & 0b0000_1000 > 0;
        self.obj_tall_mode = v & 0b0000_0100 > 0;
        self.obj_enabled = v & 0b0000_0010 > 0;
        self.bg_enabled = v & 0b0000_0001 > 0;
    }

    /// Read from the 0xFF41 STAT register.
    pub fn reg_stat_read(&self) -> u8 {
        0b1000_0000 // Unused bits
            | self.reported_mode
            | if self.stat_lyc_match   { 0b0000_0100 } else { 0 }
            | if self.interrupt_hblank { 0b0000_1000 } else { 0 }
            | if self.interrupt_vblank { 0b0001_0000 } else { 0 }
            | if self.interrupt_oam    { 0b0010_0000 } else { 0 }
            | if self.interrupt_lyc    { 0b0100_0000 } else { 0 }
    }

    /// Write to the 0xFF41 STAT register.
    pub fn reg_stat_write(&mut self, v: u8) {
        self.interrupt_hblank = v & 0b0000_1000 > 0;
        self.interrupt_vblank = v & 0b0001_0000 > 0;
        self.interrupt_oam = v & 0b0010_0000 > 0;
        self.interrupt_lyc = v & 0b0100_0000 > 0;
    }
}

impl Default for Mode {
    fn default() -> Mode {
        Mode::Mode0(51)
    }
}

impl Palette {
    pub fn pack(self) -> u8 {
        self.entries[0] | (self.entries[1] << 2) | (self.entries[2] << 4) | (self.entries[3] << 6)
    }

    pub fn update(&mut self, v: u8) {
        self.entries[0] = v & 0b0000_0011;
        self.entries[1] = (v & 0b0000_1100) >> 2;
        self.entries[2] = (v & 0b0011_0000) >> 4;
        self.entries[3] = (v & 0b1100_0000) >> 6;
    }

    /// Gets the appropriate entry in the palette, accounting for the current bus conflict if it exists.
    /// When updating any of the BGP/OBPx DMG registers, a bus conflict can occur. Because the CPU takes 4 T-cycles to
    /// update the register, during those cycles the register can be read in different states. In the first cycle, the
    /// value is still unchanged, but in the second cycle the value will be equivalent to the previous value OR'd with
    /// the new value. Even though these effects are unobservable to CPU instructions, they matter for the PPU because
    /// we're calculating pixels at 4Mhz. See the mattcurrie/mealybug-tearoom-tests/m3_bgp_change for a visual example
    /// of this phenomenon.
    pub fn entry(self, idx: usize) -> u8 {
        self.entries[idx]
    }
}

impl OAMEntry {
    fn priority(self) -> bool {
        self.attrs & 0x80 > 0
    }
    fn vert_flip(self) -> bool {
        self.attrs & 0x40 > 0
    }
    fn horz_flip(self) -> bool {
        self.attrs & 0x20 > 0
    }
    fn palette(self) -> bool {
        self.attrs & 0x10 > 0
    }

    // Calculate the y position in the OBJ tile that should be rendered given the current scanline Y position.
    fn tile_y(self, ly: u8, obj_tall: bool) -> usize {
        let base_y = (ly + 16) - self.y;
        (if self.vert_flip() {
            let h = if obj_tall { 16 } else { 8 };
            h - base_y
        } else {
            base_y
        }) as usize
    }
}

impl PixelFormat {
    fn to_u32(&self, c: Color) -> u32 {
        match self {
            PixelFormat::RGBA => ((c.0 << 24) | (c.1 << 16) | (c.2 << 8) | 0xFF),
            PixelFormat::ABGR => ((0xFF << 24) | (c.2 << 16) | (c.1 << 8) | c.0),
        }
    }
}
