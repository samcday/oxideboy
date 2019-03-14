//! The PPU is responsible for rasterizing a pretty 160x144 picture to the Gameboy LCD screen 59.7 times a second. It's
//! easily the most complicated part of the Gameboy system.
//! Right now, this implementation is *not* cycle accurate during "Mode 3" (pixel transfer). As a result, advanced
//! graphics tricks will not render correctly. First we're focussing on getting perfect timing accuracy, then we'll
//! focus on Mode 3 accuracy.

use crate::interrupt::{Interrupt, InterruptController};
use crate::memory_segment;
use crate::util::BIT_REVERSE_TABLE;
use crate::Context;
use serde::{Deserialize, Serialize};
use std::slice;

pub const SCREEN_SIZE: usize = 160 * 144;
const DEFAULT_PALETTE: Palette = Palette { entries: [0, 3, 3, 3] };

// These are the default mapping we use to convert DMG monochrome palette to the framebuffer RGB565 format.
const COLOR_MAPPING: [u16; 4] = [0xE7DA, 0x8E0E, 0x334A, 0x08c4];

memory_segment! { Tiles; u8; 0x1800 }
memory_segment! { Tilemap; u8; 0x800 }
memory_segment! { OamTable; OAMEntry; 40 }

#[derive(Deserialize, Serialize)]
pub struct Ppu {
    pub tiles: Tiles,     // 0x8000 - 0x97FF
    pub tilemap: Tilemap, // 0x9800 - 0x9FFF
    pub oam: OamTable,    // 0xFE00 - 0xFDFF

    pub enabled: bool,         // 0xFF40 LCDC register bit 7
    win_code_area_hi: bool,    // 0xFF40 LCDC register bit 6
    win_enabled: bool,         // 0xFF40 LCDC register bit 5
    pub bg_tile_area_lo: bool, // 0xFF40 LCDC register bit 4
    bg_code_area_hi: bool,     // 0xFF40 LCDC register bit 3
    obj_tall_mode: bool,       // 0xFF40 LCDC register bit 2
    pub obj_enabled: bool,     // 0xFF40 LCDC register bit 1
    pub bg_enabled: bool,      // 0xFF40 LCDC register bit 0
    stat_interrupts: StatInterrupts,
    // This is the 4 states ANDed together. Only if it goes from false to true will we request a STAT interrupt.
    stat_interrupt_active: bool,
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
    oam_allow_read: bool,
    oam_allow_write: bool,
    vram_allow_read: bool,
    vram_allow_write: bool,
    first_frame: bool,

    pub mode: Mode,
    pub scanline_objs: [(usize, usize); 10],
    pub scanline_obj_count: usize,
    pub mode_cycles: u8,

    mode3_extra_cycles: u8,

    // We're rendering frames at 59.7fps, but very often there's absolutely no need to re-render, if nothing in the PPU
    // registers, OAM, or VRAM changed since the last frame.
    pub dirty: bool,
    pub next_dirty: bool,
}

pub enum OamCorruptionType {
    READ,
    POP,
    PUSH,
    LDI,
    LDD,
}

#[derive(Debug, Default, Deserialize, Serialize)]
pub struct StatInterrupts {
    lyc_enabled: bool,    // 0xFF41 STAT register bit 6
    oam_enabled: bool,    // 0xFF41 STAT register bit 5
    vblank_enabled: bool, // 0xFF41 STAT register bit 4
    hblank_enabled: bool, // 0xFF41 STAT register bit 3
    lyc_active: bool,
    oam_active: bool,
    vblank_active: bool,
    hblank_active: bool,
}

/// The OAMEntry struct is packed in C representation (no padding) so that we can efficiently access it as a
/// contiguous byte array when performing DMA copies and OAM memory reads/writes from the CPU.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
#[repr(C)]
pub struct OAMEntry {
    pub y: u8,
    pub x: u8,
    pub code: u8,
    pub attrs: u8,
}

#[derive(Copy, Clone, Debug, Default, Deserialize, Serialize)]
pub struct Palette {
    entries: [u8; 4],
}

#[derive(Debug, Deserialize, PartialEq, Serialize)]
pub enum Mode {
    Mode0(u8),  // HBlank (51 or fewer cycles, depending on how long previous Mode3 took)
    FirstMode0, // When enabling the LCD, the first line starts in a shortened Mode0 that then goes straight to Mode0.
    Mode1,      // VBlank (1140 cycles)
    Mode2,      // OAM table scan (20 cycles)
    Mode3,      // Pixel transfer (43+ cycles, depending on sprites)
}

impl Default for Ppu {
    fn default() -> Ppu {
        Ppu {
            tiles: Default::default(),
            tilemap: Default::default(),
            oam: Default::default(),

            enabled: false,
            win_code_area_hi: false,
            win_enabled: false,
            bg_tile_area_lo: false,
            bg_code_area_hi: false,
            obj_tall_mode: false,
            obj_enabled: false,
            bg_enabled: false,

            stat_interrupts: Default::default(),
            stat_interrupt_active: false,

            scy: 0,
            scx: 0,
            ly: 0,
            lyc: 0,
            bgp: DEFAULT_PALETTE,
            obp0: DEFAULT_PALETTE,
            obp1: DEFAULT_PALETTE,
            wy: 0,
            wx: 0,

            reported_mode: 0,
            oam_allow_read: true,
            oam_allow_write: true,
            vram_allow_read: true,
            vram_allow_write: true,
            first_frame: false,

            mode: Default::default(),
            scanline_objs: [(0, 0); 10],
            scanline_obj_count: 0,
            mode_cycles: 0,

            mode3_extra_cycles: 0,
            dirty: false,
            next_dirty: false,
        }
    }
}

impl Ppu {
    pub fn new() -> Ppu {
        Default::default()
    }

    pub fn clock(&mut self, context: &mut Context, interrupts: &mut InterruptController) -> bool {
        if !self.enabled {
            return false;
        }

        let mut new_frame = false;
        self.mode_cycles += 1;

        match self.mode {
            Mode::Mode0(n) => self.mode_0_hblank(n, interrupts),
            Mode::FirstMode0 => self.first_mode_0_hblank(),
            Mode::Mode1 => self.mode_1_vblank(interrupts, &mut new_frame),
            Mode::Mode2 => self.mode_2_oam_search(interrupts),
            Mode::Mode3 => self.mode_3_pixel_transfer(context),
        }

        if new_frame {
            // Handling a new frame is slightly finicky with our dirty checking.
            // If we finished the frame and the dirty check was never set, then there's no need to flip buffers. The
            // current frame is still good (nothing changed). Otherwise, we gotta flip.
            if self.dirty && context.enable_video {
                context.swap_framebuffers();
            }
        }

        new_frame
    }

    pub fn first_mode_0_hblank(&mut self) {
        if self.mode_cycles == 1 {
            self.reported_mode = 0;
        }
        if self.mode_cycles == 18 {
            self.mode_cycles = 0;
            self.mode_cycles = 0;
            self.mode = Mode::Mode3;
        }
    }

    pub fn mode_0_hblank(&mut self, hblank_cycles: u8, interrupts: &mut InterruptController) {
        if self.mode_cycles == 1 {
            self.reported_mode = 0;
            self.oam_allow_read = true;

            if self.ly > 1 {
                self.vram_allow_read = true;
            }

            self.update_stat_interrupt(Some(interrupts), |state| {
                state.hblank_active = true;
            });
        }

        if self.mode_cycles == 2 {
            self.oam_allow_write = true;
            self.vram_allow_write = true;

            if self.ly <= 1 {
                self.vram_allow_read = true;
            }
        }

        if self.mode_cycles == hblank_cycles {
            self.mode_cycles = 0;

            self.ly += 1;

            if self.ly != self.lyc {
                self.update_stat_interrupt(Some(interrupts), |state| {
                    state.lyc_active = false;
                });
            }

            // When the PPU is first enabled, OAM is locked out one cycle earlier for lines 0-2.
            if self.first_frame && self.ly <= 2 {
                self.oam_allow_read = false;
            }

            if self.ly < 144 {
                self.mode = Mode::Mode2;
            } else {
                self.mode = Mode::Mode1;
            }
        }
    }

    pub fn mode_1_vblank(&mut self, interrupts: &mut InterruptController, new_frame: &mut bool) {
        if self.mode_cycles == 1 {
            self.reported_mode = 1;

            let lyc_match = self.ly == self.lyc;
            let vblank_start = self.ly == 144;
            self.update_stat_interrupt(Some(interrupts), |state| {
                state.lyc_active = lyc_match;
                if vblank_start {
                    state.vblank_active = true;

                    // Line 144 still triggers a STAT interrupt if OAM interrupts are enabled, even though we're not
                    // in a Mode2.
                    state.oam_active = true;
                }
            });

            if vblank_start {
                *new_frame = true;
                interrupts.request(Interrupt::VBlank);
            }
        }

        if self.mode_cycles == 20 {
            self.update_stat_interrupt(Some(interrupts), |state| {
                state.oam_active = false;
            });
        }

        if self.mode_cycles == 114 {
            self.first_frame = false;
            self.ly += 1;
            self.mode_cycles = 0;

            if self.ly == 154 {
                self.dirty = self.next_dirty;
                self.next_dirty = false;
                self.ly = 0;
                self.mode = Mode::Mode2;
            }
        }
    }

    pub fn mode_2_oam_search(&mut self, interrupts: &mut InterruptController) {
        if self.mode_cycles == 1 {
            self.reported_mode = 2;
            self.oam_allow_read = false;

            let lyc_match = self.ly == self.lyc;
            self.update_stat_interrupt(Some(interrupts), |state| {
                state.oam_active = true;
                state.hblank_active = false;
                state.vblank_active = false;
                state.lyc_active = lyc_match;
            });
        }

        if self.mode_cycles == 2 {
            self.oam_allow_write = false;
        }

        // The real hardware needs the 20 cycles to sift through all 40 entries in the OAM table.
        // Since OAM memory is inaccessible to the CPU during this time, there's no need to emulate this
        // incremental process. We just do it all in one go.
        if self.mode_cycles == 20 {
            self.mode_cycles = 0;
            self.mode = Mode::Mode3;

            self.scanline_obj_count = 0;

            let h = if self.obj_tall_mode { 16 } else { 8 };
            let ly_bound = self.ly + (h + 8);

            for (idx, obj) in self.oam.iter().enumerate() {
                if ly_bound >= obj.y && ly_bound < obj.y + h && obj.x < 168 {
                    self.scanline_objs[self.scanline_obj_count] = (idx, obj.x as usize);
                    self.scanline_obj_count += 1;
                }

                // Each scanline can display a maximum of 10 OBJs.
                if self.scanline_obj_count == 10 {
                    break;
                }
            }

            // Once the list is built, we ensure the elements are ordered by the OBJ x coord.
            // If two OBJs have the same x coord, then we prioritise the OBJ by its position in OAM table.
            // This ordering is important, we build the list here once, but then access it many times during
            // pixel transfer, so this makes things more efficient.
            (&mut self.scanline_objs[0..self.scanline_obj_count])
                .sort_unstable_by(|(a_idx, a_x), (b_idx, b_x)| a_x.cmp(b_x).then(a_idx.cmp(b_idx)));
        }
    }

    pub fn mode_3_pixel_transfer(&mut self, context: &mut Context) {
        if self.mode_cycles == 1 {
            self.reported_mode = 3;
            self.oam_allow_read = false;
            self.oam_allow_write = false;
            self.vram_allow_read = false;
            self.mode3_extra_cycles = 0;

            self.update_stat_interrupt(None, |state| state.oam_active = false);

            self.calculate_line_overhead();

            // If graphics aren't enabled, no need to draw the line.
            if context.enable_video {
                self.draw_line(&mut context.next_framebuffer);
            }
        }

        if self.mode_cycles == 2 {
            self.oam_allow_write = false;
            self.vram_allow_write = false;
        }

        if self.first_frame {
            if self.mode_cycles == 1 {
                self.oam_allow_write = true;
            } else if self.mode_cycles == 2 {
                self.oam_allow_write = false;
            }
        }

        // VRAM gets locked while we're in Mode 3. Except, of course, for the batshit timings of line 0 right after the
        // LCD is enabled.
        if self.ly == 0 && self.first_frame {
            if self.mode_cycles == 1 {
                self.vram_allow_read = true;
            }
            if self.mode_cycles == 2 {
                self.vram_allow_read = false;
            }
        }

        if self.mode_cycles == 43 + self.mode3_extra_cycles {
            self.mode = Mode::Mode0(51 - self.mode3_extra_cycles);
            self.mode_cycles = 0;
        }
    }

    fn calculate_line_overhead(&mut self) {
        // When the PPU clocks pixels out to the LCD, it actually shifts out the first skip number of pixels without the
        // display clock. This has the effect of basically throwing away those pixels, which is how scrolling is
        // achieved. This approach means that the Mode3 takes either 1 or 2 extra cycles, depending on how many pixels
        // were thrown away.
        self.mode3_extra_cycles += match self.scx % 8 {
            1...4 => 1,
            5...7 => 2,
            _ => 0,
        };

        if self.obj_enabled {
            // The PPU stalls sending out pixels while it fetches sprites. Each sprite takes an extra 6 PPU clocks to
            // fetch and draw. Depending on the X position of the sprite, it may also stall the PPU for longer, up to
            // another 5 PPU clocks. This is because while the PPU is fetching background tiles it cannot fetch sprite
            // tiles. So for example if there's a sprite at position 0, the PPU has only just started fetching the next
            // background tile and will need another 5 clocks before it gets around to starting on the 6 clock sprite
            // fetch process. This extra overhead of up to 5 clocks is only paid once per sprite X location.
            let mut sprite_overhead = 0;
            let mut penalty = (0, false);
            // for (_, sprite_x) in &self.scanline_objs {
            for i in 0..self.scanline_obj_count {
                let sprite_x = self.scanline_objs[i].1;
                // Have we paid the sprite penalty for this position yet?
                if penalty.0 != sprite_x || !penalty.1 {
                    // Nope, calculate it now.
                    penalty.0 = sprite_x;
                    penalty.1 = true;
                    sprite_overhead += 5 - std::cmp::min(5, (sprite_x + usize::from(self.scx)) % 8);
                }
                sprite_overhead += 6;
            }
            // We calculated the sprite overhead in 4Mhz PPU clock terms, now we divide it back down to the CPU M-cycle
            // speed that we're dealing with in the rest of the emulator.
            self.mode3_extra_cycles += (sprite_overhead / 4) as u8;
        }
    }

    /// Draws an entire line of graphical data as efficiently as possible.
    /// This method is used when we can "get away with it". That is, if the CPU isn't trying to modify any PPU-related
    /// registers in the middle of a Mode 3, then we don't need to run the complex (and slower) state machine, we can
    /// just rasterize the whole line in one go.
    pub fn draw_line(&mut self, framebuffer: &mut [u16]) {
        let mut skip = self.scx % 8; // Throw away the first self.scx % 8 pixels. This is how fine scrolling is done.
        let mut in_win = false;

        if !self.dirty {
            // If nothing has changed since we last drew this line, then we can skip the rest of the work.
            return;
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
            // for (sprite_idx, sprite_x) in &self.scanline_objs {
            for i in 0..self.scanline_obj_count {
                let (sprite_idx, sprite_x) = self.scanline_objs[i];
                let sprite = &self.oam[sprite_idx];
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
            let pixel = COLOR_MAPPING[pixels[i]];
            framebuffer[framebuffer_base + i] = pixel;
        }
    }

    /// Marks the current frame as dirty. This means we have to redraw the lines from here on out for the rest of this
    /// frame. Note that we may not mark ourselves dirty until midway through a frame. In that case we need to copy the
    /// pixel data from the current frame into the next one up to the current line.
    /// e.g say we were clean from LY=0 to LY=60 and then something changed. We need to copy all pixel data for lines
    /// 0.. 60 from the current framebuffer into the next one, and then continue drawing new lines into the next one
    /// as per normal.
    pub fn mark_dirty(&mut self, context: &mut Context) {
        self.dirty = true;
        self.next_dirty = true;

        // Although if we mark dirty after vblank, then that's fine. Just marking ourselves as dirty is enough.
        if self.ly >= 144 {
            return;
        }

        if self.ly > 0 {
            let bound = 0..usize::from(self.ly) * 160;
            context.next_framebuffer[bound.clone()].copy_from_slice(&context.current_framebuffer[bound.clone()]);
        }
    }

    /// Replicate the (bizarre) hardware bugs relating to OAM during PPU Mode2.
    /// In a nutshell, if the CPU even breaths in the general direction of OAM memory locations while the PPU has the
    /// OAM locked for Mode 2 (OAM Search), then weird memory corruption happens. There's different patterns of memory
    /// corruption depending on which instruction the CPU was executing. Here's the weird thing though, in all cases
    /// the CPU wasn't actually even trying to access the OAM memory. It was just doing stuff to CPU 16-bit registers
    /// with values that fall inside of the OAM memory range (0xFE00 - 0xFE9F). Crazy.
    pub fn maybe_trash_oam(&mut self, context: &mut Context, corruption_type: OamCorruptionType) {
        if self.mode == Mode::Mode2 && self.mode_cycles > 0 && self.mode_cycles < 20 {
            self.mark_dirty(context);

            let oam_mem = unsafe { slice::from_raw_parts_mut(self.oam.as_ptr() as *mut u8, 160) };
            let mut pos = (self.mode_cycles as usize) * 2 * 4;

            // TODO: there's a bit of duplication here for a bunch of the corruption patterns, can probably clean up
            // and simplify this code.

            match corruption_type {
                OamCorruptionType::READ => {
                    pos += 2;
                    for _ in 0..6 {
                        oam_mem[pos] = oam_mem[pos - 8];
                        pos += 1;
                    }
                }
                OamCorruptionType::POP => {
                    pos += 2;
                    for _ in 0..6 {
                        oam_mem[pos] = oam_mem[pos - 8];
                        pos += 1;
                    }
                    for _ in 0..8 {
                        oam_mem[pos] = oam_mem[pos - 8];
                        pos += 1;
                    }
                    pos -= 32;
                    for _ in 0..16 {
                        oam_mem[pos] = oam_mem[pos + 16];
                        pos += 1;
                    }
                }
                OamCorruptionType::PUSH => {
                    pos -= 6;
                    for _ in 0..22 {
                        oam_mem[pos] = oam_mem[pos - 8];
                        pos += 1;
                    }
                }
                OamCorruptionType::LDI => {
                    pos += 2;
                    for _ in 0..6 {
                        oam_mem[pos] = oam_mem[pos - 8];
                        pos += 1;
                    }

                    pos -= 14;
                    for _ in 0..11 {
                        oam_mem[pos] = oam_mem[pos + 8];
                        pos -= 1;
                    }
                }
                OamCorruptionType::LDD => {
                    pos += 2;
                    for _ in 0..6 {
                        oam_mem[pos] = oam_mem[pos - 8];
                        pos += 1;
                    }
                    pos -= 9;
                    for _ in 0..16 {
                        oam_mem[pos] = oam_mem[pos + 8];
                        pos -= 1;
                    }
                }
            }
        }
    }

    pub fn oam_read(&self, addr: usize) -> u8 {
        if !self.oam_allow_read {
            return 0xFF; // Reading OAM memory during Mode2 & Mode3 is not permitted.
        }
        (unsafe { slice::from_raw_parts(self.oam.as_ptr() as *const u8, 160) })[addr]
    }

    pub fn oam_write(&mut self, context: &mut Context, addr: usize, v: u8) {
        if !self.oam_allow_write {
            return; // Writing OAM memory during Mode2 & Mode3 is not permitted.
        }
        let oam = unsafe { slice::from_raw_parts_mut(self.oam.as_ptr() as *mut u8, 160) };
        if oam[addr] != v {
            self.mark_dirty(context);
        }
        oam[addr] = v;
    }

    pub fn vram_read(&self, addr: u16) -> u8 {
        if !self.vram_allow_read {
            return 0xFF; // Reading VRAM during Mode3 is not permitted.
        }

        if addr < 0x1800 {
            self.tiles[addr as usize]
        } else {
            self.tilemap[(addr - 0x1800) as usize]
        }
    }

    pub fn vram_write(&mut self, context: &mut Context, addr: u16, v: u8) {
        if !self.vram_allow_write {
            return; // Writing VRAM during Mode3 is not permitted.
        }

        let addr = addr as usize;

        if addr < 0x1800 {
            if self.tiles[addr] != v {
                self.mark_dirty(context);
            }
            self.tiles[addr] = v;
        } else {
            let addr = addr - 0x1800;
            if self.tilemap[addr] != v {
                self.mark_dirty(context);
            }
            self.tilemap[addr] = v;
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
    pub fn reg_lcdc_write(&mut self, context: &mut Context, v: u8, interrupts: &mut InterruptController) {
        let enabled = v & 0b1000_0000 > 0;

        // TODO: can do a better dirty check here. No need to set dirty flags if we didn't change anything important.
        self.mark_dirty(context);

        if enabled && !self.enabled {
            self.enabled = true;
            self.ly = 0;
            self.reported_mode = 0;

            // When the LCD is first enabled, line 0 starts in a HBlank for 19 cycles, then goes straight to Mode 3
            // (skips mode 2).
            self.mode = Mode::FirstMode0;
            self.mode_cycles = 0;
            // We need to track that we're currently drawing the first frame since enabling the PPU, since a bunch of
            // wacky timings happen during that first frame.
            self.first_frame = true;

            // When enabling the PPU, we immediately run the LY==LYC comparison. If the lyc comparison check was
            // previously unset and just became set, and LYC interrupts were already enabled in STAT prior to enabling
            // the PPU, then we immediately request the interrupt.
            let lyc_match = self.ly == self.lyc;
            self.update_stat_interrupt(Some(interrupts), |state| {
                state.lyc_active = lyc_match;
                state.oam_active = false;
                state.hblank_active = false;
                state.vblank_active = false;
            });
        } else if !enabled && self.enabled {
            for i in &mut context.next_framebuffer {
                *i = COLOR_MAPPING[0];
            }
            context.swap_framebuffers();

            // TODO: check if we're inside a VBlank.

            // Disabling the PPU immediately clears the STAT mode bits, and sets LY to 0.
            self.enabled = false;
            self.ly = 0;
            self.reported_mode = 0;

            // While PPU is disabled, OAM and VRAM read/writes are wide open.
            self.oam_allow_read = true;
            self.oam_allow_write = true;
            self.vram_allow_read = true;
            self.vram_allow_write = true;
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
            | if self.stat_interrupts.lyc_active   { 0b0000_0100 } else { 0 }
            | if self.stat_interrupts.hblank_enabled { 0b0000_1000 } else { 0 }
            | if self.stat_interrupts.vblank_enabled { 0b0001_0000 } else { 0 }
            | if self.stat_interrupts.oam_enabled    { 0b0010_0000 } else { 0 }
            | if self.stat_interrupts.lyc_enabled    { 0b0100_0000 } else { 0 }
    }

    /// Write to the 0xFF41 STAT register.
    pub fn reg_stat_write(&mut self, v: u8, interrupts: &mut InterruptController) {
        self.update_stat_interrupt(Some(interrupts), |state| {
            state.hblank_enabled = v & 0b0000_1000 > 0;
            state.vblank_enabled = v & 0b0001_0000 > 0;
            state.oam_enabled = v & 0b0010_0000 > 0;
            state.lyc_enabled = v & 0b0100_0000 > 0;
        });
    }

    /// Write to the 0xFF42 SCY register.
    pub fn reg_scy_write(&mut self, context: &mut Context, v: u8) {
        if self.scy != v {
            self.mark_dirty(context);
        }
        self.scy = v;
    }

    /// Write to the 0xFF43 SCX register.
    pub fn reg_scx_write(&mut self, context: &mut Context, v: u8) {
        if self.scx != v {
            self.mark_dirty(context);
        }
        self.scx = v;
    }

    /// Write to the 0xFF47 BGP register.
    pub fn reg_bgp_write(&mut self, context: &mut Context, v: u8) {
        if self.bgp.pack() != v {
            self.mark_dirty(context);
        }
        self.bgp.update(v);
    }

    /// Write to the 0xFF48 OBP0 register.
    pub fn reg_obp0_write(&mut self, context: &mut Context, v: u8) {
        if self.obp0.pack() != v {
            self.mark_dirty(context);
        }
        self.obp0.update(v);
    }

    /// Write to the 0xFF49 OBP1 register.
    pub fn reg_obp1_write(&mut self, context: &mut Context, v: u8) {
        if self.obp1.pack() != v {
            self.mark_dirty(context);
        }
        self.obp1.update(v);
    }

    /// Write to the 0xFF4A WY register.
    pub fn reg_wy_write(&mut self, context: &mut Context, v: u8) {
        if self.wy != v {
            self.mark_dirty(context);
        }
        self.wy = v;
    }

    /// Write to the 0xFF4B WX register.
    pub fn reg_wx_write(&mut self, context: &mut Context, v: u8) {
        if self.wx != v {
            self.mark_dirty(context);
        }
        self.wx = v;
    }

    /// Write to the 0xFF45 LYC register.
    pub fn reg_lyc_write(&mut self, v: u8, interrupts: &mut InterruptController) {
        // LYC can be set any time ...
        self.lyc = v;

        // But if the PPU isn't enabled then the STAT LYC comparison bit is not changed, even if LY and LYC match.
        if self.enabled {
            let lyc_match = self.lyc == self.ly;
            self.update_stat_interrupt(Some(interrupts), |state| state.lyc_active = lyc_match);
        }
    }

    /// STAT interrupts can behave kinda strangely. The way they work internally is the 4 comparators are running
    /// constantly. Whenever a condition is met (i.e PPU going into Mode2 with the mode2 interrupt enabled in STAT), a
    /// signal goes from 0 to 1. Only when the signal goes from inactive to active will a STAT interrupt get triggered.
    /// So what can happen is, the signal is *already* in the active state because some existing condition is already
    /// met. Then even if a new condition is met, no interrupt will be fired. So for example let's say you have OAM +
    /// hblank interrupts enabled. The first HBlank that is hit *will* trigger the interrupt, but then the transition
    /// to OAM will not trigger the interrupt, because the signal is still in the active state during the transition.
    fn update_stat_interrupt<T: FnMut(&mut StatInterrupts)>(
        &mut self,
        interrupts: Option<&mut InterruptController>,
        mut f: T,
    ) {
        let prev_state = self.stat_interrupt_active;
        f(&mut self.stat_interrupts);
        self.stat_interrupt_active = (self.stat_interrupts.lyc_enabled && self.stat_interrupts.lyc_active)
            || (self.stat_interrupts.oam_enabled && self.stat_interrupts.oam_active)
            || (self.stat_interrupts.vblank_enabled && self.stat_interrupts.vblank_active)
            || (self.stat_interrupts.hblank_enabled && self.stat_interrupts.hblank_active);

        if interrupts.is_some() && self.stat_interrupt_active && !prev_state {
            interrupts.unwrap().request(Interrupt::Stat);
        }
    }
}

impl Default for Mode {
    fn default() -> Mode {
        Mode::Mode2
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
