#![deny(
    trivial_numeric_casts,
    unstable_features,
    unused_import_braces,
    unused_extern_crates,
    variant_size_differences
)]

pub mod apu;
pub mod cartridge;
pub mod cpu;
pub mod dma;
pub mod interrupt;
pub mod joypad;
pub mod ppu;
pub mod rewind;
pub mod rom;
pub mod serial;
pub mod simple_diff;
pub mod timer;
pub mod util;

use crate::apu::Apu;
use crate::cartridge::Cartridge;
use crate::cpu::Bus;
use crate::cpu::Cpu;
use crate::dma::DmaController;
use crate::interrupt::InterruptController;
use crate::joypad::Joypad;
use crate::ppu::OamCorruptionType;
use crate::ppu::Ppu;
use crate::ppu::SCREEN_SIZE;
use crate::rom::Rom;
use crate::serial::Serial;
use crate::timer::Timer;
use bincode;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::io::{Read, Write};

/// Number of frames to keep the Nintendo boot logo visible before starting actual emulation. Only used when skipping
/// emulation of the real bootrom.
pub const BOOT_LOGO_DURATION: u64 = 30;

pub const CYCLES_PER_MICRO: f32 = 1_048_576.0 / 1_000_000.0;

const TRADEMARK_ICON: [u8; 8] = [0x3c, 0x42, 0xb9, 0xa5, 0xb9, 0xa5, 0x42, 0x3c];
static DMG0_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg0.rom");
static DMG_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg.rom");
static MGB_BOOTROM: &[u8; 256] = include_bytes!("bootroms/mgb.rom");

/// Context holds external state that is related to, but not a core part of, the Gameboy struct. Rendered frames,
/// audio samples are kept in here for example. The distinction is made because of the nature of save/load states and
/// rewinding, and also for efficiency reasons. We don't want to serialize the framebuffer and audio samples in every
/// snapshot, that would be extremely wasteful. We also don't need to calculate audio samples or frames when we're
/// in the process of replaying a Gameboy to seek to a new rewind state.
memory_segment! { Framebuffer; u16; SCREEN_SIZE }
pub struct Context {
    pub rom: Rom,

    /// If false, the PPU will not draw pixels to the framebuffer while it is running. This is useful for minimizing
    /// work while emulating. For example headless tests can disable graphics, or if the emulator knows it's not visible
    /// but expected to run in the background.
    pub enable_graphics: bool,
    pub current_framebuffer: Framebuffer,
    pub next_framebuffer: Framebuffer,

    /// Similar to enable_graphics. The APU will keep running, but not write any samples to the queue. useful if sound
    /// is muted - no point spending time generating samples.
    pub enable_sound: bool,
    audio_samples: VecDeque<f32>,
}

#[derive(Eq, Deserialize, PartialEq, Serialize)]
pub enum Model {
    /// The original Gameboy, with its terrible, ghosting, hard to see LCD screen.
    DMG0,

    /// Later revisions of the same original Gameboy, with a slightly different bootrom. Notably, the newer bootrom
    /// shows the Nintendo trademark icon on the scrolling logo.
    DMGABC,

    /// Pocket Gameboy, the final revision of the monochrome Gameboy. Smaller form factor, better LCD, and subtly
    /// different bootrom.
    MGB,
    /*
    /// Color Gameboy
    CGB,

    /// Super Gameboy - a special cartridge for the Super Nintendo that allowed one to play Gameboy games on the SNES.
    /// Has a different bootrom, and allows special extra functionality driven by the SNES, by doing some crazy
    /// bit-banging on the JOY1 register.
    SGB,
    SGB2,*/
}

memory_segment! { Ram; u8; 0x2000 }
memory_segment! { HRam; u8; 0x7F }

/// Parent structure for a Gameboy emulation session. Contains all the state of the Gameboy CPU and its various
/// components (LCD, APU, Serial port, timer, IRQ, Joypad, etc). Notably, this struct does *not* hold the framebuffer,
/// audio sample queue, or ROM. Those are externalized to the Context struct to make rewinds and snapshots cleaner.
#[derive(Default, Deserialize, Serialize)]
pub struct Gameboy {
    pub model: Model,
    pub bootrom_enabled: bool,
    pub bootrom_countdown: u64,
    pub cycle_count: u64, // The total number of M-cycles that have elapsed since emulation began
    pub frame_count: u64, // The total number of frames we've rendered since emulation began

    pub apu: Apu,
    pub cart: Cartridge,
    pub cpu: Cpu,
    pub dma: DmaController,
    pub hram: HRam, // 0xFF80 - 0xFFFE
    pub interrupts: InterruptController,
    pub joypad: Joypad,
    pub ppu: Ppu,
    pub ram: Ram, // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    pub serial: Serial,
    pub timer: Timer,
}

/// Holds the gameboy hardware components other than the CPU. This is constructed and fed into the CPU when executing
/// instructions. It can also be manually obtained (via Gameboy#bus) to query/poke internal state directly.
pub struct GameboyBus<'a> {
    model: &'a Model,
    bootrom_enabled: &'a mut bool,
    cycle_count: &'a mut u64,
    frame_count: &'a mut u64,
    context: &'a mut Context,
    apu: &'a mut Apu,
    cart: &'a mut Cartridge,
    dma: &'a mut DmaController,
    hram: &'a mut HRam, // 0xFF80 - 0xFFFE
    interrupts: &'a mut InterruptController,
    joypad: &'a mut Joypad,
    ppu: &'a mut Ppu,
    ram: &'a mut Ram, // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    serial: &'a mut Serial,
    timer: &'a mut Timer,
}

/// Saves the state of a given Gameboy + Context pair. This is a complete save state - including the framebuffers, audio
/// sample queue, and emulator state.
pub fn save_state<W: Write>(gb: &Gameboy, ctx: &Context, mut w: W) -> bincode::Result<()> {
    bincode::serialize_into(&mut w, gb)?;
    bincode::serialize_into(&mut w, &ctx.current_framebuffer)?;
    bincode::serialize_into(&mut w, &ctx.next_framebuffer)?;
    bincode::serialize_into(&mut w, &ctx.audio_samples)?;
    Ok(())
}

pub fn load_state<R: Read>(gb: &mut Gameboy, ctx: &mut Context, mut state: R) -> bincode::Result<()> {
    *gb = bincode::deserialize_from(&mut state)?;
    ctx.current_framebuffer = bincode::deserialize_from(&mut state)?;
    ctx.next_framebuffer = bincode::deserialize_from(&mut state)?;
    ctx.audio_samples = bincode::deserialize_from(&mut state)?;

    Ok(())
}

impl Context {
    pub fn new(rom: Rom) -> Context {
        Context {
            rom,
            enable_graphics: true,
            current_framebuffer: Default::default(),
            next_framebuffer: Default::default(),

            enable_sound: true,
            audio_samples: VecDeque::new(),
        }
    }

    pub fn swap_framebuffers(&mut self) {
        std::mem::swap(&mut self.current_framebuffer, &mut self.next_framebuffer);
    }

    pub fn drain_audio_samples<F: FnMut(&[f32])>(&mut self, mut f: F) {
        let (l, r) = self.audio_samples.as_slices();
        if l.len() > 0 {
            f(l);
        }
        if r.len() > 0 {
            f(r);
        }
        self.audio_samples.clear();
    }
}

impl Gameboy {
    pub fn new(model: Model, rom: &Rom, run_real_bootrom: bool) -> Gameboy {
        let cart = Cartridge::from_rom(rom);

        let mut gameboy = Gameboy {
            model,
            bootrom_enabled: true,
            bootrom_countdown: 0,
            cycle_count: 0,
            frame_count: 0,

            apu: Apu::new(),
            cart,
            cpu: Cpu::new(),
            dma: Default::default(),
            hram: Default::default(),
            interrupts: InterruptController::new(),
            joypad: Default::default(),
            ppu: Ppu::new(),
            ram: Default::default(),
            serial: Serial::new(),
            timer: Timer::new(),
        };

        if !run_real_bootrom {
            gameboy.skip_bootrom(&rom);
        }

        gameboy
    }

    pub fn bus<'a>(&'a mut self, ctx: &'a mut Context) -> (&mut Cpu, GameboyBus) {
        (
            &mut self.cpu,
            GameboyBus {
                context: ctx,

                model: &self.model,
                cycle_count: &mut self.cycle_count,
                frame_count: &mut self.frame_count,
                bootrom_enabled: &mut self.bootrom_enabled,

                apu: &mut self.apu,
                cart: &mut self.cart,
                dma: &mut self.dma,
                hram: &mut self.hram,
                interrupts: &mut self.interrupts,
                joypad: &mut self.joypad,
                ppu: &mut self.ppu,
                ram: &mut self.ram,
                serial: &mut self.serial,
                timer: &mut self.timer,
            },
        )
    }

    pub fn run_instruction(&mut self, ctx: &mut Context) {
        if self.bootrom_countdown > 0 {
            // We skipped the real bootrom phase but we're emulating a few cycles of NOP activity to show the logo on
            // the screen for a brief period.
            self.bootrom_countdown -= 1;
            if self.bootrom_countdown == 0 {
                self.finish_bootrom_skip();
            } else {
                let (_, mut bus) = self.bus(ctx);
                bus.clock();
            }
            return;
        }

        let (cpu, mut bus) = self.bus(ctx);
        cpu.step(&mut bus);
    }

    /// Skips emulating the bootrom startup process (the scrolling Nintendo logo). Emulating it is probably irritating
    /// to a lot of people that just want to pet their Pikachu and don't get the nostalgia from the distinctive startup
    /// chime. Emulating the bootrom also takes a few million CPU cycles, so skipping it in tests is a big speedup.
    pub fn skip_bootrom(&mut self, rom: &Rom) {
        // Start the bootrom countdown. The idea here is to emulate noops for a bunch of cycles to keep the Nintendo
        // logo on the screen for half a second.
        self.ppu.dirty = true;
        self.bootrom_countdown = 17_556 * BOOT_LOGO_DURATION; // 17556 clocks in a frame * number of frames

        // Ensure PPU has correct state (enabled, BG enabled, etc)
        self.ppu.enabled = true;
        self.ppu.bg_enabled = true;
        self.ppu.bg_tile_area_lo = true;

        // Setup Nintendo logo in tilemap.
        let mut tilemap_addr = 0x104;
        for v in 1..=12 {
            self.ppu.tilemap[tilemap_addr] = v;
            tilemap_addr += 1;
        }

        // Add trademark icon to VRAM
        if self.model == Model::DMGABC || self.model == Model::MGB {
            self.ppu.tilemap[0x110] = 0x19;
        }

        tilemap_addr = 0x124;
        for v in 13..=24 {
            self.ppu.tilemap[tilemap_addr] = v;
            tilemap_addr += 1;
        }
        // Copy Nintendo logo data from ROM.
        let mut vram_addr = 0x10;
        let mut src_addr = 0x104;
        for _ in 0..48 {
            let b = rom.data[src_addr];
            src_addr += 1;
            // Double up each bit in the source byte, using sorcery.
            // The best kind of sorcery too: copy/pasted from the interwebz.
            let z = ((u64::from(b).wrapping_mul(0x0101_0101_0101_0101) & 0x8040_2010_0804_0201)
                .wrapping_mul(0x0102_0408_1020_4081)
                >> 49)
                & 0x5555
                | ((u64::from(b).wrapping_mul(0x0101_0101_0101_0101) & 0x8040_2010_0804_0201)
                    .wrapping_mul(0x0102_0408_1020_4081)
                    >> 48)
                    & 0xAAAAu64;
            self.ppu.tiles[vram_addr] = (z >> 8) as u8;
            vram_addr += 2;
            self.ppu.tiles[vram_addr] = (z >> 8) as u8;
            vram_addr += 2;
            self.ppu.tiles[vram_addr] = z as u8;
            vram_addr += 2;
            self.ppu.tiles[vram_addr] = z as u8;
            vram_addr += 2;
        }

        match self.model {
            Model::DMGABC | Model::MGB => {
                for v in &TRADEMARK_ICON {
                    self.ppu.tiles[vram_addr] = *v;
                    vram_addr += 2;
                }
            }
            _ => {}
        }
    }

    pub fn finish_bootrom_skip(&mut self) {
        self.cpu.pc = 0x100;
        self.cpu.sp = 0xFFFE;

        self.cpu.a = 0x01;
        self.cpu.d = 0x00;
        self.cpu.c = 0x13;
        match self.model {
            Model::DMG0 => {
                self.cpu.b = 0xFF;
                self.cpu.e = 0xC1;
                self.cpu.h = 0x84;
                self.cpu.l = 0x03;
            }
            // DMGABC and MGB are very similar. MGB just differs in A register being 0xFF.
            Model::DMGABC | Model::MGB => {
                self.cpu.b = 0x00;
                self.cpu.h = 0x01;
                self.cpu.f.unpack(0xB0);
                self.cpu.e = 0xD8;
                self.cpu.l = 0x4D;

                if self.model == Model::MGB {
                    self.cpu.a = 0xFF;
                }
            }
        }

        self.timer.div = match self.model {
            Model::DMG0 => 0x182C,
            Model::DMGABC | Model::MGB => 0xABC8,
        };
        self.interrupts.request = 0x1;

        // PPU should be in the middle of a VBlank.
        // Where the PPU is at in terms of mode + cycles depends on which bootrom was run.
        self.ppu.mode = ppu::Mode::Mode1;
        match self.model {
            Model::DMG0 => {
                self.ppu.ly = 145;
                self.ppu.mode_cycles = 24;
            }
            Model::DMGABC | Model::MGB => {
                self.ppu.ly = 153;
                self.ppu.mode_cycles = 100;
            }
        }

        // After bootrom is finished, sound1 is still enabled but muted.
        self.apu.chan1.vol_env = apu::VolumeEnvelope {
            default: 0b1111,
            inc: false,
            steps: 0b011,
            val: 0,
            timer: 0,
        };
        self.apu.chan1.on = true;
        self.apu.reg_nr11_write(0b1000_0000);
        self.apu.reg_nr50_write(0x77);
        self.apu.reg_nr51_write(0xF3);

        // Even when the serial port isn't active, it's clocking in order to shift out a bit.
        // It's tied to the main clock cycle count, and not dependent on when SC/SB is changed. This magic number
        // ensures that mooneye serial/boot_sclk_align test passes.
        // TODO: the thing is, this makes the serial/boot_sclk_align test pass on skipped bootrom. But on real bootrom
        // it's off by a bit. I have a feeling it's somehow related to the DIV quirk. Investigate more soon.
        self.serial.transfer_clock = 124;

        self.bootrom_enabled = false;
    }

    pub fn core_panic(&self, msg: String) -> ! {
        panic!(
            "{}\nIME: {},{}\nHalt: {}\nRegs:\n\tA=0x{:02X}\n\tB=0x{:02X}\n\tC=0x{:02X}\n\tD=0x{:02X}\n\tE=0x{:02X}\n\tF=0x{:02X}\n\tH=0x{:02X}\n\tL=0x{:02X}\n\tSP={:#04X}\n\tPC={:#04X}",
            msg, self.cpu.ime, self.cpu.ime_defer, self.cpu.halted, self.cpu.a, self.cpu.b, self.cpu.c, self.cpu.d, self.cpu.e, self.cpu.f.pack(), self.cpu.h, self.cpu.l, self.cpu.sp, self.cpu.pc);
    }
}

impl<'a> GameboyBus<'a> {
    /// Handles reads from the memory bus. This method is responsible for resolving memory addresses to the correct
    /// memory segments and registers. Generally though, mem_read should be used since it ensures components are
    /// clocked correctly and reads are prevented during situations like active DMA transfers.
    pub fn memory_get(&self, addr: u16) -> u8 {
        match addr {
            0x0000...0x00FF if *self.bootrom_enabled => self.bootrom_read(addr),
            0x0000...0x3FFF => self.cart.rom_lo(&self.context.rom.data, addr as usize),
            0x4000...0x7FFF => self.cart.rom_hi(&self.context.rom.data, (addr - 0x4000) as usize),
            0x8000...0x9FFF => self.ppu.vram_read(addr - 0x8000),
            0xA000...0xBFFF => self.cart.ram((addr - 0xA000) as usize),
            0xC000...0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000...0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFE00...0xFE9F => self.ppu.oam_read((addr - 0xFE00) as usize),
            0xFF00 => self.joypad.reg_p1_read(),
            0xFF01 => self.serial.reg_sb_read(),
            0xFF02 => self.serial.reg_sc_read(),
            0xFF04 => (self.timer.div >> 8) as u8,
            0xFF05 => self.timer.tima,
            0xFF06 => self.timer.tma,
            0xFF07 => self.timer.reg_tac_read(),
            0xFF0F => 0xE0 | self.interrupts.request, // Unused IF bits are always 1
            0xFF10 => self.apu.reg_nr10_read(),
            0xFF11 => self.apu.reg_nr11_read(),
            0xFF12 => self.apu.reg_nr12_read(),
            0xFF13 => self.apu.reg_nr13_read(),
            0xFF14 => self.apu.reg_nr14_read(),
            0xFF16 => self.apu.reg_nr21_read(),
            0xFF17 => self.apu.reg_nr22_read(),
            0xFF18 => self.apu.reg_nr23_read(),
            0xFF19 => self.apu.reg_nr24_read(),
            0xFF1A => self.apu.reg_nr30_read(),
            0xFF1B => self.apu.reg_nr31_read(),
            0xFF1C => self.apu.reg_nr32_read(),
            0xFF1D => self.apu.reg_nr33_read(),
            0xFF1E => self.apu.reg_nr34_read(),
            0xFF20 => self.apu.reg_nr41_read(),
            0xFF21 => self.apu.reg_nr42_read(),
            0xFF22 => self.apu.reg_nr43_read(),
            0xFF23 => self.apu.reg_nr44_read(),
            0xFF24 => self.apu.reg_nr50_read(),
            0xFF25 => self.apu.reg_nr51_read(),
            0xFF26 => self.apu.reg_nr52_read(),
            0xFF30...0xFF3F => self.apu.wave_read(addr - 0xFF30),
            0xFF40 => self.ppu.reg_lcdc_read(),
            0xFF41 => self.ppu.reg_stat_read(),
            0xFF42 => self.ppu.scy,
            0xFF43 => self.ppu.scx,
            0xFF44 => self.ppu.ly,
            0xFF45 => self.ppu.lyc,
            0xFF46 => self.dma.reg,
            0xFF47 => self.ppu.bgp.pack(),
            0xFF48 => self.ppu.obp0.pack(),
            0xFF49 => self.ppu.obp1.pack(),
            0xFF4A => self.ppu.wy,
            0xFF4B => self.ppu.wx,
            0xFF80...0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF => self.interrupts.enable,

            _ => 0xFF, // Reads from unhandled locations see 0xFF
        }
    }

    pub fn memory_set(&mut self, addr: u16, v: u8) {
        match addr {
            0x0000...0x7FFF => self.cart.write(addr, v),
            0x8000...0x9FFF => self.ppu.vram_write(&mut self.context, addr - 0x8000, v),
            0xA000...0xBFFF => self.cart.write(addr, v),
            0xC000...0xDFFF => self.ram[(addr - 0xC000) as usize] = v,
            0xE000...0xFDFF => self.ram[(addr - 0xE000) as usize] = v,
            0xFE00...0xFE9F => self.ppu.oam_write(&mut self.context, (addr - 0xFE00) as usize, v),
            0xFF00 => self.joypad.reg_p1_write(v),
            0xFF01 => self.serial.reg_sb_write(v),
            0xFF02 => self.serial.reg_sc_write(v),
            0xFF04 => self.timer.reg_div_write(),
            0xFF05 => self.timer.reg_tima_write(v),
            0xFF06 => self.timer.reg_tma_write(v),
            0xFF07 => self.timer.reg_tac_write(v & 0x7, &mut self.interrupts),
            0xFF0F => self.interrupts.reg_if_write(v & 0x1F),
            0xFF10 => self.apu.reg_nr10_write(v),
            0xFF11 => self.apu.reg_nr11_write(v),
            0xFF12 => self.apu.reg_nr12_write(v),
            0xFF13 => self.apu.reg_nr13_write(v),
            0xFF14 => self.apu.reg_nr14_write(v),
            0xFF16 => self.apu.reg_nr21_write(v),
            0xFF17 => self.apu.reg_nr22_write(v),
            0xFF18 => self.apu.reg_nr23_write(v),
            0xFF19 => self.apu.reg_nr24_write(v),
            0xFF1A => self.apu.reg_nr30_write(v),
            0xFF1B => self.apu.reg_nr31_write(v),
            0xFF1C => self.apu.reg_nr32_write(v),
            0xFF1D => self.apu.reg_nr33_write(v),
            0xFF1E => self.apu.reg_nr34_write(v),
            0xFF20 => self.apu.reg_nr41_write(v),
            0xFF21 => self.apu.reg_nr42_write(v),
            0xFF22 => self.apu.reg_nr43_write(v),
            0xFF23 => self.apu.reg_nr44_write(v),
            0xFF24 => self.apu.reg_nr50_write(v),
            0xFF25 => self.apu.reg_nr51_write(v),
            0xFF26 => self.apu.reg_nr52_write(v),
            0xFF30...0xFF3F => self.apu.wave_write(addr - 0xFF30, v),
            0xFF40 => self.ppu.reg_lcdc_write(&mut self.context, v, &mut self.interrupts),
            0xFF41 => self.ppu.reg_stat_write(v, &mut self.interrupts),
            0xFF42 => self.ppu.reg_scy_write(&mut self.context, v),
            0xFF43 => self.ppu.reg_scx_write(&mut self.context, v),
            0xFF45 => self.ppu.reg_lyc_write(v, &mut self.interrupts),
            0xFF46 => self.dma.start(v),
            0xFF47 => self.ppu.reg_bgp_write(&mut self.context, v),
            0xFF48 => self.ppu.reg_obp0_write(&mut self.context, v),
            0xFF49 => self.ppu.reg_obp1_write(&mut self.context, v),
            0xFF4A => self.ppu.reg_wy_write(&mut self.context, v),
            0xFF4B => self.ppu.reg_wx_write(&mut self.context, v),
            0xFF50 => *self.bootrom_enabled = false,
            0xFF80...0xFFFE => self.hram[(addr - 0xFF80) as usize] = v,
            0xFFFF => self.interrupts.reg_ie_write(v),

            _ => {} // Writes to unhandled locations are simply ignored.
        }
    }

    fn bootrom_read(&self, addr: u16) -> u8 {
        match *self.model {
            Model::DMG0 => DMG0_BOOTROM[addr as usize],
            Model::DMGABC => DMG_BOOTROM[addr as usize],
            Model::MGB => MGB_BOOTROM[addr as usize],
        }
    }
}

impl<'a> Bus for GameboyBus<'a> {
    fn memory_read(&mut self, addr: u16) -> u8 {
        // While DMA transfer is in progress, reads to the OAM area will see 0xFF.
        let block_read = self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F);

        // Here's a batshit hardware quirk, the PPU is out of sync with the CPU by 2 T-cycles. To simulate this effect,
        // we perform the read *before* running the downstream hardware clocks, if we're reading a PPU register or OAM.
        if !block_read && ((addr >= 0xFF40 && addr <= 0xFF4B) || (addr >= 0xFE00 && addr <= 0xFE9F)) {
            let v = self.memory_get(addr);
            self.clock();
            return v;
        }

        // Reading from the memory bus takes a full CPU cycle.
        self.clock();

        if block_read {
            0xFF
        } else {
            self.memory_get(addr)
        }
    }

    fn memory_write(&mut self, addr: u16, v: u8) {
        // While DMA transfer is in progress, write to the OAM area will be ignored.
        let block_write = self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F);
        // Writing to the memory bus takes a full CPU cycle.
        self.clock();

        if block_write {
            return;
        }

        self.memory_set(addr, v);
    }

    fn clock(&mut self) {
        *self.cycle_count += 1;

        self.timer.clock(&mut self.interrupts);
        let (dma_active, dma_src, dma_dst) = self.dma.clock();
        if dma_active {
            let v = self.memory_get(dma_src);
            self.ppu.oam_write(&mut self.context, dma_dst, v);
        }
        self.serial.clock(&mut self.interrupts);
        if self.ppu.clock(&mut self.context, &mut self.interrupts) {
            *self.frame_count += 1;
        }
        self.apu.clock(&mut self.context);
    }

    fn interrupt_controller(&mut self) -> &mut InterruptController {
        self.interrupts
    }

    fn trigger_oam_glitch(&mut self, kind: OamCorruptionType) {
        self.ppu.maybe_trash_oam(&mut self.context, kind);
    }
}

impl Default for Model {
    fn default() -> Model {
        Model::DMG0
    }
}
