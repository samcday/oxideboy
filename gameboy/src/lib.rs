//! Gameboy LR35902 chip implementation.
//! Things were nice and simple back in the 80s. The CPU (a modified Z80), PPU and APU chips are all in this one silicon
//! package called the LR35902. Besides this chip, the Gameboy essentially only contains an LCD controller (which is
//! pointless to emulate), speaker/gamepad circuitry, and some other misc bits and pieces. So really, the LR35902 *IS*
//! the Gameboy.
//! This module contains the main interfaces and support pieces of the chip. The CPU, PPU and APU implementations are in
//! their respective modules.

pub mod apu;
pub mod cartridge;
pub mod cpu;
pub mod dma;
pub mod interrupt;
pub mod ppu;
pub mod serial;
pub mod timer;

use apu::Apu;
use cartridge::Cartridge;
use dma::DmaController;
use interrupt::{Interrupt, InterruptController};
use ppu::Ppu;
use serial::Serial;
use timer::Timer;

static DMG0_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg0.rom");
static DMG_BOOTROM:  &[u8; 256] = include_bytes!("bootroms/dmg.rom");

/// Gameboy represents an emulation session for an original Gameboy (DMG) running a Gameboy cartridge.
pub struct Gameboy {
    pub model: Model,
    pub cart: Cartridge,
    pub apu: Apu,
    pub dma: DmaController,
    pub interrupts: InterruptController,
    pub joypad: Joypad,
    pub ppu: Ppu,
    pub serial: Serial,
    pub timer: Timer,
    pub ram:  [u8; 0x2000],         // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    pub hram: [u8; 0x7F],           // 0xFF80 - 0xFFFE

    rom: Vec<u8>,
    bootrom_enabled: bool,
}

/// The CPU for the Gameboy/Gameboy Color is more or less the same. The only things that differ are how fast it runs and
/// which registers are available. The differences are encapsulated in this generic type, which is all the CPU cares
/// about when interfacing with the rest of the hardware, whether it be a DMG or a CGB.
pub trait Hardware {
    fn clock(&mut self);
    fn mem_read8(&mut self, addr: u16) -> u8;
    fn mem_write8(&mut self, addr: u16, v: u8);

    fn mem_read16(&mut self, addr: u16) -> u16 {
        let mut v = self.mem_read8(addr) as u16;
        v |= (self.mem_read8(addr + 1) as u16) << 8;
        v
    }
    fn mem_write16(&mut self, addr: u16, v: u16) {
        self.mem_write8(addr + 1, ((v & 0xFF00) >> 8) as u8);
        self.mem_write8(addr, (v & 0xFF) as u8);
    }

    fn skip_bootrom(&mut self);
    fn next_interrupt(&self) -> Option<Interrupt>;
    fn clear_interrupt(&mut self, int: Interrupt);
}

#[derive(Default)]
pub struct Joypad {
    btn: bool,
    dir: bool,

    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub a: bool,
    pub b: bool,
    pub select: bool,
    pub start: bool,
}

impl Joypad {
    fn reg_p1_read(&self) -> u8 {
        let mut v = 0xCF;

        if self.btn {
            v ^= 0x20;
            if self.a      { v ^= 1 }
            if self.b      { v ^= 2 }
            if self.select { v ^= 4 }
            if self.start  { v ^= 8 }
        } else if self.dir {
            v ^= 0x10;
            if self.right  { v ^= 1 }
            if self.left   { v ^= 2 }
            if self.up     { v ^= 4 }
            if self.down   { v ^= 8 }
        }

        v
    }

    fn reg_p1_write(&mut self, v: u8) {
        self.btn = v & 0x20 == 0;
        self.dir = v & 0x10 == 0;
    }
}

/// There are different models of the Gameboy that each behave slightly differently (different HW quirks, etc).
/// When creating a Gameboy emulation context, the desired Model must be chosen.
#[derive(Eq, PartialEq)]
pub enum Model {
    DMG0,
    DMG,
}

impl Gameboy {
    pub fn new(model: Model, rom: Vec<u8>) -> Gameboy {
        let cart = Cartridge::from_rom(&rom);

        Gameboy{
            model,
            cart,
            rom,
            apu: Apu::new(),
            dma: Default::default(),
            interrupts: InterruptController::new(),
            joypad: Default::default(),
            ppu: Ppu::new(),
            serial: Serial::new(),
            timer: Timer::new(),
            ram: [0; 0x2000],
            hram: [0; 0x7F],

            bootrom_enabled: true,
        }
    }

    fn bootrom_read(&self, addr: u16) -> u8 {
        match self.model {
            Model::DMG0 => DMG0_BOOTROM[addr as usize],
            Model::DMG  => DMG_BOOTROM[addr as usize],
        }
    }

    /// Handles reads from the memory bus. This method is responsible for resolving memory addresses to the correct
    /// memory segments and registers. Generally though, mem_read8 should be used since it ensures components are
    /// clocked correctly and reads are prevented during situations like active DMA transfers.
    fn mem_get8(&mut self, addr: u16) -> u8 {
        match addr {
            0x0000 ... 0x0100 if self.bootrom_enabled => self.bootrom_read(addr),
            0x0000 ... 0x3FFF => self.cart.rom_lo(&self.rom)[addr as usize],
            0x4000 ... 0x7FFF => self.cart.rom_hi(&self.rom)[(addr - 0x4000) as usize],
            0x8000 ... 0x9FFF => self.ppu.vram_read(addr - 0x8000),
            0xA000 ... 0xBFFF => self.cart.ram()[(addr - 0xA000) as usize],
            0xC000 ... 0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000 ... 0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFE00 ... 0xFE9F => self.ppu.oam_read((addr - 0xFE00) as usize),
            0xFEA0 ... 0xFEFF => 0x00, // Undocumented space that some ROMs seem to address...
            0xFF00            => self.joypad.reg_p1_read(),
            0xFF01            => self.serial.reg_sb_read(),
            0xFF02            => self.serial.reg_sc_read(),

            0xFF04            => (self.timer.div >> 8) as u8,
            0xFF05            => self.timer.tima,
            0xFF06            => self.timer.tma,
            0xFF07            => self.timer.reg_tac_read(),

            0xFF0F            => 0xE0 | self.interrupts.request, // Unused IF bits are always 1

            0xFF10            => self.apu.reg_nr10_read(),
            0xFF11            => self.apu.reg_nr11_read(),
            0xFF12            => self.apu.reg_nr12_read(),
            0xFF13            => self.apu.reg_nr13_read(),
            0xFF14            => self.apu.reg_nr14_read(),
            0xFF15            => { 0xFF } // Unused address that the blargg dmg_sound tests erroneously read from
            0xFF16            => self.apu.reg_nr21_read(),
            0xFF17            => self.apu.reg_nr22_read(),
            0xFF18            => self.apu.reg_nr23_read(),
            0xFF19            => self.apu.reg_nr24_read(),
            0xFF1A            => self.apu.reg_nr30_read(),
            0xFF1B            => self.apu.reg_nr31_read(),
            0xFF1C            => self.apu.reg_nr32_read(),
            0xFF1D            => self.apu.reg_nr33_read(),
            0xFF1E            => self.apu.reg_nr34_read(),
            0xFF1F            => { 0xFF } // Unused address that the blargg dmg_sound tests erroneously read from
            0xFF20            => self.apu.reg_nr41_read(),
            0xFF21            => self.apu.reg_nr42_read(),
            0xFF22            => self.apu.reg_nr43_read(),
            0xFF23            => self.apu.reg_nr44_read(),
            0xFF24            => self.apu.reg_nr50_read(),
            0xFF25            => self.apu.reg_nr51_read(),
            0xFF26            => self.apu.reg_nr52_read(),
            0xFF27 ... 0xFF2F => { 0xFF } // Unused addresses that the blargg dmg_sound tests erroneously read from
            0xFF30 ... 0xFF3F => self.apu.wave_read(addr - 0xFF30),

            0xFF40            => self.ppu.reg_lcdc_read(),
            0xFF42            => self.ppu.scy,
            0xFF43            => self.ppu.scx,
            0xFF44            => self.ppu.ly,

            0xFF46            => self.dma.reg,
            0xFF47            => self.ppu.bgp.pack(),
            0xFF48            => self.ppu.obp0.pack(),
            0xFF49            => self.ppu.obp1.pack(),
            0xFF4A            => self.ppu.wy as u8,
            0xFF4B            => self.ppu.wx as u8,

            0xFF80 ... 0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF            => self.interrupts.enable,

            _ => panic!("Unhandled memory read from 0x{:X}", addr),
        }
    }

    /// Handles writes to the memory bus. This method is responsible for resolving memory addresses to the correct
    /// memory segments and registers. Generally though, mem_write8 should be used since it ensures components are
    /// clocked correctly and writes are prevented during situations like active DMA transfers.
    fn mem_set8(&mut self, addr: u16, v: u8) {
        match addr {
            0x0000 ... 0x7FFF => { self.cart.write(addr, v); }
            0x8000 ... 0x9FFF => { self.ppu.vram_write(addr - 0x8000, v) }
            0xA000 ... 0xBFFF => { self.cart.write(addr, v); }
            0xC000 ... 0xDFFF => { self.ram[(addr - 0xC000) as usize] = v },
            0xE000 ... 0xFDFF => { self.ram[(addr - 0xE000) as usize] = v },
            0xFE00 ... 0xFE9F => { self.ppu.oam_write((addr - 0xFE00) as usize, v) }
            0xFEA0 ... 0xFEFF => { } // Undocumented space that some ROMs seem to address...
            0xFF00            => { self.joypad.reg_p1_write(v) }
            0xFF01            => { self.serial.reg_sb_write(v) },
            0xFF02            => { self.serial.reg_sc_write(v) },

            0xFF04            => { self.timer.reg_div_write() }
            0xFF05            => { self.timer.reg_tima_write(v) }
            0xFF06            => { self.timer.reg_tma_write(v) }
            0xFF07            => { if self.timer.reg_tac_write(v & 0x7) { self.interrupts.request(Interrupt::Timer) } }

            0xFF0F            => { self.interrupts.reg_if_write(v & 0x1F) }

            0xFF10            => { self.apu.reg_nr10_write(v) }
            0xFF11            => { self.apu.reg_nr11_write(v) }
            0xFF12            => { self.apu.reg_nr12_write(v) }
            0xFF13            => { self.apu.reg_nr13_write(v) }
            0xFF14            => { self.apu.reg_nr14_write(v) }
            0xFF15            => { } // Unused address that the blargg dmg_sound tests erroneously write to.
            0xFF16            => { self.apu.reg_nr21_write(v) }
            0xFF17            => { self.apu.reg_nr22_write(v) }
            0xFF18            => { self.apu.reg_nr23_write(v) }
            0xFF19            => { self.apu.reg_nr24_write(v) }
            0xFF1A            => { self.apu.reg_nr30_write(v) }
            0xFF1B            => { self.apu.reg_nr31_write(v) }
            0xFF1C            => { self.apu.reg_nr32_write(v) }
            0xFF1D            => { self.apu.reg_nr33_write(v) }
            0xFF1E            => { self.apu.reg_nr34_write(v) }
            0xFF1F            => { } // Unused address that the blargg dmg_sound tests erroneously write to.
            0xFF20            => { self.apu.reg_nr41_write(v) }
            0xFF21            => { self.apu.reg_nr42_write(v) }
            0xFF22            => { self.apu.reg_nr43_write(v) }
            0xFF23            => { self.apu.reg_nr44_write(v) }
            0xFF24            => { self.apu.reg_nr50_write(v) }
            0xFF25            => { self.apu.reg_nr51_write(v) }
            0xFF26            => { self.apu.reg_nr52_write(v) }
            0xFF27 ... 0xFF2F => { } // Unused addresses that the blargg dmg_sound tests erroneously write to.
            0xFF30 ... 0xFF3F => { self.apu.wave_write(addr - 0xFF30, v) }

            0xFF40            => { self.ppu.reg_lcdc_write(v) }
            0xFF42            => { self.ppu.scy = v }
            0xFF43            => { self.ppu.scx = v }
            0xFF46            => { self.dma.start(v) }
            0xFF47            => { self.ppu.bgp.update(v) }
            0xFF48            => { self.ppu.obp0.update(v) }
            0xFF49            => { self.ppu.obp1.update(v) }
            0xFF4A            => { self.ppu.wy = v },
            0xFF4B            => { self.ppu.wx = v },

            0xFF50 if self.bootrom_enabled && v == 1 => {
                self.bootrom_enabled = false;
            }

            0xFF80 ... 0xFFFE => { self.hram[(addr - 0xFF80) as usize] = v }
            0xFFFF            => { self.interrupts.reg_ie_write(v) }

            _ => panic!("Unhandled memory write to 0x{:X}", addr),
        }
    }
}

impl Hardware for Gameboy {
    fn mem_read8(&mut self, addr: u16) -> u8 {
        // While DMA transfer is in progress, reads to the OAM area will see 0xFF.
        if self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F) {
            self.clock();
            return 0xFF;
        }

        // Reading from the memory bus takes a full CPU cycle.
        self.clock();

        self.mem_get8(addr)
    }

    fn mem_write8(&mut self, addr: u16, v: u8) {
        // Writing to the memory bus takes a full CPU cycle.
        self.clock();

        // While DMA transfer is in progress, write to the OAM area will be ignored.
        if self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F) {
            return;
        }

        self.mem_set8(addr, v)
    }

    fn clock(&mut self) {
        if self.timer.clock() {
            self.interrupts.request(Interrupt::Timer);
        }
        let (dma_active, dma_src, dma_dst) = self.dma.clock();
        if dma_active {
            let v = self.mem_get8(dma_src);
            self.ppu.oam_write(dma_dst, v);
        }
        self.serial.clock();
        self.ppu.clock();
        self.apu.clock();
    }

    /// Skips emulating the bootrom (scrolling Nintendo logo) and just ensures all internal state looks like it ran.
    /// Don't call this directly - instead call Cpu#skip_bootrom
    fn skip_bootrom(&mut self) {
        self.timer.div = 0x1800;
        self.interrupts.request = 0x1;

        // Ensure PPU has correct state (enabled, BG enabled, etc)
        self.ppu.reg_lcdc_write(0x91);
        // PPU should be in the middle of a VBlank.
        self.ppu.mode = ppu::Mode::Mode1;
        self.ppu.ly = 145;
        self.ppu.cycle_counter = 144;

        // Setup Nintendo logo in tilemap.
        let mut addr = 0x1904;
        for v in 1..=12 {
            self.ppu.vram_write(addr, v);
            addr += 1;
        }

        if self.model == Model::DMG {
            self.ppu.vram_write(addr, 0x19);
        }

        addr = 0x1924;
        for v in 13..=24 {
            self.ppu.vram_write(addr, v);
            addr += 1;
        }
        // Copy Nintendo logo data from cart.
        let mut vram_addr = 0x10;
        let mut src_addr = 0x104;
        for _ in 0..48 {
            let b = self.mem_get8(src_addr);
            src_addr += 1;
            // Double up each bit in the source byte, using sorcery.
            // The best kind of sorcery too: copy/pasted from the interwebz.
            let z = (((b as u64).wrapping_mul(0x0101010101010101) & 0x8040201008040201).wrapping_mul(0x0102040810204081) >> 49) & 0x5555 |
                    (((b as u64).wrapping_mul(0x0101010101010101) & 0x8040201008040201).wrapping_mul(0x0102040810204081) >> 48) & 0xAAAAu64;
            self.ppu.vram_write(vram_addr, (z >> 8) as u8); vram_addr += 2;
            self.ppu.vram_write(vram_addr, (z >> 8) as u8); vram_addr += 2;
            self.ppu.vram_write(vram_addr, z as u8); vram_addr += 2;
            self.ppu.vram_write(vram_addr, z as u8); vram_addr += 2;
        }

        if self.model == Model::DMG {
            let mut src_addr = 0xD8;
            for _ in 0..8 {
                let v = self.mem_get8(src_addr);
                self.ppu.vram_write(vram_addr, v);
                src_addr += 1;
                vram_addr += 2;
            }
        }

        // After bootrom is finished, sound1 is still enabled but muted.
        self.apu.chan1.vol_env = apu::VolumeEnvelope{
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

        self.bootrom_enabled = false;
    }

    fn next_interrupt(&self) -> Option<Interrupt> {
        self.interrupts.next_interrupt()
    }

    fn clear_interrupt(&mut self, int: Interrupt) {
        self.interrupts.clear(int);
    }
}
