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
pub mod interrupt;
pub mod ppu;
pub mod serial;
pub mod timer;

use apu::Apu;
use cartridge::Cartridge;
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
    pub interrupts: InterruptController,
    pub joypad: Joypad,
    pub ppu: Ppu,
    pub serial: Serial,
    pub timer: Timer,
    pub ram:  [u8; 0x2000],
    pub hram: [u8; 0x7F],

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
}

impl Hardware for Gameboy {
    fn mem_read8(&mut self, addr: u16) -> u8 {
        // Reading from the memory bus takes a full CPU cycle.
        self.clock();

        match addr {
            0x0000 ... 0x0100 if self.bootrom_enabled => self.bootrom_read(addr),
            0x0000 ... 0x3FFF => self.cart.rom_lo(&self.rom)[addr as usize],
            0x4000 ... 0x7FFF => self.cart.rom_hi(&self.rom)[(addr - 0x4000) as usize],
            0xA000 ... 0xBFFF => self.cart.ram()[(addr - 0xA000) as usize],

            0x8000 ... 0x9FFF => self.ppu.vram_read(addr - 0x8000),
            0xC000 ... 0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000 ... 0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFF00            => self.joypad.reg_p1_read(),
            0xFF01            => self.serial.reg_sb_read(),

            0xFF04            => (self.timer.div >> 8) as u8,
            0xFF05            => self.timer.tima,
            0xFF06            => self.timer.tma,
            0xFF07            => self.timer.reg_tac_read(),

            0xFF0F            => 0xE0 | self.interrupts.request, // Unused IF bits are always 1

            // TODO: APU
            0xFF10 ... 0xFF26 => 0x00,

            0xFF40            => self.ppu.reg_lcdc_read(),
            0xFF42            => self.ppu.scy,
            0xFF43            => self.ppu.scx,
            0xFF44            => self.ppu.ly,

            0xFF47            => self.ppu.bgp.pack(),
            0xFF48            => self.ppu.obp0.pack(),
            0xFF49            => self.ppu.obp1.pack(),

            0xFF80 ... 0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF            => self.interrupts.enable,

            _ => panic!("Unhandled memory read from 0x{:X}", addr),
        }
    }

    fn mem_write8(&mut self, addr: u16, v: u8) {
        // Writing to the memory bus takes a full CPU cycle.
        self.clock();

        match addr {
            0x0000 ... 0x7FFF => { self.cart.write(addr, v); }
            0x8000 ... 0x9FFF => { self.ppu.vram_write(addr - 0x8000, v) }
            0xA000 ... 0xBFFF => { self.cart.write(addr, v); }
            0xC000 ... 0xDFFF => { self.ram[(addr - 0xC000) as usize] = v },
            0xE000 ... 0xFDFF => { self.ram[(addr - 0xE000) as usize] = v },
            0xFF00            => { self.joypad.reg_p1_write(v) }
            0xFF01            => { self.serial.reg_sb_write(v) },
            0xFF02            => { self.serial.reg_sc_write(v) },

            0xFF04            => { self.timer.reg_div_write() }
            0xFF05            => { self.timer.reg_tima_write(v) }
            0xFF06            => { self.timer.reg_tma_write(v) }
            0xFF07            => { if self.timer.reg_tac_write(v & 0x7) { self.interrupts.request(Interrupt::Timer) } }

            0xFF0F            => { self.interrupts.reg_if_write(v & 0x1F) }

            // TODO: APU
            0xFF10 ... 0xFF26 => { },

            0xFF40            => { self.ppu.reg_lcdc_write(v) }
            0xFF42            => { self.ppu.scy = v }
            0xFF43            => { self.ppu.scx = v }
            0xFF47            => { self.ppu.bgp.update(v) }
            0xFF48            => { self.ppu.obp0.update(v) }
            0xFF49            => { self.ppu.obp1.update(v) }

            0xFF50 if self.bootrom_enabled && v == 1 => {
                self.bootrom_enabled = false;
            }

            0xFF80 ... 0xFFFE => { self.hram[(addr - 0xFF80) as usize] = v }
            0xFFFF            => { self.interrupts.reg_ie_write(v) }

            _ => panic!("Unhandled memory write to 0x{:X}", addr),
        }
    }

    fn clock(&mut self) {
        if self.timer.clock() {
            self.interrupts.request(Interrupt::Timer);
        }
        // self.dma.clock();
        self.serial.clock();
        self.ppu.clock();
        self.apu.clock();
    }

    fn next_interrupt(&self) -> Option<Interrupt> {
        self.interrupts.next_interrupt()
    }

    fn clear_interrupt(&mut self, int: Interrupt) {
        self.interrupts.clear(int);
    }
}
