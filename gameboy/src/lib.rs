//! Gameboy LR35902 chip implementation.
//! Things were nice and simple back in the 80s. The CPU (a modified Z80), PPU and APU chips are all in this one silicon
//! package called the LR35902. Besides this chip, the Gameboy essentially only contains an LCD controller (which is
//! pointless to emulate), speaker/gamepad circuitry, and some other misc bits and pieces. So really, the LR35902 *IS*
//! the Gameboy.
//! This module contains the main interfaces and support pieces of the chip. The CPU, PPU and APU implementations are in
//! their respective modules.

pub mod apu;
pub mod cpu;
pub mod ppu;
pub mod serial;

use apu::Apu;
use ppu::Ppu;
use serial::Serial;

static DMG0_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg0.rom");
static DMG_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg.rom");

/// There are different models of the Gameboy that each behave slightly differently (different HW quirks, etc).
/// When creating a Gameboy emulation context, the desired Model must be chosen.
#[derive(Eq, PartialEq)]
pub enum Model {
    DMG0,
    DMG,
}

/// There's not a lot of crosstalk between the various Gameboy chips, with the exception of interrupt requests from
/// the PPU, serial port, timer and joypad. We Box this struct and share it to the systems that need it.
#[derive(Default)]
pub struct Irq {
    vblank_req: bool,
    stat_req: bool,
    timer_req: bool,
    serial_req: bool,
    joypad_req: bool,
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
}

/// Gameboy represents an emulation session for an original Gameboy (DMG) running a Gameboy cartridge.
pub struct Gameboy {
    pub model: Model,
    pub apu: Apu,
    pub irq: Irq,
    pub ppu: Ppu,
    pub serial: Serial,
    pub ram:  [u8; 0x2000],
    pub hram: [u8; 0x7F],

    bootrom_enabled: bool,
}

impl Gameboy {
    pub fn new(model: Model, rom: Vec<u8>) -> Gameboy {
        Gameboy{
            model,
            apu: Apu::new(),
            irq: Default::default(),
            ppu: Ppu::new(),
            serial: Serial::new(),
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
        match addr {
            0x0000 ... 0x0100 if self.bootrom_enabled => self.bootrom_read(addr),

            0xC000 ... 0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000 ... 0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFF01            => self.serial.reg_sb_read(),
            _ => unreachable!("TODO"),
        }
    }

    fn mem_write8(&mut self, addr: u16, v: u8) {
        match addr {
            0xC000 ... 0xDFFF => { self.ram[(addr - 0xC000) as usize] = v },
            0xE000 ... 0xFDFF => { self.ram[(addr - 0xE000) as usize] = v },
            0xFF01            => { self.serial.reg_sb_write(v) },
            0xFF02            => { self.serial.reg_sc_write(v) },
            _ => unreachable!("TODO"),
        }
    }

    fn clock(&mut self) {
        self.serial.clock();
    }
}
