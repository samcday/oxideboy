//! Gameboy cartridges themselves contain some hardware. The Gameboy CPU assigns the address space 0x0000 ... 0xBFFF
//! to the cartridge. Reads/writes to that space are handled by whatever chip is sitting inside the Gameboy cartridge
//! itself. There's a few different cartridges that have different capabilities. Some are extremely basic and only allow
//! reads to onboard ROM. Others have some onboard persistent RAM (which is used for savegames). Others have large
//! amounts of ROM data that can be paged into the available space via writes to certain locations. We implement the
//! various types of MBCs (Memory Bank Controllers) here.

use serde::{Deserialize, Serialize};
use serde_bytes;

#[derive(Default, Deserialize, Serialize)]
pub struct Cartridge {
    cart_type: CartridgeType,
    #[serde(skip)]
    pub rom: Vec<u8>,

    // Used by MBC1 + MBC3
    ram_bank_mode: bool,
    lo_rom_bank: u8,
    hi_rom_bank: u8,
    rom_bank_mask: u8,
    ram_enabled: bool,
    ram_bank: u8,
    ram_bank_mask: u8,

    #[serde(with = "serde_bytes")]
    pub ram: Vec<u8>,
}

#[derive(Debug, Deserialize, Serialize)]
enum CartridgeType {
    ROMOnly,
    MBC1,
    MBC3,
}

impl Default for CartridgeType {
    fn default() -> Self {
        CartridgeType::ROMOnly
    }
}

impl Cartridge {
    pub fn from_rom(rom: Vec<u8>) -> Self {
        let cart_type = match rom[0x147] {
            0 => CartridgeType::ROMOnly,
            1 | 2 | 3 => CartridgeType::MBC1,
            0x12 | 0x13 => CartridgeType::MBC3,
            v => panic!("Unsupported cartridge type: {:X}", v),
        };
        let rom_bank_mask = match rom[0x148] {
            v @ 0...6 => 2u8.pow(u32::from(v) + 1),
            0x52 => 72,
            0x53 => 80,
            0x54 => 96,
            v => panic!("Unexpected ROM size {}", v),
        } - 1;
        let (ram, ram_bank_mask) = match cart_type {
            CartridgeType::ROMOnly => (Vec::new(), 0),
            _ => match rom[0x149] {
                0 => (Vec::new(), 0),
                1 => (vec![0; 2048], 0),
                2 => (vec![0; 8192], 0),
                3 => (vec![0; 32768], 0b11),
                4 => (vec![0; 131_072], 0b1111),
                5 => (vec![0; 65_536], 0b111),
                v => panic!("Unexpected RAM size {} encountered", v),
            },
        };
        Self {
            cart_type,
            ram,
            rom,
            hi_rom_bank: 1,
            rom_bank_mask,
            ram_bank_mask,

            ..Default::default()
        }
    }

    pub fn rom_title(&self) -> &str {
        self.rom[0x134..=0x143]
            .split(|b| *b == 0)
            .next()
            .and_then(|v| std::str::from_utf8(v).ok())
            .unwrap_or(&"UNKNOWN")
    }

    pub fn rom_lo(&self, addr: usize) -> u8 {
        match self.cart_type {
            CartridgeType::ROMOnly => self.rom[addr],
            CartridgeType::MBC1 | CartridgeType::MBC3 => {
                let base = (self.lo_rom_bank as usize) * 0x4000;
                self.rom[base + addr]
            }
        }
    }

    pub fn rom_hi(&self, addr: usize) -> u8 {
        match self.cart_type {
            CartridgeType::ROMOnly => self.rom[0x4000 + addr],
            CartridgeType::MBC1 | CartridgeType::MBC3 => {
                let base = (self.hi_rom_bank as usize) * 0x4000;
                self.rom[base + addr]
            }
        }
    }

    pub fn ram(&self, addr: usize) -> u8 {
        if !self.ram_enabled {
            return 0xFF;
        }

        let base = (self.ram_bank as usize) * 0x2000;
        if base + addr >= self.ram.len() {
            return 0xFF;
        }
        self.ram[base + addr]
    }

    pub fn write(&mut self, addr: u16, v: u8) {
        match self.cart_type {
            CartridgeType::ROMOnly => {}
            CartridgeType::MBC1 => self.mbc1_write(addr as usize, v),
            CartridgeType::MBC3 => self.mbc3_write(addr as usize, v),
        };
    }

    fn mbc1_write(&mut self, addr: usize, v: u8) {
        match addr {
            0x0000...0x1FFF => {
                self.ram_enabled = v & 0b1111 == 0xA;
            }
            0x2000...0x3FFF => {
                // Only 5 bits are used for this register.
                let mut bank = v & 0b11111;

                // Bank 0 is not valid, lowest bank number is 1.
                if bank == 0 {
                    bank = 1;
                }
                self.hi_rom_bank = ((self.hi_rom_bank & 0b110_0000) | bank) & self.rom_bank_mask;
            }
            0x4000...0x5FFF => {
                // Only the low 2 bits are used for this register.
                let v = v & 0b11;

                self.hi_rom_bank = ((self.hi_rom_bank & 0b11111) | (v << 5)) & self.rom_bank_mask;

                if self.ram_bank_mode {
                    self.ram_bank = v & self.ram_bank_mask;
                    self.lo_rom_bank = (v << 5) & self.rom_bank_mask;
                }
            }
            0x6000...0x7FFF => {
                self.ram_bank_mode = v & 1 == 1;
                if self.ram_bank_mode {
                    self.lo_rom_bank = 0;
                    self.hi_rom_bank &= 0b11111;
                } else {
                    self.lo_rom_bank = 0;
                    self.ram_bank = 0;
                }
            }
            0xA000...0xBFFF => {
                if !self.ram_enabled {
                    return;
                }
                let addr = (self.ram_bank as usize) * 0x2000 + addr - 0xA000;
                if addr < self.ram.len() {
                    self.ram[addr] = v;
                }
            }
            _ => panic!("Unexpected cartridge write value {:X} to address: {:X}", v, addr),
        }
    }

    fn mbc3_write(&mut self, addr: usize, v: u8) {
        match addr {
            0x0000...0x1FFF => {
                self.ram_enabled = v == 0xA;
            }
            0x2000...0x3FFF => {
                self.hi_rom_bank = v.max(1);
            }
            0x4000...0x5FFF => {
                self.ram_bank = v & 0b11;
            }
            0xA000...0xBFFF => {
                if !self.ram_enabled {
                    return;
                }
                let addr = (self.ram_bank as usize) * 0x2000 + addr - 0xA000;
                if addr < self.ram.len() {
                    self.ram[addr] = v;
                }
            }
            _ => panic!("Unexpected cartridge write value {:X} to address: {:X}", v, addr),
        }
    }
}
