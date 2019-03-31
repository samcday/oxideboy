//! Gameboy cartridges themselves contain some hardware. The Gameboy CPU assigns the address space 0x0000 ... 0xBFFF
//! to the cartridge. Reads/writes to that space are handled by whatever chip is sitting inside the Gameboy cartridge
//! itself. There's a few different cartridges that have different capabilities. Some are extremely basic and only allow
//! reads to onboard ROM. Others have some onboard persistent RAM (which is used for savegames). Others have large
//! amounts of ROM data that can be paged into the available space via writes to certain locations. We implement the
//! various types of MBCs (Memory Bank Controllers) here.

use crate::rom::CartridgeType;
use crate::rom::Rom;
use serde::{Deserialize, Serialize};
use serde_bytes;

#[derive(Default, Deserialize, Serialize)]
pub struct Cartridge {
    cart_type: CartridgeType,

    // Used by MBC1 + MBC3
    ram_bank_mode: bool,
    pub lo_rom_bank: u8,
    pub hi_rom_bank: u8,
    rom_bank_mask: u8,
    ram_enabled: bool,
    ram_bank: u8,
    ram_bank_mask: u8,

    #[serde(with = "serde_bytes")]
    pub ram: Vec<u8>,
}

impl Cartridge {
    pub fn from_rom(rom: &Rom) -> Self {
        let rom_bank_mask = (rom.rom_banks - 1) as u8;
        let ram = vec![0; rom.ram_size];
        let ram_bank_mask = (ram.len() / 0x2000).saturating_sub(1) as u8;

        Self {
            cart_type: rom.cart_type,
            ram,
            hi_rom_bank: 1,
            rom_bank_mask,
            ram_bank_mask,

            ..Default::default()
        }
    }

    pub fn rom_lo<'a>(&self, rom: &'a [u8]) -> &'a [u8] {
        match self.cart_type {
            CartridgeType::ROMOnly => &rom[0..0x4000],
            CartridgeType::MBC1 | CartridgeType::MBC3 => {
                let base = (self.lo_rom_bank as usize) * 0x4000;
                &rom[base..base + 0x4000]
            }
        }
    }

    pub fn rom_hi<'a>(&self, rom: &'a [u8]) -> &'a [u8] {
        match self.cart_type {
            CartridgeType::ROMOnly => &rom[0x4000..0x8000],
            CartridgeType::MBC1 | CartridgeType::MBC3 => {
                let base = (self.hi_rom_bank as usize) * 0x4000;
                &rom[base..base + 0x4000]
            }
        }
    }

    pub fn ram(&self) -> &[u8] {
        if !self.ram_enabled {
            return &[];
        }
        let base = (self.ram_bank as usize) * 0x2000;
        let end = std::cmp::max(base + 0x2000, self.ram.len());
        &self.ram[base..end]
    }

    pub fn ram_read(&self, addr: usize) -> u8 {
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
