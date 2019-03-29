//! Utilities for extracting metadata from Gameboy ROMs.

use serde::{Deserialize, Serialize};

pub struct Rom {
    pub data: Box<[u8]>,
    pub title: String,
    pub cgb_mode: CGBMode,
    pub cart_type: CartridgeType,
    pub supports_sgb: bool,
    pub rom_banks: usize,
    pub ram_size: usize,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub enum CartridgeType {
    ROMOnly,
    MBC1,
    MBC3,
}

#[derive(PartialEq)]
pub enum CGBMode {
    None,
    BackwardsCompatible,
    CGBOnly,
}

impl Default for CartridgeType {
    fn default() -> Self {
        CartridgeType::ROMOnly
    }
}

impl Rom {
    pub fn new(data: Box<[u8]>) -> Result<Rom, String> {
        let header_checksum = data[0x134..=0x14c]
            .iter()
            .fold(0u8, |acc, b| acc.wrapping_sub(*b).wrapping_sub(1));

        if data[0x14d] != header_checksum {
            return Err("ROM failed header checksum".to_string());
        }

        let cgb_mode = match data[0x143] {
            0x80 => CGBMode::BackwardsCompatible,
            0xC0 => CGBMode::CGBOnly,
            _ => CGBMode::None,
        };

        let mut title_range = 0x134..0x144;
        if cgb_mode != CGBMode::None {
            // CGB roms use the last byte of what used to be the title sequence to denote CGB mode.
            title_range.end -= 1;
        }

        let title = data[title_range]
            .split(|b| *b == 0)
            .next()
            .and_then(|v| std::str::from_utf8(v).ok())
            .unwrap_or(&"UNKNOWN")
            .to_string();

        let cart_type = match data[0x147] {
            0 => CartridgeType::ROMOnly,
            1 | 2 | 3 => CartridgeType::MBC1,
            0x12 | 0x13 => CartridgeType::MBC3,
            v => return Err(format!("Unsupported cartridge type ${:02x}", v)),
        };

        let supports_sgb = data[0x146] == 3;

        let rom_banks = match data[0x148] {
            v @ 0...6 => 2usize.pow(u32::from(v) + 1),
            0x52 => 72,
            0x53 => 80,
            0x54 => 96,
            v => return Err(format!("Unsupported ROM bank count ${:02x}", v)),
        };

        let ram_size = match data[0x149] {
            0 => 0,
            1 => 2048,
            2 => 8192,
            3 => 32768,
            4 => 131_072,
            5 => 65_536,
            v => return Err(format!("Unsupported RAM size ${:02x}", v)),
        };

        Ok(Rom {
            data,
            title,
            cgb_mode,
            cart_type,
            supports_sgb,
            rom_banks,
            ram_size,
        })
    }

    pub fn rom_bank(&self, bank: usize) -> &[u8] {
        let bank = std::cmp::max(bank, self.rom_banks - 1);
        &self.data[bank * 0x4000..bank + 1 * 0x4000]
    }
}
