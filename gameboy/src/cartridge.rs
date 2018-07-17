#[derive(Serialize, Deserialize, Default)]
pub struct Cartridge {
    cart_type: CartridgeType,

    // Used by MBC1 + MBC3
    rom_bank: u8,
    rom_bank_count: u8,
    ram_enabled: bool,
    ram_bank: u8,
    pub ram: Vec<u8>,
}

#[derive(Serialize, Deserialize)]
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
    pub fn from_rom(rom: &[u8]) -> Self {
        let cart_type = match rom[0x147] {
            0         => CartridgeType::ROMOnly,
            1|2|3     => CartridgeType::MBC1,
            0x12|0x13 => CartridgeType::MBC3,
            v => panic!("Unsupported cartridge type: {:X}", v)
        };
        let rom_bank_count = match rom[0x148] as u32 {
            v @ 0...6 => 2u8.pow(v + 1),
            0x52 => 72,
            0x53 => 80,
            0x54 => 96,
            v => panic!("Unexpected ROM size {}", v),
        };
        let ram = match cart_type {
            CartridgeType::ROMOnly => Vec::new(),
            _ => vec![0; match rom[0x149] {
                0 => 0,
                1 => 2048,
                2 => 8192,
                3 => 32768,
                4 => 131072,
                v => panic!("Unexpected RAM size {} encountered", v),
            }]
        };
        Self{cart_type, rom_bank: 1, rom_bank_count, ram_enabled: false, ram, ram_bank: 0}
    }

    pub fn rom_lo<'a>(&self, rom: &'a[u8]) -> &'a[u8] {
        &rom[0..0x4000]
    }

    pub fn rom_hi<'a>(&self, rom: &'a[u8]) -> &'a[u8] {
        match self.cart_type {
            CartridgeType::ROMOnly => &rom[0x4000..0x8000],
            CartridgeType::MBC1|CartridgeType::MBC3 => {
                let base = (self.rom_bank as usize) * 0x4000;
                &rom[base .. base + 0x4000]
            }
        }
    }

    pub fn ram(&self) -> &[u8] {
        let base = (self.ram_bank as usize) * 0x2000;
        &self.ram[base .. base + 0x2000]
    }

    pub fn write(&mut self, addr: u16, v: u8) {
        match self.cart_type {
            CartridgeType::ROMOnly => {},
            CartridgeType::MBC1 => self.mbc1_write(addr as usize, v),
            CartridgeType::MBC3 => self.mbc3_write(addr as usize, v),
        };
    }

    fn mbc1_write(&mut self, addr: usize, v: u8) {
        match addr {
            0x0000 ... 0x1FFF => {
                self.ram_enabled = v == 0xA;
            },
            0x2000 ... 0x3FFF => {
                if v > 0x1F {
                    return;
                }
                self.rom_bank = v.max(1).min(self.rom_bank_count);
            },
            0xA000 ... 0xBFFF => {
                if !self.ram_enabled {
                    return;
                }
                let addr = (self.ram_bank as usize) * 0x2000 + addr - 0xA000;
                if addr < self.ram.len() {
                    self.ram[addr] = v;
                }
            }
            _ => panic!("Unexpected cartridge write value {:X} to address: {:X}", v, addr)
        }
    }

    fn mbc3_write(&mut self, addr: usize, v: u8) {
        match addr {
            0x0000 ... 0x1FFF => {
                self.ram_enabled = v == 0xA;
            },
            0x2000 ... 0x3FFF => {
                self.rom_bank = v.max(1);
            },
            0x4000 ... 0x5FFF => {
                self.ram_bank = v & 0b11;
            },
            0x6000 ... 0x7FFF => {
                // TODO:
            },
            0xA000 ... 0xBFFF => {
                if !self.ram_enabled {
                    return;
                }
                let addr = (self.ram_bank as usize) * 0x2000 + addr - 0xA000;
                if addr < self.ram.len() {
                    self.ram[addr] = v;
                }
            }
            _ => panic!("Unexpected cartridge write value {:X} to address: {:X}", v, addr)
        }
    }
}
