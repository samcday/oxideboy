// Implementation of the various memory bank controller (MBC) types found in Gameboy cartridges.
extern crate lr35902;

pub struct MBC1Cart<'a> {
  rom: &'a[u8],
  rom_bank_cnt: u8,
  rom_bank_cur: u8,

  ram: Box<[u8]>,
  ram_enabled: bool,
  ram_bank_cur: u8,
  _ram_bank_cnt: u8,
}

impl <'a> MBC1Cart<'a> {
  pub fn new(rom: &'a[u8]) -> MBC1Cart<'a> {
    MBC1Cart{
      rom_bank_cur: 1,
      rom_bank_cnt: match rom[0x148] {
        num @ 0 ... 6  => (2 as u8).pow((num as u32) + 1),
        0x52 => 72,
        0x53 => 80,
        0x54 => 96,
        v => panic!("Unexpected ROM size: {}", v),
      },
      ram: vec![0; match rom[0x149] {
        0 => 0,
        1 => 2048,
        2 => 8192,
        3 => 32768,
        4 => 131072,
        v => panic!("Unexpected RAM size {} encountered", v),
      }].into_boxed_slice(),
      ram_enabled: false,
      ram_bank_cur: 0,
      _ram_bank_cnt: match rom[0x149] {
        0 => 0,
        1 => 1,
        2 => 1,
        3 => 4,
        4 => 16,
        v => panic!("Unexpected RAM size: {}", v),
      },
      rom: rom
    }
  }
}

impl <'a> lr35902::Cartridge for MBC1Cart<'a> {
  fn read(&self, addr: u16) -> u8 {
    let addr = addr as usize;
    match addr {
      0x0000 ... 0x3FFF => self.rom[addr],
      0x4000 ... 0x7FFF => self.rom[(self.rom_bank_cur as usize) * 0x4000 + ((addr - 0x4000))],
      0xA000 ... 0xBFFF => {
        if !self.ram_enabled {
          panic!("Attempt to read from disabled RAM addr {}", addr);
        }
        self.ram[(self.ram_bank_cur as usize) * 0x2000 + addr - 0xA000]
      }
      _ => panic!("Unexpected cartridge read from addr: {:X}", addr),
    }
  }

  fn write(&mut self, addr: u16, v: u8) {
    let addr = addr as usize;

    match addr {
      0x0000 ... 0x1FFF => {
        self.ram_enabled = v == 0xA;
      },
      0x2000 ... 0x3FFF => {
        match v {
          0 ... 0x1F => {
            self.rom_bank_cur = v.max(1);
            if self.rom_bank_cur >= self.rom_bank_cnt {
              panic!("ROM bank {} requested, maximum is {}", self.rom_bank_cur, self.rom_bank_cnt);
            }
          },
          _ => panic!("Unexpected ROM bank {} selected", v)
        }
      },
      0xA000 ... 0xBFFF => {
        if !self.ram_enabled {
          panic!("Attempt to write to disabled RAM addr {}", addr);
        }
        let addr = (self.ram_bank_cur as usize) * 0x2000 + addr - 0xA000;
        if addr < self.ram.len() {
          self.ram[addr] = v;
        }
      }
      _ => panic!("Unexpected cartridge write value {:X} to address: {:X}", v, addr)
    }
  }
}
