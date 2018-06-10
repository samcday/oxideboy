// Implementation of the various memory bank controller (MBC) types found in Gameboy cartridges.
extern crate lr35902;

pub struct MBC1Cart<'a> {
  rom: &'a[u8],
  bank_cnt: u8,
  bank_cur: u8,
}

impl <'a> MBC1Cart<'a> {
  pub fn new(rom: &'a[u8]) -> MBC1Cart<'a> {
    println!("wtf. {}", rom[0x148]);
    MBC1Cart{
      bank_cur: 1,
      bank_cnt: match rom[0x148] {
        num @ 0 ... 6  => (2 as u8).pow((num as u32) + 1),
        0x52 => 72,
        0x53 => 80,
        0x54 => 96,
        v => panic!("Unexpected ROM size: {}", v),
      },
      rom: rom
    }
  }
}

impl <'a> lr35902::Cartridge for MBC1Cart<'a> {
  fn read(&self, addr: u16) -> u8 {
    match addr {
      0x0000 ... 0x3FFF => self.rom[addr as usize],
      0x4000 ... 0x7FFF => self.rom[(self.bank_cur as usize) * 0x4000 + (addr as usize)],
      _ => panic!("Unexpected cartridge read from addr: {:X}", addr),
    }
  }

  fn write(&mut self, addr: u16, v: u8) {
    match addr {
      0x2000 ... 0x3FFF => {
        match v {
          0 ... 0x1F => {
            self.bank_cur = v.min(1);
            println!("Selected bank {}", self.bank_cur);
            if self.bank_cur >= self.bank_cnt {
              panic!("ROM bank {} requested, maximum is {}", self.bank_cur, self.bank_cnt);
            }
          },
          _ => panic!("Unexpected ROM bank {} selected", v)
        }
      },
      _ => panic!("Unexpected cartridge write to address: {:X}", addr)
    }
  }
}
