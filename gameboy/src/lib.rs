
use lr35902::CartridgeMemory;

use lr35902::SRAM_SIZE;


use lr35902::NoopMBC;

extern crate lr35902;

pub struct Gameboy<'a> {
    pub cpu: lr35902::CPU<'a>,
    pub breakpoint_hit: bool,
}

impl <'a> Gameboy<'a> {
    /// Creates a new Gameboy instance with the given ROM data.
    pub fn new(rom: &[u8]) -> Result<Gameboy, String> {

        // let cart: Cart = cartridges::MBC1Cart::new(rom);

        let ext = lr35902::ExternalInterface{
            raw_rom: &rom,
            cart_mem: CartridgeMemory {
                rom_lo: &rom[0x0000 .. 0x4000],
                rom_hi: &rom[0x4000 .. 0x8000],
            },
            mbc: Box::new(NoopMBC{}),
            video_callback: &|_| {},
        };

        let cpu = lr35902::CPU::new(ext);

        Ok(Gameboy{
            cpu: cpu,
            breakpoint_hit: false,
        })
    }

    /// Runs the Gameboy emulation core until a whole video frame has been generated.
    /// The native Gameboy renders frames at a rate of 59.7Hz, it is up to the caller to rate limit
    /// itself to provide a realistic frame rate.
    pub fn run_frame(&mut self) -> (&[u32; lr35902::SCREEN_SIZE], &[f32]) {
        self.cpu.sound.clear_samples();

        self.cpu.run();
        while !self.cpu.is_vblank() {
            if self.cpu.pc() == 0x681 {
                self.breakpoint_hit = true;
                break;
            }
            self.cpu.run();
        }
        (self.cpu.framebuffer(), self.cpu.sound.get_samples())
    }

    pub fn joypad(&mut self) -> &mut lr35902::Joypad {
        self.cpu.joypad()
    }
}

pub struct MBC1 {
    rom_bank_cnt: u8,
    rom_bank_cur: u8,

    ram: Vec<u8>,
    ram_enabled: bool,
    ram_bank_cur: u8,
    _ram_bank_cnt: u8,
}

impl lr35902::MBC for MBC1 {
    fn write<'a>(&mut self, addr: u16, v: u8, raw_rom: &'a[u8], mem: &'a mut CartridgeMemory) {
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
                        let base = (self.rom_bank_cur as usize) * 0x4000;
                        mem.rom_hi = &raw_rom[base .. base + 0x4000];
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

pub struct MBC1Cart {
    rom: Vec<u8>,
    rom_bank_cnt: u8,
    rom_bank_cur: u8,

    ram: Vec<u8>,
    ram_enabled: bool,
    ram_bank_cur: u8,
    _ram_bank_cnt: u8,
}

impl MBC1Cart {
  pub fn new(raw: &[u8]) -> MBC1Cart {
    let mut rom = Vec::new();
    rom.extend_from_slice(raw);

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
      }],
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

impl lr35902::Cartridge for MBC1Cart {
  fn lo_rom(&self) -> &[u8] {
    &self.rom[0x0000 .. 0x4000]
  }
  fn hi_rom(&self) -> &[u8] {
    let base = (self.rom_bank_cur as usize) * 0x4000;
    &self.rom[base .. base + 0x4000]
  }
  fn ram(&self) -> &[u8] {
    let base = (self.ram_bank_cur as usize) * 0x2000;
    &self.ram[base .. base + 0x2000]
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
