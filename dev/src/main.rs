extern crate lr35902;

use std::io;
use std::io::prelude::*;
use std::fs::File;

struct Mem {
    bootrom: [u8; 0xFF],
    rom: [u8; 0x3FFF],
    lol: [u8; u16::max_value() as usize]
}

impl lr35902::MMU for Mem {
    fn read8(&self, addr: u16) -> u8 {
        if addr < 0xFF {
            return self.bootrom[addr as usize];
        }

        if addr < 0x8000 {
            return self.rom[addr as usize];
        }

        return self.lol[addr as usize];
    }

    fn read16(&self, addr: u16) -> u16 {
        if addr < 0xFF {
            let hi: u16 = self.bootrom[addr as usize].into();
            let lo: u16 = self.bootrom[(addr as usize) + 1].into();
            return hi << 8 | lo;
        }

        if addr < 0x8000 {
            let hi: u16 = self.rom[addr as usize].into();
            let lo: u16 = self.rom[(addr as usize) + 1].into();
            return hi << 8 | lo;
        }

        let hi: u16 = self.lol[addr as usize].into();
        let lo: u16 = self.lol[(addr as usize) + 1].into();
        return hi << 8 | lo;
    }

    fn write8(&mut self, addr: u16, v: u8) {
        if addr < 0xFF {
            self.bootrom[addr as usize] = v;
        }
        else if addr < 0x8000 {
            self.rom[addr as usize] = v;
        }
        else {
            self.lol[addr as usize] = v;
        }
    }

    fn write16(&mut self, addr: u16, v: u16) {
        if addr < 0xFF {
            self.bootrom[addr as usize] = (v & 0xff) as u8;
            self.bootrom[(addr+1) as usize] = (v & 0xff00 >> 8) as u8;
        }
        else if addr < 0x8000 {
            self.rom[addr as usize] = (v & 0xff) as u8;
            self.rom[(addr+1) as usize] = (v & 0xff00 >> 8) as u8;
        }
        else {
            self.lol[addr as usize] = (v & 0xff) as u8;
            self.lol[(addr+1) as usize] = (v & 0xff00 >> 8) as u8;
        }
    }
}

fn main() -> io::Result<()> {
    let mut f = File::open("boot.rom")?;
    let mut bootrom = [0; 0xFF];
    f.read(&mut bootrom)?;
    
    let mut f = File::open("game.rom")?;
    let mut rom = [0; 0x3FFF];
    f.read(&mut rom)?;

    let mut mem = Mem{bootrom, rom, lol: [0; u16::max_value() as usize]};
    let mut cpu = lr35902::CPU::new(&mut mem);

    loop {
        cpu.run();
    }
}
