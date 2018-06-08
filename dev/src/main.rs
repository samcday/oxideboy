extern crate lr35902;

use std::io;
use std::io::prelude::*;
use std::fs::File;

struct Mem {
    bootrom: [u8; 0x100],
    rom: [u8; 0x3FFF],
}

impl lr35902::MMU for Mem {
    fn read8(&self, addr: u16) -> u8 {
        if addr <= 0xFF {
            return self.bootrom[addr as usize];
        }

        return self.rom[addr as usize];
    }

    fn read16(&self, addr: u16) -> u16 {
        if addr <= 0xFF {
            let lo: u16 = self.bootrom[addr as usize].into();
            let hi: u16 = self.bootrom[(addr as usize) + 1].into();
            return hi << 8 | lo;
        }

        let lo: u16 = self.rom[addr as usize].into();
        let hi: u16 = self.rom[(addr as usize) + 1].into();
        return hi << 8 | lo;
    }

    fn write8(&mut self, addr: u16, v: u8) {
        if addr <= 0xFF {
            self.bootrom[addr as usize] = v;
        }
        else if addr < 0x8000 {
            self.rom[addr as usize] = v;
        }
    }

    fn write16(&mut self, addr: u16, v: u16) {
        if addr <= 0xFF {
            self.bootrom[addr as usize] = (v & 0xff) as u8;
            self.bootrom[(addr+1) as usize] = ((v & 0xff00) >> 8) as u8;
        }
        else if addr < 0x8000 {
            self.rom[addr as usize] = (v & 0xff) as u8;
            self.rom[(addr+1) as usize] = ((v & 0xff00) >> 8) as u8;
        }
    }
}

fn main() -> io::Result<()> {
    let mut f = File::open("boot.rom")?;
    let mut bootrom = [0; 0x100];
    f.read(&mut bootrom)?;

    let mut f = File::open("game.rom")?;
    let mut rom = [0; 0x3FFF];
    f.read(&mut rom)?;


    let mut mem = Mem{bootrom, rom};
    let mut cpu = lr35902::CPU::new(&mut mem);

    loop {
        cpu.run();
    }
}
