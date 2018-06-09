extern crate lr35902;

use std::io;
use std::io::prelude::*;
use std::fs::File;

struct Mem {
    rom: [u8; 0x8000],
}

impl lr35902::MMU for Mem {
    fn read8(&self, addr: u16) -> u8 {
        return self.rom[addr as usize];
    }

    fn read16(&self, addr: u16) -> u16 {
        let lo: u16 = self.rom[addr as usize].into();
        let hi: u16 = self.rom[(addr as usize) + 1].into();
        return hi << 8 | lo;
    }

    fn write8(&mut self, addr: u16, v: u8) {
        self.rom[addr as usize] = v;
    }

    fn write16(&mut self, addr: u16, v: u16) {
        self.rom[addr as usize] = (v & 0xff) as u8;
        self.rom[(addr+1) as usize] = ((v & 0xff00) >> 8) as u8;
    }
}

fn main() -> io::Result<()> {
    let args: Vec<String> = std::env::args().collect();

    let mut f = File::open(&args[1])?;
    let mut rom = [0; 0x8000];
    f.read(&mut rom)?;


    let mut mem = Mem{rom};
    let mut cpu = lr35902::CPU::new(&mut mem);

    loop {
        cpu.run();
    }
}
