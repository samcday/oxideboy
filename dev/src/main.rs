extern crate lr35902;
extern crate gameboy;

use std::io;
use std::io::prelude::*;
use std::fs::File;

fn main() -> io::Result<()> {
    let args: Vec<String> = std::env::args().collect();

    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

    let mut cart = gameboy::cartridges::MBC1Cart::new(&rom);
    let mut cpu = lr35902::CPU::new(&mut cart);

    loop {
        cpu.run();
    }
}
