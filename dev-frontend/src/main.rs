use gameboy;

use std::fs::File;
use std::io::prelude::*;

type Result<T> = std::result::Result<T, Box<std::error::Error>>;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

	let hw = gameboy::Gameboy::new(gameboy::Model::DMG0, rom);
	let mut cpu = gameboy::cpu::Cpu::new(hw);

	cpu.hw.ppu.test = 321;
	println!("Hmm: {}", cpu.hw.ppu.test);

	cpu.fetch_decode_execute();

	Ok(())
}
