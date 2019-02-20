use gameboy;

fn main() {
	let hw = gameboy::Gameboy::new(gameboy::Model::DMG0);
	let mut cpu = gameboy::cpu::Cpu::new(hw);

	cpu.hw.ppu.test = 321;
	println!("Hmm: {}", cpu.hw.ppu.test);


	cpu.fetch_decode_execute();
}
