extern crate gameboy;

use gameboy::*;
use std::time::Instant;

fn run_blargg_serial_test(rom: &[u8]) {
    let mut serial_output = String::new();
    let hw = gameboy::Gameboy::new(Model::DMG, rom.to_vec());
	let mut cpu = gameboy::cpu::Cpu::new(hw);
    // gameboy.skip_bootrom();

    let start = Instant::now();
    loop {
        if start.elapsed().as_secs() > 1 {
            panic!("Test ran for more than 1 second. PC={}, Output: {}", cpu.pc, serial_output);
        }

    	cpu.fetch_decode_execute();
        let sb = cpu.hw.serial.serial_out.take();
        if sb.is_some() {
            serial_output.push(sb.unwrap() as char);
        }

        if cpu.pc == 0xC18B {
            return;
        }
        if cpu.pc == 0xC1B9 {
            panic!("Tests failed.\n{}", serial_output);
        }
    }
}

#[test] fn blargg_cpu_instrs_06() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/06-ld r,r.gb")); }
