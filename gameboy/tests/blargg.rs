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
        if start.elapsed().as_secs() > 5 {
            panic!("Test ran for more than 5 seconds. PC={}, Output: {}", cpu.pc, serial_output);
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

#[test] fn blargg_cpu_instrs_01() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/01-special.gb")); }
#[test] fn blargg_cpu_instrs_02() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/02-interrupts.gb")); }
#[test] fn blargg_cpu_instrs_03() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/03-op sp,hl.gb")); }
#[test] fn blargg_cpu_instrs_04() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/04-op r,imm.gb")); }
#[test] fn blargg_cpu_instrs_05() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/05-op rp.gb")); }
#[test] fn blargg_cpu_instrs_06() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/06-ld r,r.gb")); }
#[test] fn blargg_cpu_instrs_07() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/07-jr,jp,call,ret,rst.gb")); }
#[test] fn blargg_cpu_instrs_08() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/08-misc instrs.gb")); }
#[test] fn blargg_cpu_instrs_09() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/09-op r,r.gb")); }
#[test] fn blargg_cpu_instrs_10() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/10-bit ops.gb")); }
#[test] fn blargg_cpu_instrs_11() { run_blargg_serial_test(include_bytes!("blargg/cpu_instrs/11-op a,(hl).gb")); }
