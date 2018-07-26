extern crate gameboy;

use gameboy::*;

fn run_blargg_serial_test(rom: &[u8]) {
    let mut serial_output = String::new();
    let mut gameboy = GameboyContext::new(rom.to_vec());
    gameboy.skip_bootrom();

    loop {
        gameboy::cpu::run(&mut gameboy);
        let sb = gameboy.state.cpu.sb.take();
        if sb.is_some() {
            serial_output.push(sb.unwrap() as char);
        }

        if gameboy.state.cpu.pc == 0xC18B {
            return;
        }
        if gameboy.state.cpu.pc == 0xC1B9 {
            panic!("Tests failed.\n{}", serial_output);
        }
    }
}

fn run_blargg_harness_test(rom: &[u8]) {
    let mut gameboy = GameboyContext::new(rom.to_vec());
    gameboy.skip_bootrom();

    // The test runner writes the magic value to RAM before specifying that tests are in progress.
    // Which is kinda dumb. Anyway, we force that value now so we know when tests are *actually* done.
    gameboy.state.cart.ram[0] = 0x80;

    loop {
        gameboy::cpu::run(&mut gameboy);

        // Wait until the magic value is present in RAM and the test is signalled as complete.
        if gameboy.state.cart.ram[1] == 0xDE && gameboy.state.cart.ram[2] == 0xB0 && gameboy.state.cart.ram[3] == 0x61 {
            let exit_code = gameboy.state.cart.ram[0];
            if exit_code != 0x80 {
                let mut end = 0x04;
                while gameboy.state.cart.ram[end] != 0 && end < 0x1FFF {
                    end += 1;
                }
                let output = std::str::from_utf8(&gameboy.state.cart.ram[0x04..end]).unwrap();
                if exit_code != 0 {
                    panic!("Test failed. Output: {}", output);
                }
                return;
            }
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
#[test] fn blargg_instr_timing()  { run_blargg_serial_test(include_bytes!("blargg/instr_timing.gb")); }
#[test] fn blargg_mem_timing()    { run_blargg_serial_test(include_bytes!("blargg/mem_timing.gb")); }
#[test] fn blargg_dmg_sound_01()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/01-registers.gb")); }
#[test] fn blargg_dmg_sound_02()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/02-len ctr.gb")); }
#[test] fn blargg_dmg_sound_03()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/03-trigger.gb")); }
#[test] fn blargg_dmg_sound_04()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/04-sweep.gb")); }
#[test] fn blargg_dmg_sound_05()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/05-sweep details.gb")); }
#[test] fn blargg_dmg_sound_06()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/06-overflow on trigger.gb")); }
#[test] fn blargg_dmg_sound_07()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/07-len sweep period sync.gb")); }
#[test] fn blargg_dmg_sound_08()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/08-len ctr during power.gb")); }
// #[test] fn blargg_dmg_sound_09()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/09-wave read while on.gb")); }
// #[test] fn blargg_dmg_sound_10()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/10-wave trigger while on.gb")); }
#[test] fn blargg_dmg_sound_11()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/11-regs after power.gb")); }
// #[test] fn blargg_dmg_sound_12()  { run_blargg_harness_test(include_bytes!("blargg/dmg_sound/12-wave write while on.gb")); }
// #[test] fn blargg_oam_bug_01()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/1-lcd_sync.gb")); }
// #[test] fn blargg_oam_bug_02()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/2-causes.gb")); }
// #[test] fn blargg_oam_bug_03()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/3-non_causes.gb")); }
// #[test] fn blargg_oam_bug_04()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/4-scanline_timing.gb")); }
// #[test] fn blargg_oam_bug_05()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/5-timing_bug.gb")); }
// #[test] fn blargg_oam_bug_06()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/6-timing_no_bug.gb")); }
// #[test] fn blargg_oam_bug_07()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/7-timing_effect.gb")); }
// #[test] fn blargg_oam_bug_08()    { run_blargg_harness_test(include_bytes!("blargg/oam_bug/8-instr_effect.gb")); }
