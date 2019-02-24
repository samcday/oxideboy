use oxideboy::*;
use std::time::Instant;

fn run_blargg_serial_test(rom: &[u8]) {
    let mut serial_output = String::new();
    let mut gb = Gameboy::new(Model::DMG, rom.to_vec(), NoopListener {});
    gb.skip_bootrom();

    let start = Instant::now();
    loop {
        if start.elapsed().as_secs() > 10 {
            panic!(
                "Test ran for more than 10 seconds. PC={}, Output: {}",
                gb.cpu.pc, serial_output
            );
        }

        gb.run_instruction();
        let sb = gb.hw.serial.serial_out.take();
        if sb.is_some() {
            serial_output.push(sb.unwrap() as char);
        }

        if gb.cpu.pc == 0xC18B {
            return;
        }
        if gb.cpu.pc == 0xC1B9 {
            panic!("Tests failed.\n{}", serial_output);
        }
    }
}

fn run_blargg_harness_test(rom: &[u8]) {
    let mut gb = Gameboy::new(Model::DMG, rom.to_vec(), NoopListener {});

    // The test runner writes the magic value to RAM before specifying that tests are in progress.
    // Which is kinda dumb. Anyway, we force that value now so we know when tests are *actually* done.
    gb.hw.cart.ram[0] = 0x80;

    gb.skip_bootrom();

    loop {
        gb.run_instruction();

        // Wait until the magic value is present in RAM and the test is signalled as complete.
        if gb.hw.cart.ram[1] == 0xDE && gb.hw.cart.ram[2] == 0xB0 && gb.hw.cart.ram[3] == 0x61 {
            let exit_code = gb.hw.cart.ram[0];
            if exit_code != 0x80 {
                let mut end = 0x04;
                while gb.hw.cart.ram[end] != 0 && end < 0x1FFF {
                    end += 1;
                }
                let output = std::str::from_utf8(&gb.hw.cart.ram[0x04..end]).unwrap();
                if exit_code != 0 {
                    panic!("Test failed. Output: {}", output);
                }
                return;
            }
        }
    }
}

macro_rules! serial_test_cases {
    ( $( $name:ident: $path:tt, )* ) => {
        $(
        #[test]
        fn $name () {
            run_blargg_serial_test(include_bytes!(concat!("blargg/", $path, ".gb")));
        }
        )*
    }
}
macro_rules! harness_test_cases {
    ( $( $name:ident: $path:tt, )* ) => {
        $(
        #[test]
        fn $name () {
            run_blargg_harness_test(include_bytes!(concat!("blargg/", $path, ".gb")));
        }
        )*
    }
}

serial_test_cases! {
    cpu_instrs_01: "cpu_instrs/01-special",
    blargg_cpu_instrs_02: "cpu_instrs/02-interrupts",
    blargg_cpu_instrs_03: "cpu_instrs/03-op sp,hl",
    blargg_cpu_instrs_04: "cpu_instrs/04-op r,imm",
    blargg_cpu_instrs_05: "cpu_instrs/05-op rp",
    blargg_cpu_instrs_06: "cpu_instrs/06-ld r,r",
    blargg_cpu_instrs_07: "cpu_instrs/07-jr,jp,call,ret,rst",
    blargg_cpu_instrs_08: "cpu_instrs/08-misc instrs",
    blargg_cpu_instrs_09: "cpu_instrs/09-op r,r",
    blargg_cpu_instrs_10: "cpu_instrs/10-bit ops",
    blargg_cpu_instrs_11: "cpu_instrs/11-op a,(hl)",
    blargg_instr_timing: "instr_timing",
    blargg_mem_timing: "mem_timing",
}
harness_test_cases! {
    blargg_dmg_sound_01: "dmg_sound/01-registers",
    blargg_dmg_sound_02: "dmg_sound/02-len ctr",
    blargg_dmg_sound_03: "dmg_sound/03-trigger",
    blargg_dmg_sound_04: "dmg_sound/04-sweep",
    blargg_dmg_sound_05: "dmg_sound/05-sweep details",
    blargg_dmg_sound_06: "dmg_sound/06-overflow on trigger",
    blargg_dmg_sound_07: "dmg_sound/07-len sweep period sync",
    blargg_dmg_sound_08: "dmg_sound/08-len ctr during power",
    // blargg_dmg_sound_09: "dmg_sound/09-wave read while on",
    // blargg_dmg_sound_10: "dmg_sound/10-wave trigger while on",
    blargg_dmg_sound_11: "dmg_sound/11-regs after power",
    // blargg_dmg_sound_12: "dmg_sound/12-wave write while on",
    // blargg_oam_bug_01: "oam_bug/1-lcd_sync",
    // blargg_oam_bug_02: "oam_bug/2-causes",
    // blargg_oam_bug_03: "oam_bug/3-non_causes",
    // blargg_oam_bug_04: "oam_bug/4-scanline_timing",
    // blargg_oam_bug_05: "oam_bug/5-timing_bug",
    // blargg_oam_bug_06: "oam_bug/6-timing_no_bug",
    // blargg_oam_bug_07: "oam_bug/7-timing_effect",
    // blargg_oam_bug_08: "oam_bug/8-instr_effect",
}
