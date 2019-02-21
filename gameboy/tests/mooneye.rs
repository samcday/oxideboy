extern crate gameboy;

use gameboy::*;
use std::time::Instant;

fn run_mooneye_test(rom: &[u8], model: Model, enable_bootrom: bool) {
    let hw = gameboy::Gameboy::new(model, rom.to_vec());
    let mut cpu = gameboy::cpu::Cpu::new(hw);
    if !enable_bootrom { cpu.skip_bootrom(); }

    let start = Instant::now();
    while !cpu.mooneye_breakpoint {
        cpu.fetch_decode_execute();

        if start.elapsed().as_secs() > 10 {
            panic!("Test ran for more than 10 seconds");
        }
    }

    if cpu.b != 0x03 || cpu.c != 0x05 || cpu.d != 0x08 || cpu.e != 0x0D
        || cpu.h != 0x15 || cpu.l != 0x22
    {
        cpu.core_panic(String::from("Test completed in invalid state"));
    }

    if cpu.a != 0 {
        panic!("Test failed {} assertions", cpu.a);
    }
}

#[test] fn mooneye_acceptance_bits_mem_oam() { run_mooneye_test(include_bytes!("mooneye/acceptance/bits/mem_oam.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_bits_reg_f() { run_mooneye_test(include_bytes!("mooneye/acceptance/bits/reg_f.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_bits_unused_hwio_gs() { run_mooneye_test(include_bytes!("mooneye/acceptance/bits/unused_hwio-GS.gb"), Model::DMG, false); }

#[test] fn mooneye_acceptance_instr_daa() { run_mooneye_test(include_bytes!("mooneye/acceptance/instr/daa.gb"), Model::DMG, false); }

#[test] fn mooneye_acceptance_interrupts_ie_push() { run_mooneye_test(include_bytes!("mooneye/acceptance/interrupts/ie_push.gb"), Model::DMG, false); }

#[test] fn mooneye_acceptance_oam_dma_basic() { run_mooneye_test(include_bytes!("mooneye/acceptance/oam_dma/basic.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_oam_dma_reg_read() { run_mooneye_test(include_bytes!("mooneye/acceptance/oam_dma/reg_read.gb"), Model::DMG, false); }

// #[test] fn mooneye_acceptance_ppu_hblank_ly_scx_timing_gs() { run_mooneye_test(include_bytes!("mooneye/acceptance/ppu/hblank_ly_scx_timing-GS.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_ppu_intr_1_2_timing_gs() { run_mooneye_test(include_bytes!("mooneye/acceptance/ppu/intr_1_2_timing-GS.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_ppu_intr_2_0_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ppu/intr_2_0_timing.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_ppu_intr_2_mode0_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ppu/intr_2_mode0_timing.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_ppu_intr_2_mode0_timing_sprites() { run_mooneye_test(include_bytes!("mooneye/acceptance/ppu/intr_2_mode0_timing_sprites.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_ppu_intr_2_mode3_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ppu/intr_2_mode3_timing.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_ppu_intr_2_oam_ok_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ppu/intr_2_oam_ok_timing.gb"), Model::DMG, false); }

#[test] fn mooneye_acceptance_timer_div_write() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/div_write.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_rapid_toggle() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/rapid_toggle.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim00() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim00.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim00_div_trigger() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim00_div_trigger.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim01() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim01.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim01_div_trigger() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim01_div_trigger.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim10() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim10.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim10_div_trigger() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim10_div_trigger.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim11() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim11.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tim11_div_trigger() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tim11_div_trigger.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tima_reload() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tima_reload.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tima_write_reloading() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tima_write_reloading.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_timer_tma_write_reloading() { run_mooneye_test(include_bytes!("mooneye/acceptance/timer/tma_write_reloading.gb"), Model::DMG, false); }

#[test] fn mooneye_acceptance_add_sp_e_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/add_sp_e_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_boot_hwio_dmg0() { run_mooneye_test(include_bytes!("mooneye/acceptance/boot_hwio-dmg0.gb"), Model::DMG0, false); }
// #[test] fn mooneye_acceptance_boot_hwio_dmg0_realbootrom() { run_mooneye_test(include_bytes!("mooneye/acceptance/boot_hwio-dmg0.gb"), Model::DMG0, true); }
#[test] fn mooneye_acceptance_boot_regs_dmg0() { run_mooneye_test(include_bytes!("mooneye/acceptance/boot_regs-dmg0.gb"), Model::DMG0, false); }
#[test] fn mooneye_acceptance_boot_regs_dmg0_realbootrom() { run_mooneye_test(include_bytes!("mooneye/acceptance/boot_regs-dmg0.gb"), Model::DMG0, true); }
#[test] fn mooneye_acceptance_call_cc_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/call_cc_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_call_cc_timing2() { run_mooneye_test(include_bytes!("mooneye/acceptance/call_cc_timing2.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_call_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/call_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_call_timing2() { run_mooneye_test(include_bytes!("mooneye/acceptance/call_timing2.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_div_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/div_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_ei_sequence() { run_mooneye_test(include_bytes!("mooneye/acceptance/ei_sequence.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_ei_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ei_timing.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_halt_ime0_ei() { run_mooneye_test(include_bytes!("mooneye/acceptance/halt_ime0_ei.gb"), Model::DMG, false); }
// #[test] fn mooneye_acceptance_halt_ime0_nointr_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/halt_ime0_nointr_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_halt_ime1_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/halt_ime1_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_if_ie_registers() { run_mooneye_test(include_bytes!("mooneye/acceptance/if_ie_registers.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_intr_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/intr_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_jp_cc_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/jp_cc_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_jp_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/jp_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_ld_hl_sp_e_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ld_hl_sp_e_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_oam_dma_restart() { run_mooneye_test(include_bytes!("mooneye/acceptance/oam_dma_restart.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_oam_dma_start() { run_mooneye_test(include_bytes!("mooneye/acceptance/oam_dma_start.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_oam_dma_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/oam_dma_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_pop_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/pop_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_push_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/push_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_rapid_di_ei() { run_mooneye_test(include_bytes!("mooneye/acceptance/rapid_di_ei.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_ret_cc_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ret_cc_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_ret_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/ret_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_reti_intr_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/reti_intr_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_reti_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/reti_timing.gb"), Model::DMG, false); }
#[test] fn mooneye_acceptance_rst_timing() { run_mooneye_test(include_bytes!("mooneye/acceptance/rst_timing.gb"), Model::DMG, false); }

// #[test]
// fn mooneye_sprite_priority() {
//     let rom = include_bytes!("mooneye/manual-only/sprite_priority.gb");
//     let mut gameboy = GameboyContext::new(Model::DMG, rom.to_vec());
//     gameboy.skip_bootrom();

//     let start = Instant::now();
//     while !gameboy.mooneye_breakpoint {
//         gameboy::cpu::run(&mut gameboy);

//         if start.elapsed().as_secs() > 10 {
//             panic!("Test ran for more than 10 seconds");
//         }
//     }
//     // The magic breakpoint for this test is fired immediately after LCD is enabled, so let a full frame get rendered
//     // before we run the comparison.
//     for _ in 0..17556 {
//         gameboy::cpu::run(&mut gameboy);
//     }

//     common::compare_framebuffer(gameboy.state.ppu.framebuffer(),
//                                 include_bytes!("mooneye/manual-only/sprite_priority-expected.png"),
//                                 |col| {
//                                     match col {
//                                         0xFFFFFFFF => 0xFFE0F8D0,
//                                         0xFF6F6F6F => 0xFF88C070,
//                                         0xFF000000 => 0xFF081820,
//                                         _ => panic!("Unexpected reference pixel color: {:X}", col)
//                                     }
//                                 });
// }
