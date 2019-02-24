use core::cell::RefCell;
use oxideboy::*;
use std::rc::Rc;
use std::time::Instant;

struct DebugBreakpointListener {
    breakpoint_hit: Rc<RefCell<bool>>,
}

impl EventListener for DebugBreakpointListener {
    fn on_frame(&mut self, _: &[u32]) {}
    fn on_memory_write(&mut self, _: u16, _: u8) {}
    fn on_debug_breakpoint(&mut self) {
        self.breakpoint_hit.replace(true);
    }
}

fn run_mooneye_test(rom: &[u8], model: Model, enable_bootrom: bool) {
    let breakpoint_hit = Rc::new(RefCell::new(false));
    let mut gb = Gameboy::new(
        model,
        rom.to_vec(),
        DebugBreakpointListener {
            breakpoint_hit: breakpoint_hit.clone(),
        },
    );
    if !enable_bootrom {
        gb.skip_bootrom();
    }

    let start = Instant::now();
    while !(*breakpoint_hit.borrow()) {
        gb.run_instruction();

        if start.elapsed().as_secs() > 10 {
            gb.core_panic(String::from("Test ran for more than 10 seconds"));
        }
    }

    if gb.cpu.b != 0x03
        || gb.cpu.c != 0x05
        || gb.cpu.d != 0x08
        || gb.cpu.e != 0x0D
        || gb.cpu.h != 0x15
        || gb.cpu.l != 0x22
    {
        gb.core_panic(String::from("Test completed in invalid state"));
    }

    if gb.cpu.a != 0 {
        panic!("Test failed {} assertions", gb.cpu.a);
    }
}

macro_rules! test_cases {
    ( $( $name:ident: $path:tt,$model:expr,$bootrom:expr, )* ) => {
        $(
        #[test]
        fn $name () {
            run_mooneye_test(include_bytes!(concat!("mooneye/", $path, ".gb")), $model, $bootrom);
        }
        )*
    }
}

test_cases! {
    mooneye_acceptance_bits_mem_oam: "acceptance/bits/mem_oam", Model::DMG, false,
    mooneye_acceptance_bits_reg_f: "acceptance/bits/reg_f", Model::DMG, false,
    mooneye_acceptance_bits_unused_hwio_gs: "acceptance/bits/unused_hwio-GS", Model::DMG, false,

    mooneye_acceptance_instr_daa: "acceptance/instr/daa", Model::DMG, false,

    mooneye_acceptance_interrupts_ie_push: "acceptance/interrupts/ie_push", Model::DMG, false,

    mooneye_acceptance_oam_dma_basic: "acceptance/oam_dma/basic", Model::DMG, false,
    mooneye_acceptance_oam_dma_reg_read: "acceptance/oam_dma/reg_read", Model::DMG, false,

    // mooneye_acceptance_ppu_hblank_ly_scx_timing_gs: "acceptance/ppu/hblank_ly_scx_timing-GS", Model::DMG, false,
    mooneye_acceptance_ppu_intr_1_2_timing_gs: "acceptance/ppu/intr_1_2_timing-GS", Model::DMG, false,
    mooneye_acceptance_ppu_intr_2_0_timing: "acceptance/ppu/intr_2_0_timing", Model::DMG, false,
    mooneye_acceptance_ppu_intr_2_mode0_timing: "acceptance/ppu/intr_2_mode0_timing", Model::DMG, false,
    // mooneye_acceptance_ppu_intr_2_mode0_timing_sprites: "acceptance/ppu/intr_2_mode0_timing_sprites", Model::DMG, false,
    mooneye_acceptance_ppu_intr_2_mode3_timing: "acceptance/ppu/intr_2_mode3_timing", Model::DMG, false,
    mooneye_acceptance_ppu_intr_2_oam_ok_timing: "acceptance/ppu/intr_2_oam_ok_timing", Model::DMG, false,
    // mooneye_acceptance_ppu_lcdon_write_timing_gs: "acceptance/ppu/lcdon_write_timing-GS", Model::DMG, false,
    // mooneye_acceptance_ppu_stat_irq_blocking: "acceptance/ppu/stat_irq_blocking", Model::DMG, false,
    // mooneye_acceptance_ppu_vblank_stat_intr_gs: "acceptance/ppu/vblank_stat_intr-GS", Model::DMG, false,

    mooneye_acceptance_timer_div_write: "acceptance/timer/div_write", Model::DMG, false,
    mooneye_acceptance_timer_rapid_toggle: "acceptance/timer/rapid_toggle", Model::DMG, false,
    mooneye_acceptance_timer_tim00: "acceptance/timer/tim00", Model::DMG, false,
    mooneye_acceptance_timer_tim00_div_trigger: "acceptance/timer/tim00_div_trigger", Model::DMG, false,
    mooneye_acceptance_timer_tim01: "acceptance/timer/tim01", Model::DMG, false,
    mooneye_acceptance_timer_tim01_div_trigger: "acceptance/timer/tim01_div_trigger", Model::DMG, false,
    mooneye_acceptance_timer_tim10: "acceptance/timer/tim10", Model::DMG, false,
    mooneye_acceptance_timer_tim10_div_trigger: "acceptance/timer/tim10_div_trigger", Model::DMG, false,
    mooneye_acceptance_timer_tim11: "acceptance/timer/tim11", Model::DMG, false,
    mooneye_acceptance_timer_tim11_div_trigger: "acceptance/timer/tim11_div_trigger", Model::DMG, false,
    mooneye_acceptance_timer_tima_reload: "acceptance/timer/tima_reload", Model::DMG, false,
    mooneye_acceptance_timer_tima_write_reloading: "acceptance/timer/tima_write_reloading", Model::DMG, false,
    mooneye_acceptance_timer_tma_write_reloading: "acceptance/timer/tma_write_reloading", Model::DMG, false,

    mooneye_acceptance_add_sp_e_timing: "acceptance/add_sp_e_timing", Model::DMG, false,
    // mooneye_acceptance_boot_div_dmg0: "acceptance/boot_div-dmg0", Model::DMG0, false,
    mooneye_acceptance_boot_hwio_dmg0: "acceptance/boot_hwio-dmg0", Model::DMG0, false,
    // mooneye_acceptance_boot_hwio_dmg0_realbootrom: "acceptance/boot_hwio-dmg0", Model::DMG0, true,
    mooneye_acceptance_boot_regs_dmg0: "acceptance/boot_regs-dmg0", Model::DMG0, false,
    mooneye_acceptance_boot_regs_dmg0_realbootrom: "acceptance/boot_regs-dmg0", Model::DMG0, true,
    mooneye_acceptance_call_cc_timing: "acceptance/call_cc_timing", Model::DMG, false,
    mooneye_acceptance_call_cc_timing2: "acceptance/call_cc_timing2", Model::DMG, false,
    mooneye_acceptance_call_timing: "acceptance/call_timing", Model::DMG, false,
    mooneye_acceptance_call_timing2: "acceptance/call_timing2", Model::DMG, false,
    mooneye_acceptance_div_timing: "acceptance/div_timing", Model::DMG, false,
    mooneye_acceptance_ei_sequence: "acceptance/ei_sequence", Model::DMG, false,
    mooneye_acceptance_ei_timing: "acceptance/ei_timing", Model::DMG, false,
    // mooneye_acceptance_halt_ime0_ei: "acceptance/halt_ime0_ei", Model::DMG, false,
    mooneye_acceptance_halt_ime0_nointr_timing: "acceptance/halt_ime0_nointr_timing", Model::DMG, false,
    mooneye_acceptance_halt_ime1_timing: "acceptance/halt_ime1_timing", Model::DMG, false,
    mooneye_acceptance_if_ie_registers: "acceptance/if_ie_registers", Model::DMG, false,
    mooneye_acceptance_intr_timing: "acceptance/intr_timing", Model::DMG, false,
    mooneye_acceptance_jp_cc_timing: "acceptance/jp_cc_timing", Model::DMG, false,
    mooneye_acceptance_jp_timing: "acceptance/jp_timing", Model::DMG, false,
    mooneye_acceptance_ld_hl_sp_e_timing: "acceptance/ld_hl_sp_e_timing", Model::DMG, false,
    mooneye_acceptance_oam_dma_restart: "acceptance/oam_dma_restart", Model::DMG, false,
    mooneye_acceptance_oam_dma_start: "acceptance/oam_dma_start", Model::DMG, false,
    mooneye_acceptance_oam_dma_timing: "acceptance/oam_dma_timing", Model::DMG, false,
    mooneye_acceptance_pop_timing: "acceptance/pop_timing", Model::DMG, false,
    mooneye_acceptance_push_timing: "acceptance/push_timing", Model::DMG, false,
    mooneye_acceptance_rapid_di_ei: "acceptance/rapid_di_ei", Model::DMG, false,
    mooneye_acceptance_ret_cc_timing: "acceptance/ret_cc_timing", Model::DMG, false,
    mooneye_acceptance_ret_timing: "acceptance/ret_timing", Model::DMG, false,
    mooneye_acceptance_reti_intr_timing: "acceptance/reti_intr_timing", Model::DMG, false,
    mooneye_acceptance_reti_timing: "acceptance/reti_timing", Model::DMG, false,
    mooneye_acceptance_rst_timing: "acceptance/rst_timing", Model::DMG, false,
}

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
