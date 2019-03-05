use oxideboy::*;
use paste;
use std::time::Instant;

fn run_mooneye_test(rom: &[u8], model: Model, enable_bootrom: bool) {
    let mut gb = Gameboy::new(model, rom.to_vec());
    if !enable_bootrom {
        gb.skip_bootrom();
    }

    let start = Instant::now();
    loop {
        if gb.hw.mem_get(gb.cpu.pc) == 0x40 {
            break;
        }

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
        paste::item! {
            #[test]
            fn [<mooneye_acceptance_ $name>] () {
                run_mooneye_test(include_bytes!(concat!("mooneye/acceptance/", $path, ".gb")), $model, $bootrom);
            }
        }
        )*
    }
}

test_cases! {
    bits_mem_oam:                       "bits/mem_oam",                     Model::DMG, false,
    bits_reg_f:                         "bits/reg_f",                       Model::DMG, false,
    bits_unused_hwio:                   "bits/unused_hwio-GS",              Model::DMG, false,

    instr_daa:                          "instr/daa",                        Model::DMG, false,

    interrupts_ie_push:                 "interrupts/ie_push",               Model::DMG, false,

    oam_dma_basic:                      "oam_dma/basic",                    Model::DMG, false,
    oam_dma_reg_read:                   "oam_dma/reg_read",                 Model::DMG, false,
//  oam_dma_sources:                    "oam_dma/sources-dmgABCmgbS",       Model::DMG, false,

    ppu_hblank_ly_scx_timing:           "ppu/hblank_ly_scx_timing-GS",      Model::DMG, false,
    ppu_intr_1_2_timing:                "ppu/intr_1_2_timing-GS",           Model::DMG, false,
    ppu_intr_2_0_timing:                "ppu/intr_2_0_timing",              Model::DMG, false,
    ppu_intr_2_mode0_timing:            "ppu/intr_2_mode0_timing",          Model::DMG, false,
    ppu_intr_2_mode0_timing_sprites:    "ppu/intr_2_mode0_timing_sprites",  Model::DMG, false,
    ppu_intr_2_mode3_timing:            "ppu/intr_2_mode3_timing",          Model::DMG, false,
    ppu_intr_2_oam_ok_timing:           "ppu/intr_2_oam_ok_timing",         Model::DMG, false,
    ppu_lcdon_timing:                   "ppu/lcdon_timing-dmgABCmgbS",      Model::DMG, false,
//  ppu_lcdon_write_timing:             "ppu/lcdon_write_timing-GS",        Model::DMG, false,
//  ppu_stat_irq_blocking:              "ppu/stat_irq_blocking",            Model::DMG, false,
//  ppu_stat_lyc_onoff:                 "ppu/stat_lyc_onoff",               Model::DMG, false,
    ppu_vblank_stat_intr:               "ppu/vblank_stat_intr-GS",          Model::DMG, false,

    serial_boot_sclk_align:             "serial/boot_sclk_align-dmgABCmgb", Model::DMG, false,

    timer_div_write:                    "timer/div_write",                  Model::DMG, false,
    timer_rapid_toggle:                 "timer/rapid_toggle",               Model::DMG, false,
    timer_tim00:                        "timer/tim00",                      Model::DMG, false,
    timer_tim00_div_trigger:            "timer/tim00_div_trigger",          Model::DMG, false,
    timer_tim01:                        "timer/tim01",                      Model::DMG, false,
    timer_tim01_div_trigger:            "timer/tim01_div_trigger",          Model::DMG, false,
    timer_tim10:                        "timer/tim10",                      Model::DMG, false,
    timer_tim10_div_trigger:            "timer/tim10_div_trigger",          Model::DMG, false,
    timer_tim11:                        "timer/tim11",                      Model::DMG, false,
    timer_tim11_div_trigger:            "timer/tim11_div_trigger",          Model::DMG, false,
    timer_tima_reload:                  "timer/tima_reload",                Model::DMG, false,
    timer_tima_write_reloading:         "timer/tima_write_reloading",       Model::DMG, false,
    timer_tma_write_reloading:          "timer/tma_write_reloading",        Model::DMG, false,

    add_sp_e_timing:                    "add_sp_e_timing",                  Model::DMG, false,
    boot_div_dmg0:                      "boot_div-dmg0",                    Model::DMG0, false,
    boot_div_dmg0_realbootrom:          "boot_div-dmg0",                    Model::DMG0, true,
    boot_div_dmg_abc_mgb:               "boot_div-dmgABCmgb",               Model::DMG, false,
    boot_div_dmg_abc_mgb_realbootrom:   "boot_div-dmgABCmgb",               Model::DMG, true,
    boot_hwio_dmg0:                     "boot_hwio-dmg0",                   Model::DMG0, false,
    boot_hwio_dmg0_realbootrom:         "boot_hwio-dmg0",                   Model::DMG0, true,
    boot_hwio_dmg_abc_mgb:              "boot_hwio-dmgABCmgb",              Model::DMG, false,
    boot_hwio_dmg_abc_mgb_realbootrom:  "boot_hwio-dmgABCmgb",              Model::DMG, true,
    boot_regs_dmg0:                     "boot_regs-dmg0",                   Model::DMG0, false,
    boot_regs_dmg0_realbootrom:         "boot_regs-dmg0",                   Model::DMG0, true,
    boot_regs_dmg_abc:                  "boot_regs-dmgABC",                 Model::DMG, false,
    boot_regs_dmg_abc_realbootrom:      "boot_regs-dmgABC",                 Model::DMG, true,
    call_cc_timing:                     "call_cc_timing",                   Model::DMG, false,
    call_cc_timing2:                    "call_cc_timing2",                  Model::DMG, false,
    call_timing:                        "call_timing",                      Model::DMG, false,
    call_timing2:                       "call_timing2",                     Model::DMG, false,
    di_timing:                          "di_timing-GS",                     Model::DMG, false,
    div_timing:                         "div_timing",                       Model::DMG, false,
    ei_sequence:                        "ei_sequence",                      Model::DMG, false,
    ei_timing:                          "ei_timing",                        Model::DMG, false,
    halt_ime0_ei:                       "halt_ime0_ei",                     Model::DMG, false,
    halt_ime0_nointr_timing:            "halt_ime0_nointr_timing",          Model::DMG, false,
    halt_ime1_timing:                   "halt_ime1_timing",                 Model::DMG, false,
    halt_ime1_timing2:                  "halt_ime1_timing2-GS",             Model::DMG, false,
    if_ie_registers:                    "if_ie_registers",                  Model::DMG, false,
    intr_timing:                        "intr_timing",                      Model::DMG, false,
    jp_cc_timing:                       "jp_cc_timing",                     Model::DMG, false,
    jp_timing:                          "jp_timing",                        Model::DMG, false,
    ld_hl_sp_e_timing:                  "ld_hl_sp_e_timing",                Model::DMG, false,
    oam_dma_restart:                    "oam_dma_restart",                  Model::DMG, false,
    oam_dma_start:                      "oam_dma_start",                    Model::DMG, false,
    oam_dma_timing:                     "oam_dma_timing",                   Model::DMG, false,
    pop_timing:                         "pop_timing",                       Model::DMG, false,
    push_timing:                        "push_timing",                      Model::DMG, false,
    rapid_di_ei:                        "rapid_di_ei",                      Model::DMG, false,
    ret_cc_timing:                      "ret_cc_timing",                    Model::DMG, false,
    ret_timing:                         "ret_timing",                       Model::DMG, false,
    reti_intr_timing:                   "reti_intr_timing",                 Model::DMG, false,
    reti_timing:                        "reti_timing",                      Model::DMG, false,
    rst_timing:                         "rst_timing",                       Model::DMG, false,
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
