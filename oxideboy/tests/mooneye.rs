mod common;

use oxideboy::rom::Rom;
use oxideboy::Context;
use oxideboy::{Gameboy, Model};
use paste;
use std::time::Instant;

fn run_mooneye_test(rom: &[u8], model: Model, enable_bootrom: bool) {
    let rom = Rom::new(rom.into()).unwrap();
    let mut gb = Gameboy::new(model, &rom, enable_bootrom);
    let mut gb_ctx = Context::new(rom);
    gb_ctx.enable_video = false;
    gb_ctx.enable_audio = false;

    let start = Instant::now();
    loop {
        let (cpu, bus) = gb.bus(&mut gb_ctx);
        if bus.memory_get(cpu.pc) == 0x40 {
            break;
        }

        gb.run_instruction(&mut gb_ctx);

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

macro_rules! acceptance_test_cases {
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

macro_rules! mbc1_test_cases {
    ( $( $name:ident: $path:tt,$model:expr,$bootrom:expr, )* ) => {
        $(
        paste::item! {
            #[test]
            fn [<mooneye_mbc1_ $name>] () {
                run_mooneye_test(include_bytes!(concat!("mooneye/emulator-only/mbc1/", $path, ".gb")), $model, $bootrom);
            }
        }
        )*
    }
}

acceptance_test_cases! {
    bits_mem_oam:                       "bits/mem_oam",                     Model::DMGABC, false,
    bits_reg_f:                         "bits/reg_f",                       Model::DMGABC, false,
    bits_unused_hwio:                   "bits/unused_hwio-GS",              Model::DMGABC, false,

    instr_daa:                          "instr/daa",                        Model::DMGABC, false,

    interrupts_ie_push:                 "interrupts/ie_push",               Model::DMGABC, false,

    oam_dma_basic:                      "oam_dma/basic",                    Model::DMGABC, false,
    oam_dma_reg_read:                   "oam_dma/reg_read",                 Model::DMGABC, false,
    oam_dma_sources:                    "oam_dma/sources-dmgABCmgbS",       Model::DMGABC, false,

    ppu_hblank_ly_scx_timing:           "ppu/hblank_ly_scx_timing-GS",      Model::DMGABC, false,
    ppu_intr_1_2_timing:                "ppu/intr_1_2_timing-GS",           Model::DMGABC, false,
    ppu_intr_2_0_timing:                "ppu/intr_2_0_timing",              Model::DMGABC, false,
    ppu_intr_2_mode0_timing:            "ppu/intr_2_mode0_timing",          Model::DMGABC, false,
    ppu_intr_2_mode0_timing_sprites:    "ppu/intr_2_mode0_timing_sprites",  Model::DMGABC, false,
    ppu_intr_2_mode3_timing:            "ppu/intr_2_mode3_timing",          Model::DMGABC, false,
    ppu_intr_2_oam_ok_timing:           "ppu/intr_2_oam_ok_timing",         Model::DMGABC, false,
    ppu_lcdon_timing:                   "ppu/lcdon_timing-dmgABCmgbS",      Model::DMGABC, false,
    ppu_lcdon_write_timing:             "ppu/lcdon_write_timing-GS",        Model::DMGABC, false,
    ppu_stat_irq_blocking:              "ppu/stat_irq_blocking",            Model::DMGABC, false,
    ppu_stat_lyc_onoff:                 "ppu/stat_lyc_onoff",               Model::DMGABC, false,
    ppu_vblank_stat_intr:               "ppu/vblank_stat_intr-GS",          Model::DMGABC, false,

    serial_boot_sclk_align:             "serial/boot_sclk_align-dmgABCmgb", Model::DMGABC, false,

    timer_div_write:                    "timer/div_write",                  Model::DMGABC, false,
    timer_rapid_toggle:                 "timer/rapid_toggle",               Model::DMGABC, false,
    timer_tim00:                        "timer/tim00",                      Model::DMGABC, false,
    timer_tim00_div_trigger:            "timer/tim00_div_trigger",          Model::DMGABC, false,
    timer_tim01:                        "timer/tim01",                      Model::DMGABC, false,
    timer_tim01_div_trigger:            "timer/tim01_div_trigger",          Model::DMGABC, false,
    timer_tim10:                        "timer/tim10",                      Model::DMGABC, false,
    timer_tim10_div_trigger:            "timer/tim10_div_trigger",          Model::DMGABC, false,
    timer_tim11:                        "timer/tim11",                      Model::DMGABC, false,
    timer_tim11_div_trigger:            "timer/tim11_div_trigger",          Model::DMGABC, false,
    timer_tima_reload:                  "timer/tima_reload",                Model::DMGABC, false,
    timer_tima_write_reloading:         "timer/tima_write_reloading",       Model::DMGABC, false,
    timer_tma_write_reloading:          "timer/tma_write_reloading",        Model::DMGABC, false,

    add_sp_e_timing:                    "add_sp_e_timing",                  Model::DMGABC, false,
    boot_div_dmg0:                      "boot_div-dmg0",                    Model::DMG0, false,
    boot_div_dmg0_realbootrom:          "boot_div-dmg0",                    Model::DMG0, true,
    boot_div_dmg:                       "boot_div-dmgABCmgb",               Model::DMGABC, false,
    boot_div_dmg_realbootrom:           "boot_div-dmgABCmgb",               Model::DMGABC, true,
    boot_div_mgb:                       "boot_div-dmgABCmgb",               Model::MGB, false,
    boot_div_mgb_realbootrom:           "boot_div-dmgABCmgb",               Model::MGB, true,
    boot_hwio_dmg0:                     "boot_hwio-dmg0",                   Model::DMG0, false,
    boot_hwio_dmg0_realbootrom:         "boot_hwio-dmg0",                   Model::DMG0, true,
    boot_hwio_dmg:                      "boot_hwio-dmgABCmgb",              Model::DMGABC, false,
    boot_hwio_mgb:                      "boot_hwio-dmgABCmgb",              Model::MGB, false,
    boot_hwio_dmg_realbootrom:          "boot_hwio-dmgABCmgb",              Model::DMGABC, true,
    boot_hwio_mgb_realbootrom:          "boot_hwio-dmgABCmgb",              Model::MGB, true,
    boot_regs_dmg0:                     "boot_regs-dmg0",                   Model::DMG0, false,
    boot_regs_dmg0_realbootrom:         "boot_regs-dmg0",                   Model::DMG0, true,
    boot_regs_dmg_abc:                  "boot_regs-dmgABC",                 Model::DMGABC, false,
    boot_regs_dmg_abc_realbootrom:      "boot_regs-dmgABC",                 Model::DMGABC, true,
    boot_regs_mgb:                      "boot_regs-mgb",                    Model::MGB, false,
    boot_regs_mgb_realbootrom:          "boot_regs-mgb",                    Model::MGB, true,
    call_cc_timing:                     "call_cc_timing",                   Model::DMGABC, false,
    call_cc_timing2:                    "call_cc_timing2",                  Model::DMGABC, false,
    call_timing:                        "call_timing",                      Model::DMGABC, false,
    call_timing2:                       "call_timing2",                     Model::DMGABC, false,
    di_timing:                          "di_timing-GS",                     Model::DMGABC, false,
    div_timing:                         "div_timing",                       Model::DMGABC, false,
    ei_sequence:                        "ei_sequence",                      Model::DMGABC, false,
    ei_timing:                          "ei_timing",                        Model::DMGABC, false,
    halt_ime0_ei:                       "halt_ime0_ei",                     Model::DMGABC, false,
    halt_ime0_nointr_timing:            "halt_ime0_nointr_timing",          Model::DMGABC, false,
    halt_ime1_timing:                   "halt_ime1_timing",                 Model::DMGABC, false,
    halt_ime1_timing2:                  "halt_ime1_timing2-GS",             Model::DMGABC, false,
    if_ie_registers:                    "if_ie_registers",                  Model::DMGABC, false,
    intr_timing:                        "intr_timing",                      Model::DMGABC, false,
    jp_cc_timing:                       "jp_cc_timing",                     Model::DMGABC, false,
    jp_timing:                          "jp_timing",                        Model::DMGABC, false,
    ld_hl_sp_e_timing:                  "ld_hl_sp_e_timing",                Model::DMGABC, false,
    oam_dma_restart:                    "oam_dma_restart",                  Model::DMGABC, false,
    oam_dma_start:                      "oam_dma_start",                    Model::DMGABC, false,
    oam_dma_timing:                     "oam_dma_timing",                   Model::DMGABC, false,
    pop_timing:                         "pop_timing",                       Model::DMGABC, false,
    push_timing:                        "push_timing",                      Model::DMGABC, false,
    rapid_di_ei:                        "rapid_di_ei",                      Model::DMGABC, false,
    ret_cc_timing:                      "ret_cc_timing",                    Model::DMGABC, false,
    ret_timing:                         "ret_timing",                       Model::DMGABC, false,
    reti_intr_timing:                   "reti_intr_timing",                 Model::DMGABC, false,
    reti_timing:                        "reti_timing",                      Model::DMGABC, false,
    rst_timing:                         "rst_timing",                       Model::DMGABC, false,
}

mbc1_test_cases! {
    bits_ram_en:                        "bits_ram_en",                      Model::DMGABC, false,
    ram_64_kb:                          "ram_64Kb",                         Model::DMGABC, false,
    ram_256_kb:                         "ram_256Kb",                        Model::DMGABC, false,

    rom_512_kb:                         "rom_512Kb",                        Model::DMGABC, false,
    rom_1_mb:                           "rom_1Mb",                          Model::DMGABC, false,
    rom_2_mb:                           "rom_2Mb",                          Model::DMGABC, false,
    rom_4_mb:                           "rom_4Mb",                          Model::DMGABC, false,
    rom_8_mb:                           "rom_8Mb",                          Model::DMGABC, false,
    rom_16_mb:                          "rom_16Mb",                         Model::DMGABC, false,
}

#[test]
fn mooneye_sprite_priority() {
    let rom: &[u8] = include_bytes!("mooneye/manual-only/sprite_priority.gb");
    let rom = Rom::new(rom.into()).unwrap();
    let mut gb = Gameboy::new(Model::DMGABC, &rom, false);
    let mut gb_ctx = Context::new(rom);

    let start = Instant::now();
    loop {
        let (cpu, bus) = gb.bus(&mut gb_ctx);
        if bus.memory_get(cpu.pc) == 0x40 {
            break;
        }

        gb.run_instruction(&mut gb_ctx);

        if start.elapsed().as_secs() > 10 {
            panic!("Test ran for more than 10 seconds");
        }
    }
    // The magic breakpoint for this test is fired immediately after LCD is enabled, so let a full frame get rendered
    // before we run the comparison.
    for _ in 0..17556 {
        gb.run_instruction(&mut gb_ctx);
    }

    common::compare_framebuffer(
        &gb_ctx.current_framebuffer,
        include_bytes!("mooneye/manual-only/sprite_priority-expected.png"),
        |col| match col {
            0xFFFFFFFF => 0xE7DA,
            0xFF6F6F6F => 0x8E0E,
            0xFF000000 => 0x08C4,
            _ => panic!("Unexpected reference pixel color: {:X}", col),
        },
    );
}
