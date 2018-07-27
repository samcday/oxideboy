extern crate gameboy;

mod common;

use gameboy::*;
use std::time::Instant;

fn run_mattcurrie_test(rom: &[u8], expected: &[u8]) {
    let mut gameboy = GameboyContext::new(Model::DMG, rom.to_vec());
    gameboy.skip_bootrom();

    let start = Instant::now();
    while !gameboy.mooneye_breakpoint {
        gameboy::cpu::run(&mut gameboy);

        if start.elapsed().as_secs() > 10 {
            panic!("Test ran for more than 10 seconds");
        }
    }


    common::compare_framebuffer(gameboy.state.ppu.framebuffer(),
                                expected,
                                |col| {
                                    match col {
                                        0xFFFFFFFF => 0xFFE0F8D0,
                                        0xFFAAAAAA => 0xFF88C070,
                                        0xFF555555 => 0xFF346856,
                                        0xFF000000 => 0xFF081820,
                                        _ => panic!("Unexpected reference pixel color: {:X}", col)
                                    }
                                });
}

#[test] fn mattcurrie_m3_bgp_change() { run_mattcurrie_test(include_bytes!("mattcurrie/build/m3_bgp_change.gb"), include_bytes!("mattcurrie/expected/DMG-blob/m3_bgp_change.png")); }
#[test] fn mattcurrie_m3_bgp_change_sprites() { run_mattcurrie_test(include_bytes!("mattcurrie/build/m3_bgp_change_sprites.gb"), include_bytes!("mattcurrie/expected/DMG-blob/m3_bgp_change_sprites.png")); }
