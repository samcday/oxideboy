extern crate gameboy;
extern crate image;

use gameboy::*;
use std::time::Instant;

fn run_mattcurrie_test(rom: &[u8], expected: &[u8]) {
    let mut gameboy = GameboyContext::new(rom.to_vec());
    gameboy.skip_bootrom();

    let expected_img = image::load_from_memory(expected).unwrap();
    let expected_img = expected_img.to_rgb();

    let start = Instant::now();
    while !gameboy.mooneye_breakpoint {
        gameboy::cpu::run(&mut gameboy);

        if start.elapsed().as_secs() > 10 {
            panic!("Test ran for more than 10 seconds");
        }
    }

    let framebuf = gameboy.state.ppu.framebuffer();
    for y in 0..144 {
        for x in 0..160 {
            let expected_rgb = expected_img.get_pixel(x as u32, y as u32);
            let expected_rgb = 0xFF000000 | ((expected_rgb[0] as u32) << 16) | ((expected_rgb[1] as u32) << 8) | (expected_rgb[2] as u32);

            // Our palette differs from reference images, so we convert the colours here.
            let expected_rgb = match expected_rgb {
                0xFFFFFFFF => 0xFFE0F8D0,
                0xFFAAAAAA => 0xFF88C070,
                0xFF555555 => 0xFF346856,
                0xFF000000 => 0xFF081820,
                _ => panic!("Unexpected reference pixel color: {:X}", expected_rgb)
            };

            if framebuf[(y * 160) + x] != expected_rgb {
                panic!("Pixel {},{} does not match. Expected {:X}, got {:X}", x, y, expected_rgb, framebuf[(y * 160) + x]);
            }
        }
    }
}

#[test] fn mattcurrie_m3_bgp_change() { run_mattcurrie_test(include_bytes!("mattcurrie/build/m3_bgp_change.gb"), include_bytes!("mattcurrie/expected/DMG-blob/m3_bgp_change.png")); }
