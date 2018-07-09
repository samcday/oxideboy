#![feature(test)]

extern crate test;
extern crate gameboy;

use test::Bencher;

#[bench]
fn bench_oam_search(b: &mut Bencher) {
    let mut ppu = gameboy::ppu::PPU::new();
    ppu.enabled = true;
    ppu.obj_enabled = true;

    b.iter(|| {
        ppu.ly = 66;
        ppu.oam_search();
    });
}

#[bench]
fn bench_scanline(b: &mut Bencher) {
    let mut ppu = gameboy::ppu::PPU::new();
    ppu.enabled = true;
    ppu.obj_enabled = true;

    b.iter(|| {
        ppu.ly = 66;
        while ppu.ly == 66 {
            ppu.advance();
        }
    });
}
