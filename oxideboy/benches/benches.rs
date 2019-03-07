#[macro_use]
extern crate bencher;

use bencher::Bencher;
use oxideboy::{interrupt, ppu};

fn ppu_drawline(bench: &mut Bencher) {
    let mut ppu = ppu::Ppu::new();

    ppu.dirty = true;

    bench.iter(|| {
        ppu.draw_line();
    });
}

fn ppu_drawline_sprites(bench: &mut Bencher) {
    let mut ppu = ppu::Ppu::new();
    let mut interrupts = interrupt::InterruptController::new();

    ppu.dirty = true;

    ppu.obj_enabled = true;

    ppu.oam[0].x = 10;
    ppu.oam[0].y = 10;

    ppu.mode_cycles = 20;
    ppu.mode_2_oam_search(&mut interrupts);

    bench.iter(|| {
        ppu.draw_line();
    });
}

benchmark_group!(benches, ppu_drawline, ppu_drawline_sprites);
benchmark_main!(benches);
