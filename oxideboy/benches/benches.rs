#[macro_use]
extern crate bencher;

use bencher::Bencher;
use oxideboy::{interrupt, ppu, Gameboy, Model};

fn save_state(bench: &mut Bencher) {
    let rom = include_bytes!("../tests/blargg/mem_timing.gb").to_vec();
    let mut gb = Gameboy::new(Model::DMG0, rom);
    gb.skip_bootrom();
    for _ in 0..50000 {
        gb.run_instruction();
    }

    let mut vec = Vec::new();
    gb.save_state(&mut vec);

    bench.iter(|| {
        vec.clear();
        gb.save_state(&mut vec);
        assert_eq!(vec.len(), 63242);
    });
}
fn load_state(bench: &mut Bencher) {
    let rom = include_bytes!("../tests/blargg/mem_timing.gb").to_vec();
    let mut gb = Gameboy::new(Model::DMG0, rom);
    gb.skip_bootrom();
    for _ in 0..50000 {
        gb.run_instruction();
    }

    let mut vec = Vec::new();
    gb.save_state(&mut vec);
    gb.run_instruction();
    let cycle_count = gb.cycle_count;

    bench.iter(|| {
        gb.load_state(&vec);
        assert!(gb.cycle_count < cycle_count);
    });
}

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

benchmark_group!(benches, save_state, load_state, ppu_drawline, ppu_drawline_sprites);
benchmark_main!(benches);
