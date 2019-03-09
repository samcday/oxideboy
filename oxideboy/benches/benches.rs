#[macro_use]
extern crate bencher;

use bencher::Bencher;
use oxideboy::{interrupt, ppu, Gameboy, Model};
use snap;

fn save_state(bench: &mut Bencher) {
    let rom = include_bytes!("../tests/blargg/mem_timing.gb").to_vec();
    let mut gb = Gameboy::new(Model::DMG0, rom);
    gb.skip_bootrom();
    for _ in 0..50000 {
        gb.run_instruction();
    }

    let mut vec = Vec::new();
    gb.save_state(&mut vec);

    let size = vec.len();

    bench.iter(|| {
        vec.clear();
        gb.save_state(&mut vec);
        assert_eq!(vec.len(), size);
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

fn state_snap(bench: &mut Bencher) {
    let rom = include_bytes!("/home/sam/Downloads/pokemonblue.gb").to_vec();
    let mut gb = Gameboy::new(Model::DMG, rom);
    gb.skip_bootrom();
    for _ in 0..60 {
        while !gb.new_frame {
            gb.run_instruction();
        }
        gb.run_instruction();
    }

    let mut state = Vec::new();
    gb.save_state(&mut state);

    let mut output = [0; 100000];
    let mut encoder = snap::Encoder::new();

    let size = encoder.compress(&state, &mut output).unwrap();

    bench.iter(|| {
        let new_size = encoder.compress(&state, &mut output).unwrap();
        assert_eq!(new_size, size);
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

benchmark_group!(
    benches,
    save_state,
    load_state,
    state_snap,
    ppu_drawline,
    ppu_drawline_sprites
);
benchmark_main!(benches);
