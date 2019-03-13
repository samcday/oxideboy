#[macro_use]
extern crate bencher;

use bencher::Bencher;
use lazy_static::lazy_static;
use oxideboy::rom::Rom;
use oxideboy::{interrupt, ppu, simple_diff, Context, Gameboy, Model};
use snap;

fn save_state(bench: &mut Bencher) {
    let rom = Rom::new(include_bytes!("../tests/blargg/mem_timing.gb").to_vec().into()).unwrap();
    let mut gb = Gameboy::new(Model::DMG0, &rom, false);
    let mut gb_ctx = Context::new(rom);
    for _ in 0..50000 {
        gb.run_instruction(&mut gb_ctx);
    }

    let mut vec = Vec::new();
    oxideboy::save_state(&gb, &gb_ctx, &mut vec).unwrap();

    let size = vec.len();

    bench.iter(|| {
        vec.clear();
        oxideboy::save_state(&gb, &gb_ctx, &mut vec).unwrap();
        assert_eq!(vec.len(), size);
    });
}

fn load_state(bench: &mut Bencher) {
    let rom = Rom::new(include_bytes!("../tests/blargg/mem_timing.gb").to_vec().into()).unwrap();
    let mut gb = Gameboy::new(Model::DMG0, &rom, false);
    let mut gb_ctx = Context::new(rom);
    for _ in 0..50000 {
        gb.run_instruction(&mut gb_ctx);
    }

    let mut vec = Vec::new();
    oxideboy::save_state(&gb, &gb_ctx, &mut vec).unwrap();

    gb.run_instruction(&mut gb_ctx);
    let cycle_count = gb.cycle_count;

    bench.iter(|| {
        oxideboy::load_state(&mut gb, &mut gb_ctx, &vec[..]).unwrap();
        assert!(gb.cycle_count < cycle_count);
    });
}

fn state_snap(bench: &mut Bencher) {
    let rom = Rom::new(include_bytes!("../tests/blargg/mem_timing.gb").to_vec().into()).unwrap();
    let mut gb = Gameboy::new(Model::DMG0, &rom, false);
    let mut gb_ctx = Context::new(rom);
    gb_ctx.enable_audio = false;
    while gb.frame_count < 60 {
        gb.run_instruction(&mut gb_ctx);
    }

    let mut state = Vec::new();
    bincode::serialize_into(&mut state, &gb).unwrap();

    let mut output = [0; 100000];
    let mut encoder = snap::Encoder::new();

    let size = encoder.compress(&state, &mut output).unwrap();

    bench.iter(|| {
        let new_size = encoder.compress(&state, &mut output).unwrap();
        assert_eq!(new_size, size);
    });
}

lazy_static! {
    static ref DIFF_STATES: (Vec<u8>, Vec<u8>) = {
        let rom = Rom::new(include_bytes!("../tests/blargg/mem_timing.gb").to_vec().into()).unwrap();
        let mut gb = Gameboy::new(Model::DMG0, &rom, false);
        let mut gb_ctx = Context::new(rom);

        let mut base_state = Vec::new();
        let mut new_state = Vec::new();

        while gb.frame_count < 60 {
            gb.run_instruction(&mut gb_ctx);
        }
        bincode::serialize_into(&mut base_state, &gb).unwrap();

        while gb.frame_count < 120 {
            gb.run_instruction(&mut gb_ctx);
        }
        bincode::serialize_into(&mut new_state, &gb).unwrap();

        (base_state, new_state)
    };
}

fn diff_small(bench: &mut Bencher) {
    let base_state = &DIFF_STATES.0;
    let new_state = &DIFF_STATES.1;

    let mut diff_out = vec![];
    simple_diff::generate(&base_state, &new_state, &mut diff_out).unwrap();
    let diff_size = diff_out.len();

    bench.iter(|| {
        diff_out.clear();
        simple_diff::generate(&base_state, &new_state, &mut diff_out).unwrap();
        assert_eq!(diff_out.len(), diff_size);
    });
}

fn diff_small_apply(bench: &mut Bencher) {
    let base_state = &DIFF_STATES.0;
    let new_state = &DIFF_STATES.1;

    let mut diff_out = vec![];
    simple_diff::generate(&base_state, &new_state, &mut diff_out).unwrap();

    let mut rebuilt = base_state.clone();

    bench.iter(|| {
        simple_diff::apply(&mut rebuilt, &diff_out);
    });
}

fn ppu_drawline(bench: &mut Bencher) {
    let mut ppu = ppu::Ppu::new();
    let mut framebuffer = [0; ppu::SCREEN_SIZE];

    ppu.dirty = true;

    bench.iter(|| {
        ppu.draw_line(&mut framebuffer);
    });
}

fn ppu_drawline_sprites(bench: &mut Bencher) {
    let mut ppu = ppu::Ppu::new();
    let mut framebuffer = [0; ppu::SCREEN_SIZE];
    let mut interrupts = interrupt::InterruptController::new();

    ppu.dirty = true;

    ppu.obj_enabled = true;

    ppu.oam[0].x = 10;
    ppu.oam[0].y = 10;

    ppu.mode_cycles = 20;
    ppu.mode_2_oam_search(&mut interrupts);

    bench.iter(|| {
        ppu.draw_line(&mut framebuffer);
    });
}

benchmark_group!(
    benches,
    save_state,
    load_state,
    state_snap,
    diff_small,
    diff_small_apply,
    ppu_drawline,
    ppu_drawline_sprites
);
benchmark_main!(benches);
