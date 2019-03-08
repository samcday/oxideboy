#[macro_use]
extern crate bencher;

use bencher::Bencher;
use bsdiff;
use flate2;
use libc::c_char;
use lz4::liblz4::*;
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

fn state_diff(bench: &mut Bencher) {
    let rom = include_bytes!("../tests/blargg/mem_timing.gb").to_vec();
    let mut gb = Gameboy::new(Model::DMG0, rom);
    gb.skip_bootrom();
    while !gb.new_frame {
        gb.run_instruction();
    }
    gb.run_instruction();

    let mut state1 = Vec::new();
    gb.save_state(&mut state1);

    while !gb.new_frame {
        gb.run_instruction();
    }
    gb.run_instruction();

    let mut state2 = Vec::new();
    gb.save_state(&mut state2);

    let mut diff = Vec::new();
    bsdiff::diff::diff(&state1, &state2, &mut diff).unwrap();

    bench.iter(|| {
        diff.clear();
        bsdiff::diff::diff(&state1, &state2, &mut diff).unwrap();
    });
}

fn state_flate(bench: &mut Bencher) {
    let rom = include_bytes!("../tests/blargg/mem_timing.gb").to_vec();
    let mut gb = Gameboy::new(Model::DMG0, rom);
    gb.skip_bootrom();
    while !gb.new_frame {
        gb.run_instruction();
    }
    gb.run_instruction();

    let mut state = Vec::new();
    gb.save_state(&mut state);

    let mut output = [0; 100000];

    let mut compressor = flate2::Compress::new(flate2::Compression::fast(), true);
    compressor
        .compress(&state, &mut output, flate2::FlushCompress::Finish)
        .unwrap();
    let size = output.len();
    compressor.reset();

    let mut decompressor = flate2::Decompress::new(true);
    let mut foo = [0; 100000];
    decompressor
        .decompress(&output[0..size], &mut foo, flate2::FlushDecompress::Finish)
        .unwrap();

    bench.iter(|| {
        compressor
            .compress(&state, &mut output, flate2::FlushCompress::Full)
            .unwrap();
        compressor.reset();
        assert_eq!(output.len(), size);
    });
}

fn state_lz4(bench: &mut Bencher) {
    let rom = include_bytes!("../tests/blargg/mem_timing.gb").to_vec();
    let mut gb = Gameboy::new(Model::DMG0, rom);
    gb.skip_bootrom();
    while !gb.new_frame {
        gb.run_instruction();
    }
    gb.run_instruction();

    let mut state = Vec::new();
    gb.save_state(&mut state);

    let mut output = [0; 100000];
    let compress_bound: i32 = unsafe { LZ4_compressBound(state.len() as i32) };
    let dec_size = unsafe {
        LZ4_compress_default(
            state.as_ptr() as *const c_char,
            output.as_mut_ptr() as *mut c_char,
            state.len() as i32,
            compress_bound,
        )
    };

    bench.iter(|| {
        let new_dec_size = unsafe {
            LZ4_compress_default(
                state.as_ptr() as *const c_char,
                output.as_mut_ptr() as *mut c_char,
                state.len() as i32,
                compress_bound,
            )
        };
        assert_eq!(new_dec_size, dec_size);
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

fn state_delta(bench: &mut Bencher) {
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

    for _ in 0..60 {
        while !gb.new_frame {
            gb.run_instruction();
        }
        gb.run_instruction();
    }

    let mut state2 = Vec::new();
    gb.save_state(&mut state2);

    assert_eq!(state.len(), state2.len());

    bench.iter(|| {
        let mut matching = 0;
        for i in 0..state.len() {
            if state[i] == state2[i] {
                matching += 1;
            }
        }

        assert_eq!(matching, 49427);
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
    state_diff,
    state_flate,
    state_snap,
    state_lz4,
    state_delta,
    ppu_drawline,
    ppu_drawline_sprites
);
benchmark_main!(benches);
