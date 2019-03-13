#![allow(unused)]

use byteorder::{ByteOrder, LittleEndian, ReadBytesExt, WriteBytesExt};
use oxideboy::rom::Rom;
use oxideboy::{Context, Gameboy, Model};
use snap;
use std::fs::File;
use std::io::prelude::*;
use zstd;

type Result<T> = std::result::Result<T, Box<std::error::Error>>;

static mut COMPRESS_OUT: [u8; 1000000] = [0; 1000000];

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

    let rom = Rom::new(rom.into()).unwrap();
    let mut gb = Gameboy::new(Model::DMG0, &rom, false);
    let mut gb_ctx = Context::new(rom);
    gb_ctx.enable_video = false;
    gb_ctx.enable_audio = false;

    let mut base_state = Vec::new();
    let mut new_state = Vec::new();

    let mut total_size = 0;
    let mut diff = vec![];
    let mut batch_diffs = vec![];
    let mut batch_size = 0;
    let mut next_frame_snapshot = 0;

    let base_snapshot_every = 3600;

    // Snappy
    fn compress(data: &[u8]) -> usize {
        let mut encoder = snap::Encoder::new();
        unsafe { encoder.compress(&data, &mut COMPRESS_OUT).unwrap() }
    }

    // Zstd
    // fn compress(data: &[u8]) -> usize {
    //     unsafe { zstd::block::compress_to_buffer(&data, &mut COMPRESS_OUT, 0).unwrap() }
    // }

    bincode::serialize_into(&mut new_state, &gb);
    total_size += compress(&new_state);

    loop {
        next_frame_snapshot += 60;
        while gb.frame_count < next_frame_snapshot {
            gb.run_instruction(&mut gb_ctx);
        }

        if next_frame_snapshot % 3600 == 0 {
            let seconds = (next_frame_snapshot / 60);
            println!(
                "{} seconds. Total state size so far: {}. {}kb per minute, {}kb per hour.",
                seconds,
                total_size,
                (total_size as f32) / (seconds as f32 / 60.0) / 1000.0,
                (total_size as f32) / (seconds as f32 / 60.0) * 60.0 / 1000.0
            );
        }

        std::mem::swap(&mut base_state, &mut new_state);
        new_state.clear();
        bincode::serialize_into(&mut new_state, &gb);

        if next_frame_snapshot % base_snapshot_every == 0 {
            total_size += compress(&new_state);
            continue;
        }

        // Every 60 seconds we save a base state.
        // if seconds % 60 == 0 {
        //     total_size += encoder.compress(&new_state, &mut compress_out).unwrap();
        // } else {
        //     diff_states(&base_state, &new_state, &mut diff);
        //     // Every 30 seconds we save the diffed states compressed.
        //     if seconds % 30 == 0 {
        //         let diff_compressed_size = encoder.compress(&diff, &mut compress_out).unwrap();
        //         diff.clear();
        //         total_size += diff_compressed_size;
        //     }
        // }

        let base_compressed_size = compress(&new_state);

        diff.clear();
        let diff_size = oxideboy::simple_diff::generate(&base_state, &new_state, &mut diff).unwrap();
        let diff_compressed_size = compress(&diff);

        batch_size += diff_compressed_size;
        batch_diffs.extend(&diff);

        if true {
            total_size += diff_compressed_size;
        } else {
            if next_frame_snapshot % 1000 == 0 {
                let compressed_size = compress(&batch_diffs);
                total_size += compressed_size;

                println!(
                    "last 20 diffs: {}. compressed individually: {}. compressed together: {}",
                    batch_diffs.len(),
                    batch_size,
                    compressed_size
                );
                batch_size = 0;
                batch_diffs.clear();
            }
        }

        // println!(
        //     "State size={} Diff size={} diff compressed={} Compression={}% (base state compressed: {})",
        //     base_state.len(),
        //     diff_size,
        //     diff_compressed_size,
        //     (1.0 - ((diff_compressed_size as f32) / (base_state.len() as f32))) * 100.0,
        //     base_compressed_size,
        // );

        // if base_compressed_size < diff_compressed_size {
        //     panic!("Diff ended up larger than base size, wtfux mang");
        // }
    }
}
