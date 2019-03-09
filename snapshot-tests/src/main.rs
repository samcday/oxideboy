#![allow(unused)]

use byteorder::{ByteOrder, LittleEndian, ReadBytesExt, WriteBytesExt};
use oxideboy::{Gameboy, Model};
use snap;
use std::fs::File;
use std::io::prelude::*;

type Result<T> = std::result::Result<T, Box<std::error::Error>>;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

    let mut gb = Gameboy::new(Model::DMG0, rom);
    gb.skip_bootrom();

    let mut base_state = Vec::new();
    let mut new_state = Vec::new();
    let mut diff = Vec::new();

    let mut compress_out = [0; 200000];
    let mut encoder = snap::Encoder::new();

    let mut seconds = 0;
    let mut total_size = 0;

    loop {
        for _ in 0..300 {
            while !gb.new_frame {
                gb.run_instruction();
            }
            gb.run_instruction();
        }
        seconds += 5;

        std::mem::swap(&mut base_state, &mut new_state);
        new_state.clear();
        gb.save_state(&mut new_state);

        if base_state.len() == 0 {
            let base_compressed_size = encoder.compress(&base_state, &mut compress_out).unwrap();
            total_size += base_compressed_size;
            // Very first state save, nothing to diff yet.
            continue;
        }

        // Every 60 seconds we save a base state.
        if seconds % 60 == 0 {
            total_size += encoder.compress(&base_state, &mut compress_out).unwrap();
        } else {
            diff_states(&base_state, &new_state, &mut diff);
            // Every 30 seconds we save the diffed states compressed.
            if seconds % 30 == 0 {
                let diff_compressed_size = encoder.compress(&diff, &mut compress_out).unwrap();
                diff.clear();
                total_size += diff_compressed_size;
            }
        }

        // diff.clear();
        // let diff_compressed_size = encoder.compress(&diff, &mut compress_out).unwrap();
        // total_size += diff_compressed_size;
        // apply_diff(&base_state, &new_state, &diff);

        if seconds % 60 == 0 {
            println!(
                "{} seconds. Total state size so far: {}. {}kb per minute, {}kb per hour.",
                seconds,
                total_size,
                (total_size as f32) / (seconds as f32 / 60.0) / 1000.0,
                (total_size as f32) / (seconds as f32 / 60.0) * 60.0 / 1000.0
            );
        }

        // println!(
        //     "State size={} Diff size={} snap={} Compression={}% (base state compressed: {})",
        //     base_state.len(),
        //     diff.len(),
        //     diff_compressed_size,
        //     (1.0 - ((diff_compressed_size as f32) / (base_state.len() as f32))) * 100.0,
        //     base_compressed_size,
        // );
    }
}

fn diff_states(base: &[u8], new: &[u8], diff: &mut Vec<u8>) {
    let mut chunk_start: usize = 0;
    let mut chunk_end: usize = 0;
    let mut in_chunk = false;
    let mut chunk_count = 0;

    // This will be the chunk count, we'll fill it in at the end.
    diff.write_u32::<LittleEndian>(0).unwrap();

    for (i, (l, r)) in base.iter().zip(new.iter()).enumerate() {
        if l != r {
            if !in_chunk {
                in_chunk = true;
                chunk_start = i;
            }
            chunk_end = i;
        }

        if l == r && in_chunk {
            if i - chunk_end > 16 {
                chunk_count += 1;
                diff.write_u32::<LittleEndian>(chunk_start as u32).unwrap();
                diff.write_u32::<LittleEndian>((chunk_end - chunk_start + 1) as u32)
                    .unwrap();
                diff.write(&new[chunk_start..chunk_end + 1]).unwrap();
                in_chunk = false;
            }
        }
    }

    if in_chunk {
        chunk_count += 1;
        diff.write_u32::<LittleEndian>(chunk_start as u32).unwrap();
        diff.write_u32::<LittleEndian>((chunk_end - chunk_start + 1) as u32)
            .unwrap();
        diff.write(&new[chunk_start..chunk_end + 1]).unwrap();
    }

    LittleEndian::write_u32(&mut diff[0..4], chunk_count);
}

fn apply_diff(base: &[u8], new: &[u8], mut diff: &[u8]) {
    let mut rebuilt = vec![0; base.len()];
    rebuilt.copy_from_slice(base);

    let chunk_count = diff.read_u32::<LittleEndian>().unwrap();
    for _ in 0..chunk_count {
        let offset = diff.read_u32::<LittleEndian>().unwrap() as usize;
        let len = diff.read_u32::<LittleEndian>().unwrap() as usize;
        diff.read_exact(&mut rebuilt[offset..offset + len]).unwrap();
    }

    for (i, (l, r)) in rebuilt.iter().zip(new.iter()).enumerate() {
        if l != r {
            panic!("Nope, index {} did not match. Expected={}, got {}", i, r, l);
        }
    }
}
