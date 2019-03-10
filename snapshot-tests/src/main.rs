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

    gb.save_state(&mut new_state);

    let mut diff = vec![];

    let mut compress_out = [0; 200000];
    let mut encoder = snap::Encoder::new();

    let mut seconds = 0;
    let mut total_size = 0;

    loop {
        for _ in 0..120 {
            while !gb.new_frame {
                gb.run_instruction();
            }
            gb.run_instruction();
        }
        seconds += 2;

        std::mem::swap(&mut base_state, &mut new_state);
        new_state.clear();
        gb.save_state(&mut new_state);

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

        diff.clear();
        let diff_size = oxideboy::simple_diff::generate(&base_state, &new_state, &mut diff).unwrap();
        let diff_compressed_size = encoder.compress(&diff, &mut compress_out).unwrap();
        total_size += diff_compressed_size;
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

        println!(
            "State size={} Diff size={} snap={} Compression={}% (base state compressed: {})",
            base_state.len(),
            diff_size,
            diff_compressed_size,
            (1.0 - ((diff_compressed_size as f32) / (base_state.len() as f32))) * 100.0,
            encoder.compress(&new_state, &mut compress_out).unwrap(),
        );
    }
}
