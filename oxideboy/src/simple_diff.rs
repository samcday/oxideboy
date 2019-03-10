//! The rewind functionality depends on taking regular snapshots of the emulator state. Taking snapshots of the emulator
//! is quite cheap in terms of CPU overhead, but not so cheap in terms of storage. Each snapshot is at least 40kb, and
//! could be much more depending on SRAM size, number of memory banks in use, etc. As it happens though, once we start
//! an emulation session, the save state will always be exactly the same size. Furthermore, while many small segments of
//! the snapshots will change (IO/CPU registers, OAM table, etc), there are giant chunks of it (RAM/HRAM/VRAM segments)
//! that change much less frequently.
//! So what we can do is compute the differential between the last snapshot and the new one, and store only the parts
//! that differ. Because most Gameboy games aren't rewriting all of their memory / VRAM every frame, this delta approach
//! achieves compression ratios of 90%+.

//! So how does this "simple_diff" actually work? We walk over the previous state and the new one, DIFF_WINDOW bytes at
//! a time. If all the bytes match, great, we move on. If not, we note the location where bytes started differing and
//! keep walking the data. Once we find another window (of DIFF_WINDOW) bytes that are identical, we write a "chunk"
//! starting from the first location where data started differing, up to where it stopped differing.

//! The format of the diff blob is simple. It begins with a u32 counting how many chunks are contained in the diff. Then
//! the chunks follow. Each chunk starts with a header containing first a u32 denoting the offset where the chunk should
//! be applied to, followed by a u32 indicating the length of the chunk.

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use std::io::prelude::*;

/// How big is the window of data we compare when building the diff? Setting this number higher makes the diff run
/// faster, but the compression ratio declines (since smaller patches of data that match will be overlooked). Setting
/// the number lower improves the compression ratio, but makes the diff run slower.
/// For now I've picked 64. Why 64? Well going from 64 to 32 makes the diff run almost exactly twice as slow, but the
/// compression is only ~15% better on something like Super Mario World. However, going from 64 to 128 does *not* cause
/// the diff to run twice as fast. So above 64 there seems to be diminishing returns on speed increase, while
/// compression ratio continues to suffer the higher the window gets.
pub const DIFF_WINDOW: usize = 64;

/// Generates a diff between the base data and the new data. Diff is written to provided Write sink.
pub fn generate<W: Write>(base: &[u8], new: &[u8], mut w: W) -> std::io::Result<()> {
    // This whole deal only works if our two blobs are the same size.
    assert_eq!(base.len(), new.len());

    let mut chunk_start: usize = 0;
    let mut chunk_end: usize = 0;
    let mut in_chunk = false;

    let mut chunks = base
        .chunks(DIFF_WINDOW)
        .zip(new.chunks(DIFF_WINDOW))
        .enumerate()
        .filter_map(|(i, (l, r))| {
            // This comparison is fast because https://github.com/rust-lang/rust/pull/32699/files
            // Rust specializes u8 slice comparisons with a memcmp call. Neat.
            if l != r {
                if !in_chunk {
                    in_chunk = true;
                    chunk_start = i;
                }
                chunk_end = i;
            } else if in_chunk {
                in_chunk = false;
                return Some((chunk_start * DIFF_WINDOW, (chunk_end - chunk_start + 1) * DIFF_WINDOW));
            }
            None
        })
        .collect::<Vec<(usize, usize)>>();

    // Flush the remaining chunk if there is one.
    if in_chunk {
        chunks.push((chunk_start * DIFF_WINDOW, base.len() - (chunk_start * DIFF_WINDOW)));
    }

    w.write_u32::<LittleEndian>(chunks.len() as u32)?;

    for (chunk_offset, chunk_len) in chunks {
        w.write_u32::<LittleEndian>(chunk_offset as u32)?;
        w.write_u32::<LittleEndian>(chunk_len as u32)?;
        w.write(&new[chunk_offset..chunk_offset + chunk_len])?;
    }

    Ok(())
}

/// Applies a diff to the given base data. The base data is mutated in place.
pub fn apply(base: &mut [u8], mut diff: &[u8]) {
    let chunk_count = diff.read_u32::<LittleEndian>().unwrap();
    for _ in 0..chunk_count {
        let offset = diff.read_u32::<LittleEndian>().unwrap() as usize;
        let len = diff.read_u32::<LittleEndian>().unwrap() as usize;
        diff.read_exact(&mut base[offset..offset + len]).unwrap();
    }
}
