use byteorder::{ByteOrder, LittleEndian, ReadBytesExt};
use oxideboy::simple_diff::{apply, generate, DIFF_WINDOW};
use std::io::prelude::*;

// Test that diff output is empty if base and new are the same.
#[test]
fn simple_diff_no_changes() {
    let base = vec![1, 2, 3];
    let new = vec![1, 2, 3];

    let mut diff = vec![];
    generate(&base, &new, &mut diff).unwrap();

    assert_eq!(diff.len(), 4);
    assert_eq!(LittleEndian::read_u32(&diff), 0);
}

// Test that the whole new data is returned if data has no chunks that match.
#[test]
fn simple_diff_whole_diff() {
    let base = vec![3, 2, 1];
    let new = vec![1, 2, 3];

    let mut diff = vec![];
    generate(&base, &new, &mut diff).unwrap();

    let mut rdr = &diff[..];
    // Should be 1 chunk.
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), 1);
    // Chunk should start at 0 and be 3 bytes long.
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), 0);
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), 3);
    // Chunk should have the data {1,2,3}
    let mut data = [0; 3];
    rdr.read_exact(&mut data).unwrap();
    assert_eq!(&data, &[1, 2, 3]);
}

// Test that the last chunk of data is included in diff.
#[test]
fn simple_diff_finish_last_chunk() {
    let mut base = vec![0; DIFF_WINDOW];
    let mut new = vec![0; DIFF_WINDOW];
    base.push(1);
    new.push(3);

    let mut diff = vec![];
    generate(&base, &new, &mut diff).unwrap();

    let mut rdr = &diff[..];
    // Should be 1 chunk.
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), 1);
    // Chunk should start at DIFF_WINDOW and be 1 byte long.
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), (DIFF_WINDOW) as u32);
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), 1);
    // Chunk should have the data {3}
    let mut data = [0; 1];
    rdr.read_exact(&mut data).unwrap();
    assert_eq!(&data, &[3]);
}

// Test that chunks are ended when a window of matching data is encountered.
#[test]
fn simple_diff_chunk_end() {
    let mut base = vec![0; DIFF_WINDOW * 3];
    let mut new = vec![0; DIFF_WINDOW * 3];

    // Make first and last window of data mismatch.
    base[0] = 1;
    new[0] = 3;

    base[DIFF_WINDOW * 3 - 1] = 2;
    new[DIFF_WINDOW * 3 - 1] = 4;

    let mut diff = vec![];
    generate(&base, &new, &mut diff).unwrap();

    let mut rdr = &diff[..];
    // Should be 2 chunks.
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), 2);
    // First chunk should start at 0 and be DIFF_WINDOW bytes long.
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), 0);
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), DIFF_WINDOW as u32);
    // First chunk should have the data {3,0...}
    let mut data = [0; DIFF_WINDOW];
    rdr.read_exact(&mut data).unwrap();
    assert_eq!(data[0], 3);

    // Second chunk should start at DIFF_WINDOW * 2 and be DIFF_WINDOW bytes long.
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), DIFF_WINDOW as u32 * 2);
    assert_eq!(rdr.read_u32::<LittleEndian>().unwrap(), DIFF_WINDOW as u32);
    // Second chunk should have the data {0..., 4}
    let mut data = [0; DIFF_WINDOW];
    rdr.read_exact(&mut data).unwrap();
    assert_eq!(data[DIFF_WINDOW - 1], 4);
}

// Test applying diff back to base.
#[test]
fn simple_diff_apply() {
    let mut base = vec![1, 2, 3];
    let new = vec![3, 2, 1];

    let mut diff = vec![];
    generate(&base, &new, &mut diff).unwrap();

    apply(&mut base, &diff[0..diff.len()]);

    assert_eq!(&base, &new);
}
