pub mod apu;
pub mod cartridge;
pub mod cpu;
pub mod dma;
pub mod dmg;
pub mod interrupt;
pub mod joypad;
pub mod ppu;
pub mod rom;
pub mod serial;
pub mod simple_diff;
pub mod timer;
pub mod util;

// use bincode;
// use serde::{Deserialize, Serialize};
use crate::rom::Rom;
use std::collections::VecDeque;

pub const CYCLES_PER_MICRO: f32 = 1_048_576.0 / 1_000_000.0;

/// Context holds external state that is related to, but not a core part of, the Gameboy struct. Rendered frames,
/// audio samples are kept in here for example. The distinction is made because of the nature of save/load states and
/// rewinding, and also for efficiency reasons. We don't want to serialize the framebuffer and audio samples in every
/// snapshot, that would be extremely wasteful. We also don't need to calculate audio samples or frames when we're
/// in the process of replaying a Gameboy to seek to a new rewind state.
pub struct Context {
    pub rom: Rom,
    framebuffers: [[u16; ppu::SCREEN_SIZE]; 2],
    current_framebuffer: usize,
    audio_samples: VecDeque<f32>,
}

impl Context {
    pub fn new(rom: Rom) -> Context {
        Context {
            rom,
            framebuffers: [[0; ppu::SCREEN_SIZE]; 2],
            current_framebuffer: 0,
            audio_samples: VecDeque::new(),
        }
    }

    /// Returns the framebuffer for the last fully written frame.
    pub fn current_framebuffer(&mut self) -> &mut [u16] {
        &mut self.framebuffers[self.current_framebuffer]
    }

    /// Returns the framebuffer to write the next frame into.
    pub fn next_framebuffer(&mut self) -> &mut [u16] {
        &mut self.framebuffers[self.current_framebuffer]
    }

    pub fn swap_framebuffers(&mut self) {
        self.current_framebuffer = self.current_framebuffer + 1 & 1;
    }

    pub fn drain_audio_samples<F: FnMut(&[f32])>(&mut self, mut f: F) {
        let (l, r) = self.audio_samples.as_slices();
        if l.len() > 0 {
            f(l);
        }
        if r.len() > 0 {
            f(r);
        }
        self.audio_samples.clear();
    }
}

// pub trait Gameboy {
//     pub fn memory_get();
// }

// impl Context {
// /// Run the Gameboy for a single CPU instruction. Useful for debuggers / tests.
// pub fn run_instruction<T: Gameboy>(&mut self, gb: T) {
// let context =  {};
// gb.cpu().step(&context);>
// }
// }
/*
impl Gameboy {
    /// Serializes the entire state of the emulator into the destination Vec<u8>. The emulator can be restored to the
    /// state with load_state().
    // pub fn save_state(&self, dst: &mut Vec<u8>) {
    //     bincode::serialize_into(dst, self).unwrap();
    // }
    // pub fn load_state(&mut self, state: &Vec<u8>) {
    //     // The deserialize call overwrites the whole struct with a new one from the given state. As a result the rom
    //     // field of the cartridge gets reset to an empty Vec. So we grab it out first and put it into the new struct
    //     // after the deserialize call.
    //     let rom = std::mem::replace(&mut self.cart.rom, vec![]);
    //     *self = bincode::deserialize(state).unwrap();
    //     std::mem::replace(&mut self.cart.rom, rom);
    // }

    /// Run the Gameboy for the specified number of microseconds.
    /// This entrypoint is useful for emulating the Gameboy in real-time, while adhering to a refresh rate or some other
    /// external timing control. For example, the web emulator uses requestAnimationFrame to drive emulation, which
    /// provides a microsecond-resolution timestamp that can be used to determine how many microseconds passed since the
    /// last emulation step.
    // TODO:
    // pub fn run_for_microseconds(&mut self, num_micros: f32) {
    //     let desired_cycles = (CYCLES_PER_MICRO * num_micros) as u64;

    //     while self.cycle_count < desired_cycles {
    //         self.cpu.step(&mut self);
    //     }
    // }
}
*/
