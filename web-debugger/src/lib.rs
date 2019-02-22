extern crate cfg_if;
extern crate wasm_bindgen;

mod utils;

use cfg_if::cfg_if;
use js_sys::{Uint8Array, Uint8ClampedArray};
use oxideboy::*;
use std::slice;
use wasm_bindgen::prelude::*;

macro_rules! log {
    ( $( $t:tt )* ) => {
        web_sys::console::log_1(&format!( $( $t )* ).into());
    }
}

cfg_if! {
    // When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
    // allocator.
    if #[cfg(feature = "wee_alloc")] {
        extern crate wee_alloc;
        #[global_allocator]
        static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;
    }
}

#[wasm_bindgen]
pub struct WebEmu {
    gb: Gameboy,
}

#[wasm_bindgen]
impl WebEmu {
    pub fn new(data: Uint8Array) -> WebEmu {
        utils::set_panic_hook();

        log!("Copying ROM.... {}", data.byte_length());
        // let mut rom: Vec<u8> = Vec::with_capacity(data.byte_length() as usize);
        let mut rom: Vec<u8> = vec![0; data.byte_length() as usize];
        data.copy_to(&mut rom);

        log!("Creating emu....");
        let mut gb = Gameboy::new(Model::DMG0, rom);
        gb.hw.ppu.framebuffer_fmt = ppu::PixelFormat::ABGR;
        WebEmu { gb }
    }

    pub fn run_frame(&mut self) -> Uint8ClampedArray {
        for _ in 0..17556 {
            self.gb.run_instruction();
        }

        unsafe {
            Uint8ClampedArray::view(slice::from_raw_parts_mut(
                (&mut self.gb.hw.ppu.framebuffer).as_ptr() as *mut u8,
                160 * 144 * 4,
            ))
        }
    }
}
