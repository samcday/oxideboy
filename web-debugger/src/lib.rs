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

        let mut rom: Vec<u8> = vec![0; data.byte_length() as usize];
        data.copy_to(&mut rom);

        let mut gb = Gameboy::new(Model::DMG0, rom);
        gb.skip_bootrom();
        gb.hw.ppu.framebuffer_fmt = ppu::PixelFormat::ABGR;
        WebEmu { gb }
    }

    pub fn run_frame(&mut self, microseconds: f32, framebuffer: Uint8ClampedArray) -> bool {
        let mut new_frame = false;

        let frame_cb = |ppu_buf: &[u32]| {
            new_frame = true;
            // Copy the PPU framebuffer to the JS Canvas framebuffer.
            let buf = unsafe {
                Uint8ClampedArray::view(slice::from_raw_parts((ppu_buf).as_ptr() as *const u8, 160 * 144 * 4))
            };
            framebuffer.set(&buf, 0);
        };

        self.gb.run_for_microseconds(microseconds, frame_cb);

        new_frame
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        self.gb.hw.mem_get(addr)
    }
}
