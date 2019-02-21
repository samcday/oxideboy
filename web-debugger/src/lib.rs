extern crate cfg_if;
extern crate wasm_bindgen;
extern crate gameboy;
extern crate js_sys;
extern crate web_sys;

mod utils;

use std::slice;
use gameboy::*;
use cfg_if::cfg_if;
use wasm_bindgen::prelude::*;
use js_sys::{Uint8Array, Uint8ClampedArray};

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
    cpu: cpu::Cpu<Gameboy>,
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
        let hw = Gameboy::new(Model::DMG0, rom);
        let cpu = cpu::Cpu::new(hw);
        WebEmu{cpu}
    }

    pub fn run_frame(&mut self) -> Uint8ClampedArray {
        for _  in 0..17556 {
            self.cpu.fetch_decode_execute();
        }

        unsafe { Uint8ClampedArray::view(slice::from_raw_parts_mut((&mut self.cpu.hw.ppu.framebuffer).as_ptr() as *mut u8, 160*144*4)) }
    }
}
