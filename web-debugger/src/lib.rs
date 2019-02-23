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

    pub fn reg_read(&self, reg: &str) -> u16 {
        match reg {
            "af" => self.gb.cpu.register16_get(cpu::Register16::AF),
            "bc" => self.gb.cpu.register16_get(cpu::Register16::BC),
            "de" => self.gb.cpu.register16_get(cpu::Register16::DE),
            "hl" => self.gb.cpu.register16_get(cpu::Register16::HL),
            "pc" => self.gb.cpu.pc,
            "sp" => self.gb.cpu.sp,
            v => panic!("unknown reg_read: {}", v),
        }
    }

    pub fn get_ime(&self) -> bool {
        self.gb.cpu.ime
    }

    pub fn get_ime_defer(&self) -> bool {
        self.gb.cpu.ime_defer
    }

    pub fn set_joypad_state(&mut self, key: &str, pressed: bool) {
        match key {
            "ArrowUp" | "Up" => self.gb.hw.joypad.up = pressed,
            "ArrowDown" | "Down" => self.gb.hw.joypad.down = pressed,
            "ArrowLeft" | "Left" => self.gb.hw.joypad.left = pressed,
            "ArrowRight" | "Right" => self.gb.hw.joypad.right = pressed,
            "Enter" => self.gb.hw.joypad.start = pressed,
            "Shift" => self.gb.hw.joypad.select = pressed,
            "a" => self.gb.hw.joypad.a = pressed,
            "s" => self.gb.hw.joypad.b = pressed,
            _ => {}
        }
    }
}
