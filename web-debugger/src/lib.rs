extern crate cfg_if;
extern crate wasm_bindgen;

mod utils;

use cfg_if::cfg_if;
use js_sys::Uint8ClampedArray;
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

struct DebugListener {
    frame_cb: js_sys::Function,
}

impl EventListener for DebugListener {
    fn on_frame(&mut self, ppu_buf: &[u32]) {
        // We pass the PPU buffer to the callback in JS land to update the canvas.
        let buf =
            unsafe { Uint8ClampedArray::view(slice::from_raw_parts((ppu_buf).as_ptr() as *const u8, 160 * 144 * 4)) };

        let _ = self.frame_cb.call1(&JsValue::NULL, &buf);
    }
    fn on_memory_write(&mut self, _addr: u16, _: u8) {}
    fn on_debug_breakpoint(&mut self) {}
}

#[wasm_bindgen]
pub struct WebEmu {
    gb: Gameboy<DebugListener>,
}

#[wasm_bindgen]
impl WebEmu {
    pub fn new(rom: &[u8], frame_cb: js_sys::Function) -> WebEmu {
        utils::set_panic_hook();

        let listener = DebugListener { frame_cb };

        let mut gb = Gameboy::new(Model::DMG0, rom.to_vec(), listener);
        gb.skip_bootrom();
        gb.hw.ppu.framebuffer_fmt = ppu::PixelFormat::ABGR;
        WebEmu { gb }
    }

    pub fn run(&mut self, microseconds: f32) {
        self.gb.run_for_microseconds(microseconds);
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        self.gb.hw.mem_get(addr)
    }

    pub fn reg_read(&self, reg: &str) -> u16 {
        match reg {
            "AF" => self.gb.cpu.register16_get(cpu::Register16::AF),
            "BC" => self.gb.cpu.register16_get(cpu::Register16::BC),
            "DE" => self.gb.cpu.register16_get(cpu::Register16::DE),
            "HL" => self.gb.cpu.register16_get(cpu::Register16::HL),
            "PC" => self.gb.cpu.pc,
            "SP" => self.gb.cpu.sp,
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
