extern crate cfg_if;
extern crate wasm_bindgen;

mod utils;

use cfg_if::cfg_if;
use js_sys::{Function, Uint8ClampedArray};
use oxideboy::*;
use serde::Serialize;
use std::collections::HashSet;
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
    rom_title: String,
    rom_hash: String,
    breakpoint_cb: Function,
    frame_cb: Function,
    pc_breakpoints: HashSet<u16>,
}

#[derive(Serialize)]
struct Instruction {
    pub loc: u16,
    pub txt: String,
}

#[wasm_bindgen]
impl WebEmu {
    pub fn new(rom: &[u8], frame_cb: Function, breakpoint_cb: Function) -> WebEmu {
        utils::set_panic_hook();

        let mut gb = Gameboy::new(Model::DMG0, rom.to_vec());

        gb.skip_bootrom();
        gb.hw.ppu.set_pixel_format(ppu::PixelFormat::ABGR);

        let rom_title = String::from(gb.hw.cart.rom_title());
        let rom_hash = format!("{:x}", md5::compute(&gb.hw.cart.rom));

        WebEmu {
            gb,
            pc_breakpoints: HashSet::new(),
            breakpoint_cb,
            frame_cb,
            rom_hash,
            rom_title,
        }
    }

    pub fn rom_hash(&self) -> String {
        self.rom_hash.clone()
    }

    pub fn rom_title(&self) -> String {
        self.rom_title.clone()
    }

    pub fn set_breakpoints(&mut self, val: &JsValue) {
        self.pc_breakpoints = val.into_serde().unwrap();
    }

    pub fn run(&mut self, microseconds: f32) {
        self.gb.hw.cycle_count = 0;

        let desired_cycles = (CYCLES_PER_MICRO * microseconds) as u32;

        while self.gb.hw.cycle_count < desired_cycles {
            self.step();

            let breakpoint =
                self.pc_breakpoints.contains(&self.gb.cpu.pc) || self.gb.hw.mem_get(self.gb.cpu.pc) == 0x40;

            if breakpoint {
                let _ = self.breakpoint_cb.call0(&JsValue::NULL);
                return;
            }
        }
    }

    pub fn step(&mut self) {
        self.gb.run_instruction();

        if self.gb.hw.new_frame {
            let buf = unsafe {
                Uint8ClampedArray::view(slice::from_raw_parts(
                    (self.gb.hw.ppu.framebuffer).as_ptr() as *const u8,
                    160 * 144 * 4,
                ))
            };

            let _ = self.frame_cb.call1(&JsValue::NULL, &buf);
        }
    }

    pub fn step_frame(&mut self) {
        while !self.gb.hw.new_frame {
            self.step();
        }
    }

    /// Decodes n number of instructions starting from given address.
    pub fn current_instructions(&self, addr: u16, n: usize) -> JsValue {
        let mut instrs = Vec::new();

        let mut loc = addr;

        while instrs.len() < n {
            let inst_loc = loc;
            let inst = cpu::decode_instruction(|| {
                let b = self.gb.hw.mem_get(loc);
                loc = loc.wrapping_add(1);
                b
            });

            instrs.push(Instruction {
                loc: inst_loc,
                txt: inst.to_string(),
            });
        }

        JsValue::from_serde(&instrs).unwrap()
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        self.gb.hw.mem_get(addr)
    }

    pub fn reg_read(&self, reg: &str) -> Option<u16> {
        match reg {
            "AF" => Some(self.gb.cpu.register16_get(cpu::Register16::AF)),
            "BC" => Some(self.gb.cpu.register16_get(cpu::Register16::BC)),
            "DE" => Some(self.gb.cpu.register16_get(cpu::Register16::DE)),
            "HL" => Some(self.gb.cpu.register16_get(cpu::Register16::HL)),
            "PC" => Some(self.gb.cpu.pc),
            "SP" => Some(self.gb.cpu.sp),
            _ => None,
        }
    }

    pub fn get_ime(&self) -> bool {
        self.gb.cpu.ime
    }

    pub fn get_ime_defer(&self) -> bool {
        self.gb.cpu.ime_defer
    }

    pub fn get_halted(&self) -> bool {
        self.gb.cpu.halted
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
