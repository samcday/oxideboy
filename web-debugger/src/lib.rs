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

struct DebugListener {
    frame_cb: Function,
    breakpoint_cb: Function,
    breakpoint_hit: bool,
}

impl EventListener for DebugListener {
    fn on_frame(&mut self, ppu_buf: &[u32]) {
        // We pass the PPU buffer to the callback in JS land to update the canvas.
        let buf =
            unsafe { Uint8ClampedArray::view(slice::from_raw_parts((ppu_buf).as_ptr() as *const u8, 160 * 144 * 4)) };

        let _ = self.frame_cb.call1(&JsValue::NULL, &buf);
    }
    fn on_memory_write(&mut self, _addr: u16, _: u8) {}
    fn on_debug_breakpoint(&mut self) {
        // Set breakpoint_hit so we know to stop if we're in the mai run() fn pumping instruction cycles.
        self.breakpoint_hit = true;
        let _ = self.breakpoint_cb.call0(&JsValue::NULL);
    }
}

#[wasm_bindgen]
pub struct WebEmu {
    gb: Gameboy<DebugListener>,
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

        let listener = DebugListener {
            frame_cb,
            breakpoint_cb,
            breakpoint_hit: false,
        };

        let mut gb = Gameboy::new(Model::DMG0, rom.to_vec(), listener);
        gb.skip_bootrom();
        gb.hw.ppu.framebuffer_fmt = ppu::PixelFormat::ABGR;
        WebEmu {
            gb,
            pc_breakpoints: HashSet::new(),
        }
    }

    pub fn run(&mut self, microseconds: f32) {
        self.gb.hw.listener.breakpoint_hit = false;
        self.gb.hw.cycle_count = 0;

        let desired_cycles = (CYCLES_PER_MICRO * microseconds) as u32;

        while !self.gb.hw.listener.breakpoint_hit && self.gb.hw.cycle_count < desired_cycles {
            // Check breakpoints.
            if self.pc_breakpoints.contains(&self.gb.cpu.pc) {
                let _ = self.gb.hw.listener.breakpoint_cb.call0(&JsValue::NULL);
                break;
            }
            self.gb.run_instruction();
        }
    }

    pub fn step(&mut self) {
        self.gb.run_instruction();
    }

    /// Decodes n number of instructions centered around the given memory location.
    pub fn current_instructions(&self, addr: u16, n: usize) -> JsValue {
        let mut instrs = Vec::new();

        // We're decoding n instructions total, we want to keep the initial instruction centered, so we decode n/2
        // instructions before and after the given address. Let's start by decoding forwards.
        let mut loc = addr;

        while instrs.len() < n / 2 + 1 {
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

        // Okay going backwards is kinda weird. We don't know where the instruction boundaries are, since the previous
        // instruction may be encoded in 1, 2 or 3 bytes. So each time we decode, we check if the instruction size
        // spills over where it was expected to. If that's the case, we throw that decoded instruction away and try 1
        // space back.
        let mut loc = addr.wrapping_sub(1);
        let mut boundary = addr;
        while instrs.len() < n {
            let inst_loc = loc;
            let inst = cpu::decode_instruction(|| {
                let b = self.gb.hw.mem_get(loc);
                loc = loc.wrapping_add(1);
                b
            });

            if inst_loc + u16::from(inst.size()) > boundary {
                // Oops, collision! Try going backwards a bit more.
                loc = inst_loc.wrapping_sub(1);
                continue;
            }

            instrs.insert(
                0,
                Instruction {
                    loc: inst_loc,
                    txt: inst.to_string(),
                },
            );
            boundary = boundary.wrapping_sub(u16::from(inst.size()));
            loc = inst_loc - 1;
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
