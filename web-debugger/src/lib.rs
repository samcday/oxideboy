extern crate cfg_if;
extern crate wasm_bindgen;

mod utils;

use cfg_if::cfg_if;
use js_sys::{Function, Uint16Array};
use oxideboy::*;
use serde::{Deserialize, Serialize};
use snap;
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

#[derive(Deserialize, Serialize)]
struct CpuState {
    af: u16,
    bc: u16,
    de: u16,
    hl: u16,
    pc: u16,
    sp: u16,
    ime: bool,
    ime_defer: bool,
    halted: bool,
}

#[wasm_bindgen]
impl WebEmu {
    pub fn new(rom: &[u8], frame_cb: Function, breakpoint_cb: Function) -> WebEmu {
        utils::set_panic_hook();

        let mut gb = Gameboy::new(Model::DMG0, rom.to_vec());
        gb.skip_bootrom();

        let rom_title = String::from(gb.cart.rom_title());
        let rom_hash = format!("{:x}", md5::compute(&gb.cart.rom));

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

    pub fn snapshot(&self) {
        let mut state = Vec::new();
        self.gb.save_state(&mut state);
        let mut encoder = snap::Encoder::new();

        let mut output = vec![0; snap::max_compress_len(state.len())];
        let size = encoder.compress(&state, &mut output).unwrap();
        log!("Save state size={} compressed={}", state.len(), size);
    }

    pub fn run(&mut self, microseconds: f32) {
        self.gb.cycle_count = 0;

        let desired_cycles = (CYCLES_PER_MICRO * microseconds) as u64;

        while self.gb.cycle_count < desired_cycles {
            self.step();

            let breakpoint = self.pc_breakpoints.contains(&self.gb.cpu.pc) || self.gb.mem_get(self.gb.cpu.pc) == 0x40;

            if breakpoint {
                let _ = self.breakpoint_cb.call0(&JsValue::NULL);
                return;
            }
        }
    }

    pub fn step(&mut self) {
        self.gb.apu.sample_queue.clear();

        self.gb.run_instruction();

        if self.gb.new_frame {
            let buf = unsafe {
                Uint16Array::view(slice::from_raw_parts(
                    (self.gb.ppu.framebuffer).as_ptr() as *const u16,
                    160 * 144,
                ))
            };

            let _ = self.frame_cb.call1(&JsValue::NULL, &buf);
        }
    }

    pub fn step_frame(&mut self) {
        while !self.gb.new_frame {
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
                let b = self.gb.mem_get(loc);
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
        self.gb.mem_get(addr)
    }

    pub fn mem_write(&mut self, addr: u16, v: u8) {
        self.gb.mem_set(addr, v)
    }

    pub fn cpu_state(&self) -> JsValue {
        JsValue::from_serde(&CpuState {
            af: self.gb.cpu.register16_get(cpu::Register16::AF),
            bc: self.gb.cpu.register16_get(cpu::Register16::BC),
            de: self.gb.cpu.register16_get(cpu::Register16::DE),
            hl: self.gb.cpu.register16_get(cpu::Register16::HL),
            pc: self.gb.cpu.pc,
            sp: self.gb.cpu.sp,
            ime: self.gb.cpu.ime,
            ime_defer: self.gb.cpu.ime_defer,
            halted: self.gb.cpu.halted,
        })
        .unwrap()
    }

    pub fn set_cpu_state(&mut self, val: JsValue) {
        let new_state: Result<CpuState, _> = val.into_serde();
        if new_state.is_err() {
            return;
        }

        let new_state = new_state.unwrap();
        self.gb.cpu.register16_set(cpu::Register16::AF, new_state.af);
        self.gb.cpu.register16_set(cpu::Register16::BC, new_state.bc);
        self.gb.cpu.register16_set(cpu::Register16::DE, new_state.de);
        self.gb.cpu.register16_set(cpu::Register16::HL, new_state.hl);
        self.gb.cpu.pc = new_state.pc;
        self.gb.cpu.sp = new_state.sp;
        self.gb.cpu.ime = new_state.ime;
        self.gb.cpu.ime_defer = new_state.ime_defer;
        self.gb.cpu.halted = new_state.halted;
    }

    pub fn set_joypad_state(&mut self, key: &str, pressed: bool) {
        match key {
            "ArrowUp" | "Up" => self.gb.joypad.up = pressed,
            "ArrowDown" | "Down" => self.gb.joypad.down = pressed,
            "ArrowLeft" | "Left" => self.gb.joypad.left = pressed,
            "ArrowRight" | "Right" => self.gb.joypad.right = pressed,
            "Enter" => self.gb.joypad.start = pressed,
            "Shift" => self.gb.joypad.select = pressed,
            "a" => self.gb.joypad.a = pressed,
            "s" => self.gb.joypad.b = pressed,
            _ => {}
        }
    }
}
