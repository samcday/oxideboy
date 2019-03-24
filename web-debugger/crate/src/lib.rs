extern crate cfg_if;
extern crate wasm_bindgen;

mod utils;

use cfg_if::cfg_if;
use js_sys::{Function, Uint16Array};
use oxideboy::joypad::Button;
use oxideboy::rewind::*;
use oxideboy::rom::Rom;
use oxideboy::*;
use serde::{Deserialize, Serialize};
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
    gb_ctx: Context,
    rom_title: String,
    rom_hash: String,
    breakpoint_cb: Function,
    pc_breakpoints: HashSet<u16>,
    rewind_manager: RewindManager<MemoryStorageAdapter>,
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
    pub fn new(rom: &[u8], breakpoint_cb: Function) -> WebEmu {
        utils::set_panic_hook();

        let rom = Rom::new(rom.to_vec().into()).unwrap();
        let rom_hash = format!("{:x}", md5::compute(&rom.data));
        let rom_title = rom.title.clone();

        let gb = Gameboy::new(Model::DMG0, &rom, false);
        let mut gb_ctx = Context::new(rom);
        gb_ctx.enable_audio = false;

        let rewind_manager = RewindManager::new(&gb, MemoryStorageAdapter::new()).unwrap();

        WebEmu {
            gb,
            gb_ctx,
            pc_breakpoints: HashSet::new(),
            breakpoint_cb,
            rom_hash,
            rom_title,
            rewind_manager,
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

    pub fn run(&mut self, microseconds: f64) {
        // TODO: should keep the desired cycle in struct to handle uneven cycle boundaries.
        let desired_cycle = self.gb.cycle_count + ((BASE_CLOCK_SPEED as f64 / 1_000_000.0) * microseconds) as u64;

        while self.gb.cycle_count < desired_cycle {
            self.step_forward();

            let (cpu, bus) = self.gb.bus(&mut self.gb_ctx);
            let breakpoint = self.pc_breakpoints.contains(&cpu.pc) || bus.memory_get(cpu.pc) == 0x40;

            if breakpoint {
                let _ = self.breakpoint_cb.call0(&JsValue::NULL);
                return;
            }
        }
    }

    pub fn update_state(&mut self, framebuffer: &mut [u16], mem: &mut [u8]) {
        framebuffer.copy_from_slice(&self.gb_ctx.current_framebuffer);

        let (_, bus) = self.gb.bus(&mut self.gb_ctx);

        mem[0x0000..=0x3FFF].copy_from_slice(&bus.cart.rom_lo(&bus.context.rom.data));
        if *bus.bootrom_enabled {
            mem[0x0000..0x0100].copy_from_slice(&bus.bootrom());
        }
        mem[0x4000..=0x7FFF].copy_from_slice(&bus.cart.rom_hi(&bus.context.rom.data));
        mem[0x8000..=0x97FF].copy_from_slice(&bus.ppu.tiles);
        mem[0x9800..=0x9FFF].copy_from_slice(&bus.ppu.tilemap);
        let ram = bus.cart.ram();
        mem[0xA000..0xA000 + ram.len()].copy_from_slice(ram);
        mem[0xC000..=0xDFFF].copy_from_slice(&bus.ram);
        mem[0xE000..=0xFDFF].copy_from_slice(&bus.ram[0..0x1E00]);
        mem[0xFE00..=0xFE9F].copy_from_slice(&bus.ppu.oam_memory());

        for addr in 0xFF00..=0xFFFF {
            mem[addr] = bus.memory_get(addr as u16);
        }
    }

    pub fn step_forward(&mut self) -> bool {
        self.gb.run_instruction(&mut self.gb_ctx);

        if self.gb_ctx.is_new_frame() {
            self.rewind_manager.snapshot(&self.gb);
            true
        } else {
            false
        }
    }

    pub fn step_backward(&mut self) {
        let cycles = self.gb.last_inst_cycles;
        self.rewind_manager
            .rewind_cycles(&mut self.gb, &mut self.gb_ctx, cycles);
    }

    pub fn step_frame_forward(&mut self) {
        loop {
            if self.step_forward() {
                break;
            }
        }
    }

    pub fn step_frame_backward(&mut self) {
        self.rewind_manager.rewind_frame(&mut self.gb, &mut self.gb_ctx);
    }

    /// Decodes n number of instructions starting from given address.
    pub fn current_instructions(&mut self, addr: u16, n: usize) -> JsValue {
        let mut instrs = Vec::new();

        let mut loc = addr;

        while instrs.len() < n {
            let inst_loc = loc;
            let (_, bus) = self.gb.bus(&mut self.gb_ctx);
            let inst = cpu::decode_instruction(|| {
                let b = bus.memory_get(loc);
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

    pub fn mem_read(&mut self, addr: u16) -> u8 {
        let (_, bus) = self.gb.bus(&mut self.gb_ctx);
        bus.memory_get(addr)
    }

    pub fn mem_write(&mut self, addr: u16, v: u8) {
        let (_, mut bus) = self.gb.bus(&mut self.gb_ctx);
        bus.memory_set(addr, v)
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
        let button = match key {
            "ArrowUp" | "Up" => Some(Button::Up),
            "ArrowDown" | "Down" => Some(Button::Down),
            "ArrowLeft" | "Left" => Some(Button::Left),
            "ArrowRight" | "Right" => Some(Button::Right),
            "Enter" => Some(Button::Start),
            "Shift" => Some(Button::Select),
            "a" => Some(Button::A),
            "s" => Some(Button::B),
            _ => None,
        };

        if let Some(button) = button {
            self.gb.set_joypad_button(button, pressed);
            self.rewind_manager.notify_input(&self.gb);
        }
    }
}
