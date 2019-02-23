pub mod apu;
pub mod cartridge;
pub mod cpu;
pub mod dma;
pub mod hardware;
pub mod interrupt;
pub mod joypad;
pub mod ppu;
pub mod serial;
pub mod timer;

use cpu::Cpu;
use hardware::GameboyHardware;

const CYCLES_PER_MICRO: f32 = 1048576.0 / 1000000.0;

// The main entrypoint into Oxideboy. Represents an emulation session for a Gameboy.
pub struct Gameboy {
    pub cpu: Cpu,
    pub hw: GameboyHardware,
}

/// There are different models of the Gameboy that each behave slightly differently (different HW quirks, etc).
/// When creating a Gameboy emulation context, the desired Model must be chosen.
#[derive(Eq, PartialEq)]
pub enum Model {
    DMG0,
    DMG,
}

impl Gameboy {
    pub fn new(model: Model, rom: Vec<u8>) -> Gameboy {
        Gameboy {
            cpu: Cpu::new(),
            hw: GameboyHardware::new(model, rom),
        }
    }

    /// Run the Gameboy for a single CPU instruction. Useful for debuggers / tests.
    pub fn run_instruction(&mut self) {
        self.cpu.step(&mut self.hw);
    }

    /// Run the Gameboy for the specified number of microseconds.
    /// This entrypoint is useful for emulating the Gameboy in real-time, while adhering to a refresh rate or some other
    /// external timing control. For example, the web emulator uses requestAnimationFrame to drive emulation, which
    /// provides a microsecond-resolution timestamp that can be used to determine how many microseconds passed since the
    /// last emulation step.
    /// While executing, a new video frame may become available. If so, the provided closure will be called.
    pub fn run_for_microseconds<T: FnMut(&[u32])>(&mut self, num_micros: f32, mut frame_cb: T) {
        self.hw.cycle_count = 0;
        let desired_cycles = (CYCLES_PER_MICRO * num_micros) as u32;

        while self.hw.cycle_count < desired_cycles {
            self.cpu.step(&mut self.hw);

            if self.hw.new_frame {
                self.hw.new_frame = false;
                frame_cb(&self.hw.ppu.framebuffer);
            }
        }
    }

    pub fn skip_bootrom(&mut self) {
        self.cpu.pc = 0x100;
        self.cpu.sp = 0xFFFE;
        self.cpu.a = 0x01;
        self.cpu.b = 0xFF;
        self.cpu.c = 0x13;
        self.cpu.e = 0xC1;
        self.cpu.h = 0x84;
        self.cpu.l = 0x03;

        self.hw.timer.div = 0x1800;
        self.hw.interrupts.request = 0x1;

        // Ensure PPU has correct state (enabled, BG enabled, etc)
        self.hw.ppu.reg_lcdc_write(0x91);
        // PPU should be in the middle of a VBlank.
        self.hw.ppu.mode = ppu::Mode::Mode1;
        self.hw.ppu.ly = 145;
        self.hw.ppu.cycles = 144;

        // Setup Nintendo logo in tilemap.
        let mut addr = 0x1904;
        for v in 1..=12 {
            self.hw.ppu.vram_write(addr, v);
            addr += 1;
        }

        if self.hw.model == Model::DMG {
            self.hw.ppu.vram_write(addr, 0x19);
        }

        addr = 0x1924;
        for v in 13..=24 {
            self.hw.ppu.vram_write(addr, v);
            addr += 1;
        }
        // Copy Nintendo logo data from cart.
        let mut vram_addr = 0x10;
        let mut src_addr = 0x104;
        for _ in 0..48 {
            let b = self.hw.mem_get(src_addr);
            src_addr += 1;
            // Double up each bit in the source byte, using sorcery.
            // The best kind of sorcery too: copy/pasted from the interwebz.
            let z = (((b as u64).wrapping_mul(0x0101010101010101) & 0x8040201008040201)
                .wrapping_mul(0x0102040810204081)
                >> 49)
                & 0x5555
                | (((b as u64).wrapping_mul(0x0101010101010101) & 0x8040201008040201).wrapping_mul(0x0102040810204081)
                    >> 48)
                    & 0xAAAAu64;
            self.hw.ppu.vram_write(vram_addr, (z >> 8) as u8);
            vram_addr += 2;
            self.hw.ppu.vram_write(vram_addr, (z >> 8) as u8);
            vram_addr += 2;
            self.hw.ppu.vram_write(vram_addr, z as u8);
            vram_addr += 2;
            self.hw.ppu.vram_write(vram_addr, z as u8);
            vram_addr += 2;
        }

        if self.hw.model == Model::DMG {
            let mut src_addr = 0xD8;
            for _ in 0..8 {
                let v = self.hw.mem_get(src_addr);
                self.hw.ppu.vram_write(vram_addr, v);
                src_addr += 1;
                vram_addr += 2;
            }
        }

        // After bootrom is finished, sound1 is still enabled but muted.
        self.hw.apu.chan1.vol_env = apu::VolumeEnvelope {
            default: 0b1111,
            inc: false,
            steps: 0b011,
            val: 0,
            timer: 0,
        };
        self.hw.apu.chan1.on = true;
        self.hw.apu.reg_nr11_write(0b1000_0000);
        self.hw.apu.reg_nr50_write(0x77);
        self.hw.apu.reg_nr51_write(0xF3);

        self.hw.bootrom_enabled = false;
    }

    pub fn core_panic(&self, msg: String) -> ! {
        panic!(
            "{}\nIME: {},{}\nHalt: {}\nRegs:\n\tA=0x{:02X}\n\tB=0x{:02X}\n\tC=0x{:02X}\n\tD=0x{:02X}\n\tE=0x{:02X}\n\tF=0x{:02X}\n\tH=0x{:02X}\n\tL=0x{:02X}\n\tSP={:#04X}\n\tPC={:#04X}",
            msg, self.cpu.ime, self.cpu.ime_defer, self.cpu.halted, self.cpu.a, self.cpu.b, self.cpu.c, self.cpu.d, self.cpu.e, self.cpu.f.pack(), self.cpu.h, self.cpu.l, self.cpu.sp, self.cpu.pc);
    }
}
