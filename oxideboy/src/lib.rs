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
pub mod util;

use cpu::Cpu;
use hardware::GameboyHardware;

pub const CYCLES_PER_MICRO: f32 = 1_048_576.0 / 1_000_000.0;
pub const NOOP_LISTENER: NoopListener = NoopListener {};

// The main entrypoint into Oxideboy. Represents an emulation session for a Gameboy.
pub struct Gameboy<T: EventListener> {
    pub cpu: Cpu,
    pub hw: GameboyHardware<T>,
}

/// There are different models of the Gameboy that each behave slightly differently (different HW quirks, etc).
/// When creating a Gameboy emulation context, the desired Model must be chosen.
#[derive(Eq, PartialEq)]
pub enum Model {
    DMG0,
    DMG,
}

/// This trait can be implemented to get notified when interesting things occur inside the emulator.
pub trait EventListener {
    /// Called when the PPU has completed a frame.
    fn on_frame(&mut self, frame: &[u32]);

    /// Called when a memory address is written to.
    fn on_memory_write(&mut self, addr: u16, v: u8);

    /// Called before each instruction step is run. If this method returns false the instruction is not run.
    /// Used to implement debugger breakpoints.
    fn before_instruction(&mut self, pc: u16, inst: cpu::Instruction) -> bool;
}

/// An empty EventListener. Use NOOP_LISTENER if you're not interested in anything that occurs inside the emulator.
pub struct NoopListener {}

impl EventListener for NoopListener {
    fn on_frame(&mut self, _: &[u32]) {}
    fn on_memory_write(&mut self, _: u16, _: u8) {}
    fn before_instruction(&mut self, _: u16, _: cpu::Instruction) -> bool {
        true
    }
}

impl<T: EventListener> Gameboy<T> {
    pub fn new(model: Model, rom: Vec<u8>, listener: T) -> Gameboy<T> {
        Gameboy {
            cpu: Cpu::new(),
            hw: GameboyHardware::new(model, rom, listener),
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
    pub fn run_for_microseconds(&mut self, num_micros: f32) {
        self.hw.cycle_count = 0;
        let desired_cycles = (CYCLES_PER_MICRO * num_micros) as u32;

        while self.hw.cycle_count < desired_cycles {
            self.cpu.step(&mut self.hw);
        }
    }

    pub fn skip_bootrom(&mut self) {
        self.cpu.pc = 0x100;
        self.cpu.sp = 0xFFFE;

        self.cpu.a = 0x01;
        self.cpu.d = 0x00;
        self.cpu.c = 0x13;
        match self.hw.model {
            Model::DMG0 => {
                self.cpu.b = 0xFF;
                self.cpu.e = 0xC1;
                self.cpu.h = 0x84;
                self.cpu.l = 0x03;
            }
            Model::DMG => {
                self.cpu.b = 0x00;
                self.cpu.h = 0x01;
                self.cpu.f.unpack(0xB0);
                self.cpu.e = 0xD8;
                self.cpu.l = 0x4D;
            }
        }

        // TODO: I'm not really happy with these DIV magic numbers yet.
        // I worked backwards from the mooneye boot_div tests to get these, but when running the real bootroms, I end
        // up with different DIV numbers. When we run real bootroms, the DIV values are as follows: DMG0=1848, DMG=CF44.
        // This means either the DIV register starts at some specific value on boot up, or the emulation of the bootrom
        // isn't cycle accurate with the hardware. I think the latter is more likely given that the PPU emulation isn't
        // accurate yet. In fact, when I think about it, the first Mode0 is supposed to be shorter after the LCD is
        // enabled, which we haven't implemented yet. So that's probably why the DIV is a few cycles further along with
        // the real bootrom.
        self.hw.timer.div = match self.hw.model {
            Model::DMG0 => 0x182C,
            Model::DMG => 0xABC8,
        };
        self.hw.interrupts.request = 0x1;

        // Ensure PPU has correct state (enabled, BG enabled, etc)
        self.hw.ppu.reg_lcdc_write(0x91);
        // PPU should be in the middle of a VBlank.
        self.hw.ppu.mode = ppu::Mode::Mode1;
        self.hw.ppu.ly = 145;
        self.hw.ppu.mode_cycles = 30;

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
            let z = ((u64::from(b).wrapping_mul(0x0101_0101_0101_0101) & 0x8040_2010_0804_0201)
                .wrapping_mul(0x0102_0408_1020_4081)
                >> 49)
                & 0x5555
                | ((u64::from(b).wrapping_mul(0x0101_0101_0101_0101) & 0x8040_2010_0804_0201)
                    .wrapping_mul(0x0102_0408_1020_4081)
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

        // TODO: not 100% sure on this magic number.
        // Basically, even when the serial port isn't active, it's clocking in order to shift out a bit.
        // It's tied to the main clock cycle count, and not dependent on when SC/SB is changed. This magic number
        // ensures that mooneye serial/boot_sclk_align test passes.
        self.hw.serial.transfer_clock = 124;

        self.hw.bootrom_enabled = false;
    }

    pub fn core_panic(&self, msg: String) -> ! {
        panic!(
            "{}\nIME: {},{}\nHalt: {}\nRegs:\n\tA=0x{:02X}\n\tB=0x{:02X}\n\tC=0x{:02X}\n\tD=0x{:02X}\n\tE=0x{:02X}\n\tF=0x{:02X}\n\tH=0x{:02X}\n\tL=0x{:02X}\n\tSP={:#04X}\n\tPC={:#04X}",
            msg, self.cpu.ime, self.cpu.ime_defer, self.cpu.halted, self.cpu.a, self.cpu.b, self.cpu.c, self.cpu.d, self.cpu.e, self.cpu.f.pack(), self.cpu.h, self.cpu.l, self.cpu.sp, self.cpu.pc);
    }
}
