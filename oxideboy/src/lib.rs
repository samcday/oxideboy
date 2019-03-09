pub mod apu;
pub mod cartridge;
pub mod cpu;
pub mod dma;
pub mod interrupt;
pub mod joypad;
pub mod ppu;
pub mod serial;
pub mod simple_diff;
pub mod timer;
pub mod util;

use crate::apu::Apu;
use crate::cartridge::Cartridge;
use crate::cpu::Cpu;
use crate::dma::DmaController;
use crate::interrupt::InterruptController;
use crate::joypad::Joypad;
use crate::ppu::Ppu;
use crate::serial::Serial;
use crate::timer::Timer;
use bincode;
use serde::{Deserialize, Serialize};

pub const CYCLES_PER_MICRO: f32 = 1_048_576.0 / 1_000_000.0;

static DMG0_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg0.rom");
static DMG_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg.rom");

memory_segment! { Ram; u8; 0x2000 }
memory_segment! { HRam; u8; 0x7F }

// The main entrypoint into Oxideboy. Represents an emulation session for a Gameboy.
#[derive(Deserialize, Serialize)]
pub struct Gameboy {
    pub model: Model,
    pub bootrom_enabled: bool,
    pub cycle_count: u64,
    pub new_frame: bool, // Set to true when the PPU has completed rendering of a whole frame.

    pub apu: Apu,
    pub cart: Cartridge,
    pub cpu: Cpu,
    pub dma: DmaController,
    pub hram: HRam, // 0xFF80 - 0xFFFE
    pub interrupts: InterruptController,
    pub joypad: Joypad,
    pub ppu: Ppu,
    pub ram: Ram, // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    pub serial: Serial,
    pub timer: Timer,
}

/// There are different models of the Gameboy that each behave slightly differently (different HW quirks, etc).
/// When creating a Gameboy emulation context, the desired Model must be chosen.
#[derive(Eq, Deserialize, PartialEq, Serialize)]
pub enum Model {
    DMG0,
    DMG,
}

impl Gameboy {
    pub fn new(model: Model, rom: Vec<u8>) -> Gameboy {
        let cart = Cartridge::from_rom(rom);

        Gameboy {
            model,
            bootrom_enabled: true,
            cycle_count: 0,
            new_frame: false,

            apu: Apu::new(),
            cart,
            cpu: Cpu::new(),
            dma: Default::default(),
            hram: Default::default(),
            interrupts: InterruptController::new(),
            joypad: Default::default(),
            ppu: Ppu::new(),
            ram: Default::default(),
            serial: Serial::new(),
            timer: Timer::new(),
        }
    }

    /// Serializes the entire state of the emulator into the destination Vec<u8>. The emulator can be restored to the
    /// state with load_state().
    pub fn save_state(&self, dst: &mut Vec<u8>) {
        bincode::serialize_into(dst, self).unwrap();
    }

    ///
    pub fn load_state(&mut self, state: &Vec<u8>) {
        // The deserialize call overwrites the whole struct with a new one from the given state. As a result the rom
        // field of the cartridge gets reset to an empty Vec. So we grab it out first and put it into the new struct
        // after the deserialize call.
        let rom = std::mem::replace(&mut self.cart.rom, vec![]);
        *self = bincode::deserialize(state).unwrap();
        std::mem::replace(&mut self.cart.rom, rom);
    }

    /// Run the Gameboy for a single CPU instruction. Useful for debuggers / tests.
    pub fn run_instruction(&mut self) {
        self.new_frame = false;
        cpu::step(self);
    }

    /// Run the Gameboy for the specified number of microseconds.
    /// This entrypoint is useful for emulating the Gameboy in real-time, while adhering to a refresh rate or some other
    /// external timing control. For example, the web emulator uses requestAnimationFrame to drive emulation, which
    /// provides a microsecond-resolution timestamp that can be used to determine how many microseconds passed since the
    /// last emulation step.
    // TODO:
    // pub fn run_for_microseconds(&mut self, num_micros: f32) {
    //     let desired_cycles = (CYCLES_PER_MICRO * num_micros) as u64;

    //     while self.cycle_count < desired_cycles {
    //         self.cpu.step(&mut self);
    //     }
    // }

    pub fn skip_bootrom(&mut self) {
        self.cpu.pc = 0x100;
        self.cpu.sp = 0xFFFE;

        self.cpu.a = 0x01;
        self.cpu.d = 0x00;
        self.cpu.c = 0x13;
        match self.model {
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

        self.timer.div = match self.model {
            Model::DMG0 => 0x182C,
            Model::DMG => 0xABC8,
        };
        self.interrupts.request = 0x1;

        // Ensure PPU has correct state (enabled, BG enabled, etc)
        self.ppu.enabled = true;
        self.ppu.bg_enabled = true;
        self.ppu.bg_tile_area_lo = true;

        // PPU should be in the middle of a VBlank.
        // Where the PPU is at in terms of mode + cycles depends on which bootrom was run.
        self.ppu.mode = ppu::Mode::Mode1;
        match self.model {
            Model::DMG => {
                self.ppu.ly = 153;
                self.ppu.mode_cycles = 100;
            }
            Model::DMG0 => {
                self.ppu.ly = 145;
                self.ppu.mode_cycles = 24;
            }
        }

        // Setup Nintendo logo in tilemap.
        let mut addr = 0x1904;
        for v in 1..=12 {
            self.ppu.vram_write(addr, v);
            addr += 1;
        }

        if self.model == Model::DMG {
            self.ppu.vram_write(addr, 0x19);
        }

        addr = 0x1924;
        for v in 13..=24 {
            self.ppu.vram_write(addr, v);
            addr += 1;
        }
        // Copy Nintendo logo data from cart.
        let mut vram_addr = 0x10;
        let mut src_addr = 0x104;
        for _ in 0..48 {
            let b = self.mem_get(src_addr);
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
            self.ppu.vram_write(vram_addr, (z >> 8) as u8);
            vram_addr += 2;
            self.ppu.vram_write(vram_addr, (z >> 8) as u8);
            vram_addr += 2;
            self.ppu.vram_write(vram_addr, z as u8);
            vram_addr += 2;
            self.ppu.vram_write(vram_addr, z as u8);
            vram_addr += 2;
        }

        if self.model == Model::DMG {
            let mut src_addr = 0xD8;
            for _ in 0..8 {
                let v = self.mem_get(src_addr);
                self.ppu.vram_write(vram_addr, v);
                src_addr += 1;
                vram_addr += 2;
            }
        }

        // After bootrom is finished, sound1 is still enabled but muted.
        self.apu.chan1.vol_env = apu::VolumeEnvelope {
            default: 0b1111,
            inc: false,
            steps: 0b011,
            val: 0,
            timer: 0,
        };
        self.apu.chan1.on = true;
        self.apu.reg_nr11_write(0b1000_0000);
        self.apu.reg_nr50_write(0x77);
        self.apu.reg_nr51_write(0xF3);

        // TODO: not 100% sure on this magic number.
        // Basically, even when the serial port isn't active, it's clocking in order to shift out a bit.
        // It's tied to the main clock cycle count, and not dependent on when SC/SB is changed. This magic number
        // ensures that mooneye serial/boot_sclk_align test passes.
        self.serial.transfer_clock = 124;

        self.bootrom_enabled = false;
    }

    pub fn core_panic(&self, msg: String) -> ! {
        panic!(
            "{}\nIME: {},{}\nHalt: {}\nRegs:\n\tA=0x{:02X}\n\tB=0x{:02X}\n\tC=0x{:02X}\n\tD=0x{:02X}\n\tE=0x{:02X}\n\tF=0x{:02X}\n\tH=0x{:02X}\n\tL=0x{:02X}\n\tSP={:#04X}\n\tPC={:#04X}",
            msg, self.cpu.ime, self.cpu.ime_defer, self.cpu.halted, self.cpu.a, self.cpu.b, self.cpu.c, self.cpu.d, self.cpu.e, self.cpu.f.pack(), self.cpu.h, self.cpu.l, self.cpu.sp, self.cpu.pc);
    }

    fn bootrom_read(&self, addr: u16) -> u8 {
        match self.model {
            Model::DMG0 => DMG0_BOOTROM[addr as usize],
            Model::DMG => DMG_BOOTROM[addr as usize],
        }
    }

    /// Handles reads from the memory bus. This method is responsible for resolving memory addresses to the correct
    /// memory segments and registers. Generally though, mem_read should be used since it ensures components are
    /// clocked correctly and reads are prevented during situations like active DMA transfers.
    pub fn mem_get(&self, addr: u16) -> u8 {
        match addr {
            0x0000...0x0100 if self.bootrom_enabled => self.bootrom_read(addr),
            0x0000...0x3FFF => self.cart.rom_lo(addr as usize),
            0x4000...0x7FFF => self.cart.rom_hi((addr - 0x4000) as usize),
            0x8000...0x9FFF => self.ppu.vram_read(addr - 0x8000),
            0xA000...0xBFFF => self.cart.ram((addr - 0xA000) as usize),
            0xC000...0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000...0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFE00...0xFE9F => self.ppu.oam_read((addr - 0xFE00) as usize),
            0xFF00 => self.joypad.reg_p1_read(),
            0xFF01 => self.serial.reg_sb_read(),
            0xFF02 => self.serial.reg_sc_read(),
            0xFF04 => (self.timer.div >> 8) as u8,
            0xFF05 => self.timer.tima,
            0xFF06 => self.timer.tma,
            0xFF07 => self.timer.reg_tac_read(),
            0xFF0F => 0xE0 | self.interrupts.request, // Unused IF bits are always 1
            0xFF10 => self.apu.reg_nr10_read(),
            0xFF11 => self.apu.reg_nr11_read(),
            0xFF12 => self.apu.reg_nr12_read(),
            0xFF13 => self.apu.reg_nr13_read(),
            0xFF14 => self.apu.reg_nr14_read(),
            0xFF16 => self.apu.reg_nr21_read(),
            0xFF17 => self.apu.reg_nr22_read(),
            0xFF18 => self.apu.reg_nr23_read(),
            0xFF19 => self.apu.reg_nr24_read(),
            0xFF1A => self.apu.reg_nr30_read(),
            0xFF1B => self.apu.reg_nr31_read(),
            0xFF1C => self.apu.reg_nr32_read(),
            0xFF1D => self.apu.reg_nr33_read(),
            0xFF1E => self.apu.reg_nr34_read(),
            0xFF20 => self.apu.reg_nr41_read(),
            0xFF21 => self.apu.reg_nr42_read(),
            0xFF22 => self.apu.reg_nr43_read(),
            0xFF23 => self.apu.reg_nr44_read(),
            0xFF24 => self.apu.reg_nr50_read(),
            0xFF25 => self.apu.reg_nr51_read(),
            0xFF26 => self.apu.reg_nr52_read(),
            0xFF30...0xFF3F => self.apu.wave_read(addr - 0xFF30),
            0xFF40 => self.ppu.reg_lcdc_read(),
            0xFF41 => self.ppu.reg_stat_read(),
            0xFF42 => self.ppu.scy,
            0xFF43 => self.ppu.scx,
            0xFF44 => self.ppu.ly,
            0xFF45 => self.ppu.lyc,
            0xFF46 => self.dma.reg,
            0xFF47 => self.ppu.bgp.pack(),
            0xFF48 => self.ppu.obp0.pack(),
            0xFF49 => self.ppu.obp1.pack(),
            0xFF4A => self.ppu.wy as u8,
            0xFF4B => self.ppu.wx as u8,
            0xFF80...0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF => self.interrupts.enable,

            _ => 0xFF, // Reads from unhandled locations see 0xFF
        }
    }

    pub fn mem_read(&mut self, addr: u16) -> u8 {
        // While DMA transfer is in progress, reads to the OAM area will see 0xFF.
        let block_read = self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F);

        // Here's a batshit hardware quirk, the PPU is out of sync with the CPU by 2 T-cycles. To simulate this effect,
        // we perform the read *before* running the downstream hardware clocks, if we're reading a PPU register or OAM.
        if !block_read && ((addr >= 0xFF40 && addr <= 0xFF4B) || (addr >= 0xFE00 && addr <= 0xFE9F)) {
            let v = self.mem_get(addr);
            self.clock();
            return v;
        }

        // Reading from the memory bus takes a full CPU cycle.
        self.clock();

        if block_read {
            0xFF
        } else {
            self.mem_get(addr)
        }
    }
    pub fn mem_read16(&mut self, addr: u16) -> u16 {
        let mut v = u16::from(self.mem_read(addr));
        v |= u16::from(self.mem_read(addr + 1)) << 8;
        v
    }

    pub fn mem_write(&mut self, addr: u16, v: u8) {
        // While DMA transfer is in progress, write to the OAM area will be ignored.
        let block_write = self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F);
        // Writing to the memory bus takes a full CPU cycle.
        self.clock();

        if block_write {
            return;
        }

        self.mem_set(addr, v);
    }

    pub fn mem_set(&mut self, addr: u16, v: u8) {
        match addr {
            0x0000...0x7FFF => self.cart.write(addr, v),
            0x8000...0x9FFF => self.ppu.vram_write(addr - 0x8000, v),
            0xA000...0xBFFF => self.cart.write(addr, v),
            0xC000...0xDFFF => self.ram[(addr - 0xC000) as usize] = v,
            0xE000...0xFDFF => self.ram[(addr - 0xE000) as usize] = v,
            0xFE00...0xFE9F => self.ppu.oam_write((addr - 0xFE00) as usize, v),
            0xFF00 => self.joypad.reg_p1_write(v),
            0xFF01 => self.serial.reg_sb_write(v),
            0xFF02 => self.serial.reg_sc_write(v),
            0xFF04 => self.timer.reg_div_write(),
            0xFF05 => self.timer.reg_tima_write(v),
            0xFF06 => self.timer.reg_tma_write(v),
            0xFF07 => self.timer.reg_tac_write(v & 0x7, &mut self.interrupts),
            0xFF0F => self.interrupts.reg_if_write(v & 0x1F),
            0xFF10 => self.apu.reg_nr10_write(v),
            0xFF11 => self.apu.reg_nr11_write(v),
            0xFF12 => self.apu.reg_nr12_write(v),
            0xFF13 => self.apu.reg_nr13_write(v),
            0xFF14 => self.apu.reg_nr14_write(v),
            0xFF16 => self.apu.reg_nr21_write(v),
            0xFF17 => self.apu.reg_nr22_write(v),
            0xFF18 => self.apu.reg_nr23_write(v),
            0xFF19 => self.apu.reg_nr24_write(v),
            0xFF1A => self.apu.reg_nr30_write(v),
            0xFF1B => self.apu.reg_nr31_write(v),
            0xFF1C => self.apu.reg_nr32_write(v),
            0xFF1D => self.apu.reg_nr33_write(v),
            0xFF1E => self.apu.reg_nr34_write(v),
            0xFF20 => self.apu.reg_nr41_write(v),
            0xFF21 => self.apu.reg_nr42_write(v),
            0xFF22 => self.apu.reg_nr43_write(v),
            0xFF23 => self.apu.reg_nr44_write(v),
            0xFF24 => self.apu.reg_nr50_write(v),
            0xFF25 => self.apu.reg_nr51_write(v),
            0xFF26 => self.apu.reg_nr52_write(v),
            0xFF30...0xFF3F => self.apu.wave_write(addr - 0xFF30, v),
            0xFF40 => self.ppu.reg_lcdc_write(v, &mut self.interrupts),
            0xFF41 => self.ppu.reg_stat_write(v, &mut self.interrupts),
            0xFF42 => self.ppu.reg_scy_write(v),
            0xFF43 => self.ppu.reg_scx_write(v),
            0xFF45 => self.ppu.reg_lyc_write(v, &mut self.interrupts),
            0xFF46 => self.dma.start(v),
            0xFF47 => self.ppu.reg_bgp_write(v),
            0xFF48 => self.ppu.reg_obp0_write(v),
            0xFF49 => self.ppu.reg_obp1_write(v),
            0xFF4A => self.ppu.reg_wy_write(v),
            0xFF4B => self.ppu.reg_wx_write(v),
            0xFF50 if v == 1 => self.bootrom_enabled = false,
            0xFF80...0xFFFE => self.hram[(addr - 0xFF80) as usize] = v,
            0xFFFF => self.interrupts.reg_ie_write(v),

            _ => {} // Writes to unhandled locations are simply ignored.
        }
    }
    pub fn mem_write16(&mut self, addr: u16, v: u16) {
        self.mem_write(addr + 1, ((v & 0xFF00) >> 8) as u8);
        self.mem_write(addr, (v & 0xFF) as u8);
    }

    pub fn clock(&mut self) {
        self.cycle_count += 1;

        self.timer.clock(&mut self.interrupts);
        let (dma_active, dma_src, dma_dst) = self.dma.clock();
        if dma_active {
            let v = self.mem_get(dma_src);
            self.ppu.oam_write(dma_dst, v);
        }
        self.serial.clock(&mut self.interrupts);
        self.ppu.clock(&mut self.interrupts, &mut self.new_frame);
        self.apu.clock();
    }
}

// Until Rust gets support for const generics, we need to create "glue" types to wrap our big array types in order to
// get Serde to correctly serialize them as byte array blobs, which is *significantly* faster than the "Seq" fallback
// that shims like the serde-big-array crate use. Seriously, like, nearly 100x faster. Thankfully Rust is awesome and
// lets us implement a bunch of std::ops traits, so we can still index/deref the wrapped data transparently.
// IMPORTANT: memory segments can only be created out of primitive types or structs with repr(C) layout. We do some
// ugly unsafe shit below to shuffle things in and out as &[u8] slices.
#[macro_export]
macro_rules! memory_segment {
    ( $name:ident; $type:tt; $size:tt ) => {
        #[repr(C)]
        pub struct $name([$type; $size]);

        impl Default for $name {
            fn default() -> $name {
                $name {
                    0: [Default::default(); $size],
                }
            }
        }

        #[allow(unused)]
        impl $name {
            fn as_ptr(&self) -> *const $type {
                self.0.as_ptr()
            }

            fn iter(&self) -> std::slice::Iter<$type> {
                self.0.iter()
            }
        }

        impl std::ops::Index<usize> for $name {
            type Output = $type;

            fn index(&self, index: usize) -> &$type {
                &self.0[index]
            }
        }
        impl std::ops::IndexMut<usize> for $name {
            fn index_mut(&mut self, index: usize) -> &mut $type {
                &mut self.0[index]
            }
        }

        impl std::ops::Deref for $name {
            type Target = [$type];

            fn deref(&self) -> &[$type] {
                &self.0
            }
        }

        impl serde::Serialize for $name {
            fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
            where
                S: serde::Serializer,
            {
                let slice: &[u8] = unsafe {
                    std::slice::from_raw_parts(self.0.as_ptr() as *const u8, std::mem::size_of::<$type>() * $size)
                };
                serializer.serialize_bytes(slice)
            }
        }

        impl<'de> serde::Deserialize<'de> for $name {
            fn deserialize<D>(deserializer: D) -> Result<$name, D::Error>
            where
                D: serde::Deserializer<'de>,
            {
                struct ArrayVisitor;

                impl<'de> serde::de::Visitor<'de> for ArrayVisitor {
                    type Value = $name;

                    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
                        formatter.write_str(&format!("a byte array of {} elements", $size))
                    }

                    fn visit_bytes<E>(self, value: &[u8]) -> Result<Self::Value, E>
                    where
                        E: serde::de::Error,
                    {
                        let expected_size = std::mem::size_of::<$type>() * $size;

                        if value.len() != expected_size {
                            return Err(E::custom(format!(
                                "expected byte array of {:x}, but this is {:x}",
                                $size,
                                value.len()
                            )));
                        }
                        let mut segment: $name = Default::default();
                        unsafe {
                            let dst = segment.0.as_mut_ptr() as *mut u8;
                            std::ptr::copy(value.as_ptr(), dst, expected_size);
                        }
                        Ok(segment)
                    }
                }

                deserializer.deserialize_bytes(ArrayVisitor)
            }
        }
    };
}
