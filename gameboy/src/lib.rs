#[macro_use] extern crate serde_derive;
extern crate serde;
extern crate bincode;

use bincode::{serialize, deserialize};

pub mod apu;
pub mod cartridge;
pub mod cpu;
pub mod dma;
pub mod interrupt;
pub mod ppu;
pub mod timer;

pub const SCREEN_SIZE: usize = 160 * 144;
pub const FRAME_RATE: f64 = 1048576.0 / 17556.0;

static DMG0_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg0.rom");

#[derive(Serialize, Deserialize, Default)]
pub struct JoypadState {
    btn: bool,
    dir: bool,

    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub a: bool,
    pub b: bool,
    pub select: bool,
    pub start: bool,
}

impl JoypadState {
    fn reg_p1_read(&self) -> u8 {
        let mut v = 0xCF;

        if self.btn {
            v ^= 0x20;
            if self.a      { v ^= 1 }
            if self.b      { v ^= 2 }
            if self.select { v ^= 4 }
            if self.start  { v ^= 8 }
        } else if self.dir {
            v ^= 0x10;
            if self.right  { v ^= 1 }
            if self.left   { v ^= 2 }
            if self.up     { v ^= 4 }
            if self.down   { v ^= 8 }
        }

        v
    }

    fn reg_p1_write(&mut self, v: u8) {
        self.btn = v & 0x20 == 0;
        self.dir = v & 0x10 == 0;
    }
}

#[derive(Serialize, Deserialize)]
pub struct GameboyState {
    pub cpu: cpu::CPUState,
    pub apu: apu::APUState,
    pub dma: dma::DMAState,
    pub int: interrupt::InterruptState,
    pub ppu: ppu::PPUState,
    pub timer: timer::TimerState,
    pub joypad: JoypadState,
    pub cart: cartridge::Cartridge,

    // RAM segments.
    #[serde(with = "BigArray")]
    pub ram:  [u8; 0x2000], // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    #[serde(with = "BigArray")]
    pub hram: [u8; 0x7F],   // 0xFF80 - 0xFFFE
}

impl Default for GameboyState {
    fn default() -> Self {
        Self {
            cpu: cpu::CPUState::new(),
            apu: apu::APUState::new(),
            ppu: ppu::PPUState::new(),
            dma: Default::default(),
            int: Default::default(),
            timer: Default::default(),
            joypad: Default::default(),
            cart: Default::default(),
            ram: [0; 0x2000], hram: [0; 0x7F],
        }
    }
}

impl GameboyState {
    fn new(cart: cartridge::Cartridge) -> Self {
        Self { cart, ..Default::default() }
    }
}

/// Represents a running gameboy emulation session. Contains the cartridge being run, and the
/// state of the CPU / PPU / APU / timer / etc.
pub struct GameboyContext {
    pub state: GameboyState,
    rom: Vec<u8>,

    pub cycle_count: u64,
    next_frame_cycles: u64,

    // Debugging stuff.
    pub instr_addr: u16,            // Address of the current instruction
    pub mooneye_breakpoint: bool,
}

impl GameboyContext {
    pub fn new(rom: Vec<u8>) -> Self {
        let cart = cartridge::Cartridge::from_rom(&rom);

        Self {
            state: GameboyState::new(cart),
            rom,
            cycle_count: 0,
            next_frame_cycles: 0,
            instr_addr: 0,
            mooneye_breakpoint: false,
        }
    }

    pub fn save_state(&self) -> Vec<u8> {
        serialize(&self.state).unwrap()
    }

    pub fn load_state(&mut self, data: &[u8]) {
        self.state = deserialize(data).unwrap();
    }

    // Advances the CPU clock by 1 "instruction cycle", which is actually 4 low level machine cycles.
    // The accuracy of clock counting is important, as it affects the accuracy of everything else - the timer
    // PPU, sound, etc.
    // It's worth noting that the clock can advance in the middle of processing an instruction that
    // spans multiple instruction cycles. Because of this, it means that the timer can actually increment
    // in the middle of a load instruction.
    // For example, we might be processing a LDH A,(0xFF00+$05) instruction which spans 3 instruction cycles.
    // At the time we decoded the instruction (first cycle) the TIMA register (0xFF05) might have been "10".
    // But when we actually fetch that location for the LDH instruction another cycle later, the value could have
    // incremented to 11. This cycle accuracy is tested in the mem_timing.gb test ROM.
    fn clock(&mut self) {
        self.cycle_count += 1;
        timer::clock(&mut self.state.timer, &mut self.state.int);
        dma::clock(self);
        ppu::clock(&mut self.state.ppu, &mut self.state.int);
        apu::clock(&mut self.state.apu);
    }

    /// Skips emulating the bootrom (scrolling Nintendo logo) and just ensures all internal state looks like it ran.
    pub fn skip_bootrom(&mut self) {
        self.state.cpu.pc = 0x100;
        self.state.cpu.sp = 0xFFFE;
        self.state.timer.div = 0x1800;
        self.state.int.requested = 0x1;

        self.state.cpu.a = 0x01;
        self.state.cpu.b = 0xFF;
        self.state.cpu.c = 0x13;
        self.state.cpu.e = 0xC1;
        self.state.cpu.h = 0x84;
        self.state.cpu.l = 0x03;

        // Ensure PPU has correct state (enabled, BG enabled, etc)
        self.state.ppu.reg_lcdc_write(0x91);
        // PPU should be in the middle of a VBlank.
        self.state.ppu.mode = ppu::Mode::VBlank;
        self.state.ppu.ly = 145;
        self.state.ppu.cycles = 144;
        // Setup Nintendo logo in tilemap.
        let mut addr = 0x1904;
        for v in 1..=12 {
            self.state.ppu.vram_write(addr, v);
            addr += 1;
        }
        // TODO: not in DMG0.
        // self.state.ppu.vram_write(addr, 0x19);
        addr = 0x1924;
        for v in 13..=24 {
            self.state.ppu.vram_write(addr, v);
            addr += 1;
        }
        // Copy Nintendo logo data from cart.
        let mut vram_addr = 0x10;
        let mut src_addr = 0x104;
        for _ in 0..48 {
            let b = self.mem_get8(src_addr);
            src_addr += 1;
            // Double up each bit in the source byte, using sorcery.
            // The best kind of sorcery too: copy/pasted from the interwebz.
            let z = (((b as u64).wrapping_mul(0x0101010101010101) & 0x8040201008040201).wrapping_mul(0x0102040810204081) >> 49) & 0x5555 |
                    (((b as u64).wrapping_mul(0x0101010101010101) & 0x8040201008040201).wrapping_mul(0x0102040810204081) >> 48) & 0xAAAAu64;
            self.state.ppu.vram_write(vram_addr, (z >> 8) as u8); vram_addr += 2;
            self.state.ppu.vram_write(vram_addr, (z >> 8) as u8); vram_addr += 2;
            self.state.ppu.vram_write(vram_addr, z as u8); vram_addr += 2;
            self.state.ppu.vram_write(vram_addr, z as u8); vram_addr += 2;
        }

        // TODO: not in DMG0 mode.
        // let mut src_addr = 0xD8;
        // for _ in 0..8 {
        //     let v = self.mem_get8(src_addr);
        //     self.state.ppu.vram_write(vram_addr, v);
        //     src_addr += 1;
        //     vram_addr += 2;
        // }

        // After bootrom is finished, sound1 is still enabled but muted.
        self.state.apu.chan1.vol_env = apu::VolumeEnvelope{
            default: 0b1111,
            inc: false,
            steps: 0b011,
            val: 0,
            timer: 0,
        };
        self.state.apu.chan1.on = true;
        self.state.apu.reg_nr11_write(0b1000_0000);
        self.state.apu.reg_nr50_write(0x77);
        self.state.apu.reg_nr51_write(0xF3);

        self.state.cpu.bootrom_enabled = false;
    }

    fn mem_read8(&mut self, addr: u16) -> u8 {
        // While DMA transfer is in progress, reads to the OAM area will see 0xFF.
        if self.state.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F) {
            self.clock();
            return 0xFF;
        }

        self.clock();
        self.mem_get8(addr)
    }

    fn mem_get8(&mut self, addr: u16) -> u8 {
        match addr {
            0x0000 ... 0x0100 if self.state.cpu.bootrom_enabled => DMG0_BOOTROM[addr as usize],

            0x0000 ... 0x3FFF => self.state.cart.rom_lo(&self.rom)[addr as usize],
            0x4000 ... 0x7FFF => self.state.cart.rom_hi(&self.rom)[(addr - 0x4000) as usize],
            0x8000 ... 0x9FFF => self.state.ppu.vram_read(addr - 0x8000),
            0xA000 ... 0xBFFF => self.state.cart.ram()[(addr - 0xA000) as usize],
            0xC000 ... 0xDFFF => self.state.ram[(addr - 0xC000) as usize],
            0xE000 ... 0xFDFF => self.state.ram[(addr - 0xE000) as usize],
            0xFE00 ... 0xFE9F => self.state.ppu.oam_read((addr - 0xFE00) as usize),
            0xFEA0 ... 0xFEFF => 0x00,

            0xFF00            => self.state.joypad.reg_p1_read(),

            // Serial
            0xFF01            => self.state.cpu.sb.unwrap_or(0),
            0xFF02            => self.state.cpu.reg_sc_read(),

            // Timer
            0xFF04            => (self.state.timer.div >> 8) as u8,
            0xFF05            => self.state.timer.tima,
            0xFF06            => self.state.timer.tma,
            0xFF07            => self.state.timer.reg_tac_read(),

            // Pending interrupts
            0xFF0F            => 0xE0 | self.state.int.requested, // Unused IF bits are always 1

            // Sound
            0xFF10            => self.state.apu.reg_nr10_read(),
            0xFF11            => self.state.apu.reg_nr11_read(),
            0xFF12            => self.state.apu.reg_nr12_read(),
            0xFF13            => self.state.apu.reg_nr13_read(),
            0xFF14            => self.state.apu.reg_nr14_read(),
            0xFF16            => self.state.apu.reg_nr21_read(),
            0xFF17            => self.state.apu.reg_nr22_read(),
            0xFF18            => self.state.apu.reg_nr23_read(),
            0xFF19            => self.state.apu.reg_nr24_read(),
            0xFF1A            => self.state.apu.reg_nr30_read(),
            0xFF1B            => self.state.apu.reg_nr31_read(),
            0xFF1C            => self.state.apu.reg_nr32_read(),
            0xFF1D            => self.state.apu.reg_nr33_read(),
            0xFF1E            => self.state.apu.reg_nr34_read(),
            0xFF20            => self.state.apu.reg_nr41_read(),
            0xFF21            => self.state.apu.reg_nr42_read(),
            0xFF22            => self.state.apu.reg_nr43_read(),
            0xFF23            => self.state.apu.reg_nr44_read(),
            0xFF24            => self.state.apu.reg_nr50_read(),
            0xFF25            => self.state.apu.reg_nr51_read(),
            0xFF26            => self.state.apu.reg_nr52_read(),
            0xFF30 ... 0xFF3F => self.state.apu.wave_read(addr - 0xFF30),

            // PPU
            0xFF40            => self.state.ppu.reg_lcdc_read(),
            0xFF41            => self.state.ppu.reg_stat_read(),
            0xFF42            => self.state.ppu.scy as u8,
            0xFF43            => self.state.ppu.scx as u8,
            0xFF44            => self.state.ppu.ly as u8,
            0xFF45            => self.state.ppu.lyc as u8,
            0xFF46            => self.state.dma.reg,
            0xFF47            => self.state.ppu.bgp.pack(),
            0xFF48            => self.state.ppu.obp0.pack(),
            0xFF49            => self.state.ppu.obp1.pack(),
            0xFF4A            => self.state.ppu.wy as u8,
            0xFF4B            => self.state.ppu.wx as u8,
            0xFF80 ... 0xFFFE => self.state.hram[(addr - 0xFF80) as usize],
            0xFFFF            => self.state.int.enabled,

            _                 => 0xFF,
        }
    }

    fn mem_write8(&mut self, addr: u16, v: u8) {
        self.clock();

        // While DMA transfer is in progress, write to the OAM area will be ignored.
        if self.state.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F) {
            return;
        }

        match addr {
            0x0000 ... 0x7FFF => { self.state.cart.write(addr, v); }
            0x8000 ... 0x9FFF => { self.state.ppu.vram_write(addr - 0x8000, v) }
            0xA000 ... 0xBFFF => { self.state.cart.write(addr, v); }
            0xC000 ... 0xDFFF => { self.state.ram[(addr - 0xC000) as usize] = v }
            0xE000 ... 0xFDFF => { self.state.ram[(addr - 0xE000) as usize] = v }
            0xFE00 ... 0xFE9F => { self.state.ppu.oam_write((addr - 0xFE00) as usize, v) }
            0xFEA0 ... 0xFEFF => { } // Undocumented space that some ROMs seem to address...
            0xFF00            => { self.state.joypad.reg_p1_write(v) }
            0xFF01            => { self.state.cpu.sb = Some(v) }
            0xFF02            => { self.state.cpu.reg_sc_write(v) }
            0xFF04            => { self.state.timer.reg_div_write() }
            0xFF05            => { self.state.timer.reg_tima_write(v) }
            0xFF06            => { self.state.timer.reg_tma_write(v) }
            0xFF07            => { self.state.timer.reg_tac_write(v & 0x7, &mut self.state.int) }
            0xFF0F            => { self.state.int.reg_if_write(v & 0x1F) }

            // Sound
            0xFF10            => { self.state.apu.reg_nr10_write(v) }
            0xFF11            => { self.state.apu.reg_nr11_write(v) }
            0xFF12            => { self.state.apu.reg_nr12_write(v) }
            0xFF13            => { self.state.apu.reg_nr13_write(v) }
            0xFF14            => { self.state.apu.reg_nr14_write(v) }
            0xFF16            => { self.state.apu.reg_nr21_write(v) }
            0xFF17            => { self.state.apu.reg_nr22_write(v) }
            0xFF18            => { self.state.apu.reg_nr23_write(v) }
            0xFF19            => { self.state.apu.reg_nr24_write(v) }
            0xFF1A            => { self.state.apu.reg_nr30_write(v) }
            0xFF1B            => { self.state.apu.reg_nr31_write(v) }
            0xFF1C            => { self.state.apu.reg_nr32_write(v) }
            0xFF1D            => { self.state.apu.reg_nr33_write(v) }
            0xFF1E            => { self.state.apu.reg_nr34_write(v) }
            0xFF20            => { self.state.apu.reg_nr41_write(v) }
            0xFF21            => { self.state.apu.reg_nr42_write(v) }
            0xFF22            => { self.state.apu.reg_nr43_write(v) }
            0xFF23            => { self.state.apu.reg_nr44_write(v) }
            0xFF24            => { self.state.apu.reg_nr50_write(v) }
            0xFF25            => { self.state.apu.reg_nr51_write(v) }
            0xFF26            => { self.state.apu.reg_nr52_write(v) }
            0xFF30 ... 0xFF3F => { self.state.apu.wave_write(addr - 0xFF30, v) }

            // PPU
            0xFF40            => { self.state.ppu.reg_lcdc_write(v) }
            0xFF41            => { self.state.ppu.reg_stat_write(v) }
            0xFF42            => { self.state.ppu.scy = v.into() }
            0xFF43            => { self.state.ppu.scx = v.into() }
            0xFF44            => { }                   // LY is readonly.
            0xFF45            => { self.state.ppu.lyc = v.into() }
            0xFF46            => { dma::start(self, v) }
            0xFF47            => { self.state.ppu.bgp.update(v) }
            0xFF48            => { self.state.ppu.obp0.update(v) }
            0xFF49            => { self.state.ppu.obp1.update(v) }
            0xFF50 if self.state.cpu.bootrom_enabled && v == 1 => {
                self.state.cpu.bootrom_enabled = false;
            }
 
            0xFF4A            => { self.state.ppu.wy = v.into() },
            0xFF4B            => { self.state.ppu.wx = v.into() },

            0xFF4D            => { }      // KEY1 for CGB.
            0xFF7F            => { }      // No idea what this is.
            0xFF80 ... 0xFFFE => { self.state.hram[(addr - 0xFF80) as usize] = v }
            0xFFFF            => { self.state.int.reg_ie_write(v) }

            // TODO:
            _                 => {
                // println!("Unhandled write to ${:X}", addr)
            }
        };
    }

    fn mem_read16(&mut self, addr: u16) -> u16 {
        let mut v = self.mem_read8(addr) as u16;
        v |= (self.mem_read8(addr + 1) as u16) << 8;
        v
    }

    fn mem_write16(&mut self, addr: u16, v: u16) {
        self.mem_write8(addr + 1, ((v & 0xFF00) >> 8) as u8);
        self.mem_write8(addr, (v & 0xFF) as u8);
    }

    pub fn core_panic(&self, msg: String) -> ! {
        panic!(
            "{}\nRegs:\n\tA=0x{:02X}\n\tB=0x{:02X}\n\tC=0x{:02X}\n\tD=0x{:02X}\n\tE=0x{:02X}\n\tF=0x{:02X}\n\tH=0x{:02X}\n\tL=0x{:02X}\n\tSP={:#04X}\n\tPC={:#04X}",
            msg, self.state.cpu.a, self.state.cpu.b, self.state.cpu.c, self.state.cpu.d, self.state.cpu.e, self.state.cpu.f.pack(), self.state.cpu.h, self.state.cpu.l, self.state.cpu.sp, self.state.cpu.pc);
    }

    /// Runs the CPU for a whole video frame. A video frame is exactly 17556 CPU cycles.
    /// This higher level abstraction over the CPU is useful for running the emulator at the correct speed.
    /// Instead of trying to execute the CPU at 1Mhz, you can execute this method at 59.7Hz.
    /// Because we emulate at the instruction level, sometimes the last instruction we execute in this loop
    /// may take us over the 17556 cycle threshold. We return a delta number of cycles that should be fed in to the
    /// next call to ensure we don't run too fast.
    pub fn run_frame(&mut self) {
        self.state.apu.sample_queue.clear();

        self.next_frame_cycles += 17556;
        while self.cycle_count < self.next_frame_cycles {
            cpu::run(self);
        }
    }
}

use std::fmt;
use std::marker::PhantomData;
use serde::ser::{Serialize, Serializer, SerializeTuple};
use serde::de::{Deserialize, Deserializer, Visitor, SeqAccess, Error};

trait BigArray<'de>: Sized {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where S: Serializer;
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where D: Deserializer<'de>;
}

macro_rules! big_array {
    ($($len:expr,)+) => {
        $(
            impl<'de, T> BigArray<'de> for [T; $len]
                where T: Default + Copy + Serialize + Deserialize<'de>
            {
                fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
                    where S: Serializer
                {
                    let mut seq = serializer.serialize_tuple(self.len())?;
                    for elem in &self[..] {
                        seq.serialize_element(elem)?;
                    }
                    seq.end()
                }

                fn deserialize<D>(deserializer: D) -> Result<[T; $len], D::Error>
                    where D: Deserializer<'de>
                {
                    struct ArrayVisitor<T> {
                        element: PhantomData<T>,
                    }

                    impl<'de, T> Visitor<'de> for ArrayVisitor<T>
                        where T: Default + Copy + Deserialize<'de>
                    {
                        type Value = [T; $len];

                        fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                            formatter.write_str(concat!("an array of length ", $len))
                        }

                        fn visit_seq<A>(self, mut seq: A) -> Result<[T; $len], A::Error>
                            where A: SeqAccess<'de>
                        {
                            let mut arr = [T::default(); $len];
                            for i in 0..$len {
                                arr[i] = seq.next_element()?
                                    .ok_or_else(|| Error::invalid_length(i, &self))?;
                            }
                            Ok(arr)
                        }
                    }

                    let visitor = ArrayVisitor { element: PhantomData };
                    deserializer.deserialize_tuple($len, visitor)
                }
            }
        )+
    }
}
big_array! { 40, 127, 176, 384, 2048, 8192, 46080, }
