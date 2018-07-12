pub mod ppu;
mod sound;

use std::fmt;
use std::fmt::Display;

static BOOT_ROM: &[u8; 256] = include_bytes!("boot.rom");

pub const SCREEN_SIZE: usize = 160 * 144;
pub const FRAME_RATE: f64 = 1048576.0 / 17556.0;

#[derive(Default)]
struct CpuFlags {
    z: bool,
    n: bool,
    h: bool,
    c: bool,
}

impl CpuFlags {
    fn reset(&mut self) -> &mut Self {
        self.z = false;
        self.n = false;
        self.h = false;
        self.c = false;
        self
    }

    fn pack(&self) -> u8 {
        0
            | if self.z { 0b1000_0000 } else { 0 }
            | if self.n { 0b0100_0000 } else { 0 }
            | if self.h { 0b0010_0000 } else { 0 }
            | if self.c { 0b0001_0000 } else { 0 }
    }

    fn unpack(&mut self, v: u8) {
        self.z = v & 0b1000_0000 > 0;
        self.n = v & 0b0100_0000 > 0;
        self.h = v & 0b0010_0000 > 0;
        self.c = v & 0b0001_0000 > 0;
    }
}

enum CartridgeType {
    ROMOnly,
    MBC1,
    MBC3,
}

pub struct Cartridge {
    cart_type: CartridgeType,
    rom: Vec<u8>,

    // Used by MBC1 + MBC3
    rom_bank: u8,
    ram_enabled: bool,
    ram_bank: u8,
    ram: Vec<u8>,
}

impl Cartridge {
    fn from_rom(rom: Vec<u8>) -> Self {
        let cart_type = match rom[0x147] {
            0         => CartridgeType::ROMOnly,
            1|2|3     => CartridgeType::MBC1,
            0x12|0x13 => CartridgeType::MBC3,
            v => panic!("Unsupported cartridge type: {:X}", v)
        };
        let ram = match cart_type {
            CartridgeType::ROMOnly => Vec::new(),
            _ => vec![0; match rom[0x149] {
                0 => 0,
                1 => 2048,
                2 => 8192,
                3 => 32768,
                4 => 131072,
                v => panic!("Unexpected RAM size {} encountered", v),
            }]
        };
        Self{cart_type, rom, rom_bank: 1, ram_enabled: false, ram, ram_bank: 0}
    }

    fn rom_lo(&self) -> &[u8] {
        &self.rom[0..0x4000]
    }

    fn rom_hi(&self) -> &[u8] {
        match self.cart_type {
            CartridgeType::ROMOnly => &self.rom[0x4000..0x8000],
            CartridgeType::MBC1|CartridgeType::MBC3 => {
                let base = (self.rom_bank as usize) * 0x4000;
                &self.rom[base .. base + 0x4000]
            }
        }
    }

    fn ram(&self) -> &[u8] {
        let base = (self.ram_bank as usize) * 0x2000;
        &self.ram[base .. base + 0x2000]
    }

    fn write(&mut self, addr: u16, v: u8) {
        match self.cart_type {
            CartridgeType::ROMOnly => {},
            CartridgeType::MBC1 => self.mbc1_write(addr as usize, v),
            CartridgeType::MBC3 => self.mbc3_write(addr as usize, v),
        };
    }

    fn mbc1_write(&mut self, addr: usize, v: u8) {
        match addr {
            0x0000 ... 0x1FFF => {
                self.ram_enabled = v == 0xA;
            },
            0x2000 ... 0x3FFF => {
                if v > 0x1F {
                    return;
                }
                self.rom_bank = v.max(1);
                // if self.rom_bank >= self.rom_bank_cnt {
                //     // TODO:
                //     panic!("ROM bank {} requested, maximum is {}", self.rom_bank, self.rom_bank_cnt);
                // }
            },
            0xA000 ... 0xBFFF => {
                if !self.ram_enabled {
                    return;
                }
                let addr = (self.ram_bank as usize) * 0x2000 + addr - 0xA000;
                if addr < self.ram.len() {
                    self.ram[addr] = v;
                }
            }
            _ => panic!("Unexpected cartridge write value {:X} to address: {:X}", v, addr)
        }
    }

    fn mbc3_write(&mut self, addr: usize, v: u8) {
        match addr {
            0x0000 ... 0x1FFF => {
                self.ram_enabled = v == 0xA;
            },
            0x2000 ... 0x3FFF => {
                self.rom_bank = v.max(1);
            },
            0x4000 ... 0x5FFF => {
                self.ram_bank = v & 0b11;
            },
            0x6000 ... 0x7FFF => {
                // TODO:
            },
            0xA000 ... 0xBFFF => {
                if !self.ram_enabled {
                    return;
                }
                let addr = (self.ram_bank as usize) * 0x2000 + addr - 0xA000;
                if addr < self.ram.len() {
                    self.ram[addr] = v;
                }
            }
            _ => panic!("Unexpected cartridge write value {:X} to address: {:X}", v, addr)
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Reg8 { A, B, C, D, E, H, L }

#[derive(Clone, Copy, Debug)]
enum Reg16 { AF, BC, DE, HL, SP }

use Reg8::{*};
use Reg16::{*};

#[derive(Clone, Copy, Debug)]
enum FlagCondition { NZ, Z, NC, C }

enum BitwiseOp { XOR, OR, AND }

#[derive(Clone, Copy, Debug)]
enum Operand8 {
    Reg(Reg8),        // Contents of an 8 bit register.
    Imm(u8),          // Immediate value.
    Addr(Reg16),      // 16 bit register value interpreted as memory address.
    AddrInc(Reg16),   // 16 bit register value interpreted as memory address. Increment register after use.
    AddrDec(Reg16),   // 16 bit register value interpreted as memory address. Decrement register after use.
    ImmAddr(u16),     // Immediate 16 bit value interpreted as memory address.
    ImmAddrHigh(u8),  // Immediate 8 bit value interpreted as memory address from $FF00.
    AddrHigh(Reg8),   // 8 bit register value interpreted as memory address from $FF00.
}

impl Operand8 {
    fn get(&self, cpu: &mut CPU) -> u8 {
        match *self {
            Operand8::Reg(r) => cpu.get_reg8(r),
            Operand8::Imm(d) => d,
            Operand8::Addr(rr) => { let addr = cpu.get_reg16(rr); cpu.mem_read8(addr) },
            Operand8::AddrInc(rr) => {
                let addr = cpu.get_reg16(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    cpu.ppu.maybe_trash_oam();
                }
                cpu.set_reg16(rr, addr.wrapping_add(1));
                cpu.mem_read8(addr)
            },
            Operand8::AddrDec(rr) => {
                let addr = cpu.get_reg16(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    cpu.ppu.maybe_trash_oam();
                }
                cpu.set_reg16(rr, addr.wrapping_sub(1));
                cpu.mem_read8(addr)
            },
            Operand8::ImmAddr(addr) => cpu.mem_read8(addr),
            Operand8::ImmAddrHigh(addr) => cpu.mem_read8(0xFF00 + (addr as u16)),
            Operand8::AddrHigh(r) => { let addr = 0xFF00 + (cpu.get_reg8(r) as u16); cpu.mem_read8(addr) },
        }
    }

    fn set(&self, cpu: &mut CPU, v: u8) -> u8 {
        match *self {
            Operand8::Reg(r) => cpu.set_reg8(r, v),
            Operand8::Imm(_) => { panic!("Attempted to write to immediate operand") },
            Operand8::Addr(rr) => {
                let addr = cpu.get_reg16(rr);
                cpu.mem_write8(addr, v)
            },
            Operand8::AddrInc(rr) => {
                let addr = cpu.get_reg16(rr);
                cpu.set_reg16(rr, addr.wrapping_add(1));
                cpu.mem_write8(addr, v)
            },
            Operand8::AddrDec(rr) => {
                let addr = cpu.get_reg16(rr);
                cpu.set_reg16(rr, addr.wrapping_sub(1));
                cpu.mem_write8(addr, v)
            },
            Operand8::ImmAddr(addr) => cpu.mem_write8(addr, v),
            Operand8::ImmAddrHigh(addr) => cpu.mem_write8(0xFF00 + (addr as u16), v),
            Operand8::AddrHigh(r) => { let addr = cpu.get_reg8(r); cpu.mem_write8(0xFF00 + (addr as u16), v) },
        };
        v
    }
}

impl Display for Operand8 {
    fn fmt(&self, f: &mut ::fmt::Formatter) -> ::fmt::Result {
        match *self {
            Operand8::Reg(r) => write!(f, "{:?}", r),
            Operand8::Imm(d) => write!(f, "${:02X}", d),
            Operand8::Addr(rr) => write!(f, "({:?})", rr),
            Operand8::AddrInc(rr) => write!(f, "({:?}+)", rr),
            Operand8::AddrDec(rr) => write!(f, "({:?}-)", rr),
            Operand8::ImmAddr(addr) => write!(f, "({:#04X})", addr),
            Operand8::ImmAddrHigh(addr) => write!(f, "(0xFF00+${:02X})", addr),
            Operand8::AddrHigh(r) => write!(f, "(0xFF00+{:?})", r),
        }
    }
}


#[derive(Clone, Copy, Debug)]
enum Operand16 {
    Reg(Reg16),        // Contents of a 16 bit register.
    Imm(u16),          // Immediate value.
    ImmAddr(u16),      // Immediate 16 bit value interpreted as memory address.
}

impl Operand16 {
    fn get(&self, cpu: &mut CPU) -> u16 {
        match *self {
            Operand16::Reg(r) => cpu.get_reg16(r),
            Operand16::Imm(d) => d,
            Operand16::ImmAddr(addr) => cpu.mem_read16(addr),
        }
    }

    fn set(&self, cpu: &mut CPU, v: u16) {
        match *self {
            Operand16::Reg(r) => cpu.set_reg16(r, v),
            Operand16::Imm(_) => panic!("Attempted to write to immediate operand"),
            Operand16::ImmAddr(addr) => cpu.mem_write16(addr, v),
        }
    }
}

impl Display for Operand16 {
    fn fmt(&self, f: &mut ::fmt::Formatter) -> ::fmt::Result {
        match *self {
            Operand16::Reg(r) => write!(f, "{:?}", r),
            Operand16::Imm(d) => write!(f, "${:04X}", d),
            Operand16::ImmAddr(d) => write!(f, "({:#04X})", d),
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
enum Inst {
    NOP,
    EI,
    DI,
    STOP,
    HALT,

    ADD(Operand8),
    ADD16(Reg16),
    ADD_SP_r8(i8),
    ADC(Operand8),
    SUB(Operand8),
    SBC(Operand8),
    INC8(Operand8),
    DEC8(Operand8),
    INC16(Reg16),
    DEC16(Reg16),
    DAA,

    LD8(Operand8, Operand8),
    LD16(Operand16, Operand16),
    LD_HL_SP(i8),

    CP(Operand8),
    AND(Operand8),
    OR(Operand8),
    XOR(Operand8),

    RLA,
    RLCA,
    RRA,
    RRCA,
    RLC(Operand8),
    RRC(Operand8),
    RL(Operand8),
    RR(Operand8),
    SLA(Operand8),
    SRA(Operand8),
    SRL(Operand8),

    BIT(u8, Operand8),
    SET(u8, Operand8),
    RES(u8, Operand8),
    SWAP(Operand8),

    JP(Option<FlagCondition>, Operand16),
    JR(Option<FlagCondition>, u8),
    CALL(Option<FlagCondition>, u16),
    RET(Option<FlagCondition>),
    PUSH(Reg16),
    POP(Reg16),
    RST(u8),
    RETI,

    CPL,
    CCF,
    SCF,
}

impl Display for Inst {
    fn fmt(&self, f: &mut ::fmt::Formatter) -> ::fmt::Result {
        match self {
            Inst::ADC(o) => write!(f, "ADC A, {}", o),
            Inst::ADD(o) => write!(f, "ADD A, {}", o),
            Inst::ADD16(rr) => write!(f, "ADD HL, {:?}", rr),
            Inst::ADD_SP_r8(d) => write!(f, "ADD SP, ${:X}", d),
            Inst::AND(o) => write!(f, "AND {}", o),
            Inst::BIT(b, o) => write!(f, "BIT {}, {}", b, o),
            Inst::CALL(None, n) => write!(f, "CALL ${:X}", n),
            Inst::CALL(Some(cc), n) => write!(f, "CALL {:?}, ${:X}", cc, n),
            Inst::CCF => write!(f, "CCF"),
            Inst::CP(o) => write!(f, "CP {}", o),
            Inst::CPL => write!(f, "CPL"),
            Inst::DAA => write!(f, "DAA"),
            Inst::DEC16(o) => write!(f, "DEC {:?}", o),
            Inst::DEC8(o) => write!(f, "DEC {}", o),
            Inst::DI => write!(f,  "DI"),
            Inst::EI => write!(f,  "EI"),
            Inst::HALT => write!(f,  "HALT"),
            Inst::INC16(rr) => write!(f, "INC {:?}", rr),
            Inst::INC8(o) => write!(f, "INC {}", o),
            Inst::JP(None, o) => write!(f, "JP {}", o),
            Inst::JP(Some(cc), o) => write!(f, "JP {:?}, {}", cc, o),
            Inst::JR(None, n) => write!(f, "JR ${:X}", n),
            Inst::JR(Some(cc), n) => write!(f, "JR {:?}, ${:X}", cc, n),
            Inst::LD16(l, r) => write!(f, "LD {}, {}", l, r),
            Inst::LD8(l, r) => write!(f, "LD {}, {}", l, r),
            Inst::LD_HL_SP(d) => write!(f, "LD HL, (SP+${:X})", d),
            Inst::NOP => write!(f,  "NOP"),
            Inst::OR(o) => write!(f, "OR {}", o),
            Inst::POP(rr) => write!(f, "POP {:?}", rr),
            Inst::PUSH(rr) => write!(f, "PUSH {:?}", rr),
            Inst::RES(b, o) => write!(f, "RES {}, {:?}", b, o),
            Inst::RET(None) => write!(f, "RET"),
            Inst::RET(Some(cc)) => write!(f, "RET {:?}", cc),
            Inst::RETI => write!(f,  "RETI"),
            Inst::RL(o) => write!(f, "RL {}", o),
            Inst::RLA => write!(f, "RLA"),
            Inst::RLC(o) => write!(f, "RLC {}", o),
            Inst::RLCA => write!(f, "RLCA"),
            Inst::RR(o) => write!(f, "RR {}", o),
            Inst::RRA => write!(f, "RRA"),
            Inst::RRC(o) => write!(f, "RRC {}", o),
            Inst::RRCA => write!(f, "RRCA"),
            Inst::RST(n) => write!(f, "RST ${:X}", n),
            Inst::SBC(o) => write!(f, "SBC A, {}", o),
            Inst::SCF => write!(f, "SCF"),
            Inst::SET(b, o) => write!(f, "SET {}, {:?}", b, o),
            Inst::SLA(o) => write!(f, "SLA {}", o),
            Inst::SRA(o) => write!(f, "SRA {}", o),
            Inst::SRL(o) => write!(f, "SRL {}", o),
            Inst::STOP => write!(f,  "STOP"),
            Inst::SUB(o) => write!(f, "SUB A, {}", o),
            Inst::SWAP(o) => write!(f, "SWAP {}", o),
            Inst::XOR(o) => write!(f, "XOR {}", o),
        }
    }
}

#[derive(Default)]
pub struct Joypad {
    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub a: bool,
    pub b: bool,
    pub select: bool,
    pub start: bool,
}

pub struct CPU {
    // Registers.
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    f: CpuFlags,
    sp: u16,
    pub pc: u16,

    bootrom_enabled: bool,

    // RAM segments.
    ram:  [u8; 0x2000], // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    hram: [u8; 0x7F],   // 0xFF80 - 0xFFFE

    // Interrupts.
    halted: bool,
    ie: u8,
    if_: u8,
    ime: bool,
    ime_defer: bool,

    // PPU
    ppu: ppu::PPU,

    // Sound
    sound: sound::SoundController,

    // Timer.
    div: u16,               // Increments every CPU clock cycle (4.194304Mhz). Only top 8 bits are visible to programs.
    timer_enabled: bool,    // Controls whether the timer unit is running
    timer_freq: u16,        // Expressed as a divisor of the DIV register.
    tima: u8,               // Every time DIV register ticks timer_freq times, TIMA is incremented.
    tma: u8,                // When TIMA overflows, the timer interrupt is set and TIMA is reset to this value
    tima_overflow: bool,    // The reloading of TIMA with TMA is delayed by 1 instruction cycle.
    tima_new: bool,         // If we just reloaded TIMA, this flag will be set for that cycle.

    // Joypad.
    joypad_btn: bool,
    joypad_dir: bool,
    joypad: Joypad,

    // Serial I/O
    pub sb: Option<u8>,
    serial_transfer: bool,
    serial_internal_clock: bool,

    dma_reg: u8,
    dma_request: bool,
    dma_active: bool,
    dma_from: u16,
    dma_idx: usize,

    cart: Cartridge,

    pub cycle_count: u64,

    // Debugging stuff.
    instr_addr: u16,            // Address of the current instruction
    mooneye_breakpoint: bool,
}

impl CPU {
    pub fn new(rom: Vec<u8>) -> Self {
        Self{
            a: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0, f: Default::default(),
            sp: 0, pc: 0,
            bootrom_enabled: true,
            ram: [0; 0x2000], hram: [0; 0x7F],
            halted: false, ime: false, ime_defer: false, ie: 0, if_: 0,
            ppu: ppu::PPU::new(),
            sound: sound::SoundController::new(),
            div: 0, tima: 0, tma: 0, timer_enabled: false, timer_freq: 0, tima_overflow: false, tima_new: false,
            joypad_btn: false, joypad_dir: false, joypad: Default::default(),
            sb: None, serial_transfer: false, serial_internal_clock: false,
            dma_reg: 0, dma_request: false, dma_active: false, dma_from: 0, dma_idx: 0,
            cart: Cartridge::from_rom(rom),
            cycle_count: 0,
            instr_addr: 0,
            mooneye_breakpoint: false,
        }
    }

    /// Skips emulating the bootrom (scrolling Nintendo logo) and just ensures all internal state looks like it ran.
    pub fn skip_bootrom(&mut self) {
        self.pc = 0x100;
        self.sp = 0xFFFE;
        self.div = 0x1800;
        self.if_ = 0x1;
        
        self.a = 0x01;
        self.b = 0xFF;
        self.c = 0x13;
        self.e = 0xC1;
        self.h = 0x84;
        self.l = 0x03;

        // Ensure PPU has correct state (enabled, BG enabled, etc)
        self.ppu.reg_lcdc_write(0x91);
        // PPU should be in the middle of a VBlank.
        self.ppu.state = ppu::PPUState::VBlank;
        self.ppu.prev_state = ppu::PPUState::VBlank;
        self.ppu.ly = 145;
        self.ppu.cycles = 144;
        // Setup Nintendo logo in tilemap.
        let mut addr = 0x1904;
        for v in 1..=12 {
            self.ppu.vram_write(addr, v);
            addr += 1;
        }
        // TODO: not in DMG0.
        // self.ppu.vram_write(addr, 0x19);
        addr = 0x1924;
        for v in 13..=24 {
            self.ppu.vram_write(addr, v);
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
            self.ppu.vram_write(vram_addr, (z >> 8) as u8); vram_addr += 2;
            self.ppu.vram_write(vram_addr, (z >> 8) as u8); vram_addr += 2;
            self.ppu.vram_write(vram_addr, z as u8); vram_addr += 2;
            self.ppu.vram_write(vram_addr, z as u8); vram_addr += 2;
        }

        // TODO: not in DMG0 mode.
        // let mut src_addr = 0xD8;
        // for _ in 0..8 {
        //     let v = self.mem_get8(src_addr);
        //     self.ppu.vram_write(vram_addr, v);
        //     src_addr += 1;
        //     vram_addr += 2;
        // }

        // After bootrom is finished, sound1 is still enabled but muted.
        self.sound.chan1.vol_env = sound::VolumeEnvelope{
            default: 0b1111,
            inc: false,
            steps: 0b011,
            val: 0,
            timer: 0,
        };
        self.sound.chan1.on = true;
        self.sound.write_nr11(0b1000_0000);
        self.sound.write_nr50(0x77);
        self.sound.write_nr51(0xF3);

        self.bootrom_enabled = false;
    }

    pub fn pc(&self) -> u16 {
        return self.pc;
    }

    /// Runs the CPU for a whole "frame". A frame is exactly 17556 CPU cycles.
    /// This higher level abstraction over the CPU is useful for running the emulator at the correct speed.
    /// Instead of trying to execute the CPU at 1Mhz, you can execute this method at 59.7Hz.
    /// Because we emulate at the instruction level, sometimes the last instruction we execute in this loop
    /// may take us over the 17556 cycle threshold. We return a delta number of cycles that should be fed in to the
    /// next call to ensure we don't run too fast.
    pub fn run_frame(&mut self, delta: u16) -> (u16, &[u32], &[f32]) {
        self.sound.sample_queue.clear();

        let mut counter = 0;
        while counter < 17556 - delta {
            counter += self.run();
        }

        (counter - (17556 - delta), &self.ppu.framebuffer(), &self.sound.sample_queue[..])
    }

    // Runs the CPU for a single instruction.
    // This is the main "Fetch, decode, execute" cycle.
    // Returns the number of CPU cycles consumed.
    pub fn run(&mut self) -> u16 {
        let start_count = self.cycle_count;

        if self.halted {
            self.advance_clock();
        }

        self.process_interrupts();

        // Apply deferred change to IME.
        if self.ime_defer {
            self.ime = true;
            self.ime_defer = false;
        }

        if !self.halted {
            self.instr_addr = self.pc;
            let inst = self.decode();

            self.execute(inst);
            if !self.bootrom_enabled {
                // println!("Instr: {} ;${:04X} ({})", inst, self.instr_addr, self.cycle_count);
            }
        }

        return (self.cycle_count - start_count) as u16;
    }

    pub fn joypad(&mut self) -> &mut Joypad {
        &mut self.joypad
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
    fn advance_clock(&mut self) {
        self.cycle_count += 1;
        self.advance_timer();
        self.advance_dma();
        self.sound.advance();
        self.if_ |= self.ppu.advance();
    }

    /// Runs the DMA procedure when it's active. DMA transfers copy 160 bytes, one per cycle, from a configurable
    /// source address into OAM memory. DMA transfers begin one extra cycle after being requested. They cannot be
    /// stopped, but they can be restarted from a new location by writing to the DMA register again.
    fn advance_dma(&mut self) {
        // When a DMA is initiated, it doesn't begin on the next cycle, but the one after.
        if self.dma_request {
            self.dma_request = false;
            self.dma_active = true;
            return;
        }

        if !self.dma_active {
            return;
        }

        let addr = self.dma_from + (self.dma_idx as u16);
        let v = self.mem_get8(addr);
        self.ppu.oam_write(self.dma_idx, v);
        self.dma_idx += 1;
        if self.dma_idx == 160 {
            self.dma_active = false;
        }
    }

    // Advances the TIMA register depending on how many CPU clock cycles have passed, and the
    // state of TMA / TAC.
    fn advance_timer(&mut self) {
        // DIV is a 16 bit register (of which only the upper 8 bits are addressable) that increments
        // at the same speed as the CPU clock, 4.194304Mhz.
        self.div = self.div.wrapping_add(4);
        self.tima_new = false;

        // If TIMA overflowed on the previous cycle, this is where we reload it to TMA and request timer interrupt.
        // See the code below here for when we set tima_overflow.
        if self.tima_overflow {
            self.tima = self.tma;
            self.if_ |= 0x4;
            self.tima_overflow = false;
            self.tima_new = true;
        }

        if self.timer_enabled && self.div % self.timer_freq == 0 {
            self.tima = self.tima.wrapping_add(1);
            if self.tima == 0 {
                // When TIMA overflows, we don't actually reload it with TMA and request timer interrupt immediately.
                // That happens on the next cycle.
                self.tima_overflow = true;
            }
        }
    }

    fn get_tac(&self) -> u8 {
        0b1111_1000 | (if self.timer_enabled { 0b100 } else { 0 }) | match self.timer_freq {
            0 => 0,
            1024 => 0,
            16 => 1,
            64 => 2,
            256 => 3,
            _ => unreachable!("timer_freq has fixed set of values")
        }
    }

    fn set_tac(&mut self, v: u8) {
        let was_enabled = self.timer_enabled;
        let orig_freq = self.timer_freq;

        self.timer_enabled = v & 0b100 > 0;
        if self.timer_enabled {
            self.timer_freq = match v & 0b11 {
                0b00 => 1024,
                0b01 => 16,
                0b10 => 64,
                0b11 => 256,
                _ => unreachable!("Matched all possible 2 bit values"),
            }
        }

        // The original DMG had a glitch that could cause TIMA to sometimes spuriously increment.
        // We emulate that quirk here.
        // More info: http://gbdev.gg8.se/wiki/articles/Timer_Obscure_Behaviour
        let glitch = if was_enabled {
            if !self.timer_enabled {
                self.div & (orig_freq / 2) != 0
            } else {
                (self.div & (orig_freq / 2) != 0) && (self.div & (self.timer_freq/2) == 0)
            }
        } else { false };
        if glitch {
            // A cut down version of what we do in advance_timer.
            // Note how here we're not delaying the TIMA reset or interrupt request to the next cycle.
            // We do it immediately in the glitch case. Hardware is weird.
            self.tima = self.tima.wrapping_add(1);
            if self.tima == 0 {
                self.tima = self.tma;
                self.if_ |= 0x4;
            }
        }
    }

    fn execute(&mut self, inst: Inst) {
        match inst {
            Inst::NOP => (),
            Inst::ADD(o) => self.add8(o, false),
            Inst::ADC(o) => self.add8(o, true),
            Inst::SUB(o) => self.sub8(o, false, true),
            Inst::SBC(o) => self.sub8(o, true, true),
            Inst::CP(o)  => self.sub8(o, false, false),
            Inst::LD8(l, r) => self.ld8(l, r),
            Inst::LD16(l, r) => self.ld16(l, r),
            Inst::INC8(o) => self.inc8(o),
            Inst::DEC8(o) => self.dec8(o),
            Inst::INC16(r) => self.inc16(r),
            Inst::DEC16(r) => self.dec16(r),
            Inst::AND(o) => self.bitwise(BitwiseOp::AND, o),
            Inst::OR(o) => self.bitwise(BitwiseOp::OR, o),
            Inst::XOR(o) => self.bitwise(BitwiseOp::XOR, o),
            Inst::RLC(o) => self.rlc(o, true),
            Inst::RRC(o) => self.rrc(o, true),
            Inst::RL(o) => self.rl(o, true, true),
            Inst::RR(o) => self.rr(o, true),
            Inst::SLA(o) => self.rl(o, true, false),
            Inst::SRA(o) => self.shift_right(o, true),
            Inst::SRL(o) => self.shift_right(o, false),
            Inst::SWAP(o) => self.swap(o),
            Inst::BIT(b, o) => self.bit(b, o),
            Inst::RES(b, o) => self.setbit(b, o, false),
            Inst::SET(b, o) => self.setbit(b, o, true),
            Inst::DI => self.set_ime(false),
            Inst::EI => self.set_ime(true),
            Inst::JR(cc, n) => self.jr(cc, n),
            Inst::JP(cc, o) => self.jp(cc, o),
            Inst::RETI => self.ret(None, true),
            Inst::STOP => self.stop(),
            Inst::HALT => self.halt(),
            Inst::DAA => self.daa(),
            Inst::CPL => self.cpl(),
            Inst::CCF => self.ccf(),
            Inst::SCF => self.scf(),
            Inst::ADD_SP_r8(d) => self.add_sp_r8(d),
            Inst::ADD16(rr) => self.add16(rr),
            Inst::RLA => self.rl(Operand8::Reg(A), false, true),
            Inst::RLCA => self.rlc(Operand8::Reg(A), false),
            Inst::RRA => self.rr(Operand8::Reg(A), false),
            Inst::RRCA => self.rrc(Operand8::Reg(A), false),
            Inst::LD_HL_SP(d) => self.ld_hl_sp(d), 
            Inst::CALL(cc, addr) => self.call(cc, addr),
            Inst::RET(cc) => self.ret(cc, false),
            Inst::PUSH(r) => self.push(r),
            Inst::POP(r) => self.pop(r),
            Inst::RST(a) => self.rst(a),
        }
    }

    fn process_interrupts(&mut self) {
        // We exit HALT state if there's any pending interrupts.
        // We do this regardless of IME state.
        if self.halted && self.if_ & self.ie > 0 {
            self.halted = false;
        }

        // Service interrupts.
        let intr = self.if_ & self.ie;
        if !self.ime || intr == 0 {
            return;
        }

        let (addr, flag) = if intr & 0x1 > 0 {
            // Vertical blanking
            (0x40, 0x1)
        } else if intr & 0x2 > 0 {
            // LCDC status interrupt
            (0x48, 0x2)
        } else if intr & 0x4 > 0 {
            // Timer overflow
            (0x50, 0x4)
        } else if intr & 0x8 > 0 {
            // Serial transfer completion
            (0x58, 0x8)
        } else if intr & 0x10 > 0 {
            // P10-P13 input signal goes low
            (0x60, 0x10)
        } else {
            unreachable!("All interrupt flags covered");
        };

        // Interrupt handling needs 3 internal cycles to do interrupt-y stuff.
        self.advance_clock();
        self.advance_clock();
        self.advance_clock();

        // Here's an interesting quirk. If the stack pointer was set to 0000 or 0001, then the push we just did
        // above would have overwritten IE. If the new IE value no longer matches the interrupt we were processing,
        // then we cancel that interrupt and set PC to 0. We then try and find another interrupt.
        // If there isn't one, we end up running code from 0000. Crazy.

        // ... That said, all the checking for that weird edge case is expensive. We don't need to do it if SP isn't
        // either 0000 or 0001.
        if self.sp > 1 {
            self.ime = false;
            self.if_ ^= flag;
            self.push_and_jump(addr);
            return;
        }

        let pc = self.pc;
        let mut sp = self.sp;
        sp = sp.wrapping_sub(1);
        self.mem_write8(sp, ((pc & 0xFF00) >> 8) as u8);
        // This is where we capture what IE is after pushing the upper byte. Pushing the lower byte might
        // also overwrite IE, but in that case we ignore that occurring.
        let newie = self.ie;
        sp = sp.wrapping_sub(1);
        self.mem_write8(sp, pc as u8);
        self.sp = self.sp.wrapping_sub(2);

        if newie & flag == 0 {
            self.pc = 0;
            // Okay so this interrupt didn't go so good. Let's see if there's another one.
            self.process_interrupts();
            // Maybe that call found an interrupt and disabled IME, maybe it didn't. Either way this
            // cancellation needs to result in IME being disabled.
            self.ime = false;
            return;
        }

        self.if_ ^= flag;
        self.ime = false;
        self.pc = addr;
    }

    fn mem_read8(&mut self, addr: u16) -> u8 {
        // While DMA transfer is in progress, reads to the OAM area will see 0xFF.
        if self.dma_active && (addr >= 0xFE00 && addr <= 0xFE9F) {
            self.advance_clock();
            return 0xFF;
        }

        self.advance_clock();
        self.mem_get8(addr)
    }

    fn mem_get8(&mut self, addr: u16) -> u8 {
        match addr {
            0x0000 ... 0x100 if self.bootrom_enabled => BOOT_ROM[addr as usize],

            0x0000 ... 0x3FFF => self.cart.rom_lo()[addr as usize],
            0x4000 ... 0x7FFF => self.cart.rom_hi()[(addr - 0x4000) as usize],
            0x8000 ... 0x9FFF => self.ppu.vram_read(addr - 0x8000),
            0xA000 ... 0xBFFF => self.cart.ram()[(addr - 0xA000) as usize],
            0xC000 ... 0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000 ... 0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFE00 ... 0xFE9F => self.ppu.oam_read((addr - 0xFE00) as usize),
            0xFEA0 ... 0xFEFF => 0x00,

            0xFF00            => self.read_joypad(),

            // Serial
            0xFF01            => self.sb.unwrap_or(0),
            0xFF02            => self.read_sc(),

            // Timer
            0xFF04            => (self.div >> 8) as u8,
            0xFF05            => self.tima,
            0xFF06            => self.tma,
            0xFF07            => self.get_tac(),

            // Pending interrupts
            0xFF0F            => 0xE0 | self.if_, // Unused IF bits are always 1

            // Sound
            0xFF10            => self.sound.read_nr10(),
            0xFF11            => self.sound.read_nr11(),
            0xFF12            => self.sound.read_nr12(),
            0xFF13            => self.sound.read_nr13(),
            0xFF14            => self.sound.read_nr14(),
            0xFF16            => self.sound.read_nr21(),
            0xFF17            => self.sound.read_nr22(),
            0xFF18            => self.sound.read_nr23(),
            0xFF19            => self.sound.read_nr24(),
            0xFF1A            => self.sound.read_nr30(),
            0xFF1B            => self.sound.read_nr31(),
            0xFF1C            => self.sound.read_nr32(),
            0xFF1D            => self.sound.read_nr33(),
            0xFF1E            => self.sound.read_nr34(),
            0xFF20            => self.sound.read_nr41(),
            0xFF21            => self.sound.read_nr42(),
            0xFF22            => self.sound.read_nr43(),
            0xFF23            => self.sound.read_nr44(),
            0xFF24            => self.sound.read_nr50(),
            0xFF25            => self.sound.read_nr51(),
            0xFF26            => self.sound.read_nr52(),
            0xFF30 ... 0xFF3F => self.sound.wave_read(addr - 0xFF30),

            // PPU
            0xFF40            => self.ppu.reg_lcdc_read(),
            0xFF41            => self.ppu.reg_stat_read(),
            0xFF42            => self.ppu.scy as u8,
            0xFF43            => self.ppu.scx as u8,
            0xFF44            => self.ppu.ly as u8,
            0xFF45            => self.ppu.lyc as u8,
            0xFF46            => self.dma_reg,
            0xFF47            => self.ppu.bgp.pack(),
            0xFF48            => self.ppu.obp0.pack(),
            0xFF49            => self.ppu.obp1.pack(),
            0xFF4A            => self.ppu.wy as u8,
            0xFF4B            => self.ppu.wx as u8,
            0xFF80 ... 0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF            => self.ie,

            _                 => 0xFF,
        }
    }

    fn mem_write8(&mut self, addr: u16, v: u8) {
        // While DMA transfer is in progress, write to the OAM area will be ignored.
        if self.dma_active && (addr >= 0xFE00 && addr <= 0xFE9F) {
            self.advance_clock();
            return;
        }

        self.advance_clock();
        match addr {
            0x0000 ... 0x7FFF => { self.cart.write(addr, v); }
            0x8000 ... 0x9FFF => { self.ppu.vram_write(addr - 0x8000, v) }
            0xA000 ... 0xBFFF => { self.cart.write(addr, v); }
            0xC000 ... 0xDFFF => { self.ram[(addr - 0xC000) as usize] = v }
            0xE000 ... 0xFDFF => { self.ram[(addr - 0xE000) as usize] = v }
            0xFE00 ... 0xFE9F => { self.ppu.oam_write((addr - 0xFE00) as usize, v) }
            0xFEA0 ... 0xFEFF => { } // Undocumented space that some ROMs seem to address...
            0xFF01            => { self.sb = Some(v) }
            0xFF02            => { self.write_sc(v) }
            0xFF04            => { self.write_div() }
            0xFF05            => {
                // If TIMA is written to during the same cycle it overflowed, then the interrupt and reloading of TIMA
                // with TMA is cancelled.
                // See tima_write_reloading mooneye test.
                self.tima_overflow = false;

                // Converse to above, if we're in the next cycle where TIMA was reloaded with TMA, we actually swallow
                // this write to TIMA.
                if !self.tima_new {
                    self.tima = v;
                }
            }
            0xFF06            => {
                self.tma = v;

                // Another timer quirk. If TMA is written to in the same cycle that we refreshed TIMA, then
                // we actually write this new value into TIMA immediately.
                if self.tima_new {
                    self.tima = self.tma;
                }
            }
            0xFF07            => { self.set_tac(v & 0x7) }
            0xFF0F            => { self.if_ = v & 0x1F }

            0xFF00            => { self.write_joypad(v) }

            // Sound
            0xFF10            => { self.sound.write_nr10(v) }
            0xFF11            => { self.sound.write_nr11(v) }
            0xFF12            => { self.sound.write_nr12(v) }
            0xFF13            => { self.sound.write_nr13(v) }
            0xFF14            => { self.sound.write_nr14(v) }
            0xFF16            => { self.sound.write_nr21(v) }
            0xFF17            => { self.sound.write_nr22(v) }
            0xFF18            => { self.sound.write_nr23(v) }
            0xFF19            => { self.sound.write_nr24(v) }
            0xFF1A            => { self.sound.write_nr30(v) }
            0xFF1B            => { self.sound.write_nr31(v) }
            0xFF1C            => { self.sound.write_nr32(v) }
            0xFF1D            => { self.sound.write_nr33(v) }
            0xFF1E            => { self.sound.write_nr34(v) }
            0xFF20            => { self.sound.write_nr41(v) }
            0xFF21            => { self.sound.write_nr42(v) }
            0xFF22            => { self.sound.write_nr43(v) }
            0xFF23            => { self.sound.write_nr44(v) }
            0xFF24            => { self.sound.write_nr50(v) }
            0xFF25            => { self.sound.write_nr51(v) }
            0xFF26            => { self.sound.write_nr52(v) }
            0xFF30 ... 0xFF3F => { self.sound.wave_write(addr - 0xFF30, v) }

            // PPU
            0xFF40            => { self.ppu.reg_lcdc_write(v) }
            0xFF41            => { self.ppu.reg_stat_write(v) }
            0xFF42            => { self.ppu.scy = v.into() }
            0xFF43            => { self.ppu.scx = v.into() }
            0xFF44            => { }                   // LY is readonly.
            0xFF45            => { self.ppu.lyc = v.into() }
            0xFF46            => { self.dma(v) }
            0xFF47            => { self.ppu.bgp.unpack(v) }
            0xFF48            => { self.ppu.obp0.unpack(v) }
            0xFF49            => { self.ppu.obp1.unpack(v) }
            0xFF50 if self.bootrom_enabled && v == 1 => {
                self.bootrom_enabled = false;
            }

            0xFF4A            => { self.ppu.wy = v.into() },
            0xFF4B            => { self.ppu.wx = v.into() },

            0xFF4D            => { }      // KEY1 for CGB.
            0xFF7F            => { }      // No idea what this is.
            0xFF80 ... 0xFFFE => { self.hram[(addr - 0xFF80) as usize] = v }
            0xFFFF            => { self.ie = v }

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

    fn read_joypad(&self) -> u8 {
        let mut v = 0xCF;

        if self.joypad_btn {
            v ^= 0x20;
            if self.joypad.a      { v ^= 1 }
            if self.joypad.b      { v ^= 2 }
            if self.joypad.select { v ^= 4 }
            if self.joypad.start  { v ^= 8 }
        } else if self.joypad_dir {
            v ^= 0x10;
            if self.joypad.right  { v ^= 1 }
            if self.joypad.left   { v ^= 2 }
            if self.joypad.up     { v ^= 4 }
            if self.joypad.down   { v ^= 8 }
        }

        v
    }

    fn write_div(&mut self) {
        // Writes to DIV are sort-of ignored. They just reset DIV to 0.
        // HOWEVER, there's also a side effect to note ...
        let old_div = self.div;
        self.div = 0;

        // ... If the timer is currently running, then depending on the frequency and which bits in DIV just went
        // from 1 to 0, we may trigger another timer increment. This is because of how the actual timer
        // circuitry works.
        // See http://gbdev.gg8.se/wiki/articles/Timer_Obscure_Behaviour
        if self.timer_enabled {
            let should_inc = old_div & match self.timer_freq {
                1024 => 0b0000_0010_0000_0000,
                16   => 0b0000_0000_0000_1000,
                64   => 0b0000_0000_0010_0000,
                256  => 0b0000_0000_1000_0000,
                _ => unreachable!("All timer frequencies covered")
            } > 0;
            if should_inc {
                self.tima = self.tima.wrapping_add(1);
                if self.tima == 0 {
                    self.tima_overflow = true;
                }
            }
        }
    }

    fn write_joypad(&mut self, v: u8) {
        self.joypad_btn = v & 0x20 == 0;
        self.joypad_dir = v & 0x10 == 0;
    }

    fn read_sc(&self) -> u8 {
        0x7E  // Unused bits are set to 1
            | if self.serial_transfer       { 0b1000_0000 } else { 0 }
            | if self.serial_internal_clock { 0b0000_0001 } else { 0 }
    }

    fn write_sc(&mut self, v: u8) {
        self.serial_transfer       = v & 0b1000_0000 > 0;
        self.serial_internal_clock = v & 0b0000_0001 > 0;
    }

    fn dma(&mut self, v: u8) {
        self.dma_reg = v;

        if !self.ppu.dma_ok() {
            return;
        }

        self.dma_request = true;
        self.dma_active = false;
        self.dma_idx = 0;
        self.dma_from = ((v & 0xFF) as u16) << 8;
    }

    // Fetches next byte from PC and increments PC.
    fn fetch8(&mut self) -> u8 {
        let addr = self.pc;
        self.pc = self.pc.wrapping_add(1);
        self.mem_read8(addr)
    }

    // Fetches next short from PC and increments PC by 2.
    fn fetch16(&mut self) -> u16 {
        let addr = self.pc;
        self.pc = self.pc.wrapping_add(2);
        self.mem_read16(addr)
    }

    // Decodes the next instruction located at PC.
    fn decode(&mut self) -> Inst {
        match self.fetch8() {
            0x00 => Inst::NOP,
            0x01 => Inst::LD16(Operand16::Reg(BC), Operand16::Imm(self.fetch16())),
            0x02 => Inst::LD8(Operand8::Addr(BC), Operand8::Reg(A)),
            0x03 => Inst::INC16(BC),
            0x04 => Inst::INC8(Operand8::Reg(B)),
            0x05 => Inst::DEC8(Operand8::Reg(B)),
            0x06 => Inst::LD8(Operand8::Reg(B), Operand8::Imm(self.fetch8())),
            0x07 => Inst::RLCA,
            0x08 => Inst::LD16(Operand16::ImmAddr(self.fetch16()), Operand16::Reg(SP)),
            0x09 => Inst::ADD16(BC),
            0x0A => Inst::LD8(Operand8::Reg(A), Operand8::Addr(BC)),
            0x0B => Inst::DEC16(BC),
            0x0C => Inst::INC8(Operand8::Reg(C)),
            0x0D => Inst::DEC8(Operand8::Reg(C)),
            0x0E => Inst::LD8(Operand8::Reg(C), Operand8::Imm(self.fetch8())),
            0x0F => Inst::RRCA,
            0x10 => Inst::STOP,
            0x11 => Inst::LD16(Operand16::Reg(DE), Operand16::Imm(self.fetch16())),
            0x12 => Inst::LD8(Operand8::Addr(DE), Operand8::Reg(A)),
            0x13 => Inst::INC16(DE),
            0x14 => Inst::INC8(Operand8::Reg(D)),
            0x15 => Inst::DEC8(Operand8::Reg(D)),
            0x16 => Inst::LD8(Operand8::Reg(D), Operand8::Imm(self.fetch8())), 
            0x17 => Inst::RLA,
            0x18 => Inst::JR(None, self.fetch8()),
            0x19 => Inst::ADD16(DE),
            0x1A => Inst::LD8(Operand8::Reg(A), Operand8::Addr(DE)),
            0x1B => Inst::DEC16(DE),
            0x1C => Inst::INC8(Operand8::Reg(E)),
            0x1D => Inst::DEC8(Operand8::Reg(E)),
            0x1E => Inst::LD8(Operand8::Reg(E), Operand8::Imm(self.fetch8())),
            0x1F => Inst::RRA,
            0x20 => Inst::JR(Some(FlagCondition::NZ), self.fetch8()),
            0x21 => Inst::LD16(Operand16::Reg(HL), Operand16::Imm(self.fetch16())),
            0x22 => Inst::LD8(Operand8::AddrInc(HL), Operand8::Reg(A)),
            0x23 => Inst::INC16(HL),
            0x24 => Inst::INC8(Operand8::Reg(H)),
            0x25 => Inst::DEC8(Operand8::Reg(H)),
            0x26 => Inst::LD8(Operand8::Reg(H), Operand8::Imm(self.fetch8())),
            0x27 => Inst::DAA,
            0x28 => Inst::JR(Some(FlagCondition::Z), self.fetch8()),
            0x29 => Inst::ADD16(HL),
            0x2A => Inst::LD8(Operand8::Reg(A), Operand8::AddrInc(HL)),
            0x2B => Inst::DEC16(HL),
            0x2C => Inst::INC8(Operand8::Reg(L)),
            0x2D => Inst::DEC8(Operand8::Reg(L)),
            0x2E => Inst::LD8(Operand8::Reg(L), Operand8::Imm(self.fetch8())),
            0x2F => Inst::CPL,
            0x30 => Inst::JR(Some(FlagCondition::NC), self.fetch8()),
            0x31 => Inst::LD16(Operand16::Reg(SP), Operand16::Imm(self.fetch16())),
            0x32 => Inst::LD8(Operand8::AddrDec(HL), Operand8::Reg(A)),
            0x33 => Inst::INC16(SP),
            0x34 => Inst::INC8(Operand8::Addr(HL)),
            0x35 => Inst::DEC8(Operand8::Addr(HL)),
            0x36 => Inst::LD8(Operand8::Addr(HL), Operand8::Imm(self.fetch8())),
            0x37 => Inst::SCF,
            0x38 => Inst::JR(Some(FlagCondition::C), self.fetch8()),
            0x39 => Inst::ADD16(SP),
            0x3A => Inst::LD8(Operand8::Reg(A), Operand8::AddrDec(HL)),
            0x3B => Inst::DEC16(SP),
            0x3C => Inst::INC8(Operand8::Reg(A)),
            0x3D => Inst::DEC8(Operand8::Reg(A)),
            0x3E => Inst::LD8(Operand8::Reg(A), Operand8::Imm(self.fetch8())),
            0x3F => Inst::CCF,
            0x40 => {
                if cfg!(test) {
                    self.mooneye_breakpoint = true;
                }
                Inst::LD8(Operand8::Reg(B), Operand8::Reg(B))
            },
            0x41 => Inst::LD8(Operand8::Reg(B), Operand8::Reg(C)),
            0x42 => Inst::LD8(Operand8::Reg(B), Operand8::Reg(D)),
            0x43 => Inst::LD8(Operand8::Reg(B), Operand8::Reg(E)),
            0x44 => Inst::LD8(Operand8::Reg(B), Operand8::Reg(H)),
            0x45 => Inst::LD8(Operand8::Reg(B), Operand8::Reg(L)),
            0x46 => Inst::LD8(Operand8::Reg(B), Operand8::Addr(HL)),
            0x47 => Inst::LD8(Operand8::Reg(B), Operand8::Reg(A)),
            0x48 => Inst::LD8(Operand8::Reg(C), Operand8::Reg(B)),
            0x49 => Inst::LD8(Operand8::Reg(C), Operand8::Reg(C)),
            0x4A => Inst::LD8(Operand8::Reg(C), Operand8::Reg(D)),
            0x4B => Inst::LD8(Operand8::Reg(C), Operand8::Reg(E)),
            0x4C => Inst::LD8(Operand8::Reg(C), Operand8::Reg(H)),
            0x4D => Inst::LD8(Operand8::Reg(C), Operand8::Reg(L)),
            0x4E => Inst::LD8(Operand8::Reg(C), Operand8::Addr(HL)),
            0x4F => Inst::LD8(Operand8::Reg(C), Operand8::Reg(A)),
            0x50 => Inst::LD8(Operand8::Reg(D), Operand8::Reg(B)),
            0x51 => Inst::LD8(Operand8::Reg(D), Operand8::Reg(C)),
            0x52 => Inst::LD8(Operand8::Reg(D), Operand8::Reg(D)),
            0x53 => Inst::LD8(Operand8::Reg(D), Operand8::Reg(E)),
            0x54 => Inst::LD8(Operand8::Reg(D), Operand8::Reg(H)),
            0x55 => Inst::LD8(Operand8::Reg(D), Operand8::Reg(L)),
            0x56 => Inst::LD8(Operand8::Reg(D), Operand8::Addr(HL)),
            0x57 => Inst::LD8(Operand8::Reg(D), Operand8::Reg(A)),
            0x58 => Inst::LD8(Operand8::Reg(E), Operand8::Reg(B)),
            0x59 => Inst::LD8(Operand8::Reg(E), Operand8::Reg(C)),
            0x5A => Inst::LD8(Operand8::Reg(E), Operand8::Reg(D)),
            0x5B => Inst::LD8(Operand8::Reg(E), Operand8::Reg(E)),
            0x5C => Inst::LD8(Operand8::Reg(E), Operand8::Reg(H)),
            0x5D => Inst::LD8(Operand8::Reg(E), Operand8::Reg(L)),
            0x5E => Inst::LD8(Operand8::Reg(E), Operand8::Addr(HL)),
            0x5F => Inst::LD8(Operand8::Reg(E), Operand8::Reg(A)),
            0x60 => Inst::LD8(Operand8::Reg(H), Operand8::Reg(B)),
            0x61 => Inst::LD8(Operand8::Reg(H), Operand8::Reg(C)),
            0x62 => Inst::LD8(Operand8::Reg(H), Operand8::Reg(D)),
            0x63 => Inst::LD8(Operand8::Reg(H), Operand8::Reg(E)),
            0x64 => Inst::LD8(Operand8::Reg(H), Operand8::Reg(H)),
            0x65 => Inst::LD8(Operand8::Reg(H), Operand8::Reg(L)),
            0x66 => Inst::LD8(Operand8::Reg(H), Operand8::Addr(HL)),
            0x67 => Inst::LD8(Operand8::Reg(H), Operand8::Reg(A)),
            0x68 => Inst::LD8(Operand8::Reg(L), Operand8::Reg(B)),
            0x69 => Inst::LD8(Operand8::Reg(L), Operand8::Reg(C)),
            0x6A => Inst::LD8(Operand8::Reg(L), Operand8::Reg(D)),
            0x6B => Inst::LD8(Operand8::Reg(L), Operand8::Reg(E)),
            0x6C => Inst::LD8(Operand8::Reg(L), Operand8::Reg(H)),
            0x6D => Inst::LD8(Operand8::Reg(L), Operand8::Reg(L)),
            0x6E => Inst::LD8(Operand8::Reg(L), Operand8::Addr(HL)),
            0x6F => Inst::LD8(Operand8::Reg(L), Operand8::Reg(A)),
            0x70 => Inst::LD8(Operand8::Addr(HL), Operand8::Reg(B)),
            0x71 => Inst::LD8(Operand8::Addr(HL), Operand8::Reg(C)),
            0x72 => Inst::LD8(Operand8::Addr(HL), Operand8::Reg(D)),
            0x73 => Inst::LD8(Operand8::Addr(HL), Operand8::Reg(E)),
            0x74 => Inst::LD8(Operand8::Addr(HL), Operand8::Reg(H)),
            0x75 => Inst::LD8(Operand8::Addr(HL), Operand8::Reg(L)),
            0x76 => Inst::HALT,
            0x77 => Inst::LD8(Operand8::Addr(HL), Operand8::Reg(A)),
            0x78 => Inst::LD8(Operand8::Reg(A), Operand8::Reg(B)),
            0x79 => Inst::LD8(Operand8::Reg(A), Operand8::Reg(C)),
            0x7A => Inst::LD8(Operand8::Reg(A), Operand8::Reg(D)),
            0x7B => Inst::LD8(Operand8::Reg(A), Operand8::Reg(E)),
            0x7C => Inst::LD8(Operand8::Reg(A), Operand8::Reg(H)),
            0x7D => Inst::LD8(Operand8::Reg(A), Operand8::Reg(L)),
            0x7E => Inst::LD8(Operand8::Reg(A), Operand8::Addr(HL)),
            0x7F => Inst::LD8(Operand8::Reg(A), Operand8::Reg(A)),
            0x80 => Inst::ADD(Operand8::Reg(B)),
            0x81 => Inst::ADD(Operand8::Reg(C)),
            0x82 => Inst::ADD(Operand8::Reg(D)),
            0x83 => Inst::ADD(Operand8::Reg(E)),
            0x84 => Inst::ADD(Operand8::Reg(H)),
            0x85 => Inst::ADD(Operand8::Reg(L)),
            0x86 => Inst::ADD(Operand8::Addr(HL)),
            0x87 => Inst::ADD(Operand8::Reg(A)),
            0x88 => Inst::ADC(Operand8::Reg(B)),
            0x89 => Inst::ADC(Operand8::Reg(C)),
            0x8A => Inst::ADC(Operand8::Reg(D)),
            0x8B => Inst::ADC(Operand8::Reg(E)),
            0x8C => Inst::ADC(Operand8::Reg(H)),
            0x8D => Inst::ADC(Operand8::Reg(L)),
            0x8E => Inst::ADC(Operand8::Addr(HL)),
            0x8F => Inst::ADC(Operand8::Reg(A)),
            0x90 => Inst::SUB(Operand8::Reg(B)),
            0x91 => Inst::SUB(Operand8::Reg(C)),
            0x92 => Inst::SUB(Operand8::Reg(D)),
            0x93 => Inst::SUB(Operand8::Reg(E)),
            0x94 => Inst::SUB(Operand8::Reg(H)),
            0x95 => Inst::SUB(Operand8::Reg(L)),
            0x96 => Inst::SUB(Operand8::Addr(HL)),
            0x97 => Inst::SUB(Operand8::Reg(A)),
            0x98 => Inst::SBC(Operand8::Reg(B)),
            0x99 => Inst::SBC(Operand8::Reg(C)),
            0x9A => Inst::SBC(Operand8::Reg(D)),
            0x9B => Inst::SBC(Operand8::Reg(E)),
            0x9C => Inst::SBC(Operand8::Reg(H)),
            0x9D => Inst::SBC(Operand8::Reg(L)),
            0x9E => Inst::SBC(Operand8::Addr(HL)),
            0x9F => Inst::SBC(Operand8::Reg(A)),
            0xA0 => Inst::AND(Operand8::Reg(B)),
            0xA1 => Inst::AND(Operand8::Reg(C)),
            0xA2 => Inst::AND(Operand8::Reg(D)),
            0xA3 => Inst::AND(Operand8::Reg(E)),
            0xA4 => Inst::AND(Operand8::Reg(H)),
            0xA5 => Inst::AND(Operand8::Reg(L)),
            0xA6 => Inst::AND(Operand8::Addr(HL)),
            0xA7 => Inst::AND(Operand8::Reg(A)),
            0xA8 => Inst::XOR(Operand8::Reg(B)),
            0xA9 => Inst::XOR(Operand8::Reg(C)),
            0xAA => Inst::XOR(Operand8::Reg(D)),
            0xAB => Inst::XOR(Operand8::Reg(E)),
            0xAC => Inst::XOR(Operand8::Reg(H)),
            0xAD => Inst::XOR(Operand8::Reg(L)),
            0xAE => Inst::XOR(Operand8::Addr(HL)),
            0xAF => Inst::XOR(Operand8::Reg(A)),
            0xB0 => Inst::OR(Operand8::Reg(B)),
            0xB1 => Inst::OR(Operand8::Reg(C)),
            0xB2 => Inst::OR(Operand8::Reg(D)),
            0xB3 => Inst::OR(Operand8::Reg(E)),
            0xB4 => Inst::OR(Operand8::Reg(H)),
            0xB5 => Inst::OR(Operand8::Reg(L)),
            0xB6 => Inst::OR(Operand8::Addr(HL)),
            0xB7 => Inst::OR(Operand8::Reg(A)),
            0xB8 => Inst::CP(Operand8::Reg(B)),
            0xB9 => Inst::CP(Operand8::Reg(C)),
            0xBA => Inst::CP(Operand8::Reg(D)),
            0xBB => Inst::CP(Operand8::Reg(E)),
            0xBC => Inst::CP(Operand8::Reg(H)),
            0xBD => Inst::CP(Operand8::Reg(L)),
            0xBE => Inst::CP(Operand8::Addr(HL)),
            0xBF => Inst::CP(Operand8::Reg(A)),
            0xC0 => Inst::RET(Some(FlagCondition::NZ)),
            0xC1 => Inst::POP(BC),
            0xC2 => Inst::JP(Some(FlagCondition::NZ), Operand16::Imm(self.fetch16())),
            0xC3 => Inst::JP(None, Operand16::Imm(self.fetch16())),
            0xC4 => Inst::CALL(Some(FlagCondition::NZ), self.fetch16()),
            0xC5 => Inst::PUSH(BC),
            0xC6 => Inst::ADD(Operand8::Imm(self.fetch8())),
            0xC7 => Inst::RST(0x00),
            0xC8 => Inst::RET(Some(FlagCondition::Z)),
            0xC9 => Inst::RET(None),
            0xCA => Inst::JP(Some(FlagCondition::Z), Operand16::Imm(self.fetch16())),
            0xCB => self.decode_extended(),
            0xCC => Inst::CALL(Some(FlagCondition::Z), self.fetch16()),
            0xCD => Inst::CALL(None, self.fetch16()),
            0xCE => Inst::ADC(Operand8::Imm(self.fetch8())),
            0xCF => Inst::RST(0x08),
            0xD0 => Inst::RET(Some(FlagCondition::NC)),
            0xD1 => Inst::POP(DE),
            0xD2 => Inst::JP(Some(FlagCondition::NC), Operand16::Imm(self.fetch16())),
            0xD4 => Inst::CALL(Some(FlagCondition::NC), self.fetch16()),
            0xD5 => Inst::PUSH(DE),
            0xD6 => Inst::SUB(Operand8::Imm(self.fetch8())),
            0xD7 => Inst::RST(0x10),
            0xD8 => Inst::RET(Some(FlagCondition::C)),
            0xD9 => Inst::RETI,
            0xDA => Inst::JP(Some(FlagCondition::C), Operand16::Imm(self.fetch16())),
            0xDC => Inst::CALL(Some(FlagCondition::C), self.fetch16()),
            0xDE => Inst::SBC(Operand8::Imm(self.fetch8())),
            0xDF => Inst::RST(0x18),
            0xE0 => Inst::LD8(Operand8::ImmAddrHigh(self.fetch8()), Operand8::Reg(A)),
            0xE1 => Inst::POP(HL),
            0xE2 => Inst::LD8(Operand8::AddrHigh(C), Operand8::Reg(A)),
            0xE5 => Inst::PUSH(HL),
            0xE6 => Inst::AND(Operand8::Imm(self.fetch8())),
            0xE7 => Inst::RST(0x20),
            0xE8 => Inst::ADD_SP_r8(self.fetch8() as i8),
            0xE9 => Inst::JP(None, Operand16::Reg(HL)),
            0xEA => Inst::LD8(Operand8::ImmAddr(self.fetch16()), Operand8::Reg(A)),
            0xEE => Inst::XOR(Operand8::Imm(self.fetch8())),
            0xEF => Inst::RST(0x28),
            0xF0 => Inst::LD8(Operand8::Reg(A), Operand8::ImmAddrHigh(self.fetch8())),
            0xF1 => Inst::POP(AF),
            0xF2 => Inst::LD8(Operand8::Reg(A), Operand8::AddrHigh(C)),
            0xF3 => Inst::DI,
            0xF5 => Inst::PUSH(AF),
            0xF6 => Inst::OR(Operand8::Imm(self.fetch8())),
            0xF7 => Inst::RST(0x30),
            0xF8 => Inst::LD_HL_SP(self.fetch8() as i8),
            0xF9 => Inst::LD16(Operand16::Reg(SP), Operand16::Reg(HL)),
            0xFA => Inst::LD8(Operand8::Reg(A), Operand8::ImmAddr(self.fetch16())),
            0xFB => Inst::EI,
            0xFE => Inst::CP(Operand8::Imm(self.fetch8())),
            0xFF => Inst::RST(0x38),

            n => {
                self.core_panic(format!("Unexpected opcode 0x{:X} encountered", n));
            }
        }
    }

    // Decodes extended instruction following 0xCB instruction.
    fn decode_extended(&mut self) -> Inst {
        match self.fetch8() {
            0x00 => Inst::RLC(Operand8::Reg(B)),
            0x01 => Inst::RLC(Operand8::Reg(C)),
            0x02 => Inst::RLC(Operand8::Reg(D)),
            0x03 => Inst::RLC(Operand8::Reg(E)),
            0x04 => Inst::RLC(Operand8::Reg(H)),
            0x05 => Inst::RLC(Operand8::Reg(L)),
            0x06 => Inst::RLC(Operand8::Addr(HL)),
            0x07 => Inst::RLC(Operand8::Reg(A)),
            0x08 => Inst::RRC(Operand8::Reg(B)),
            0x09 => Inst::RRC(Operand8::Reg(C)),
            0x0A => Inst::RRC(Operand8::Reg(D)),
            0x0B => Inst::RRC(Operand8::Reg(E)),
            0x0C => Inst::RRC(Operand8::Reg(H)),
            0x0D => Inst::RRC(Operand8::Reg(L)),
            0x0E => Inst::RRC(Operand8::Addr(HL)),
            0x0F => Inst::RRC(Operand8::Reg(A)),
            0x10 => Inst::RL(Operand8::Reg(B)),
            0x11 => Inst::RL(Operand8::Reg(C)),
            0x12 => Inst::RL(Operand8::Reg(D)),
            0x13 => Inst::RL(Operand8::Reg(E)),
            0x14 => Inst::RL(Operand8::Reg(H)),
            0x15 => Inst::RL(Operand8::Reg(L)),
            0x16 => Inst::RL(Operand8::Addr(HL)),
            0x17 => Inst::RL(Operand8::Reg(A)),
            0x18 => Inst::RR(Operand8::Reg(B)),
            0x19 => Inst::RR(Operand8::Reg(C)),
            0x1A => Inst::RR(Operand8::Reg(D)),
            0x1B => Inst::RR(Operand8::Reg(E)),
            0x1C => Inst::RR(Operand8::Reg(H)),
            0x1D => Inst::RR(Operand8::Reg(L)),
            0x1E => Inst::RR(Operand8::Addr(HL)),
            0x1F => Inst::RR(Operand8::Reg(A)),
            0x20 => Inst::SLA(Operand8::Reg(B)),
            0x21 => Inst::SLA(Operand8::Reg(C)),
            0x22 => Inst::SLA(Operand8::Reg(D)),
            0x23 => Inst::SLA(Operand8::Reg(E)),
            0x24 => Inst::SLA(Operand8::Reg(H)),
            0x25 => Inst::SLA(Operand8::Reg(L)),
            0x26 => Inst::SLA(Operand8::Addr(HL)),
            0x27 => Inst::SLA(Operand8::Reg(A)),
            0x28 => Inst::SRA(Operand8::Reg(B)),
            0x29 => Inst::SRA(Operand8::Reg(C)),
            0x2A => Inst::SRA(Operand8::Reg(D)),
            0x2B => Inst::SRA(Operand8::Reg(E)),
            0x2C => Inst::SRA(Operand8::Reg(H)),
            0x2D => Inst::SRA(Operand8::Reg(L)),
            0x2E => Inst::SRA(Operand8::Addr(HL)),
            0x2F => Inst::SRA(Operand8::Reg(A)),
            0x30 => Inst::SWAP(Operand8::Reg(B)),
            0x31 => Inst::SWAP(Operand8::Reg(C)),
            0x32 => Inst::SWAP(Operand8::Reg(D)),
            0x33 => Inst::SWAP(Operand8::Reg(E)),
            0x34 => Inst::SWAP(Operand8::Reg(H)),
            0x35 => Inst::SWAP(Operand8::Reg(L)),
            0x36 => Inst::SWAP(Operand8::Addr(HL)),
            0x37 => Inst::SWAP(Operand8::Reg(A)),
            0x38 => Inst::SRL(Operand8::Reg(B)),
            0x39 => Inst::SRL(Operand8::Reg(C)),
            0x3A => Inst::SRL(Operand8::Reg(D)),
            0x3B => Inst::SRL(Operand8::Reg(E)),
            0x3C => Inst::SRL(Operand8::Reg(H)),
            0x3D => Inst::SRL(Operand8::Reg(L)),
            0x3E => Inst::SRL(Operand8::Addr(HL)),
            0x3F => Inst::SRL(Operand8::Reg(A)),
            0x40 => Inst::BIT(0, Operand8::Reg(B)),
            0x41 => Inst::BIT(0, Operand8::Reg(C)),
            0x42 => Inst::BIT(0, Operand8::Reg(D)),
            0x43 => Inst::BIT(0, Operand8::Reg(E)),
            0x44 => Inst::BIT(0, Operand8::Reg(H)),
            0x45 => Inst::BIT(0, Operand8::Reg(L)),
            0x46 => Inst::BIT(0, Operand8::Addr(HL)),
            0x47 => Inst::BIT(0, Operand8::Reg(A)),
            0x48 => Inst::BIT(1, Operand8::Reg(B)),
            0x49 => Inst::BIT(1, Operand8::Reg(C)),
            0x4A => Inst::BIT(1, Operand8::Reg(D)),
            0x4B => Inst::BIT(1, Operand8::Reg(E)),
            0x4C => Inst::BIT(1, Operand8::Reg(H)),
            0x4D => Inst::BIT(1, Operand8::Reg(L)),
            0x4E => Inst::BIT(1, Operand8::Addr(HL)),
            0x4F => Inst::BIT(1, Operand8::Reg(A)),
            0x50 => Inst::BIT(2, Operand8::Reg(B)),
            0x51 => Inst::BIT(2, Operand8::Reg(C)),
            0x52 => Inst::BIT(2, Operand8::Reg(D)),
            0x53 => Inst::BIT(2, Operand8::Reg(E)),
            0x54 => Inst::BIT(2, Operand8::Reg(H)),
            0x55 => Inst::BIT(2, Operand8::Reg(L)),
            0x56 => Inst::BIT(2, Operand8::Addr(HL)),
            0x57 => Inst::BIT(2, Operand8::Reg(A)),
            0x58 => Inst::BIT(3, Operand8::Reg(B)),
            0x59 => Inst::BIT(3, Operand8::Reg(C)),
            0x5A => Inst::BIT(3, Operand8::Reg(D)),
            0x5B => Inst::BIT(3, Operand8::Reg(E)),
            0x5C => Inst::BIT(3, Operand8::Reg(H)),
            0x5D => Inst::BIT(3, Operand8::Reg(L)),
            0x5E => Inst::BIT(3, Operand8::Addr(HL)),
            0x5F => Inst::BIT(3, Operand8::Reg(A)),
            0x60 => Inst::BIT(4, Operand8::Reg(B)),
            0x61 => Inst::BIT(4, Operand8::Reg(C)),
            0x62 => Inst::BIT(4, Operand8::Reg(D)),
            0x63 => Inst::BIT(4, Operand8::Reg(E)),
            0x64 => Inst::BIT(4, Operand8::Reg(H)),
            0x65 => Inst::BIT(4, Operand8::Reg(L)),
            0x66 => Inst::BIT(4, Operand8::Addr(HL)),
            0x67 => Inst::BIT(4, Operand8::Reg(A)),
            0x68 => Inst::BIT(5, Operand8::Reg(B)),
            0x69 => Inst::BIT(5, Operand8::Reg(C)),
            0x6A => Inst::BIT(5, Operand8::Reg(D)),
            0x6B => Inst::BIT(5, Operand8::Reg(E)),
            0x6C => Inst::BIT(5, Operand8::Reg(H)),
            0x6D => Inst::BIT(5, Operand8::Reg(L)),
            0x6E => Inst::BIT(5, Operand8::Addr(HL)),
            0x6F => Inst::BIT(5, Operand8::Reg(A)),
            0x70 => Inst::BIT(6, Operand8::Reg(B)),
            0x71 => Inst::BIT(6, Operand8::Reg(C)),
            0x72 => Inst::BIT(6, Operand8::Reg(D)),
            0x73 => Inst::BIT(6, Operand8::Reg(E)),
            0x74 => Inst::BIT(6, Operand8::Reg(H)),
            0x75 => Inst::BIT(6, Operand8::Reg(L)),
            0x76 => Inst::BIT(6, Operand8::Addr(HL)),
            0x77 => Inst::BIT(6, Operand8::Reg(A)),
            0x78 => Inst::BIT(7, Operand8::Reg(B)),
            0x79 => Inst::BIT(7, Operand8::Reg(C)),
            0x7A => Inst::BIT(7, Operand8::Reg(D)),
            0x7B => Inst::BIT(7, Operand8::Reg(E)),
            0x7C => Inst::BIT(7, Operand8::Reg(H)),
            0x7D => Inst::BIT(7, Operand8::Reg(L)),
            0x7E => Inst::BIT(7, Operand8::Addr(HL)),
            0x7F => Inst::BIT(7, Operand8::Reg(A)),
            0x80 => Inst::RES(0, Operand8::Reg(B)),
            0x81 => Inst::RES(0, Operand8::Reg(C)),
            0x82 => Inst::RES(0, Operand8::Reg(D)),
            0x83 => Inst::RES(0, Operand8::Reg(E)),
            0x84 => Inst::RES(0, Operand8::Reg(H)),
            0x85 => Inst::RES(0, Operand8::Reg(L)),
            0x86 => Inst::RES(0, Operand8::Addr(HL)),
            0x87 => Inst::RES(0, Operand8::Reg(A)),
            0x88 => Inst::RES(1, Operand8::Reg(B)),
            0x89 => Inst::RES(1, Operand8::Reg(C)),
            0x8A => Inst::RES(1, Operand8::Reg(D)),
            0x8B => Inst::RES(1, Operand8::Reg(E)),
            0x8C => Inst::RES(1, Operand8::Reg(H)),
            0x8D => Inst::RES(1, Operand8::Reg(L)),
            0x8E => Inst::RES(1, Operand8::Addr(HL)),
            0x8F => Inst::RES(1, Operand8::Reg(A)),
            0x90 => Inst::RES(2, Operand8::Reg(B)),
            0x91 => Inst::RES(2, Operand8::Reg(C)),
            0x92 => Inst::RES(2, Operand8::Reg(D)),
            0x93 => Inst::RES(2, Operand8::Reg(E)),
            0x94 => Inst::RES(2, Operand8::Reg(H)),
            0x95 => Inst::RES(2, Operand8::Reg(L)),
            0x96 => Inst::RES(2, Operand8::Addr(HL)),
            0x97 => Inst::RES(2, Operand8::Reg(A)),
            0x98 => Inst::RES(3, Operand8::Reg(B)),
            0x99 => Inst::RES(3, Operand8::Reg(C)),
            0x9A => Inst::RES(3, Operand8::Reg(D)),
            0x9B => Inst::RES(3, Operand8::Reg(E)),
            0x9C => Inst::RES(3, Operand8::Reg(H)),
            0x9D => Inst::RES(3, Operand8::Reg(L)),
            0x9E => Inst::RES(3, Operand8::Addr(HL)),
            0x9F => Inst::RES(3, Operand8::Reg(A)),
            0xA0 => Inst::RES(4, Operand8::Reg(B)),
            0xA1 => Inst::RES(4, Operand8::Reg(C)),
            0xA2 => Inst::RES(4, Operand8::Reg(D)),
            0xA3 => Inst::RES(4, Operand8::Reg(E)),
            0xA4 => Inst::RES(4, Operand8::Reg(H)),
            0xA5 => Inst::RES(4, Operand8::Reg(L)),
            0xA6 => Inst::RES(4, Operand8::Addr(HL)),
            0xA7 => Inst::RES(4, Operand8::Reg(A)),
            0xA8 => Inst::RES(5, Operand8::Reg(B)),
            0xA9 => Inst::RES(5, Operand8::Reg(C)),
            0xAA => Inst::RES(5, Operand8::Reg(D)),
            0xAB => Inst::RES(5, Operand8::Reg(E)),
            0xAC => Inst::RES(5, Operand8::Reg(H)),
            0xAD => Inst::RES(5, Operand8::Reg(L)),
            0xAE => Inst::RES(5, Operand8::Addr(HL)),
            0xAF => Inst::RES(5, Operand8::Reg(A)),
            0xB0 => Inst::RES(6, Operand8::Reg(B)),
            0xB1 => Inst::RES(6, Operand8::Reg(C)),
            0xB2 => Inst::RES(6, Operand8::Reg(D)),
            0xB3 => Inst::RES(6, Operand8::Reg(E)),
            0xB4 => Inst::RES(6, Operand8::Reg(H)),
            0xB5 => Inst::RES(6, Operand8::Reg(L)),
            0xB6 => Inst::RES(6, Operand8::Addr(HL)),
            0xB7 => Inst::RES(6, Operand8::Reg(A)),
            0xB8 => Inst::RES(7, Operand8::Reg(B)),
            0xB9 => Inst::RES(7, Operand8::Reg(C)),
            0xBA => Inst::RES(7, Operand8::Reg(D)),
            0xBB => Inst::RES(7, Operand8::Reg(E)),
            0xBC => Inst::RES(7, Operand8::Reg(H)),
            0xBD => Inst::RES(7, Operand8::Reg(L)),
            0xBE => Inst::RES(7, Operand8::Addr(HL)),
            0xBF => Inst::RES(7, Operand8::Reg(A)),
            0xC0 => Inst::SET(0, Operand8::Reg(B)),
            0xC1 => Inst::SET(0, Operand8::Reg(C)),
            0xC2 => Inst::SET(0, Operand8::Reg(D)),
            0xC3 => Inst::SET(0, Operand8::Reg(E)),
            0xC4 => Inst::SET(0, Operand8::Reg(H)),
            0xC5 => Inst::SET(0, Operand8::Reg(L)),
            0xC6 => Inst::SET(0, Operand8::Addr(HL)),
            0xC7 => Inst::SET(0, Operand8::Reg(A)),
            0xC8 => Inst::SET(1, Operand8::Reg(B)),
            0xC9 => Inst::SET(1, Operand8::Reg(C)),
            0xCA => Inst::SET(1, Operand8::Reg(D)),
            0xCB => Inst::SET(1, Operand8::Reg(E)),
            0xCC => Inst::SET(1, Operand8::Reg(H)),
            0xCD => Inst::SET(1, Operand8::Reg(L)),
            0xCE => Inst::SET(1, Operand8::Addr(HL)),
            0xCF => Inst::SET(1, Operand8::Reg(A)),
            0xD0 => Inst::SET(2, Operand8::Reg(B)),
            0xD1 => Inst::SET(2, Operand8::Reg(C)),
            0xD2 => Inst::SET(2, Operand8::Reg(D)),
            0xD3 => Inst::SET(2, Operand8::Reg(E)),
            0xD4 => Inst::SET(2, Operand8::Reg(H)),
            0xD5 => Inst::SET(2, Operand8::Reg(L)),
            0xD6 => Inst::SET(2, Operand8::Addr(HL)),
            0xD7 => Inst::SET(2, Operand8::Reg(A)),
            0xD8 => Inst::SET(3, Operand8::Reg(B)),
            0xD9 => Inst::SET(3, Operand8::Reg(C)),
            0xDA => Inst::SET(3, Operand8::Reg(D)),
            0xDB => Inst::SET(3, Operand8::Reg(E)),
            0xDC => Inst::SET(3, Operand8::Reg(H)),
            0xDD => Inst::SET(3, Operand8::Reg(L)),
            0xDE => Inst::SET(3, Operand8::Addr(HL)),
            0xDF => Inst::SET(3, Operand8::Reg(A)),
            0xE0 => Inst::SET(4, Operand8::Reg(B)),
            0xE1 => Inst::SET(4, Operand8::Reg(C)),
            0xE2 => Inst::SET(4, Operand8::Reg(D)),
            0xE3 => Inst::SET(4, Operand8::Reg(E)),
            0xE4 => Inst::SET(4, Operand8::Reg(H)),
            0xE5 => Inst::SET(4, Operand8::Reg(L)),
            0xE6 => Inst::SET(4, Operand8::Addr(HL)),
            0xE7 => Inst::SET(4, Operand8::Reg(A)),
            0xE8 => Inst::SET(5, Operand8::Reg(B)),
            0xE9 => Inst::SET(5, Operand8::Reg(C)),
            0xEA => Inst::SET(5, Operand8::Reg(D)),
            0xEB => Inst::SET(5, Operand8::Reg(E)),
            0xEC => Inst::SET(5, Operand8::Reg(H)),
            0xED => Inst::SET(5, Operand8::Reg(L)),
            0xEE => Inst::SET(5, Operand8::Addr(HL)),
            0xEF => Inst::SET(5, Operand8::Reg(A)),
            0xF0 => Inst::SET(6, Operand8::Reg(B)),
            0xF1 => Inst::SET(6, Operand8::Reg(C)),
            0xF2 => Inst::SET(6, Operand8::Reg(D)),
            0xF3 => Inst::SET(6, Operand8::Reg(E)),
            0xF4 => Inst::SET(6, Operand8::Reg(H)),
            0xF5 => Inst::SET(6, Operand8::Reg(L)),
            0xF6 => Inst::SET(6, Operand8::Addr(HL)),
            0xF7 => Inst::SET(6, Operand8::Reg(A)),
            0xF8 => Inst::SET(7, Operand8::Reg(B)),
            0xF9 => Inst::SET(7, Operand8::Reg(C)),
            0xFA => Inst::SET(7, Operand8::Reg(D)),
            0xFB => Inst::SET(7, Operand8::Reg(E)),
            0xFC => Inst::SET(7, Operand8::Reg(H)),
            0xFD => Inst::SET(7, Operand8::Reg(L)),
            0xFE => Inst::SET(7, Operand8::Addr(HL)),
            0xFF => Inst::SET(7, Operand8::Reg(A)),

            n => {
                self.core_panic(format!("Unexpected extended opcode 0x{:X} encountered", n));
            }
        }
    }

    fn get_reg8(&self, reg: Reg8) -> u8 {
        match reg {
            A => self.a,
            B => self.b,
            C => self.c,
            D => self.d,
            E => self.e,
            H => self.h,
            L => self.l,
        }
    }

    fn set_reg8(&mut self, reg: Reg8, v: u8) {
        match reg {
            A => { self.a = v },
            B => { self.b = v },
            C => { self.c = v },
            D => { self.d = v },
            E => { self.e = v },
            H => { self.h = v },
            L => { self.l = v },
        };
    }

    fn get_reg16(&self, reg: Reg16) -> u16 {
        let (hi, lo) = match reg {
            AF => { (self.a, self.f.pack()) },
            BC => { (self.b, self.c) },
            DE => { (self.d, self.e) },
            HL => { (self.h, self.l) },
            SP => { return self.sp },
        };

        (hi as u16) << 8 | (lo as u16)
    }

    fn set_reg16(&mut self, reg: Reg16, v: u16) {
        let (hi, lo) = match reg {
            AF => {
                self.a = ((v & 0xFF00) >> 8) as u8;
                self.f.unpack(v as u8);
                return;
            },
            BC => { (&mut self.b, &mut self.c) },
            DE => { (&mut self.d, &mut self.e) },
            HL => { (&mut self.h, &mut self.l) },
            SP => { self.sp = v; return },
        };

        *hi = ((v & 0xFF00) >> 8) as u8;
        *lo = v as u8;
    }

    fn stack_push(&mut self, v: u16) {
        if self.sp >= 0xFE00 && self.sp <= 0xFEFF {
            self.ppu.maybe_trash_oam();
        }

        self.sp = self.sp.wrapping_sub(2);
        let sp = self.sp;
        self.mem_write16(sp, v);
    }

    fn stack_pop(&mut self) -> u16 {
        if self.sp >= 0xFDFF && self.sp <= 0xFEFE {
            self.ppu.maybe_trash_oam();
        }

        let addr = self.sp;
        let v = self.mem_read16(addr);
        self.sp = self.sp.wrapping_add(2);

        v
    }

    fn push_and_jump(&mut self, addr: u16) {
        let pc = self.pc;
        self.stack_push(pc);
        self.pc = addr;
    }

    // EI | DI
    // Flags = Z:- N:- H:- C:-
    fn set_ime(&mut self, state: bool) {
        if state {
            // Enabling interrupts happens on the next cycle ...
            self.ime_defer = true;
        } else {
            // ... However, disabling interrupts is immediate.
            self.ime = false;
        }
    }

    // STOP
    // Flags = Z:- N:- H:- C:-
    fn stop(&mut self) {
        // TODO: this should be more than a noop.
        self.pc = self.pc.wrapping_add(1);
    }

    // HALT
    // Flags = Z:- N:- H:- C:-
    fn halt(&mut self) {
        // TODO: pretty sure I need to check interrupt states here.
        self.halted = true;
    }

    // INC %r8 | INC (HL)
    // Flags = Z:* N:0 H:* C:-
    fn inc8(&mut self, o: Operand8) {
        let v = o.get(self).wrapping_add(1);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = v & 0x0F == 0;
        o.set(self, v);
    }

    // DEC %r8 | DEC (HL)
    // Flags = Z:* N:1 H:* C:-
    fn dec8(&mut self, o: Operand8) {
        let v = o.get(self).wrapping_sub(1);
        self.f.z = v == 0;
        self.f.n = true;
        self.f.h = v & 0x0F == 0x0F;
        o.set(self, v);
    }

    // INC rr
    // Flags = Z:- N:- H:- C:-
    fn inc16(&mut self, r: Reg16) {
        let v = self.get_reg16(r);
        if v >= 0xFE00 && v <= 0xFEFF {
            self.ppu.maybe_trash_oam();
        }
        self.set_reg16(r, v.wrapping_add(1));
        self.advance_clock();
    }

    // DEC rr
    // Flags = Z:- N:- H:- C:-
    fn dec16(&mut self, r: Reg16) {
        let v = self.get_reg16(r);
        if v >= 0xFE00 && v <= 0xFEFF {
            self.ppu.maybe_trash_oam();
        }
        self.set_reg16(r, v.wrapping_sub(1));
        self.advance_clock();
    }

    // DAA
    // Flags = Z:* N:- H:0 C:*
    // TODO: clean this dumpster fire up.
    fn daa(&mut self) {
        let mut carry = false;

        if !self.f.n {
            if self.f.c || self.a > 0x99 {
                self.a = self.a.wrapping_add(0x60);
                carry = true;
            }
            if self.f.h || (self.a & 0xF) > 9 {
                self.a = self.a.wrapping_add(0x06);
            }
        } else if self.f.c {
            carry = true;
            self.a = self.a.wrapping_add(if self.f.h { 0x9A } else { 0xA0 });
        } else if self.f.h {
            self.a = self.a.wrapping_add(0xFA);
        }

        self.f.z = self.a == 0;
        self.f.h = false;
        self.f.c = carry;
    }

    // CPL
    // Flags = Z:- N:1 H:1 C:-
    fn cpl(&mut self) {
        self.a = !self.a;
        self.f.n = true;
        self.f.h = true;
    }

    // CCF
    // Flags = Z:- N:0 H:0 C:*
    fn ccf(&mut self) {
        self.f.n = false;
        self.f.h = false;
        self.f.c = !self.f.c;
    }

    // SCF
    // Flags = Z:- N:0 H:0 C:1
    fn scf(&mut self) {
        self.f.n = false;
        self.f.h = false;
        self.f.c = true;
    }

    // ADD A, %r8 | ADD A, (HL) | ADD A, $d8
    // ADC A, %r8 | ADC A, (HL) | ADC A, $d8
    // Flags = Z:* N:0 H:* C:*
    fn add8(&mut self, o: Operand8, carry: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let old = self.a;
        let v = o.get(self);
        let new = old.wrapping_add(v).wrapping_add(carry);
        self.a = new;
        self.f.z = new == 0;
        self.f.n = false;
        self.f.h = (((old & 0xF) + (v & 0xF)) + carry) & 0x10 == 0x10;
        self.f.c = (old as u16) + (v as u16) + (carry as u16) > 0xFF;
    }

    // SUB    %r8 | SUB    (HL) | SUB    $d8
    // SBC A, %r8 | SBC A, (HL) | SBC A, $d8
    // Flags = Z:* N:1 H:* C:*
    fn sub8(&mut self, o: Operand8, carry: bool, store: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let a = self.a;
        let v = o.get(self);
        let new_a = a.wrapping_sub(v).wrapping_sub(carry);
        if store {
            self.a = new_a;
        }

        self.f.z = new_a == 0;
        self.f.n = true;
        self.f.h = ((a & 0xF) as u16) < ((v & 0xF) as u16) + (carry as u16);
        self.f.c = (a as u16) < (v as u16) + (carry as u16);
    }

    // ADD SP, r8
    // Flags = Z:0 N:0 H:* C:*
    fn add_sp_r8(&mut self, d: i8) {
        let d = d as i16 as u16;
        let sp = self.sp;

        self.sp = sp.wrapping_add(d as i16 as u16);

        self.advance_clock();
        self.advance_clock();

        self.f.z = false;
        self.f.n = false;
        self.f.h = ((sp & 0xF) + (d & 0xF)) & 0x10 > 0;
        self.f.c = ((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0;
    }

    // ADD HL, rr
    // Flags = Z:- N:0 H:* C:*
    fn add16(&mut self, r: Reg16) {
        let hl = self.get_reg16(HL);
        let v = self.get_reg16(r);
        let (new_hl, overflow) = hl.overflowing_add(v);
        self.set_reg16(HL, new_hl);

        self.advance_clock();

        self.f.n = false;
        self.f.h = ((hl & 0xFFF) + (v & 0xFFF)) & 0x1000 > 0;
        self.f.c = overflow;
    }

    // LD %r8, %r8
    // LD %r8, $d8
    // LD %r8, (%r16)
    // LD (%r16), %r8
    // LD (HL+), A | LD (HL-), A | LD A, (HL+) | LD A, (HL-)
    // Flags = Z:- N:- H:- C:-
    fn ld8(&mut self, l: Operand8, r: Operand8) {
        let v = r.get(self);
        l.set(self, v);
    }

    // LD %r16, $d16
    // LD (%a16), SP
    // Flags = Z:- N:- H:- C:-
    fn ld16(&mut self, l: Operand16, r: Operand16) {
        let v = r.get(self);
        l.set(self, v);
        
        // In the specific case of loading a 16bit reg into another 16bit reg, this consumes
        // another CPU cycle. I don't truly understand why, since in every instruction the cost
        // of reading/writing a 16bit reg appears to be free.
        if let Operand16::Reg(_) = l {
            if let Operand16::Reg(_) = r {
                self.advance_clock();
            }
        }
    }

    // LD HL, SP+r8
    // Flags = Z:0 N:0 H:* C:*
    fn ld_hl_sp(&mut self, d: i8) {
        let sp = self.sp;
        let d = d as i16 as u16;
        let v = sp.wrapping_add(d);
        self.set_reg16(HL, v);
        self.f.reset();
        self.f.h = (sp & 0xF) + (d & 0xF) & 0x10 > 0;
        self.f.c = (sp & 0xFF) + (d & 0xFF) & 0x100 > 0;
        self.advance_clock();
    }

    // AND %r8 | AND $d8 | AND (HL)
    // XOR %r8 | XOR $d8 | XOR (HL)
    // OR  %r8 | OR  $d8 | OR  (HL)
    // Flags = Z:* N:0 H:* C:0
    fn bitwise(&mut self, op: BitwiseOp, o: Operand8) {
        let a = self.a;
        let v = o.get(self);
        let mut hc = false;
        self.a = match op {
            BitwiseOp::AND => { hc = true; a & v },
            BitwiseOp::OR => a | v,
            BitwiseOp::XOR => a ^ v,
        };

        self.f.reset();
        self.f.z = self.a == 0;
        self.f.h = hc;
    }

    // BIT b, r
    // Flags = Z:* N:0 H:1 C:-
    fn bit(&mut self, b: u8, o: Operand8) {
        let v = o.get(self) & (1 << b);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = true;
    }

    // RES b, r
    // SET b, r
    // Flags = Z:- N:- H:- C:-
    fn setbit(&mut self, b: u8, o: Operand8, on: bool) {
        let v = o.get(self);
        o.set(self, if on { v | 1 << b } else { v & !(1 << b) });
    }

    // RR r
    // RRA
    // Flags = Z:* N:0 H:0 C:*
    fn rr(&mut self, o: Operand8, extended: bool) {
        let v = o.get(self);
        let msb = if self.f.c { 0x80 } else { 0 };
        let carry = v & 0x1 > 0;
        let v = o.set(self, v >> 1 | msb);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    // RL %r8
    // SL %r8
    // RLA
    // Flags = Z:* N:0 H:0 C:*
    fn rl(&mut self, o: Operand8, set_zero: bool, preserve_lsb: bool) {
        let v = o.get(self);
        let lsb = if preserve_lsb && self.f.c { 1 } else { 0 };
        let carry = v & 0x80 > 0;
        let v = o.set(self, v << 1 | lsb);
        self.f.reset();
        self.f.z = set_zero && v == 0;
        self.f.c = carry;
    }

    // RLC %r8
    // RLCA
    // Flags = Z:* N:0 H:0 C:*
    fn rlc(&mut self, o: Operand8, extended: bool) {
        let v = o.get(self);
        let carry = v & 0x80 > 0;
        let lsb = if carry { 1 } else { 0 };
        let v = o.set(self, v << 1 | lsb);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    // SRA r
    // Flags = Z:* N:0 H:0 C:*
    fn shift_right(&mut self, o: Operand8, preserve_msb: bool) {
        let v = o.get(self);
        let carry = v & 0x01 > 0;
        let preserve = if preserve_msb { v & 0x80 } else { 0 };
        let v = o.set(self, v >> 1 | preserve);
        self.f.reset();
        self.f.z = v == 0;
        self.f.c = carry;
    }

    // RRC r
    // Flags = Z:* N:0 H:0 C:*
    fn rrc(&mut self, o: Operand8, extended: bool) {
        let v = o.get(self);
        let carry = v & 0x1 > 0;
        let msb = if carry { 0x80 } else { 0 };
        let v = o.set(self, v >> 1 | msb);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    // SWAP r
    // Flags = Z:* N:0 H:0 C:0
    fn swap(&mut self, o: Operand8) {
        let v = o.get(self);
        let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
        o.set(self, v);
        self.f.reset();
        self.f.z = v == 0;
    }

    fn check_jmp_condition(&mut self, cc: Option<FlagCondition>) -> bool {
        match cc {
            None => true,
            Some(FlagCondition::NZ) => !self.f.z,
            Some(FlagCondition::Z)  => self.f.z,
            Some(FlagCondition::NC) => !self.f.c,
            Some(FlagCondition::C)  => self.f.c,
        }
    }

    // CALL cc, $a16 | CALL $a16
    // Flags = Z:- N:- H:- C:-
    fn call(&mut self, cc: Option<FlagCondition>, addr: u16) {
        if !self.check_jmp_condition(cc) {
            return;
        }
        self.advance_clock();
        self.push_and_jump(addr);
    }

    // RET cc | RET | RETI
    // Flags = Z:- N:- H:- C:-
    fn ret(&mut self, cc: Option<FlagCondition>, ei: bool) {
        if !self.check_jmp_condition(cc) {
            self.advance_clock();
            return;
        }
        if cc.is_some() {
            self.advance_clock();
        }
        if ei {
            // RETI immediately enables IME, it's not deferred like eith an EI or DI call.
            self.ime = true;
        }
        let pc = self.stack_pop();
        self.advance_clock();
        self.pc = pc;
    }

    // JP cc $a16 | JP $a16
    // Flags = Z:- N:- H:- C:-
    fn jp(&mut self, cc: Option<FlagCondition>, o: Operand16) {
        if !self.check_jmp_condition(cc) {
            return;
        }

        let addr = o.get(self);
        self.pc = addr;

        if let Operand16::Reg(HL) = o {
            // For some reason, JP (HL) doesn't cause an extra clock cycle. Very mysterious.
        } else {
            self.advance_clock();
        }
    }

    // RST 00H | RST 08H | RST 10H ... RST 38H
    // Flags = Z:- N:- H:- C:-
    fn rst(&mut self, a: u8) {
        self.advance_clock();
        self.push_and_jump(a as u16);
    }

    // PUSH %r16
    // Flags = Z:- N:- H:- C:-
    fn push(&mut self, r: Reg16) {
        let v = self.get_reg16(r);
        self.advance_clock();
        self.stack_push(v);
    }

    // POP %r16
    // Flags = Z:* N:* H:* C:*
    fn pop(&mut self, r: Reg16) {
        let mut v = self.stack_pop();
        if let AF = r {
            // Reset bits 0-3 in F.
            v &= 0xFFF0;
        }
        self.set_reg16(r, v);
    }

    // JR cc $r8 | JP $r8
    // Flags = Z:- N:- H:- C:-
    fn jr(&mut self, cc: Option<FlagCondition>, n: u8) {
        if !self.check_jmp_condition(cc) {
            return;
        }
        self.advance_clock();
        self.pc = self.pc.wrapping_add(n as i8 as i16 as u16);
    }

    fn core_panic(&self, msg: String) -> ! {
        panic!("{}\nRegs:\n\tA=0x{:02X}\n\tB=0x{:02X}\n\tC=0x{:02X}\n\tD=0x{:02X}\n\tE=0x{:02X}\n\tF=0x{:02X}\n\tH=0x{:02X}\n\tL=0x{:02X}\n\tSP={:#04X}\n\tPC={:#04X}", msg, self.a, self.b, self.c, self.d, self.e, self.f.pack(), self.h, self.l, self.sp, self.pc);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    fn run_mooneye_test(rom: &[u8], enable_bootrom: bool) {
        let mut cpu = CPU::new(rom.to_vec());
        if !enable_bootrom { cpu.skip_bootrom(); }

        let start = Instant::now();
        while !cpu.mooneye_breakpoint {
            cpu.run();

            if start.elapsed().as_secs() > 10 {
                panic!("Test ran for more than 10 seconds");
            }
        }

        if cpu.b != 3 || cpu.c != 5 || cpu.d != 8 || cpu.e != 13 || cpu.h != 21 || cpu.l != 34 {
            cpu.core_panic(String::from("Test completed in invalid state"));
        }

        if cpu.a != 0 {
            panic!("Test failed {} assertions", cpu.a);
        }
    }

    #[test] fn mooneye_acceptance_bits_mem_oam() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/bits/mem_oam.gb"), false); }
    #[test] fn mooneye_acceptance_bits_reg_f() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/bits/reg_f.gb"), false); }
    #[test] fn mooneye_acceptance_bits_unused_hwio_gs() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/bits/unused_hwio-GS.gb"), false); }

    #[test] fn mooneye_acceptance_interrupts_ie_push() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/interrupts/ie_push.gb"), false); }

    #[test] fn mooneye_acceptance_oam_dma_basic() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/oam_dma/basic.gb"), false); }
    #[test] fn mooneye_acceptance_oam_dma_reg_read() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/oam_dma/reg_read.gb"), false); }

    #[test] fn mooneye_acceptance_ppu_hblank_ly_scx_timing_gs() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ppu/hblank_ly_scx_timing-GS.gb"), false); }
    #[test] fn mooneye_acceptance_ppu_intr_1_2_timing_gs() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ppu/intr_1_2_timing-GS.gb"), false); }
    #[test] fn mooneye_acceptance_ppu_intr_2_0_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ppu/intr_2_0_timing.gb"), false); }
    #[test] fn mooneye_acceptance_ppu_intr_2_mode0_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ppu/intr_2_mode0_timing.gb"), false); }
    #[test] fn mooneye_acceptance_ppu_intr_2_mode0_timing_sprites() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ppu/intr_2_mode0_timing_sprites.gb"), false); }
    #[test] fn mooneye_acceptance_ppu_intr_2_mode3_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ppu/intr_2_mode3_timing.gb"), false); }
    #[test] fn mooneye_acceptance_ppu_intr_2_oam_ok_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ppu/intr_2_oam_ok_timing.gb"), false); }

    #[test] fn mooneye_acceptance_timer_div_write() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/div_write.gb"), false); }
    #[test] fn mooneye_acceptance_timer_rapid_toggle() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/rapid_toggle.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim00() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim00.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim00_div_trigger() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim00_div_trigger.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim01() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim01.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim01_div_trigger() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim01_div_trigger.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim10() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim10.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim10_div_trigger() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim10_div_trigger.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim11() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim11.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tim11_div_trigger() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tim11_div_trigger.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tima_reload() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tima_reload.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tima_write_reloading() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tima_write_reloading.gb"), false); }
    #[test] fn mooneye_acceptance_timer_tma_write_reloading() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/timer/tma_write_reloading.gb"), false); }

    #[test] fn mooneye_acceptance_add_sp_e_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/add_sp_e_timing.gb"), false); }
    #[test] fn mooneye_acceptance_boot_hwio_dmg0() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/boot_hwio-dmg0.gb"), false); }
    #[test] fn mooneye_acceptance_boot_hwio_dmg0_realbootrom() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/boot_hwio-dmg0.gb"), true); }
    #[test] fn mooneye_acceptance_boot_regs_dmg0() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/boot_regs-dmg0.gb"), false); }
    #[test] fn mooneye_acceptance_boot_regs_dmg0_realbootrom() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/boot_regs-dmg0.gb"), true); }
    #[test] fn mooneye_acceptance_call_cc_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/call_cc_timing.gb"), false); }
    #[test] fn mooneye_acceptance_call_cc_timing2() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/call_cc_timing2.gb"), false); }
    #[test] fn mooneye_acceptance_call_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/call_timing.gb"), false); }
    #[test] fn mooneye_acceptance_call_timing2() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/call_timing2.gb"), false); }
    #[test] fn mooneye_acceptance_div_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/div_timing.gb"), false); }
    #[test] fn mooneye_acceptance_ei_sequence() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ei_sequence.gb"), false); }
    #[test] fn mooneye_acceptance_ei_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ei_timing.gb"), false); }
    #[test] fn mooneye_acceptance_halt_ime0_ei() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/halt_ime0_ei.gb"), false); }
    #[test] fn mooneye_acceptance_halt_ime0_nointr_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/halt_ime0_nointr_timing.gb"), false); }
    #[test] fn mooneye_acceptance_halt_ime1_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/halt_ime1_timing.gb"), false); }
    #[test] fn mooneye_acceptance_if_ie_registers() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/if_ie_registers.gb"), false); }
    #[test] fn mooneye_acceptance_intr_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/intr_timing.gb"), false); }
    #[test] fn mooneye_acceptance_jp_cc_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/jp_cc_timing.gb"), false); }
    #[test] fn mooneye_acceptance_jp_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/jp_timing.gb"), false); }
    #[test] fn mooneye_acceptance_ld_hl_sp_e_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ld_hl_sp_e_timing.gb"), false); }
    #[test] fn mooneye_acceptance_oam_dma_restart() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/oam_dma_restart.gb"), false); }
    #[test] fn mooneye_acceptance_oam_dma_start() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/oam_dma_start.gb"), false); }
    #[test] fn mooneye_acceptance_oam_dma_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/oam_dma_timing.gb"), false); }
    #[test] fn mooneye_acceptance_pop_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/pop_timing.gb"), false); }
    #[test] fn mooneye_acceptance_push_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/push_timing.gb"), false); }
    #[test] fn mooneye_acceptance_rapid_di_ei() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/rapid_di_ei.gb"), false); }
    #[test] fn mooneye_acceptance_ret_cc_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ret_cc_timing.gb"), false); }
    #[test] fn mooneye_acceptance_ret_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/ret_timing.gb"), false); }
    #[test] fn mooneye_acceptance_reti_intr_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/reti_intr_timing.gb"), false); }
    #[test] fn mooneye_acceptance_reti_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/reti_timing.gb"), false); }
    #[test] fn mooneye_acceptance_rst_timing() { run_mooneye_test(include_bytes!("../../mooneye-gb-tests/build/acceptance/rst_timing.gb"), false); }

    fn run_blargg_serial_test(rom: &[u8]) {
        let mut serial_output = String::new();
        let mut cpu = CPU::new(rom.to_vec());
        cpu.skip_bootrom();

        loop {
            cpu.run();
            let sb = cpu.sb.take();
            if sb.is_some() {
                serial_output.push(sb.unwrap() as char);
            }

            if cpu.pc == 0xC18B {
                return;
            }
            if cpu.pc == 0xC1B9 {
                panic!("Tests failed.\n{}", serial_output);
            }
        }
    }

    fn run_blargg_harness_test(rom: &[u8]) {
        let mut cpu = CPU::new(rom.to_vec());
        cpu.skip_bootrom();

        // The test runner writes the magic value to RAM before specifying that tests are in progress.
        // Which is kinda dumb. Anyway, we force that value now so we know when tests are *actually* done.
        cpu.cart.ram[0] = 0x80;

        loop {
            cpu.run();

            // Wait until the magic value is present in RAM and the test is signalled as complete.
            if cpu.cart.ram[1] == 0xDE && cpu.cart.ram[2] == 0xB0 && cpu.cart.ram[3] == 0x61 {
                let exit_code = cpu.cart.ram[0];
                if exit_code != 0x80 {
                    let mut end = 0x04;
                    while cpu.cart.ram[end] != 0 && end < 0x1FFF {
                        end += 1;
                    }
                    let output = std::str::from_utf8(&cpu.cart.ram[0x04..end]).unwrap();
                    if exit_code != 0 {
                        panic!("Test failed. Output: {}", output);
                    }
                    return;
                }
            }
        }
    }

    #[test] fn blargg_cpu_instrs_01() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/01-special.gb")); }
    #[test] fn blargg_cpu_instrs_02() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/02-interrupts.gb")); }
    #[test] fn blargg_cpu_instrs_03() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/03-op sp,hl.gb")); }
    #[test] fn blargg_cpu_instrs_04() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/04-op r,imm.gb")); }
    #[test] fn blargg_cpu_instrs_05() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/05-op rp.gb")); }
    #[test] fn blargg_cpu_instrs_06() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/06-ld r,r.gb")); }
    #[test] fn blargg_cpu_instrs_07() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/07-jr,jp,call,ret,rst.gb")); }
    #[test] fn blargg_cpu_instrs_08() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/08-misc instrs.gb")); }
    #[test] fn blargg_cpu_instrs_09() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/09-op r,r.gb")); }
    #[test] fn blargg_cpu_instrs_10() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/10-bit ops.gb")); }
    #[test] fn blargg_cpu_instrs_11() { run_blargg_serial_test(include_bytes!("../test/blargg/cpu_instrs/11-op a,(hl).gb")); }
    #[test] fn blargg_instr_timing()  { run_blargg_serial_test(include_bytes!("../test/blargg/instr_timing.gb")); }
    #[test] fn blargg_mem_timing()    { run_blargg_serial_test(include_bytes!("../test/blargg/mem_timing.gb")); }
    #[test] fn blargg_dmg_sound_01() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/01-registers.gb")); }
    #[test] fn blargg_dmg_sound_02() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/02-len ctr.gb")); }
    #[test] fn blargg_dmg_sound_03() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/03-trigger.gb")); }
    #[test] fn blargg_dmg_sound_04() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/04-sweep.gb")); }
    #[test] fn blargg_dmg_sound_05() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/05-sweep details.gb")); }
    #[test] fn blargg_dmg_sound_06() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/06-overflow on trigger.gb")); }
    // #[test] fn blargg_dmg_sound_07() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/07-len sweep period sync.gb")); }
    #[test] fn blargg_dmg_sound_08() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/08-len ctr during power.gb")); }
    // #[test] fn blargg_dmg_sound_09() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/09-wave read while on.gb")); }
    // #[test] fn blargg_dmg_sound_10() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/10-wave trigger while on.gb")); }
    #[test] fn blargg_dmg_sound_11() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/11-regs after power.gb")); }
    // #[test] fn blargg_dmg_sound_12() { run_blargg_harness_test(include_bytes!("../test/blargg/dmg_sound/12-wave write while on.gb")); }
    #[test] fn blargg_oam_bug_01() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/1-lcd_sync.gb")); }
    #[test] fn blargg_oam_bug_02() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/2-causes.gb")); }
    #[test] fn blargg_oam_bug_03() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/3-non_causes.gb")); }
    #[test] fn blargg_oam_bug_04() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/4-scanline_timing.gb")); }
    #[test] fn blargg_oam_bug_05() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/5-timing_bug.gb")); }
    #[test] fn blargg_oam_bug_06() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/6-timing_no_bug.gb")); }
    // #[test] fn blargg_oam_bug_07() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/7-timing_effect.gb")); }
    // #[test] fn blargg_oam_bug_08() { run_blargg_harness_test(include_bytes!("../test/blargg/oam_bug/8-instr_effect.gb")); }
}
