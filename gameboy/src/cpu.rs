use std::fmt;

use ::GameboyContext;
use self::Reg8::{*};
use self::Reg16::{*};

#[derive(Default)]
pub struct CPUState {
    // CPU Registers
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub h: u8,
    pub l: u8,
    pub f: self::Flags,
    pub sp: u16,
    pub pc: u16,

    pub bootrom_enabled: bool,

    pub halted: bool,

    // Serial I/O
    pub sb: Option<u8>,
    serial_transfer: bool,
    serial_internal_clock: bool,
}

/// CPU flags contained in the "F" register:
/// Z: Zero flag, N: subtract flag, H: half carry flag, C: carry flag
#[derive(Default)]
pub struct Flags {
    pub z: bool,
    pub n: bool,
    pub h: bool,
    pub c: bool,
}

/// The 4 conditions available to branching instructions (CALL/JP/JR/RET).
#[derive(Clone, Copy, Debug)]
enum FlagCondition {
    NZ, // CPU flag Z is clear
    Z,  // CPU flag Z is set
    NC, // CPU flag C is clear
    C,  // CPU flag C is set
}

enum BitwiseOp { XOR, OR, AND }

/// Enum for 7 of the 8 available 8-bit CPU registers. The F register is not included here because
/// no instructions reference it directly.
#[derive(Clone, Copy, Debug)]
enum Reg8 { A, B, C, D, E, H, L }

/// Enum for the 5 available 16-bit CPU registers. With the exception of HL and SP, these registers are just the
/// 8-bit ones viewed as 16 bits.
#[derive(Clone, Copy, Debug)]
enum Reg16 { AF, BC, DE, HL, SP }

/// Enum containing the LR35902 instruction set.
/// There's 501 instructions total, but many of them are the same instruction executed on different registers,
/// or different permutations that use immediate values or data in memory instead of CPU registers.
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

/// Many instructions do the same thing, but just operate on different different operands.
/// For example, you can ADD a value to the A register using either another register, the 8 bit value pointed to by HL,
/// or an immediate 8-bit value. We encapsulate the various operands here and the instructions operate on them.
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

/// Same as Operand8, but for 16 bit operands.
#[derive(Clone, Copy, Debug)]
enum Operand16 {
    Reg(Reg16),        // Contents of a 16 bit register.
    Imm(u16),          // Immediate value.
    ImmAddr(u16),      // Immediate 16 bit value interpreted as memory address.
}

impl Flags {
    /// Clears all CPU flags.
    fn reset(&mut self) -> &mut Self {
        self.z = false;
        self.n = false;
        self.h = false;
        self.c = false;
        self
    }

    /// Converts the CPU flags into an 8bit value. Used by instructions that operate on AF register.
    pub fn pack(&self) -> u8 {
        0
            | if self.z { 0b1000_0000 } else { 0 }
            | if self.n { 0b0100_0000 } else { 0 }
            | if self.h { 0b0010_0000 } else { 0 }
            | if self.c { 0b0001_0000 } else { 0 }
    }

    /// Converts 8 bit value into CPU flags. Used by instructions that operate on AF register.
    pub fn unpack(&mut self, v: u8) {
        self.z = v & 0b1000_0000 > 0;
        self.n = v & 0b0100_0000 > 0;
        self.h = v & 0b0010_0000 > 0;
        self.c = v & 0b0001_0000 > 0;
    }

    fn check_jmp_condition(&self, cc: Option<FlagCondition>) -> bool {
        match cc {
            None => true,
            Some(FlagCondition::NZ) => !self.z,
            Some(FlagCondition::Z)  => self.z,
            Some(FlagCondition::NC) => !self.c,
            Some(FlagCondition::C)  => self.c,
        }
    }
}

impl Operand8 {
    fn get(&self, ctx: &mut GameboyContext) -> u8 {
        match *self {
            Operand8::Reg(r) => ctx.state.cpu.get_reg8(r),
            Operand8::Imm(d) => d,
            Operand8::Addr(rr) => { let addr = ctx.state.cpu.get_reg16(rr); ctx.mem_read8(addr) },
            Operand8::AddrInc(rr) => {
                let addr = ctx.state.cpu.get_reg16(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    ctx.state.ppu.maybe_trash_oam();
                }
                ctx.state.cpu.set_reg16(rr, addr.wrapping_add(1));
                ctx.mem_read8(addr)
            },
            Operand8::AddrDec(rr) => {
                let addr = ctx.state.cpu.get_reg16(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    ctx.state.ppu.maybe_trash_oam();
                }
                ctx.state.cpu.set_reg16(rr, addr.wrapping_sub(1));
                ctx.mem_read8(addr)
            },
            Operand8::ImmAddr(addr) => ctx.mem_read8(addr),
            Operand8::ImmAddrHigh(addr) => ctx.mem_read8(0xFF00 + (addr as u16)),
            Operand8::AddrHigh(r) => { let addr = 0xFF00 + (ctx.state.cpu.get_reg8(r) as u16); ctx.mem_read8(addr) },
        }
    }

    fn set(&self, ctx: &mut GameboyContext, v: u8) -> u8 {
        match *self {
            Operand8::Reg(r) => ctx.state.cpu.set_reg8(r, v),
            Operand8::Imm(_) => { panic!("Attempted to write to immediate operand") },
            Operand8::Addr(rr) => {
                let addr = ctx.state.cpu.get_reg16(rr);
                ctx.mem_write8(addr, v)
            },
            Operand8::AddrInc(rr) => {
                let addr = ctx.state.cpu.get_reg16(rr);
                ctx.state.cpu.set_reg16(rr, addr.wrapping_add(1));
                ctx.mem_write8(addr, v)
            },
            Operand8::AddrDec(rr) => {
                let addr = ctx.state.cpu.get_reg16(rr);
                ctx.state.cpu.set_reg16(rr, addr.wrapping_sub(1));
                ctx.mem_write8(addr, v)
            },
            Operand8::ImmAddr(addr) => ctx.mem_write8(addr, v),
            Operand8::ImmAddrHigh(addr) => ctx.mem_write8(0xFF00 + (addr as u16), v),
            Operand8::AddrHigh(r) => { let addr = ctx.state.cpu.get_reg8(r); ctx.mem_write8(0xFF00 + (addr as u16), v) },
        };
        v
    }
}

impl fmt::Display for Operand8 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
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

impl Operand16 {
    fn get(&self, ctx: &mut GameboyContext) -> u16 {
        match *self {
            Operand16::Reg(r) => ctx.state.cpu.get_reg16(r),
            Operand16::Imm(d) => d,
            Operand16::ImmAddr(addr) => ctx.mem_read16(addr),
        }
    }

    fn set(&self, ctx: &mut GameboyContext, v: u16) {
        match *self {
            Operand16::Reg(r) => ctx.state.cpu.set_reg16(r, v),
            Operand16::Imm(_) => panic!("Attempted to write to immediate operand"),
            Operand16::ImmAddr(addr) => ctx.mem_write16(addr, v),
        }
    }
}

impl fmt::Display for Operand16 {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            Operand16::Reg(r) => write!(f, "{:?}", r),
            Operand16::Imm(d) => write!(f, "${:04X}", d),
            Operand16::ImmAddr(d) => write!(f, "({:#04X})", d),
        }
    }
}

impl fmt::Display for Inst {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
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

impl CPUState {
    pub fn new() -> Self {
        Self{
            a: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0, f: Default::default(),
            sp: 0, pc: 0,
            bootrom_enabled: true,
            halted: false,
            sb: None, serial_transfer: false, serial_internal_clock: false,
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

    pub fn reg_sc_read(&self) -> u8 {
        0x7E  // Unused bits are set to 1
            | if self.serial_transfer       { 0b1000_0000 } else { 0 }
            | if self.serial_internal_clock { 0b0000_0001 } else { 0 }
    }

    pub fn reg_sc_write(&mut self, v: u8) {
        self.serial_transfer       = v & 0b1000_0000 > 0;
        self.serial_internal_clock = v & 0b0000_0001 > 0;
    }
}

/// Runs the CPU for a single "instruction cycle". This will be one or more actual "CPU cycles", each of which
/// takes 4 "machine cycles".
pub fn run(ctx: &mut GameboyContext) {
    if ctx.state.cpu.halted {
        ctx.clock();
    }

    ::interrupt::process(ctx);

    // Apply deferred change to IME.
    if ctx.state.int.ime_defer {
        ctx.state.int.ime = true;
        ctx.state.int.ime_defer = false;
    }

    if !ctx.state.cpu.halted {
        ctx.instr_addr = ctx.state.cpu.pc;
        let inst = decode(ctx);

        execute_instruction(ctx, inst);
        if !ctx.state.cpu.bootrom_enabled {
            // println!("Instr: {} ;${:04X}", inst, ctx.instr_addr);
        }
    }
}

// Fetches next byte PC points to, and increments PC.
fn fetch8(ctx: &mut GameboyContext) -> u8 {
    let addr = ctx.state.cpu.pc;
    ctx.state.cpu.pc = ctx.state.cpu.pc.wrapping_add(1);
    ctx.mem_read8(addr)
}

// Fetches next short PC points to, and increments PC by 2.
fn fetch16(ctx: &mut GameboyContext) -> u16 {
    let addr = ctx.state.cpu.pc;
    ctx.state.cpu.pc = ctx.state.cpu.pc.wrapping_add(2);
    ctx.mem_read16(addr)
}

/// Decodes the next instruction located at PC.
fn decode(ctx: &mut GameboyContext) -> Inst {
    match fetch8(ctx) {
        0x00 => Inst::NOP,
        0x01 => Inst::LD16(Operand16::Reg(BC), Operand16::Imm(fetch16(ctx))),
        0x02 => Inst::LD8(Operand8::Addr(BC), Operand8::Reg(A)),
        0x03 => Inst::INC16(BC),
        0x04 => Inst::INC8(Operand8::Reg(B)),
        0x05 => Inst::DEC8(Operand8::Reg(B)),
        0x06 => Inst::LD8(Operand8::Reg(B), Operand8::Imm(fetch8(ctx))),
        0x07 => Inst::RLCA,
        0x08 => Inst::LD16(Operand16::ImmAddr(fetch16(ctx)), Operand16::Reg(SP)),
        0x09 => Inst::ADD16(BC),
        0x0A => Inst::LD8(Operand8::Reg(A), Operand8::Addr(BC)),
        0x0B => Inst::DEC16(BC),
        0x0C => Inst::INC8(Operand8::Reg(C)),
        0x0D => Inst::DEC8(Operand8::Reg(C)),
        0x0E => Inst::LD8(Operand8::Reg(C), Operand8::Imm(fetch8(ctx))),
        0x0F => Inst::RRCA,
        0x10 => Inst::STOP,
        0x11 => Inst::LD16(Operand16::Reg(DE), Operand16::Imm(fetch16(ctx))),
        0x12 => Inst::LD8(Operand8::Addr(DE), Operand8::Reg(A)),
        0x13 => Inst::INC16(DE),
        0x14 => Inst::INC8(Operand8::Reg(D)),
        0x15 => Inst::DEC8(Operand8::Reg(D)),
        0x16 => Inst::LD8(Operand8::Reg(D), Operand8::Imm(fetch8(ctx))), 
        0x17 => Inst::RLA,
        0x18 => Inst::JR(None, fetch8(ctx)),
        0x19 => Inst::ADD16(DE),
        0x1A => Inst::LD8(Operand8::Reg(A), Operand8::Addr(DE)),
        0x1B => Inst::DEC16(DE),
        0x1C => Inst::INC8(Operand8::Reg(E)),
        0x1D => Inst::DEC8(Operand8::Reg(E)),
        0x1E => Inst::LD8(Operand8::Reg(E), Operand8::Imm(fetch8(ctx))),
        0x1F => Inst::RRA,
        0x20 => Inst::JR(Some(FlagCondition::NZ), fetch8(ctx)),
        0x21 => Inst::LD16(Operand16::Reg(HL), Operand16::Imm(fetch16(ctx))),
        0x22 => Inst::LD8(Operand8::AddrInc(HL), Operand8::Reg(A)),
        0x23 => Inst::INC16(HL),
        0x24 => Inst::INC8(Operand8::Reg(H)),
        0x25 => Inst::DEC8(Operand8::Reg(H)),
        0x26 => Inst::LD8(Operand8::Reg(H), Operand8::Imm(fetch8(ctx))),
        0x27 => Inst::DAA,
        0x28 => Inst::JR(Some(FlagCondition::Z), fetch8(ctx)),
        0x29 => Inst::ADD16(HL),
        0x2A => Inst::LD8(Operand8::Reg(A), Operand8::AddrInc(HL)),
        0x2B => Inst::DEC16(HL),
        0x2C => Inst::INC8(Operand8::Reg(L)),
        0x2D => Inst::DEC8(Operand8::Reg(L)),
        0x2E => Inst::LD8(Operand8::Reg(L), Operand8::Imm(fetch8(ctx))),
        0x2F => Inst::CPL,
        0x30 => Inst::JR(Some(FlagCondition::NC), fetch8(ctx)),
        0x31 => Inst::LD16(Operand16::Reg(SP), Operand16::Imm(fetch16(ctx))),
        0x32 => Inst::LD8(Operand8::AddrDec(HL), Operand8::Reg(A)),
        0x33 => Inst::INC16(SP),
        0x34 => Inst::INC8(Operand8::Addr(HL)),
        0x35 => Inst::DEC8(Operand8::Addr(HL)),
        0x36 => Inst::LD8(Operand8::Addr(HL), Operand8::Imm(fetch8(ctx))),
        0x37 => Inst::SCF,
        0x38 => Inst::JR(Some(FlagCondition::C), fetch8(ctx)),
        0x39 => Inst::ADD16(SP),
        0x3A => Inst::LD8(Operand8::Reg(A), Operand8::AddrDec(HL)),
        0x3B => Inst::DEC16(SP),
        0x3C => Inst::INC8(Operand8::Reg(A)),
        0x3D => Inst::DEC8(Operand8::Reg(A)),
        0x3E => Inst::LD8(Operand8::Reg(A), Operand8::Imm(fetch8(ctx))),
        0x3F => Inst::CCF,
        0x40 => {
            ctx.mooneye_breakpoint = true;
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
        0xC2 => Inst::JP(Some(FlagCondition::NZ), Operand16::Imm(fetch16(ctx))),
        0xC3 => Inst::JP(None, Operand16::Imm(fetch16(ctx))),
        0xC4 => Inst::CALL(Some(FlagCondition::NZ), fetch16(ctx)),
        0xC5 => Inst::PUSH(BC),
        0xC6 => Inst::ADD(Operand8::Imm(fetch8(ctx))),
        0xC7 => Inst::RST(0x00),
        0xC8 => Inst::RET(Some(FlagCondition::Z)),
        0xC9 => Inst::RET(None),
        0xCA => Inst::JP(Some(FlagCondition::Z), Operand16::Imm(fetch16(ctx))),
        0xCB => decode_extended(ctx),
        0xCC => Inst::CALL(Some(FlagCondition::Z), fetch16(ctx)),
        0xCD => Inst::CALL(None, fetch16(ctx)),
        0xCE => Inst::ADC(Operand8::Imm(fetch8(ctx))),
        0xCF => Inst::RST(0x08),
        0xD0 => Inst::RET(Some(FlagCondition::NC)),
        0xD1 => Inst::POP(DE),
        0xD2 => Inst::JP(Some(FlagCondition::NC), Operand16::Imm(fetch16(ctx))),
        0xD4 => Inst::CALL(Some(FlagCondition::NC), fetch16(ctx)),
        0xD5 => Inst::PUSH(DE),
        0xD6 => Inst::SUB(Operand8::Imm(fetch8(ctx))),
        0xD7 => Inst::RST(0x10),
        0xD8 => Inst::RET(Some(FlagCondition::C)),
        0xD9 => Inst::RETI,
        0xDA => Inst::JP(Some(FlagCondition::C), Operand16::Imm(fetch16(ctx))),
        0xDC => Inst::CALL(Some(FlagCondition::C), fetch16(ctx)),
        0xDE => Inst::SBC(Operand8::Imm(fetch8(ctx))),
        0xDF => Inst::RST(0x18),
        0xE0 => Inst::LD8(Operand8::ImmAddrHigh(fetch8(ctx)), Operand8::Reg(A)),
        0xE1 => Inst::POP(HL),
        0xE2 => Inst::LD8(Operand8::AddrHigh(C), Operand8::Reg(A)),
        0xE5 => Inst::PUSH(HL),
        0xE6 => Inst::AND(Operand8::Imm(fetch8(ctx))),
        0xE7 => Inst::RST(0x20),
        0xE8 => Inst::ADD_SP_r8(fetch8(ctx) as i8),
        0xE9 => Inst::JP(None, Operand16::Reg(HL)),
        0xEA => Inst::LD8(Operand8::ImmAddr(fetch16(ctx)), Operand8::Reg(A)),
        0xEE => Inst::XOR(Operand8::Imm(fetch8(ctx))),
        0xEF => Inst::RST(0x28),
        0xF0 => Inst::LD8(Operand8::Reg(A), Operand8::ImmAddrHigh(fetch8(ctx))),
        0xF1 => Inst::POP(AF),
        0xF2 => Inst::LD8(Operand8::Reg(A), Operand8::AddrHigh(C)),
        0xF3 => Inst::DI,
        0xF5 => Inst::PUSH(AF),
        0xF6 => Inst::OR(Operand8::Imm(fetch8(ctx))),
        0xF7 => Inst::RST(0x30),
        0xF8 => Inst::LD_HL_SP(fetch8(ctx) as i8),
        0xF9 => Inst::LD16(Operand16::Reg(SP), Operand16::Reg(HL)),
        0xFA => Inst::LD8(Operand8::Reg(A), Operand8::ImmAddr(fetch16(ctx))),
        0xFB => Inst::EI,
        0xFE => Inst::CP(Operand8::Imm(fetch8(ctx))),
        0xFF => Inst::RST(0x38),

        n => {
            ctx.core_panic(format!("Unexpected opcode 0x{:X} encountered", n));
        }
    }
}

// Decodes extended instruction following 0xCB instruction.
fn decode_extended(ctx: &mut GameboyContext) -> Inst {
    match fetch8(ctx) {
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
            ctx.core_panic(format!("Unexpected extended opcode 0x{:X} encountered", n));
        }
    }
}

/// Executes the given instruction inside the provided GameboyState.
fn execute_instruction(ctx: &mut GameboyContext, inst: Inst) {
    match inst {
        Inst::NOP => (),
        Inst::ADD(o) => add8(ctx, o, false),
        Inst::ADC(o) => add8(ctx, o, true),
        Inst::SUB(o) => sub8(ctx, o, false, true),
        Inst::SBC(o) => sub8(ctx, o, true, true),
        Inst::CP(o)  => sub8(ctx, o, false, false),
        Inst::LD8(l, r) => ld8(ctx, l, r),
        Inst::LD16(l, r) => ld16(ctx, l, r),
        Inst::INC8(o) => inc8(ctx, o),
        Inst::DEC8(o) => dec8(ctx, o),
        Inst::INC16(r) => inc16(ctx, r),
        Inst::DEC16(r) => dec16(ctx, r),
        Inst::AND(o) => bitwise(ctx, BitwiseOp::AND, o),
        Inst::OR(o) => bitwise(ctx, BitwiseOp::OR, o),
        Inst::XOR(o) => bitwise(ctx, BitwiseOp::XOR, o),
        Inst::RLC(o) => rlc(ctx, o, true),
        Inst::RRC(o) => rrc(ctx, o, true),
        Inst::RL(o) => rl(ctx, o, true, true),
        Inst::RR(o) => rr(ctx, o, true),
        Inst::SLA(o) => rl(ctx, o, true, false),
        Inst::SRA(o) => shift_right(ctx, o, true),
        Inst::SRL(o) => shift_right(ctx, o, false),
        Inst::SWAP(o) => swap(ctx, o),
        Inst::BIT(b, o) => bit(ctx, b, o),
        Inst::RES(b, o) => setbit(ctx, b, o, false),
        Inst::SET(b, o) => setbit(ctx, b, o, true),
        Inst::DI => ctx.state.int.set_ime(false),
        Inst::EI => ctx.state.int.set_ime(true),
        Inst::JR(cc, n) => jr(ctx, cc, n),
        Inst::JP(cc, o) => jp(ctx, cc, o),
        Inst::RETI => ret(ctx, None, true),
        Inst::STOP => stop(ctx),
        Inst::HALT => halt(ctx),
        Inst::DAA => daa(ctx),
        Inst::CPL => cpl(ctx),
        Inst::CCF => ccf(ctx),
        Inst::SCF => scf(ctx),
        Inst::ADD_SP_r8(d) => add_sp_r8(ctx, d),
        Inst::ADD16(rr) => add16(ctx, rr),
        Inst::RLA => rl(ctx, Operand8::Reg(A), false, true),
        Inst::RLCA => rlc(ctx, Operand8::Reg(A), false),
        Inst::RRA => rr(ctx, Operand8::Reg(A), false),
        Inst::RRCA => rrc(ctx, Operand8::Reg(A), false),
        Inst::LD_HL_SP(d) => ld_hl_sp(ctx, d), 
        Inst::CALL(cc, addr) => call(ctx, cc, addr),
        Inst::RET(cc) => ret(ctx, cc, false),
        Inst::PUSH(r) => push(ctx, r),
        Inst::POP(r) => pop(ctx, r),
        Inst::RST(a) => rst(ctx, a),
    }
}

// STOP
// Flags = Z:- N:- H:- C:-
fn stop(ctx: &mut GameboyContext) {
    // TODO: this should be more than a noop.
    ctx.state.cpu.pc = ctx.state.cpu.pc.wrapping_add(1);
}

// HALT
// Flags = Z:- N:- H:- C:-
fn halt(ctx: &mut GameboyContext) {
    // TODO: pretty sure I need to check interrupt states here.
    ctx.state.cpu.halted = true;
}

// INC %r8 | INC (HL)
// Flags = Z:* N:0 H:* C:-
fn inc8(ctx: &mut GameboyContext, o: Operand8) {
    let v = o.get(ctx).wrapping_add(1);
    ctx.state.cpu.f.z = v == 0;
    ctx.state.cpu.f.n = false;
    ctx.state.cpu.f.h = v & 0x0F == 0;
    o.set(ctx, v);
}

// DEC %r8 | DEC (HL)
// Flags = Z:* N:1 H:* C:-
fn dec8(ctx: &mut GameboyContext, o: Operand8) {
    let v = o.get(ctx).wrapping_sub(1);
    ctx.state.cpu.f.z = v == 0;
    ctx.state.cpu.f.n = true;
    ctx.state.cpu.f.h = v & 0x0F == 0x0F;
    o.set(ctx, v);
}

// INC rr
// Flags = Z:- N:- H:- C:-
fn inc16(ctx: &mut GameboyContext, r: Reg16) {
    let v = ctx.state.cpu.get_reg16(r);
    if v >= 0xFE00 && v <= 0xFEFF {
        ctx.state.ppu.maybe_trash_oam();
    }
    ctx.state.cpu.set_reg16(r, v.wrapping_add(1));
    ctx.clock();
}

// DEC rr
// Flags = Z:- N:- H:- C:-
fn dec16(ctx: &mut GameboyContext, r: Reg16) {
    let v = ctx.state.cpu.get_reg16(r);
    if v >= 0xFE00 && v <= 0xFEFF {
        ctx.state.ppu.maybe_trash_oam();
    }
    ctx.state.cpu.set_reg16(r, v.wrapping_sub(1));
    ctx.clock();
}

// DAA
// Flags = Z:* N:- H:0 C:*
// TODO: clean this dumpster fire up.
fn daa(ctx: &mut GameboyContext) {
    let mut carry = false;

    if !ctx.state.cpu.f.n {
        if ctx.state.cpu.f.c || ctx.state.cpu.a > 0x99 {
            ctx.state.cpu.a = ctx.state.cpu.a.wrapping_add(0x60);
            carry = true;
        }
        if ctx.state.cpu.f.h || (ctx.state.cpu.a & 0xF) > 9 {
            ctx.state.cpu.a = ctx.state.cpu.a.wrapping_add(0x06);
        }
    } else if ctx.state.cpu.f.c {
        carry = true;
        ctx.state.cpu.a = ctx.state.cpu.a.wrapping_add(if ctx.state.cpu.f.h { 0x9A } else { 0xA0 });
    } else if ctx.state.cpu.f.h {
        ctx.state.cpu.a = ctx.state.cpu.a.wrapping_add(0xFA);
    }

    ctx.state.cpu.f.z = ctx.state.cpu.a == 0;
    ctx.state.cpu.f.h = false;
    ctx.state.cpu.f.c = carry;
}

// CPL
// Flags = Z:- N:1 H:1 C:-
fn cpl(ctx: &mut GameboyContext) {
    ctx.state.cpu.a = !ctx.state.cpu.a;
    ctx.state.cpu.f.n = true;
    ctx.state.cpu.f.h = true;
}

// CCF
// Flags = Z:- N:0 H:0 C:*
fn ccf(ctx: &mut GameboyContext) {
    ctx.state.cpu.f.n = false;
    ctx.state.cpu.f.h = false;
    ctx.state.cpu.f.c = !ctx.state.cpu.f.c;
}

// SCF
// Flags = Z:- N:0 H:0 C:1
fn scf(ctx: &mut GameboyContext) {
    ctx.state.cpu.f.n = false;
    ctx.state.cpu.f.h = false;
    ctx.state.cpu.f.c = true;
}

// ADD A, %r8 | ADD A, (HL) | ADD A, $d8
// ADC A, %r8 | ADC A, (HL) | ADC A, $d8
// Flags = Z:* N:0 H:* C:*
fn add8(ctx: &mut GameboyContext, o: Operand8, carry: bool) {
    let carry = if carry && ctx.state.cpu.f.c { 1 } else { 0 };

    let old = ctx.state.cpu.a;
    let v = o.get(ctx);
    let new = old.wrapping_add(v).wrapping_add(carry);
    ctx.state.cpu.a = new;
    ctx.state.cpu.f.z = new == 0;
    ctx.state.cpu.f.n = false;
    ctx.state.cpu.f.h = (((old & 0xF) + (v & 0xF)) + carry) & 0x10 == 0x10;
    ctx.state.cpu.f.c = (old as u16) + (v as u16) + (carry as u16) > 0xFF;
}

// SUB    %r8 | SUB    (HL) | SUB    $d8
// SBC A, %r8 | SBC A, (HL) | SBC A, $d8
// Flags = Z:* N:1 H:* C:*
fn sub8(ctx: &mut GameboyContext, o: Operand8, carry: bool, store: bool) {
    let carry = if carry && ctx.state.cpu.f.c { 1 } else { 0 };

    let a = ctx.state.cpu.a;
    let v = o.get(ctx);
    let new_a = a.wrapping_sub(v).wrapping_sub(carry);
    if store {
        ctx.state.cpu.a = new_a;
    }

    ctx.state.cpu.f.z = new_a == 0;
    ctx.state.cpu.f.n = true;
    ctx.state.cpu.f.h = ((a & 0xF) as u16) < ((v & 0xF) as u16) + (carry as u16);
    ctx.state.cpu.f.c = (a as u16) < (v as u16) + (carry as u16);
}

// ADD SP, r8
// Flags = Z:0 N:0 H:* C:*
fn add_sp_r8(ctx: &mut GameboyContext, d: i8) {
    let d = d as i16 as u16;
    let sp = ctx.state.cpu.sp;

    ctx.state.cpu.sp = sp.wrapping_add(d as i16 as u16);

    ctx.clock();
    ctx.clock();

    ctx.state.cpu.f.z = false;
    ctx.state.cpu.f.n = false;
    ctx.state.cpu.f.h = ((sp & 0xF) + (d & 0xF)) & 0x10 > 0;
    ctx.state.cpu.f.c = ((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0;
}

// ADD HL, rr
// Flags = Z:- N:0 H:* C:*
fn add16(ctx: &mut GameboyContext, r: Reg16) {
    let hl = ctx.state.cpu.get_reg16(HL);
    let v = ctx.state.cpu.get_reg16(r);
    let (new_hl, overflow) = hl.overflowing_add(v);
    ctx.state.cpu.set_reg16(HL, new_hl);

    ctx.clock();

    ctx.state.cpu.f.n = false;
    ctx.state.cpu.f.h = ((hl & 0xFFF) + (v & 0xFFF)) & 0x1000 > 0;
    ctx.state.cpu.f.c = overflow;
}

// LD %r8, %r8
// LD %r8, $d8
// LD %r8, (%r16)
// LD (%r16), %r8
// LD (HL+), A | LD (HL-), A | LD A, (HL+) | LD A, (HL-)
// Flags = Z:- N:- H:- C:-
fn ld8(ctx: &mut GameboyContext, l: Operand8, r: Operand8) {
    let v = r.get(ctx);
    l.set(ctx, v);
}

// LD %r16, $d16
// LD (%a16), SP
// Flags = Z:- N:- H:- C:-
fn ld16(ctx: &mut GameboyContext, l: Operand16, r: Operand16) {
    let v = r.get(ctx);
    l.set(ctx, v);
    
    // In the specific case of loading a 16bit reg into another 16bit reg, this consumes
    // another CPU cycle. I don't truly understand why, since in every instruction the cost
    // of reading/writing a 16bit reg appears to be free.
    if let Operand16::Reg(_) = l {
        if let Operand16::Reg(_) = r {
            ctx.clock();
        }
    }
}

// LD HL, SP+r8
// Flags = Z:0 N:0 H:* C:*
fn ld_hl_sp(ctx: &mut GameboyContext, d: i8) {
    let sp = ctx.state.cpu.sp;
    let d = d as i16 as u16;
    let v = sp.wrapping_add(d);
    ctx.state.cpu.set_reg16(HL, v);
    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.h = (sp & 0xF) + (d & 0xF) & 0x10 > 0;
    ctx.state.cpu.f.c = (sp & 0xFF) + (d & 0xFF) & 0x100 > 0;
    ctx.clock();
}

// AND %r8 | AND $d8 | AND (HL)
// XOR %r8 | XOR $d8 | XOR (HL)
// OR  %r8 | OR  $d8 | OR  (HL)
// Flags = Z:* N:0 H:* C:0
fn bitwise(ctx: &mut GameboyContext, op: BitwiseOp, o: Operand8) {
    let a = ctx.state.cpu.a;
    let v = o.get(ctx);
    let mut hc = false;
    ctx.state.cpu.a = match op {
        BitwiseOp::AND => { hc = true; a & v },
        BitwiseOp::OR => a | v,
        BitwiseOp::XOR => a ^ v,
    };

    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.z = ctx.state.cpu.a == 0;
    ctx.state.cpu.f.h = hc;
}

// BIT b, r
// Flags = Z:* N:0 H:1 C:-
fn bit(ctx: &mut GameboyContext, b: u8, o: Operand8) {
    let v = o.get(ctx) & (1 << b);
    ctx.state.cpu.f.z = v == 0;
    ctx.state.cpu.f.n = false;
    ctx.state.cpu.f.h = true;
}

// RES b, r
// SET b, r
// Flags = Z:- N:- H:- C:-
fn setbit(ctx: &mut GameboyContext, b: u8, o: Operand8, on: bool) {
    let v = o.get(ctx);
    o.set(ctx, if on { v | 1 << b } else { v & !(1 << b) });
}

// RR r
// RRA
// Flags = Z:* N:0 H:0 C:*
fn rr(ctx: &mut GameboyContext, o: Operand8, extended: bool) {
    let v = o.get(ctx);
    let msb = if ctx.state.cpu.f.c { 0x80 } else { 0 };
    let carry = v & 0x1 > 0;
    let v = o.set(ctx, v >> 1 | msb);
    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.z = extended && v == 0;
    ctx.state.cpu.f.c = carry;
}

// RL %r8
// SL %r8
// RLA
// Flags = Z:* N:0 H:0 C:*
fn rl(ctx: &mut GameboyContext, o: Operand8, set_zero: bool, preserve_lsb: bool) {
    let v = o.get(ctx);
    let lsb = if preserve_lsb && ctx.state.cpu.f.c { 1 } else { 0 };
    let carry = v & 0x80 > 0;
    let v = o.set(ctx, v << 1 | lsb);
    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.z = set_zero && v == 0;
    ctx.state.cpu.f.c = carry;
}

// RLC %r8
// RLCA
// Flags = Z:* N:0 H:0 C:*
fn rlc(ctx: &mut GameboyContext, o: Operand8, extended: bool) {
    let v = o.get(ctx);
    let carry = v & 0x80 > 0;
    let lsb = if carry { 1 } else { 0 };
    let v = o.set(ctx, v << 1 | lsb);
    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.z = extended && v == 0;
    ctx.state.cpu.f.c = carry;
}

// SRA r
// Flags = Z:* N:0 H:0 C:*
fn shift_right(ctx: &mut GameboyContext, o: Operand8, preserve_msb: bool) {
    let v = o.get(ctx);
    let carry = v & 0x01 > 0;
    let preserve = if preserve_msb { v & 0x80 } else { 0 };
    let v = o.set(ctx, v >> 1 | preserve);
    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.z = v == 0;
    ctx.state.cpu.f.c = carry;
}

// RRC r
// Flags = Z:* N:0 H:0 C:*
fn rrc(ctx: &mut GameboyContext, o: Operand8, extended: bool) {
    let v = o.get(ctx);
    let carry = v & 0x1 > 0;
    let msb = if carry { 0x80 } else { 0 };
    let v = o.set(ctx, v >> 1 | msb);
    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.z = extended && v == 0;
    ctx.state.cpu.f.c = carry;
}

// SWAP r
// Flags = Z:* N:0 H:0 C:0
fn swap(ctx: &mut GameboyContext, o: Operand8) {
    let v = o.get(ctx);
    let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
    o.set(ctx, v);
    ctx.state.cpu.f.reset();
    ctx.state.cpu.f.z = v == 0;
}

// CALL cc, $a16 | CALL $a16
// Flags = Z:- N:- H:- C:-
fn call(ctx: &mut GameboyContext, cc: Option<FlagCondition>, addr: u16) {
    if !ctx.state.cpu.f.check_jmp_condition(cc) {
        return;
    }
    ctx.clock();
    push_and_jump(ctx, addr);
}

// RET cc | RET | RETI
// Flags = Z:- N:- H:- C:-
fn ret(ctx: &mut GameboyContext, cc: Option<FlagCondition>, ei: bool) {
    if !ctx.state.cpu.f.check_jmp_condition(cc) {
        ctx.clock();
        return;
    }
    if cc.is_some() {
        ctx.clock();
    }
    if ei {
        // RETI immediately enables IME, it's not deferred like eith an EI or DI call.
        ctx.state.int.ime = true;
    }
    let pc = stack_pop(ctx);
    ctx.clock();
    ctx.state.cpu.pc = pc;
}

// JP cc $a16 | JP $a16
// Flags = Z:- N:- H:- C:-
fn jp(ctx: &mut GameboyContext, cc: Option<FlagCondition>, o: Operand16) {
    if !ctx.state.cpu.f.check_jmp_condition(cc) {
        return;
    }

    let addr = o.get(ctx);
    ctx.state.cpu.pc = addr;

    if let Operand16::Reg(HL) = o {
        // For some reason, JP (HL) doesn't cause an extra clock cycle. Very mysterious.
    } else {
        ctx.clock();
    }
}

// RST 00H | RST 08H | RST 10H ... RST 38H
// Flags = Z:- N:- H:- C:-
fn rst(ctx: &mut GameboyContext, a: u8) {
    ctx.clock();
    push_and_jump(ctx, a as u16);
}

// PUSH %r16
// Flags = Z:- N:- H:- C:-
fn push(ctx: &mut GameboyContext, r: Reg16) {
    let v = ctx.state.cpu.get_reg16(r);
    ctx.clock();
    stack_push(ctx, v);
}

// POP %r16
// Flags = Z:* N:* H:* C:*
fn pop(ctx: &mut GameboyContext, r: Reg16) {
    let mut v = stack_pop(ctx);
    if let AF = r {
        // Reset bits 0-3 in F.
        v &= 0xFFF0;
    }
    ctx.state.cpu.set_reg16(r, v);
}

// JR cc $r8 | JP $r8
// Flags = Z:- N:- H:- C:-
fn jr(ctx: &mut GameboyContext, cc: Option<FlagCondition>, n: u8) {
    if !ctx.state.cpu.f.check_jmp_condition(cc) {
        return;
    }
    ctx.clock();
    ctx.state.cpu.pc = ctx.state.cpu.pc.wrapping_add(n as i8 as i16 as u16);
}

fn stack_push(ctx: &mut GameboyContext, v: u16) {
    if ctx.state.cpu.sp >= 0xFE00 && ctx.state.cpu.sp <= 0xFEFF {
        ctx.state.ppu.maybe_trash_oam();
    }

    ctx.state.cpu.sp = ctx.state.cpu.sp.wrapping_sub(2);
    let sp = ctx.state.cpu.sp;
    ctx.mem_write16(sp, v);
}

fn stack_pop(ctx: &mut GameboyContext) -> u16 {
    if ctx.state.cpu.sp >= 0xFDFF && ctx.state.cpu.sp <= 0xFEFE {
        ctx.state.ppu.maybe_trash_oam();
    }

    let addr = ctx.state.cpu.sp;
    let v = ctx.mem_read16(addr);
    ctx.state.cpu.sp = ctx.state.cpu.sp.wrapping_add(2);

    v
}

fn push_and_jump(ctx: &mut GameboyContext, addr: u16) {
    let pc = ctx.state.cpu.pc;
    stack_push(ctx, pc);
    ctx.state.cpu.pc = addr;
}
