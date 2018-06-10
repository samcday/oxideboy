use std::fmt;

use std::io::{self, Write};

static BOOT_ROM: &[u8; 256] = include_bytes!("boot.rom");

const FLAG_ZERO: u8      = 0b10000000;
const FLAG_SUBTRACT: u8  = 0b01000000;
const FLAG_HALF_CARRY:u8 = 0b00100000;
const FLAG_CARRY:u8      = 0b00010000;

enum FlagOp {
    Z(bool),
    N(bool),
    H(bool),
    C(bool),
}

pub trait MMU {
    fn read8(&self, addr: u16) -> u8;
    fn write8(&mut self, addr: u16, v: u8);

    fn read16(&self, addr: u16) -> u16;
    fn write16(&mut self, addr: u16, v: u16);
}

#[derive(Clone, Copy, Debug)]
enum Reg8 {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}
use Reg8::{*};

#[derive(Clone, Copy, Debug)]
enum Reg16 {
    AF,
    BC,
    DE,
    HL,
    SP,
}
use Reg16::{*};

#[derive(Clone, Copy, Debug)]
enum FlagCondition {
    NZ,
    Z,
    NC,
    C,
}

enum BitwiseOp {
    XOR,
    OR,
    AND,
}

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
            Operand8::Addr(rr) => cpu.mem_read8(cpu.get_reg16(rr)),
            Operand8::AddrInc(rr) => {
                let addr = cpu.get_reg16(rr);
                cpu.set_reg16(rr, addr.wrapping_add(1));
                cpu.mem_read8(addr)
            },
            Operand8::AddrDec(rr) => {
                let addr = cpu.get_reg16(rr);
                cpu.set_reg16(rr, addr.wrapping_sub(1));
                cpu.mem_read8(addr)
            },
            Operand8::ImmAddr(addr) => cpu.mem_read8(addr),
            Operand8::ImmAddrHigh(addr) => cpu.mem_read8(0xFF00 + (addr as u16)),
            Operand8::AddrHigh(r) => cpu.mem_read8(0xFF00 + (cpu.get_reg8(r) as u16)),
        }
    }

    fn set(&self, cpu: &mut CPU, v: u8) {
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
    }

    fn cycle_cost(&self) -> u8 {
        match *self {
            Operand8::Reg(_) => 0,
            Operand8::Imm(_) => 4,
            Operand8::Addr(_) | Operand8::AddrInc(_) | Operand8::AddrDec(_) => 4,
            Operand8::ImmAddr(_) => 12,
            Operand8::ImmAddrHigh(_) => 8,
            Operand8::AddrHigh(_) => 4,
        }
    }
}

impl ::fmt::Display for Operand8 {
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
    fn get(&self, cpu: &CPU) -> u16 {
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

    fn cycle_cost(&self) -> u8 {
        match *self {
            Operand16::Reg(_) => 0,
            Operand16::Imm(_) => 4,
            Operand16::ImmAddr(_) => 12,
        }
    }
}

impl ::fmt::Display for Operand16 {
    fn fmt(&self, f: &mut ::fmt::Formatter) -> ::fmt::Result {
        match *self {
            Operand16::Reg(r) => write!(f, "{:?}", r),
            Operand16::Imm(d) => write!(f, "${:04X}", d),
            Operand16::ImmAddr(d) => write!(f, "({:#04X})", d),
        }
    }
}

#[allow(non_camel_case_types)]
enum Inst {
    NOP,
    EI,
    DI,
    STOP,
    HALT,

    ADD(Operand8),
    ADD16(Reg16),
    ADC(Operand8),
    SUB(Operand8),
    SBC(Operand8),
    INC8(Operand8),
    DEC8(Operand8),
    INC16(Reg16),
    DEC16(Reg16),

    RLA,
    RLCA,
    RRA,
    RRCA,

    LD8(Operand8, Operand8),
    LD16(Operand16, Operand16),

    CP(Operand8),
    AND(Operand8),
    OR(Operand8),
    XOR(Operand8),

    RLC(Operand8),
    RRC(Operand8),
    RL(Operand8),
    RR(Operand8),
    SLA(Operand8),
    SRA(Operand8),
    SWAP(Operand8),
    SRL(Operand8),
    BIT(u8, Operand8),
    RES(u8, Operand8),
    SET(u8, Operand8),

    JP(Option<FlagCondition>, Operand16),
    JR(Option<FlagCondition>, u8),
    CALL(Option<FlagCondition>, u16),
    RET(Option<FlagCondition>),
    PUSH(Reg16),
    POP(Reg16),
    RST(u8),
    RETI,

    DAA,
    CPL,
    CCF,
    SCF,

    ADD_SP_r8(i8),
    LD_HL_SP(i8),
}

impl ::fmt::Display for Inst {
    fn fmt(&self, f: &mut ::fmt::Formatter) -> ::fmt::Result {
        match self {
            Inst::NOP => write!(f,  "NOP"),
            Inst::ADD(o) => write!(f, "ADD A, {}", o),
            Inst::ADC(o) => write!(f, "ADC A, {}", o),
            Inst::SUB(o) => write!(f, "SUB A, {}", o),
            Inst::SBC(o) => write!(f, "SBC A, {}", o),
            Inst::CP(o) => write!(f, "CP {}", o),
            Inst::LD8(l, r) => write!(f, "LD {}, {}", l, r),
            Inst::LD16(l, r) => write!(f, "LD {}, {}", l, r),
            Inst::INC8(o) => write!(f, "INC {}", o),
            Inst::DEC8(o) => write!(f, "DEC {}", o),
            Inst::AND(o) => write!(f, "AND {}", o),
            Inst::OR(o) => write!(f, "OR {}", o),
            Inst::XOR(o) => write!(f, "XOR {}", o),

            Inst::RLC(o) => write!(f, "RLC {}", o),
            Inst::RRC(o) => write!(f, "RRC {}", o),
            Inst::RL(o) => write!(f, "RL {}", o),
            Inst::RR(o) => write!(f, "RR {}", o),
            Inst::SLA(o) => write!(f, "SLA {}", o),
            Inst::SRA(o) => write!(f, "SRA {}", o),
            Inst::SWAP(o) => write!(f, "SWAP {}", o),
            Inst::SRL(o) => write!(f, "SRL {}", o),
            Inst::BIT(b, o) => write!(f, "BIT {}, {:?}", b, o),
            Inst::RES(b, o) => write!(f, "RES {}, {:?}", b, o),
            Inst::SET(b, o) => write!(f, "SET {}, {:?}", b, o),

            Inst::EI => write!(f,  "EI"),
            Inst::DI => write!(f,  "DI"),
            Inst::RETI => write!(f,  "RETI"),
            Inst::STOP => write!(f,  "STOP"),
            Inst::HALT => write!(f,  "HALT"),
            Inst::DAA => write!(f, "DAA"),
            Inst::CPL => write!(f, "CPL"),
            Inst::CCF => write!(f, "CCF"),
            Inst::SCF => write!(f, "SCF"),
            Inst::ADD16(rr) => write!(f, "ADD HL, {:?}", rr),
            Inst::ADD_SP_r8(d) => write!(f, "ADD SP, ${:X}", d),
            Inst::INC16(rr) => write!(f, "INC {:?}", rr),
            Inst::DEC16(o) => write!(f, "DEC {:?}", o),
            Inst::RLA => write!(f, "RLA"),
            Inst::RLCA => write!(f, "RLCA"),
            Inst::RRA => write!(f, "RRA"),
            Inst::RRCA => write!(f, "RRCA"),
            Inst::JP(None, o) => write!(f, "JP {}", o),
            Inst::JP(Some(cc), o) => write!(f, "JP {:?}, {}", cc, o),
            Inst::JR(None, n) => write!(f, "JR ${:X}", n),
            Inst::JR(Some(cc), n) => write!(f, "JR {:?}, ${:X}", cc, n),
            Inst::CALL(Some(cc), n) => write!(f, "CALL {:?} ${:X}", cc, n),
            Inst::CALL(None, n) => write!(f, "CALL ${:X}", n),
            Inst::RET(Some(cc)) => write!(f, "RET {:?}", cc),
            Inst::RET(None) => write!(f, "RET"),
            Inst::PUSH(rr) => write!(f, "PUSH {:?}", rr),
            Inst::POP(rr) => write!(f, "POP {:?}", rr),
            Inst::RST(n) => write!(f, "RST ${:X}", n),
            Inst::LD_HL_SP(d) => write!(f, "LD HL, (SP+${:X})", d),
        }
    }
}

pub struct CPU<'a> {
    // Registers.
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    f: u8,
    sp: u16,
    pc: u16,

    bootrom_enabled: bool,

    // RAM segments.
    vram: [u8; 0x2000], // 0x8000 - 0x9FFF 
    ram:  [u8; 0x2000], // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    oam:  [u8; 0x9F],   // 0xFE00 - 0xFDFF
    wram: [u8; 0x7F],   // 0xFF80 - 0xFFFE

    // Interrupts.
    halted: bool,
    ie: u8,
    if_: u8,
    ime: bool,
    ime_defer: Option<bool>,

    // Timer.
    clock_count: u32,
    tima: u8,
    tma: u8,
    tac: u8,

    // Serial I/O
    sb: u8,
    sc: u8,

    mmu: &'a mut (MMU + 'a),
}

impl <'a> CPU<'a> {
    pub fn new(mmu: &'a mut (MMU + 'a)) -> CPU {
        CPU{
            a: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0, f: 0, sp: 0, pc: 0,
            bootrom_enabled: true,
            vram: [0; 0x2000], ram: [0; 0x2000], oam: [0; 0x9F], wram: [0; 0x7F],
            halted: false, ime: false, ime_defer: None, ie: 0, if_: 0,
            clock_count: 0, tima: 0, tma: 0, tac: 0,
            sb: 0, sc: 0,
            mmu}
    }

    // Runs the CPU for a single instruction.
    // This is the main "Fetch, decode, execute" cycle.
    pub fn run(&mut self) {
        if self.halted {
            self.clock_count += 1;
        }

        if self.sb > 0 {
            io::stdout().write(&[self.sb]).unwrap();
            io::stdout().flush().unwrap();
            self.sb = 0;
        }

        // Advance timer if needed.
        let timer_freq = match self.tac & 3 {
            0 => 1024,
            1 => 16,
            2 => 64,
            3 => 256,
            _ => unreachable!("self.tac & 3 will never get here"),
        };
        if self.clock_count > timer_freq {
            self.clock_count %= timer_freq;

            if self.tac & 4 > 0 {
                self.tima = self.tima.wrapping_add(1);
                if self.tima == 0 {
                    self.tima = self.tma;
                    self.if_ |= 0x4;
                }
            }
        }

        // We exit HALT state if there's any pending interrupts.
        // We do this regardless of IME state.
        if self.halted && self.if_ & self.ie > 0 {
            self.halted = false;
        }

        // Service interrupts.
        if self.ime && (self.if_ & self.ie) > 0 {
            let addr = match self.if_ & self.ie {
                // P10-P13 input signal goes low
                0x10 => {
                    self.if_ ^= 0x10;
                    0x60
                },
                // Serial transfer completion
                0x8 => {
                    self.if_ ^= 0x8;
                    0x58
                },
                // Timer overflow
                0x4 => {
                    self.if_ ^= 0x4;
                    0x50
                },
                // LCDC status interrupt
                0x2 => {
                    self.if_ ^= 0x2;
                    0x48
                },
                // Vertical blanking
                0x1 => {
                    self.if_ ^= 0x1;
                    0x40
                },
                _ => unreachable!("Non-existent interrupt ${:X} encountered", 123)
            };

            self.push_and_jump(addr);
        }

        if self.halted {
            return;
        }

        let _addr = self.pc;
        let inst = self.decode();
        // println!("Inst: {} ; ${:04X}", inst, _addr);

        // Apply deferred change to IME.
        if self.ime_defer.is_some() {
           self.ime = self.ime_defer.unwrap();
           self.ime_defer = None; 
        }

        let inst_cycles = match inst {
            Inst::NOP => 4,
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
            Inst::RL(o) => self.rl(o, true),
            Inst::RR(o) => self.rr(o, true),
            Inst::SLA(o) => self.sla(o),
            Inst::SRA(o) => self.sra(o),
            Inst::SWAP(o) => self.swap(o),
            Inst::SRL(o) => self.srl(o),
            Inst::BIT(b, o) => self.bit(b, o),
            Inst::RES(b, o) => self.setbit(b, o, false),
            Inst::SET(b, o) => self.setbit(b, o, true),

            Inst::DI => self.di(),
            Inst::EI => self.ei(),
            Inst::JR(cc, n) => self.jr(cc, n),
            Inst::JP(cc, o) => self.jp(cc, o),
            Inst::RETI => self.reti(),
            Inst::STOP => self.stop(),
            Inst::HALT => self.halt(),

            Inst::DAA => self.daa(),
            Inst::CPL => self.cpl(),
            Inst::CCF => self.ccf(),
            Inst::SCF => self.scf(),
            Inst::ADD_SP_r8(d) => self.add_sp_r8(d),
            Inst::ADD16(rr) => self.add16(rr),
            Inst::RLA => self.rl(Operand8::Reg(A), false),
            Inst::RLCA => self.rlc(Operand8::Reg(A), false),
            Inst::RRA => self.rr(Operand8::Reg(A), false),
            Inst::RRCA => self.rrc(Operand8::Reg(A), false),

            Inst::LD_HL_SP(d) => self.ld_hl_sp(d), 

            Inst::CALL(cc, addr) => self.call(cc, addr),
            Inst::RET(cc) => self.ret(cc),
            Inst::PUSH(r) => self.push(r),
            Inst::POP(r) => self.pop(r),
            Inst::RST(a) => self.rst(a),
        };
        self.clock_count += inst_cycles as u32;
    }

    fn mem_read8(&self, addr: u16) -> u8 {
        match addr {
            0x0000 ... 0x100 if self.bootrom_enabled => BOOT_ROM[addr as usize],

            0x0000 ... 0x7FFF => self.mmu.read8(addr),
            0x8000 ... 0x9FFF => self.vram[(addr - 0x8000) as usize],
            0xA000 ... 0xBFFF => self.mmu.read8(addr),
            0xC000 ... 0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000 ... 0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFE00 ... 0xFEFF => self.oam[(addr - 0xFE00) as usize],
            0xFF01            => self.sb,
            0xFF02            => self.sc,
            0xFF05            => self.tima,
            0xFF06            => self.tma,
            0xFF07            => self.tac,
            0xFF0F            => self.if_,

            // TODO: LCD
            0xFF42            => 0x00,
            0xFF44            => 0x90, // TODO: temp hack

            0xFF4D            => 0x00,      // KEY1 for CGB.
            0xFF50            => if self.bootrom_enabled { 0 } else { 1 },
            0xFF80 ... 0xFFFE => self.wram[(addr - 0xFF80) as usize],
            0xFFFF            => self.ie,

            _                 => panic!("Unhandled read from ${:X}", addr),
        }
    }

    fn mem_read16(&self, addr: u16) -> u16 {
        let mut v = self.mem_read8(addr) as u16;
        v |= (self.mem_read8(addr + 1) as u16) << 8;
        v
    }

    fn mem_write8(&mut self, addr: u16, v: u8) {
        match addr {
            0xFF50 if self.bootrom_enabled && v == 1 => { self.bootrom_enabled = false; },

            0x0000 ... 0x7FFF => { panic!("Write ${:X} to ${:X}", v, addr); }, // TODO: this is forwarded to MBC.
            0x8000 ... 0x9FFF => { self.vram[(addr - 0x8000) as usize] = v },
            0xA000 ... 0xBFFF => { panic!("Write ${:X} to ${:X}", v, addr); }, // TODO: this is forwarded to MBC.
            0xC000 ... 0xDFFF => { self.ram[(addr - 0xC000) as usize] = v },
            0xE000 ... 0xFDFF => { self.ram[(addr - 0xE000) as usize] = v },
            0xFE00 ... 0xFEFF => { self.oam[(addr - 0xFE00) as usize] = v },
            0xFF01            => { self.sb = v },
            0xFF02            => { self.sc = v },
            0xFF05            => { self.tima = v },
            0xFF06            => { self.tma = v },
            0xFF07            => { self.tac = v & 0x7 },
            0xFF0F            => { self.if_ = v & 0x1F },

            // TODO: joypad
            0xFF00            => { }, // TODO:

            // TODO: sound
            0xFF11 ... 0xFF14 => { },
            0xFF24 ... 0xFF26 => { },

            // TODO: LCD
            0xFF40 ... 0xFF43 => { },
            0xFF47            => { },

            0xFF4D            => { },      // KEY1 for CGB.
            0xFF80 ... 0xFFFE => { self.wram[(addr - 0xFF80) as usize] = v },
            0xFFFF            => { self.ie = v & 0x1F },

            _                 => { panic!("Unhandled write to ${:X}", addr) },
        };
    }

    fn mem_write16(&mut self, addr: u16, v: u16) {
        self.mem_write8(addr, (v & 0xFF) as u8);
        self.mem_write8(addr + 1, ((v & 0xFF00) >> 8) as u8);
    }

    // Fetches next byte from PC and increments PC.
    fn fetch8(&mut self) -> u8 {
        self.pc += 1;
        self.mem_read8(self.pc - 1)
    }

    // Fetches next short from PC and increments PC by 2.
    fn fetch16(&mut self) -> u16 {
        self.pc += 2;
        self.mem_read16(self.pc - 2)
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
            0x10 => { self.fetch8(); Inst::STOP },
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
            0x40 => Inst::LD8(Operand8::Reg(B), Operand8::Reg(B)),
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
                panic!("Unexpected opcode 0x{:X} encountered", n);
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
                panic!("Unexpected extended opcode 0x{:X} encountered", n);
            }
        }
    }

    fn flags_query(&self, f: u8) -> bool {
        self.f & f > 0
    }

    fn flags_update(&mut self, ops: &[FlagOp]) {
        let mut mask: u8 = 0;
        let mut new: u8 = 0;
        for op in ops {
            match op {
                FlagOp::Z(true)  => { mask |= FLAG_ZERO; new |= FLAG_ZERO },
                FlagOp::Z(false) => { mask |= FLAG_ZERO; },
                FlagOp::N(true)  => { mask |= FLAG_SUBTRACT; new |= FLAG_SUBTRACT },
                FlagOp::N(false) => { mask |= FLAG_SUBTRACT; },
                FlagOp::H(true)  => { mask |= FLAG_HALF_CARRY; new |= FLAG_HALF_CARRY },
                FlagOp::H(false) => { mask |= FLAG_HALF_CARRY; },
                FlagOp::C(true)  => { mask |= FLAG_CARRY; new |= FLAG_CARRY },
                FlagOp::C(false) => { mask |= FLAG_CARRY; },
            }
        }
        self.f = (self.f & !(mask)) | new;
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
            AF => { (self.a, self.f) },
            BC => { (self.b, self.c) },
            DE => { (self.d, self.e) },
            HL => { (self.h, self.l) },
            SP => { return self.sp },
        };

        (hi as u16) << 8 | (lo as u16)
    }

    fn set_reg16(&mut self, reg: Reg16, v: u16) {
        let (hi, lo) = match reg {
            AF => { (&mut self.a, &mut self.f) },
            BC => { (&mut self.b, &mut self.c) },
            DE => { (&mut self.d, &mut self.e) },
            HL => { (&mut self.h, &mut self.l) },
            SP => { self.sp = v; return },
        };

        *hi = ((v & 0xFF00) >> 8) as u8;
        *lo = (v & 0xFF) as u8;
    }

    fn stack_push(&mut self, v: u16) {
        self.sp -= 2;
        let sp = self.sp;
        self.mem_write16(sp, v);
    }

    fn stack_pop(&mut self) -> u16 {
        let v = self.mem_read16(self.sp);
        self.sp += 2;
        v
    }

    fn push_and_jump(&mut self, addr: u16) {
        let pc = self.pc;
        self.stack_push(pc);
        self.pc = addr;
    }

    fn check_jmp_condition(&self, cc: FlagCondition) -> bool {
        match cc {
            FlagCondition::NZ => !self.flags_query(FLAG_ZERO),
            FlagCondition::Z  => self.flags_query(FLAG_ZERO),
            FlagCondition::NC => !self.flags_query(FLAG_CARRY),
            FlagCondition::C  => self.flags_query(FLAG_CARRY),
        }
    }

    fn di(&mut self) -> u8 {
        self.ime_defer = Some(false);
        4
    }

    fn ei(&mut self) -> u8 {
        self.ime_defer = Some(true);
        4
    }

    fn reti(&mut self) -> u8 {
        self.ime_defer = Some(true);
        self.ret(None)
    }

    // STOP
    // Flags:
    // Z N H C
    // - - - -
    fn stop(&mut self) -> u8 {
        // TODO: this should be more than a noop.
        4
    }

    // HALT
    // Flags:
    // Z N H C
    // - - - -
    fn halt(&mut self) -> u8 {
        // TODO: pretty sure I need to check interrupt states here.
        self.halted = true;
        4
    }

    // INC %r8 | INC (HL)
    // Flags = Z:* N:0 H:* C:-
    fn inc8(&mut self, o: Operand8) -> u8 {
        let v = o.get(self).wrapping_add(1);
        self.flags_update(&[FlagOp::Z(v == 0), FlagOp::N(false), FlagOp::H(v & 0x0F == 0)]);
        o.set(self, v);
        4 + o.cycle_cost() * 2
    }

    // DEC %r8 | DEC (HL)
    // Flags = Z:* N:1 H:* C:-
    fn dec8(&mut self, o: Operand8) -> u8 {
        let v = o.get(self).wrapping_sub(1);
        self.flags_update(&[FlagOp::Z(v == 0), FlagOp::N(true), FlagOp::H(v & 0x0F == 0x0F)]);
        o.set(self, v);
        4 + o.cycle_cost() * 2
    }

    // INC rr
    // Flags = Z:- N:- H:- C:-
    fn inc16(&mut self, r: Reg16) -> u8 {
        let v = self.get_reg16(r).wrapping_add(1);
        self.set_reg16(r, v);
        8
    }

    // DEC rr
    // Flags = Z:- N:- H:- C:-
    fn dec16(&mut self, r: Reg16) -> u8 {
        let v = self.get_reg16(r).wrapping_sub(1);
        self.set_reg16(r, v);
        8
    }

    // DAA
    // Flags = Z:* N:- H:0 C:*
    // TODO: clean this cancer up.
    fn daa(&mut self) -> u8 {
        let mut carry = false;

        if self.f & FLAG_SUBTRACT == 0 {
            if self.f & FLAG_CARRY > 0 || self.a > 0x99 {
                self.a = self.a.wrapping_add(0x60);
                carry = true;
            }
            if self.f & FLAG_HALF_CARRY > 0 || (self.a & 0xF) > 9 {
                self.a = self.a.wrapping_add(0x06);
            }
        } else if self.f & FLAG_CARRY > 0 {
            carry = true;
            self.a = self.a.wrapping_add(if self.f & FLAG_HALF_CARRY > 0 { 0x9A } else { 0xA0 });
        } else if self.f & FLAG_HALF_CARRY > 0 {
            self.a = self.a.wrapping_add(0xFA);
        }

        self.f &= !(FLAG_HALF_CARRY | FLAG_ZERO | FLAG_CARRY);
        if carry {
            self.f |= FLAG_CARRY;
        }
        if self.a == 0 {
            self.f |= FLAG_ZERO;
        }

        4
    }

    // CPL
    // Flags = Z:- N:1 H:1 C:-
    fn cpl(&mut self) -> u8 {
        self.a = !self.a;
        self.flags_update(&[FlagOp::N(true), FlagOp::H(true)]);
        4
    }

    // CCF
    // Flags = Z:- N:0 H:0 C:*
    fn ccf(&mut self) -> u8 {
        let carry = self.flags_query(FLAG_CARRY);
        self.flags_update(&[FlagOp::N(false), FlagOp::H(false), FlagOp::C(!carry)]);
        4
    }

    // SCF
    // Flags = Z:- N:0 H:0 C:1
    fn scf(&mut self) -> u8 {
        self.flags_update(&[FlagOp::N(false), FlagOp::H(false), FlagOp::C(true)]);
        4
    }

    // ADD A, %r8 | ADD A, (HL) | ADD A, $d8
    // ADC A, %r8 | ADC A, (HL) | ADC A, $d8
    // Flags = Z:* N:0 H:* C:*
    fn add8(&mut self, o: Operand8, carry: bool) -> u8 {
        let carry = if carry && (self.f & FLAG_CARRY > 0) { 1 } else { 0 };

        let old = self.a;
        let v = o.get(self);
        let new = old.wrapping_add(v).wrapping_add(carry);
        self.a = new;
        self.flags_update(&[
            FlagOp::Z(new == 0),
            FlagOp::N(false),
            FlagOp::H((((old & 0xF) + (v & 0xF)) + carry) & 0x10 == 0x10),
            FlagOp::C((old as u16) + (v as u16) + (carry as u16) > 0xFF),
        ]);

        4 + o.cycle_cost()
    }

    // SUB    %r8 | SUB    (HL) | SUB    $d8
    // SBC A, %r8 | SBC A, (HL) | SBC A, $d8
    // Flags = Z:* N:1 H:* C:*
    fn sub8(&mut self, o: Operand8, carry: bool, store: bool) -> u8 {
        let carry = if carry && (self.f & FLAG_CARRY != 0) { 1 } else { 0 };

        let a = self.a;
        let v = o.get(self);
        let new_a = a.wrapping_sub(v).wrapping_sub(carry);

        if store {
            self.a = new_a;
        }

        self.flags_update(&[
            FlagOp::Z(new_a == 0),
            FlagOp::N(true),
            FlagOp::H(((a & 0xF) as u16) < ((v & 0xF) as u16) + (carry as u16)),
            FlagOp::C((a as u16) < (v as u16) + (carry as u16)),
        ]);

        4 + o.cycle_cost()
    }

    // ADD SP, r8
    // Flags = Z:0 N:0 H:* C:*
    fn add_sp_r8(&mut self, d: i8) -> u8 {
        let d = d as i16 as u16;
        let sp = self.sp;

        self.sp = sp.wrapping_add(d as i16 as u16);

        self.flags_update(&[
            FlagOp::Z(false),
            FlagOp::N(false),
            FlagOp::H(((sp & 0xF) + (d & 0xF)) & 0x10 > 0),
            FlagOp::C(((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0),
        ]);

        16
    }

    // ADD HL, rr
    // Flags = Z:- N:0 H:* C:*
    fn add16(&mut self, r: Reg16) -> u8 {
        let hl = self.get_reg16(HL);
        let v = self.get_reg16(r);
        let (new_hl, overflow) = hl.overflowing_add(v);
        self.set_reg16(HL, new_hl);

        self.flags_update(&[
            FlagOp::N(false),
            FlagOp::H(((hl & 0xFFF) + (v & 0xFFF)) & 0x1000 > 0),
            FlagOp::C(overflow),
        ]);

        8
    }

    // LD %r8, %r8
    // LD %r8, $d8
    // LD %r8, (%r16)
    // LD (%r16), %r8
    // LD (HL+), A | LD (HL-), A | LD A, (HL+) | LD A, (HL-)
    // Flags = Z:- N:- H:- C:-
    fn ld8(&mut self, l: Operand8, r: Operand8) -> u8 {
        let v = r.get(self);
        l.set(self, v);
        4 + l.cycle_cost() + r.cycle_cost()
    }

    // LD %r16, $d16
    // LD (%a16), SP
    // Flags = Z:- N:- H:- C:-
    fn ld16(&mut self, l: Operand16, r: Operand16) -> u8 {
        let v = r.get(self);
        l.set(self, v);
        8 + l.cycle_cost() + r.cycle_cost()
    }

    // LD HL, SP+r8
    // Flags = Z:0 N:0 H:* C:*
    fn ld_hl_sp(&mut self, d: i8) -> u8 {
        let sp = self.sp;
        let d = d as i16 as u16;
        let v = sp.wrapping_add(d);
        self.set_reg16(HL, v);
        self.flags_update(&[
            FlagOp::Z(false),
            FlagOp::N(false),
            FlagOp::H((sp & 0xF) + (d & 0xF) & 0x10 > 0),
            FlagOp::C((sp & 0xFF) + (d & 0xFF) & 0x100 > 0),
        ]);

        12
    }

    // AND %r8 | AND $d8 | AND (HL)
    // XOR %r8 | XOR $d8 | XOR (HL)
    // OR  %r8 | OR  $d8 | OR  (HL)
    // Flags = Z:* N:0 H:* C:0
    fn bitwise(&mut self, op: BitwiseOp, o: Operand8) -> u8 {
        let a = self.a;
        let v = o.get(self);
        self.a = match op {
            BitwiseOp::AND => a & v,
            BitwiseOp::OR => a | v,
            BitwiseOp::XOR => a ^ v,
        };
        let z = self.a == 0;
        let hc = if let BitwiseOp::AND = op { true } else { false };
        self.flags_update(&[
            FlagOp::Z(z),
            FlagOp::N(false),
            FlagOp::H(hc),
            FlagOp::C(false),
        ]);

        4 + o.cycle_cost()
    }

    // BIT b, r
    // Flags = Z:* N:0 H:1 C:-
    fn bit(&mut self, b: u8, o: Operand8) -> u8 {
        let v = o.get(self) & (1 << b);
        self.flags_update(&[
            FlagOp::Z(v == 0),
            FlagOp::N(false),
            FlagOp::H(true),
        ]);
        8 + o.cycle_cost() * 2
    }

    // RES b, r
    // SET b, r
    // Flags = Z:- N:- H:- C:-
    fn setbit(&mut self, b: u8, o: Operand8, on: bool) -> u8 {
        let mut v = o.get(self);
        if on {
            v |= 1 << b;
        } else {
            v &= !(1 << b);
        }
        o.set(self, v);
        8 + o.cycle_cost() * 2
    }

    // RL r
    // Flags:
    // Z N H C
    // * 0 0 *
    // TODO:
    fn rl(&mut self, o: Operand8, extended: bool) -> u8 {
        let v = o.get(self);
        let carry = self.f & FLAG_CARRY > 0;

        self.f = 0;
        if v & 0x80 > 0 {
            self.f |= FLAG_CARRY;
        }

        let mut v = v << 1;

        if carry {
            v |= 1;
        }

        // Only extended RL instructions set zero flag.
        if extended && v == 0 {
            self.f |= FLAG_ZERO;
        }
        o.set(self, v);

        (if extended { 8 } else { 4 }) + o.cycle_cost() * 2
    }

    // RLC r
    // Flags:
    // Z N H C
    // * 0 0 *
    // RLCA
    // Flags:
    // Z N H C
    // 0 0 0 *
    // TODO:
    fn rlc(&mut self, o: Operand8, extended: bool) -> u8 {
        let mut v = o.get(self);
        self.f = 0;
        if v & 0x80 > 0 {
            self.f |= FLAG_CARRY;
        }
        v <<= 1;
        if self.f & FLAG_CARRY > 0 {
            v |= 1;
        }
        if extended && v == 0 {
            self.f |= FLAG_ZERO;
        }
        o.set(self, v);

        (if extended { 8 } else { 4 }) + o.cycle_cost() * 2
    }

    // RR r
    // Flags:
    // Z N H C
    // * 0 0 *
    fn rr(&mut self, o: Operand8, extended: bool) -> u8 {
        let v = o.get(self);
        let carry = self.f & FLAG_CARRY > 0;

        self.f = 0;
        if v & 0x1 > 0 {
            self.f |= FLAG_CARRY;
        }

        let mut v = v >> 1;

        if carry {
            v |= 0x80;
        }

        if extended && v == 0 {
            self.f |= FLAG_ZERO;
        }

        o.set(self, v);

        (if extended { 8 } else { 4 }) + o.cycle_cost() * 2
    }

    // SLA r
    // Flags:
    // Z N H C
    // * 0 0 C
    // TODO:
    fn sla(&mut self, o: Operand8) -> u8 {
        let v = o.get(self);
        self.f = 0;
        if v & 0x80 > 0 {
            self.f |= FLAG_CARRY;
        }
        let v = v << 1;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        o.set(self, v);
        8 + o.cycle_cost() * 2
    }

    // SRA r
    // Flags:
    // Z N H C
    // * 0 0 *
    fn sra(&mut self, o: Operand8) -> u8 {
        let v = o.get(self);
        self.f = 0;
        if v & 0x01 > 0 {
            self.f |= FLAG_CARRY;
        }
        let v = (v >> 1) | (v & 0x80);
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        o.set(self, v);
        8 + o.cycle_cost() * 2
    }

    // RRC r
    // Flags:
    // Z N H C
    // * 0 0 *
    // TODO:
    fn rrc(&mut self, o: Operand8, extended: bool) -> u8 {
        let v = o.get(self);

        self.f = 0;
        if v & 0x1 > 0 {
            self.f |= FLAG_CARRY;
        }

        let mut v = v >> 1;
        if self.f & FLAG_CARRY > 0 {
            v |= 0x80;
        }
        if extended && v == 0 {
            self.f |= FLAG_ZERO;
        }

        o.set(self, v);

        (if extended { 8 } else { 4 }) + o.cycle_cost() * 2
    }

    // SWAP r
    // Flags:
    // Z N H C
    // * 0 0 0
    // TODO:
    fn swap(&mut self, o: Operand8) -> u8 {
        let v = o.get(self);
        let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
        self.f = 0;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        o.set(self, v);
        8 + o.cycle_cost() * 2
    }

    // SRL r
    // SRL (HL)
    // Flags:
    // Z N H C
    // * 0 0 *
    // TODO:
    fn srl(&mut self, o: Operand8) -> u8 {
        let v = o.get(self);

        self.f = 0;
        if v & 0x1 == 1 {
            self.f |= FLAG_CARRY;
        }

        let v = v >> 1;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }

        o.set(self, v);

        8 + o.cycle_cost() * 2
    }

    fn call(&mut self, cc: Option<FlagCondition>, addr: u16) -> u8 {
        if let Some(cc) = cc {
            if !self.check_jmp_condition(cc) {
                return 12;
            }
        }

        self.push_and_jump(addr);
        24
    }

    fn ret(&mut self, cc: Option<FlagCondition>) -> u8 {
        let mut ic = 16;
        if let Some(cc) = cc {
            if !self.check_jmp_condition(cc) {
                return 8;
            }
            ic += 4;
        }

        self.pc = self.mem_read16(self.sp);
        self.sp += 2;
        ic
    }

    fn jp(&mut self, cc: Option<FlagCondition>, o: Operand16) -> u8 {
        if let Some(cc) = cc {
            if !self.check_jmp_condition(cc) {
                return 12;
            }
        }

        let addr = o.get(self);
        self.pc = addr;

        match o {
            Operand16::Reg(HL) => 4,
            _ => 16
        }
    }

    fn rst(&mut self, a: u8) -> u8 {
        self.push_and_jump(a as u16);
        16
    }

    fn push(&mut self, r: Reg16) -> u8 {
        let v = self.get_reg16(r);
        self.stack_push(v);
        16
    }

    fn pop(&mut self, r: Reg16) -> u8 {
        let mut v = self.stack_pop();
        if let AF = r {
            // Reset bits 0-3 in F.
            v &= 0xFFF0;
        }
        self.set_reg16(r, v);
        12
    }

    fn jr(&mut self, cc: Option<FlagCondition>, n: u8) -> u8 {
        if let Some(cc) = cc {
            if !self.check_jmp_condition(cc) {
                return 8;
            }
        }

        self.pc = self.pc.wrapping_add(n as i8 as i16 as u16);
        12
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
