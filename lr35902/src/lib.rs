use std::fmt;

use std::io::{self, Write};

static BOOT_ROM: &[u8; 256] = include_bytes!("boot.rom");

const FLAG_ZERO: u8      = 0b10000000;
const FLAG_SUBTRACT: u8  = 0b01000000;
const FLAG_HALF_CARRY:u8 = 0b00100000;
const FLAG_CARRY:u8      = 0b00010000;

pub trait MMU {
    fn read8(&self, addr: u16) -> u8;
    fn write8(&mut self, addr: u16, v: u8);

    fn read16(&self, addr: u16) -> u16;
    fn write16(&mut self, addr: u16, v: u16);
}

#[derive(Clone, Copy, Debug)]
enum Register8 {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

#[derive(Clone, Copy, Debug)]
enum Register16 {
    AF,
    BC,
    DE,
    HL,
    SP,
}

#[derive(Debug)]
enum ConditionFlag {
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

#[allow(non_camel_case_types)]
enum Instruction {
    NOP,

    EI,
    DI,
    RETI,
    STOP,

    DAA,
    CPL,
    CCF,
    SCF,
    ADD_A_r(Register8),
    ADD_A_d8(u8),
    ADD_A_HL,
    ADD_HL_rr(Register16),
    ADD_SP_r8(i8),
    ADC_A_r(Register8),
    ADC_A_d8(u8),
    ADC_A_HL,
    SUB_r(Register8),
    SUB_d8(u8),
    SUB_HL,
    SBC_r(Register8),
    SBC_d8(u8),
    SBC_HL,
    INC_r(Register8),
    INC_rr(Register16),
    DEC_r(Register8),
    DEC_HL,
    DEC_rr(Register16),
    AND_r(Register8),
    AND_d8(u8),
    AND_HL,
    OR_r(Register8),
    OR_d8(u8),
    OR_HL,
    XOR_r(Register8),
    XOR_d8(u8),
    XOR_HL,
    CP_r(Register8),
    CP_d8(u8),
    CP_HL,
    RLA,
    RLCA,
    RRA,
    RRCA,

    JP(Option<ConditionFlag>, u16),
    JP_HL,
    JR_cc_n(ConditionFlag, u8),
    JR_n(u8),
    CALL(Option<ConditionFlag>, u16),
    RET(Option<ConditionFlag>),
    PUSH(Register16),
    POP(Register16),
    RST(u8),

    LD_r_rr(Register8, Register16),
    LD_r_d8(Register8, u8),
    LD_rr_d16(Register16, u16),
    LD_r16_r(Register16, Register8),
    LD_r_r(Register8, Register8),
    LD_C_A,
    LD_A_C,
    LD_HLD_A,
    LD_HLI_A,
    LD_A_HLD,
    LD_A_HLI,
    LD_HL_SP(i8),
    LD_HL_d8(u8),
    LD_SP_HL,
    LDH_A_n(u8),
    LDH_n_A(u8),
    LD_a16_SP(u16),
    LD_a16_A(u16),
    LD_A_a16(u16),

    BIT_b_r(u8, Register8),
    BIT_b_HL(u8),
    RES_b_r(u8, Register8),
    RES_b_HL(u8),
    SET_b_r(u8, Register8),
    SET_b_HL(u8),
    RL_r(Register8),
    RLC_r(Register8),
    RLC_HL,
    SRL_r(Register8),
    SRL_HL,
    RR_r(Register8),
    RR_HL,
    RRC_r(Register8),
    RRC_HL,
    SLA_r(Register8),
    SLA_HL,
    SRA_r(Register8),
    SRA_HL,
    SWAP_r(Register8),
    SWAP_HL,
}

impl ::fmt::Display for Instruction {
    fn fmt(&self, f: &mut ::fmt::Formatter) -> ::fmt::Result {
        match self {
            Instruction::NOP => write!(f,  "NOP"),
            Instruction::EI => write!(f,  "EI"),
            Instruction::DI => write!(f,  "DI"),
            Instruction::RETI => write!(f,  "RETI"),
            Instruction::STOP => write!(f,  "STOP"),
            Instruction::DAA => write!(f, "DAA"),
            Instruction::CPL => write!(f, "CPL"),
            Instruction::CCF => write!(f, "CCF"),
            Instruction::SCF => write!(f, "SCF"),
            Instruction::ADD_A_r(r) => write!(f, "ADD A, {:?}", r),
            Instruction::ADD_A_d8(d) => write!(f, "ADD A, ${:X}", d),
            Instruction::ADD_A_HL => write!(f, "ADD A, (HL)"),
            Instruction::ADD_HL_rr(rr) => write!(f, "ADD HL, {:?}", rr),
            Instruction::ADD_SP_r8(d) => write!(f, "ADD SP, ${:X}", d),
            Instruction::ADC_A_r(r) => write!(f, "ADC A, {:?}", r),
            Instruction::ADC_A_d8(d) => write!(f, "ADC A, ${:X}", d),
            Instruction::ADC_A_HL => write!(f, "ADC A, (HL)"),
            Instruction::SUB_r(r) => write!(f, "SUB {:?}", r),
            Instruction::SUB_d8(d) => write!(f, "SUB ${:X}", d),
            Instruction::SUB_HL => write!(f, "SUB (HL)"),
            Instruction::SBC_r(r) => write!(f, "SBC A, {:?}", r),
            Instruction::SBC_d8(d) => write!(f, "SBC A, ${:X}", d),
            Instruction::SBC_HL => write!(f, "SBC (HL)"),
            Instruction::INC_r(r) => write!(f, "INC {:?}", r),
            Instruction::INC_rr(rr) => write!(f, "INC {:?}", rr),
            Instruction::DEC_r(r) => write!(f, "DEC {:?}", r),
            Instruction::DEC_rr(rr) => write!(f, "DEC {:?}", rr),
            Instruction::DEC_HL => write!(f, "DEC (HL)"),
            Instruction::AND_r(r) => write!(f, "AND {:?}", r),
            Instruction::AND_d8(d) => write!(f, "AND ${:X}", d),
            Instruction::AND_HL => write!(f, "AND (HL)"),
            Instruction::OR_r(r) => write!(f, "OR {:?}", r),
            Instruction::OR_d8(d) => write!(f, "OR ${:X}", d),
            Instruction::OR_HL => write!(f, "OR (HL)"),
            Instruction::XOR_r(r) => write!(f, "XOR {:?}", r),
            Instruction::XOR_d8(d) => write!(f, "XOR ${:X}", d),
            Instruction::XOR_HL => write!(f, "XOR (HL)"),
            Instruction::CP_r(r) => write!(f, "CP {:?}", r),
            Instruction::CP_d8(d) => write!(f, "CP ${:X}", d),
            Instruction::CP_HL => write!(f, "CP (HL)"),
            Instruction::RLA => write!(f, "RLA"),
            Instruction::RLCA => write!(f, "RLCA"),
            Instruction::RRA => write!(f, "RRA"),
            Instruction::RRCA => write!(f, "RRCA"),
            Instruction::JP(None, addr) => write!(f, "JP ${:X}", addr),
            Instruction::JP(Some(cc), addr) => write!(f, "JP {:?}, ${:X}", cc, addr),
            Instruction::JP_HL => write!(f, "JP (HL)"),
            Instruction::JR_cc_n(cc, n) => write!(f, "JR {:?}, ${:X}", cc, n),
            Instruction::JR_n(n) => write!(f, "JR ${:X}", n),
            Instruction::CALL(Some(cc), n) => write!(f, "CALL {:?} ${:X}", cc, n),
            Instruction::CALL(None, n) => write!(f, "CALL ${:X}", n),
            Instruction::RET(Some(cc)) => write!(f, "RET {:?}", cc),
            Instruction::RET(None) => write!(f, "RET"),
            Instruction::PUSH(rr) => write!(f, "PUSH {:?}", rr),
            Instruction::POP(rr) => write!(f, "POP {:?}", rr),
            Instruction::RST(n) => write!(f, "RST ${:X}", n),
            Instruction::LD_r_d8(r, n) => write!(f, "LD {:?}, ${:X}", r, n),
            Instruction::LD_r_r(r, r2) => write!(f, "LD {:?}, {:?}", r, r2),
            Instruction::LD_r_rr(r, rr) => write!(f, "LD {:?}, ({:?})", r, rr),
            Instruction::LD_rr_d16(rr, n) => write!(f, "LD {:?}, ${:X}", rr, n),
            Instruction::LD_r16_r(rr, r) => write!(f, "LD ({:?}), {:?}", rr, r),
            Instruction::LD_C_A => write!(f, "LD (C), A"),
            Instruction::LD_A_C => write!(f, "LD A, (C)"),
            Instruction::LD_HLD_A => write!(f, "LD (HL-), A"),
            Instruction::LD_HLI_A => write!(f, "LD (HL+), A"),
            Instruction::LD_A_HLD => write!(f, "LD A, (HL-)"),
            Instruction::LD_A_HLI => write!(f, "LD A, (HL+)"),
            Instruction::LD_HL_SP(d) => write!(f, "LD HL, (SP+${:X})", d),
            Instruction::LD_HL_d8(d) => write!(f, "LD (HL), ${:X}", d),
            Instruction::LD_SP_HL => write!(f, "LD SP, HL"),
            Instruction::LDH_A_n(n) => write!(f, "LDH A, (0xFF00+${:X})", n),
            Instruction::LDH_n_A(n) => write!(f, "LDH (0xFF00+${:X}), A", n),
            Instruction::LD_a16_SP(a) => write!(f, "LD (${:X}), SP", a),
            Instruction::LD_a16_A(a) => write!(f, "LD (${:X}), A", a),
            Instruction::LD_A_a16(a) => write!(f, "LD A, (${:X})", a),
            Instruction::BIT_b_r(b, r) => write!(f, "BIT {}, {:?}", b, r),
            Instruction::BIT_b_HL(b) => write!(f, "BIT {}, (HL)", b),
            Instruction::RES_b_r(b, r) => write!(f, "RES {}, {:?}", b, r),
            Instruction::RES_b_HL(b) => write!(f, "RES {}, (HL)", b),
            Instruction::SET_b_r(b, r) => write!(f, "SET {}, {:?}", b, r),
            Instruction::SET_b_HL(b) => write!(f, "SET {}, (HL)", b),
            Instruction::RL_r(r) => write!(f, "RL {:?}", r),
            Instruction::RLC_r(r) => write!(f, "RLC {:?}", r),
            Instruction::RLC_HL => write!(f, "RLC HL"),
            Instruction::SRL_r(r) => write!(f, "SRL {:?}", r),
            Instruction::SRL_HL => write!(f, "SRL (HL)"),
            Instruction::RR_r(r) => write!(f, "RR {:?}", r),
            Instruction::RR_HL => write!(f, "RR (HL)"),
            Instruction::RRC_r(r) => write!(f, "RRC {:?}", r),
            Instruction::RRC_HL => write!(f, "RRC (HL)"),
            Instruction::SLA_r(r) => write!(f, "SLA {:?}", r),
            Instruction::SLA_HL => write!(f, "SLA (HL)"),
            Instruction::SRA_r(r) => write!(f, "SRA {:?}", r),
            Instruction::SRA_HL => write!(f, "SRA (HL)"),
            Instruction::SWAP_r(r) => write!(f, "SWAP {:?}", r),
            Instruction::SWAP_HL => write!(f, "SWAP (HL)"),
        }
    }
}

pub struct CPU<'a> {
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
    ic: u32,
    bootrom_enabled: bool,

    // Interrupts.
    ie: u8,
    if_: u8,
    ime: bool,
    ime_defer: Option<bool>,

    // RAM segments.
    vram: [u8; 0x2000], // 0x8000 - 0x9FFF 
    ram:  [u8; 0x2000], // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    oam:  [u8; 0x9F],   // 0xFE00 - 0xFDFF
    wram: [u8; 0x7E],   // 0xFF80 - 0xFFFE

    // Serial I/O
    sb: u8,
    sc: u8,

    mmu: &'a mut (MMU + 'a),
}

impl <'a> CPU<'a> {
    pub fn new(mmu: &'a mut (MMU + 'a)) -> CPU {
        CPU{
            a: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0, f: 0, sp: 0, pc: 0, ic: 0, bootrom_enabled: true,
            ime: false, ime_defer: None, ie: 0, if_: 0,
            vram: [0; 0x2000],
            ram: [0; 0x2000],
            oam: [0; 0x9F],
            wram: [0; 0x7E],
            sb: 0, sc: 0,
            mmu}
    }

    // Runs the CPU for a single instruction.
    // This is the main "Fetch, decode, execute" cycle.
    pub fn run(&mut self) {
        if self.sb > 0 {
            io::stdout().write(&[self.sb]).unwrap();
            self.sb = 0;
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

            // println!("Servicing interrupt");
            self.sp -= 2;
            let sp = self.sp;
            let pc = self.pc;
            self.mem_write16(sp, pc);
            self.pc = addr;
        }
        
        let _addr = self.pc;
        let inst = self.decode();
        // println!("Inst: {} ; ${:04X}", inst, _addr);

        // Apply deferred change to IME.
        if self.ime_defer.is_some() {
           self.ime = self.ime_defer.unwrap();
           self.ime_defer = None; 
        }

        self.ic += match inst {
            Instruction::NOP => 4,

            Instruction::DI => self.di(),
            Instruction::EI => self.ei(),
            Instruction::RETI => self.reti(),
            Instruction::STOP => self.stop(),

            Instruction::DAA => self.daa(),
            Instruction::CPL => self.cpl(),
            Instruction::CCF => self.ccf(),
            Instruction::SCF => self.scf(),
            Instruction::ADD_A_r(r) => self.add_r(r, false),
            Instruction::ADD_SP_r8(d) => self.add_sp_r8(d),
            Instruction::ADD_A_d8(d) => self.add_d8(d, false),
            Instruction::ADD_A_HL => self.add_hl(false),
            Instruction::ADD_HL_rr(rr) => self.add_rr(rr),
            Instruction::ADC_A_r(r) => self.add_r(r, true),
            Instruction::ADC_A_d8(d) => self.add_d8(d, true),
            Instruction::ADC_A_HL => self.add_hl(true),
            Instruction::INC_r(r) => self.inc_r(r),
            Instruction::INC_rr(r) => self.inc_rr(r),
            Instruction::DEC_r(r) => self.dec_r(r),
            Instruction::DEC_HL => self.dec_hl(),
            Instruction::DEC_rr(r) => self.dec_rr(r),
            Instruction::AND_r(r) => self.bitwise_r(BitwiseOp::AND, r),
            Instruction::AND_d8(d) => self.bitwise_d8(BitwiseOp::AND, d),
            Instruction::AND_HL => self.bitwise_hl(BitwiseOp::AND),
            Instruction::XOR_r(r) => self.bitwise_r(BitwiseOp::XOR, r),
            Instruction::XOR_d8(d) => self.bitwise_d8(BitwiseOp::XOR, d),
            Instruction::XOR_HL => self.bitwise_hl(BitwiseOp::XOR),
            Instruction::OR_r(r) => self.bitwise_r(BitwiseOp::OR, r),
            Instruction::OR_d8(d) => self.bitwise_d8(BitwiseOp::OR, d),
            Instruction::OR_HL => self.bitwise_hl(BitwiseOp::OR),
            Instruction::SUB_r(r) => self.sub_r(r, false, true),
            Instruction::SUB_d8(d) => self.sub_d8(d, false, true),
            Instruction::SUB_HL => self.sub_hl(false, true),
            Instruction::SBC_r(r) => self.sub_r(r, true, true),
            Instruction::SBC_d8(d) => self.sub_d8(d, true, true),
            Instruction::SBC_HL => self.sub_hl(true, true),
            Instruction::CP_r(r) => self.sub_r(r, false, false),
            Instruction::CP_d8(d) => self.sub_d8(d, false, false),
            Instruction::CP_HL => self.sub_hl(false, false),
            Instruction::RLA => self.rl_r(Register8::A, false),
            Instruction::RLCA => self.rlc_r(Register8::A, false),
            Instruction::RRA => self.rr_r(Register8::A, false),
            Instruction::RRCA => self.rrc_r(Register8::A, false),

            Instruction::LD_r_d8(r, v) => self.ld_r_d8(r, v),
            Instruction::LD_rr_d16(r, v) => self.ld_rr_d16(r, v),
            Instruction::LD_r16_r(r16, r) => self.ld_r16_r(r16, r),
            Instruction::LD_r_r(to, from) => self.ld_r_r(to, from),
            Instruction::LD_r_rr(r, rr) => self.ld_r_rr(r, rr),
            Instruction::LD_C_A => self.ldd_c_a(),
            Instruction::LD_A_C => self.ldd_a_c(),
            Instruction::LD_HLD_A => self.ld_hld_a(),
            Instruction::LD_HLI_A => self.ld_hli_a(),
            Instruction::LD_A_HLD => self.ld_a_hld(),
            Instruction::LD_A_HLI => self.ld_a_hli(),
            Instruction::LD_HL_SP(d) => self.ld_hl_sp(d),
            Instruction::LD_HL_d8(d) => self.ld_hl_d8(d),
            Instruction::LD_SP_HL => self.ld_sp_hl(),
            Instruction::LDH_n_A(n) => self.ldh_n_a(n),
            Instruction::LDH_A_n(n) => self.ldh_a_n(n),
            Instruction::LD_A_a16(a) => self.ld_a_a16(a),
            Instruction::LD_a16_A(a) => self.ld_a16_a(a),
            Instruction::LD_a16_SP(a) => self.ld_a16_sp(a),

            Instruction::JP(cc, n) => self.jp(cc, n),
            Instruction::JP_HL => self.jp_hl(),
            Instruction::JR_n(n) => self.jr_n(n),
            Instruction::JR_cc_n(cc, n) => self.jr_cc_n(cc, n),
            Instruction::CALL(cc, addr) => self.call(cc, addr),
            Instruction::RET(cc) => self.ret(cc),
            Instruction::PUSH(r) => self.push(r),
            Instruction::POP(r) => self.pop(r),
            Instruction::RST(a) => self.rst(a),

            Instruction::BIT_b_r(b, r) => self.bit_b_r(b, r),
            Instruction::BIT_b_HL(b) => self.bit_b_hl(b),
            Instruction::RES_b_r(b, r) => self.res_b_r(b, r),
            Instruction::RES_b_HL(b) => self.res_b_hl(b),
            Instruction::SET_b_r(b, r) => self.set_b_r(b, r),
            Instruction::SET_b_HL(b) => self.set_b_hl(b),
            Instruction::RL_r(r) => self.rl_r(r, true),
            Instruction::RLC_r(r) => self.rlc_r(r, true),
            Instruction::RLC_HL => self.rlc_hl(),
            Instruction::SRL_r(r) => self.srl_r(r),
            Instruction::SRL_HL => self.srl_hl(),
            Instruction::RR_r(r) => self.rr_r(r, true),
            Instruction::RR_HL => self.rr_hl(),
            Instruction::SLA_r(r) => self.sla_r(r),
            Instruction::SLA_HL => self.sla_hl(),
            Instruction::SRA_r(r) => self.sra_r(r),
            Instruction::SRA_HL => self.sra_hl(),
            Instruction::RRC_r(r) => self.rrc_r(r, true),
            Instruction::RRC_HL => self.rrc_hl(),
            Instruction::SWAP_r(r) => self.swap_r(r),
            Instruction::SWAP_HL => self.swap_hl(),
        };
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
            0xFF44            => 0x90, // temp hack
            0xFF00            => 0xFF, // TODO: P1 (joypad info)
            0xFF01            => self.sb,
            0xFF02            => self.sc,
            0xFF0F            => self.if_,
            0xFF50            => if self.bootrom_enabled { 0 } else { 1 },
            0xFF03 ... 0xFF79 => 0xFF, // TODO
            0xFF80 ... 0xFFFE => self.wram[(addr - 0xFF80) as usize],
            0xFFFF            => self.ie,
            _ => unreachable!("Emulator not designed to run in an environment where the laws of physics no longer apply")
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

            0x0000 ... 0x7FFF => { }, // TODO: this is forwarded to MBC.
            0x8000 ... 0x9FFF => { self.vram[(addr - 0x8000) as usize] = v },
            0xA000 ... 0xBFFF => { }, // TODO: this is forwarded to MBC.
            0xC000 ... 0xDFFF => { self.ram[(addr - 0xC000) as usize] = v },
            0xE000 ... 0xFDFF => { self.ram[(addr - 0xE000) as usize] = v },
            0xFE00 ... 0xFEFF => { self.oam[(addr - 0xFE00) as usize] = v },
            0xFF00            => { }, // TODO: P1 (joypad info)
            0xFF01            => { self.sb = v },
            0xFF02            => { self.sc = v },
            0xFF0F            => { self.if_ = v & 0x1F },
            0xFF03 ... 0xFF79 => { }, // TODO
            0xFF80 ... 0xFFFE => { self.wram[(addr - 0xFF80) as usize] = v },
            0xFFFF            => { self.ie = v & 0x1F },
            _ => unreachable!("Emulator not designed to run in an environment where the laws of physics no longer apply")
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
    fn decode(&mut self) -> Instruction {
        match self.fetch8() {
            0x00 => Instruction::NOP,
            0x01 => Instruction::LD_rr_d16(Register16::BC, self.fetch16()),
            0x02 => Instruction::LD_r16_r(Register16::BC, Register8::A),
            0x03 => Instruction::INC_rr(Register16::BC),
            0x04 => Instruction::INC_r(Register8::B),
            0x05 => Instruction::DEC_r(Register8::B),
            0x06 => Instruction::LD_r_d8(Register8::B, self.fetch8()),
            0x07 => Instruction::RLCA,
            0x08 => Instruction::LD_a16_SP(self.fetch16()),
            0x09 => Instruction::ADD_HL_rr(Register16::BC),
            0x0A => Instruction::LD_r_rr(Register8::A, Register16::BC),
            0x0B => Instruction::DEC_rr(Register16::BC),
            0x0C => Instruction::INC_r(Register8::C),
            0x0D => Instruction::DEC_r(Register8::C),
            0x0E => Instruction::LD_r_d8(Register8::C, self.fetch8()),
            0x0F => Instruction::RRCA,
            0x10 => { self.fetch8(); Instruction::STOP },
            0x11 => Instruction::LD_rr_d16(Register16::DE, self.fetch16()),
            0x12 => Instruction::LD_r16_r(Register16::DE, Register8::A),
            0x13 => Instruction::INC_rr(Register16::DE),
            0x14 => Instruction::INC_r(Register8::D),
            0x15 => Instruction::DEC_r(Register8::D),
            0x16 => Instruction::LD_r_d8(Register8::D, self.fetch8()), 
            0x17 => Instruction::RLA,
            0x18 => Instruction::JR_n(self.fetch8()),
            0x19 => Instruction::ADD_HL_rr(Register16::DE),
            0x1A => Instruction::LD_r_rr(Register8::A, Register16::DE),
            0x1B => Instruction::DEC_rr(Register16::DE),
            0x1C => Instruction::INC_r(Register8::E),
            0x1D => Instruction::DEC_r(Register8::E),
            0x1E => Instruction::LD_r_d8(Register8::E, self.fetch8()),
            0x1F => Instruction::RRA,
            0x20 => Instruction::JR_cc_n(ConditionFlag::NZ, self.fetch8()),
            0x21 => Instruction::LD_rr_d16(Register16::HL, self.fetch16()),
            0x22 => Instruction::LD_HLI_A,
            0x23 => Instruction::INC_rr(Register16::HL),
            0x24 => Instruction::INC_r(Register8::H),
            0x25 => Instruction::DEC_r(Register8::H),
            0x26 => Instruction::LD_r_d8(Register8::H, self.fetch8()),
            0x27 => Instruction::DAA,
            0x28 => Instruction::JR_cc_n(ConditionFlag::Z, self.fetch8()),
            0x29 => Instruction::ADD_HL_rr(Register16::HL),
            0x2A => Instruction::LD_A_HLI,
            0x2B => Instruction::DEC_rr(Register16::HL),
            0x2C => Instruction::INC_r(Register8::L),
            0x2D => Instruction::DEC_r(Register8::L),
            0x2E => Instruction::LD_r_d8(Register8::L, self.fetch8()),
            0x2F => Instruction::CPL,
            0x30 => Instruction::JR_cc_n(ConditionFlag::NC, self.fetch8()),
            0x31 => Instruction::LD_rr_d16(Register16::SP, self.fetch16()),
            0x32 => Instruction::LD_HLD_A,
            0x33 => Instruction::INC_rr(Register16::SP),

            0x35 => Instruction::DEC_HL,
            0x36 => Instruction::LD_HL_d8(self.fetch8()),
            0x37 => Instruction::SCF,
            0x38 => Instruction::JR_cc_n(ConditionFlag::C, self.fetch8()),
            0x39 => Instruction::ADD_HL_rr(Register16::SP),
            0x3A => Instruction::LD_A_HLD,
            0x3B => Instruction::DEC_rr(Register16::SP),
            0x3C => Instruction::INC_r(Register8::A),
            0x3D => Instruction::DEC_r(Register8::A),
            0x3E => Instruction::LD_r_d8(Register8::A, self.fetch8()),
            0x3F => Instruction::CCF,
            0x40 => Instruction::LD_r_r(Register8::B, Register8::B),
            0x41 => Instruction::LD_r_r(Register8::B, Register8::C),
            0x42 => Instruction::LD_r_r(Register8::B, Register8::D),
            0x43 => Instruction::LD_r_r(Register8::B, Register8::E),
            0x44 => Instruction::LD_r_r(Register8::B, Register8::H),
            0x45 => Instruction::LD_r_r(Register8::B, Register8::L),
            0x46 => Instruction::LD_r_rr(Register8::B, Register16::HL),
            0x47 => Instruction::LD_r_r(Register8::B, Register8::A),
            0x48 => Instruction::LD_r_r(Register8::C, Register8::B),
            0x49 => Instruction::LD_r_r(Register8::C, Register8::C),
            0x4A => Instruction::LD_r_r(Register8::C, Register8::D),
            0x4B => Instruction::LD_r_r(Register8::C, Register8::E),
            0x4C => Instruction::LD_r_r(Register8::C, Register8::H),
            0x4D => Instruction::LD_r_r(Register8::C, Register8::L),
            0x4E => Instruction::LD_r_rr(Register8::C, Register16::HL),
            0x4F => Instruction::LD_r_r(Register8::C, Register8::A),
            0x50 => Instruction::LD_r_r(Register8::D, Register8::B),
            0x51 => Instruction::LD_r_r(Register8::D, Register8::C),
            0x52 => Instruction::LD_r_r(Register8::D, Register8::D),
            0x53 => Instruction::LD_r_r(Register8::D, Register8::E),
            0x54 => Instruction::LD_r_r(Register8::D, Register8::H),
            0x55 => Instruction::LD_r_r(Register8::D, Register8::L),
            0x56 => Instruction::LD_r_rr(Register8::D, Register16::HL),
            0x57 => Instruction::LD_r_r(Register8::D, Register8::A),
            0x58 => Instruction::LD_r_r(Register8::E, Register8::B),
            0x59 => Instruction::LD_r_r(Register8::E, Register8::C),
            0x5A => Instruction::LD_r_r(Register8::E, Register8::D),
            0x5B => Instruction::LD_r_r(Register8::E, Register8::E),
            0x5C => Instruction::LD_r_r(Register8::E, Register8::H),
            0x5D => Instruction::LD_r_r(Register8::E, Register8::L),
            0x5E => Instruction::LD_r_rr(Register8::E, Register16::HL),
            0x5F => Instruction::LD_r_r(Register8::E, Register8::A),
            0x60 => Instruction::LD_r_r(Register8::H, Register8::B),
            0x61 => Instruction::LD_r_r(Register8::H, Register8::C),
            0x62 => Instruction::LD_r_r(Register8::H, Register8::D),
            0x63 => Instruction::LD_r_r(Register8::H, Register8::E),
            0x64 => Instruction::LD_r_r(Register8::H, Register8::H),
            0x65 => Instruction::LD_r_r(Register8::H, Register8::L),
            0x66 => Instruction::LD_r_rr(Register8::H, Register16::HL),
            0x67 => Instruction::LD_r_r(Register8::H, Register8::A),
            0x68 => Instruction::LD_r_r(Register8::L, Register8::B),
            0x69 => Instruction::LD_r_r(Register8::L, Register8::C),
            0x6A => Instruction::LD_r_r(Register8::L, Register8::D),
            0x6B => Instruction::LD_r_r(Register8::L, Register8::E),
            0x6C => Instruction::LD_r_r(Register8::L, Register8::H),
            0x6D => Instruction::LD_r_r(Register8::L, Register8::L),
            0x6E => Instruction::LD_r_rr(Register8::L, Register16::HL),
            0x6F => Instruction::LD_r_r(Register8::L, Register8::A),
            0x70 => Instruction::LD_r16_r(Register16::HL, Register8::B),
            0x71 => Instruction::LD_r16_r(Register16::HL, Register8::C),
            0x72 => Instruction::LD_r16_r(Register16::HL, Register8::D),
            0x73 => Instruction::LD_r16_r(Register16::HL, Register8::E),
            0x74 => Instruction::LD_r16_r(Register16::HL, Register8::H),
            0x75 => Instruction::LD_r16_r(Register16::HL, Register8::L),

            0x77 => Instruction::LD_r16_r(Register16::HL, Register8::A),
            0x78 => Instruction::LD_r_r(Register8::A, Register8::B),
            0x79 => Instruction::LD_r_r(Register8::A, Register8::C),
            0x7A => Instruction::LD_r_r(Register8::A, Register8::D),
            0x7B => Instruction::LD_r_r(Register8::A, Register8::E),
            0x7C => Instruction::LD_r_r(Register8::A, Register8::H),
            0x7D => Instruction::LD_r_r(Register8::A, Register8::L),
            0x7E => Instruction::LD_r_rr(Register8::A, Register16::HL),
            0x7F => Instruction::LD_r_r(Register8::A, Register8::A),
            0x80 => Instruction::ADD_A_r(Register8::B),
            0x81 => Instruction::ADD_A_r(Register8::C),
            0x82 => Instruction::ADD_A_r(Register8::D),
            0x83 => Instruction::ADD_A_r(Register8::E),
            0x84 => Instruction::ADD_A_r(Register8::H),
            0x85 => Instruction::ADD_A_r(Register8::L),
            0x86 => Instruction::ADD_A_HL,
            0x87 => Instruction::ADD_A_r(Register8::A),
            0x88 => Instruction::ADC_A_r(Register8::B),
            0x89 => Instruction::ADC_A_r(Register8::C),
            0x8A => Instruction::ADC_A_r(Register8::D),
            0x8B => Instruction::ADC_A_r(Register8::E),
            0x8C => Instruction::ADC_A_r(Register8::H),
            0x8D => Instruction::ADC_A_r(Register8::L),
            0x8E => Instruction::ADC_A_HL,
            0x8F => Instruction::ADC_A_r(Register8::A),
            0x90 => Instruction::SUB_r(Register8::B),
            0x91 => Instruction::SUB_r(Register8::C),
            0x92 => Instruction::SUB_r(Register8::D),
            0x93 => Instruction::SUB_r(Register8::E),
            0x94 => Instruction::SUB_r(Register8::H),
            0x95 => Instruction::SUB_r(Register8::L),
            0x96 => Instruction::SUB_HL,
            0x97 => Instruction::SUB_r(Register8::A),
            0x98 => Instruction::SBC_r(Register8::B),
            0x99 => Instruction::SBC_r(Register8::C),
            0x9A => Instruction::SBC_r(Register8::D),
            0x9B => Instruction::SBC_r(Register8::E),
            0x9C => Instruction::SBC_r(Register8::H),
            0x9D => Instruction::SBC_r(Register8::L),
            0x9E => Instruction::SBC_HL,
            0x9F => Instruction::SBC_r(Register8::A),
            0xA0 => Instruction::AND_r(Register8::B),
            0xA1 => Instruction::AND_r(Register8::C),
            0xA2 => Instruction::AND_r(Register8::D),
            0xA3 => Instruction::AND_r(Register8::E),
            0xA4 => Instruction::AND_r(Register8::H),
            0xA5 => Instruction::AND_r(Register8::L),
            0xA6 => Instruction::AND_HL,
            0xA7 => Instruction::AND_r(Register8::A),
            0xA8 => Instruction::XOR_r(Register8::B),
            0xA9 => Instruction::XOR_r(Register8::C),
            0xAA => Instruction::XOR_r(Register8::D),
            0xAB => Instruction::XOR_r(Register8::E),
            0xAC => Instruction::XOR_r(Register8::H),
            0xAD => Instruction::XOR_r(Register8::L),
            0xAE => Instruction::XOR_HL,
            0xAF => Instruction::XOR_r(Register8::A),
            0xB0 => Instruction::OR_r(Register8::B),
            0xB1 => Instruction::OR_r(Register8::C),
            0xB2 => Instruction::OR_r(Register8::D),
            0xB3 => Instruction::OR_r(Register8::E),
            0xB4 => Instruction::OR_r(Register8::H),
            0xB5 => Instruction::OR_r(Register8::L),
            0xB6 => Instruction::OR_HL,
            0xB7 => Instruction::OR_r(Register8::A),
            0xB8 => Instruction::CP_r(Register8::B),
            0xB9 => Instruction::CP_r(Register8::C),
            0xBA => Instruction::CP_r(Register8::D),
            0xBB => Instruction::CP_r(Register8::E),
            0xBC => Instruction::CP_r(Register8::H),
            0xBD => Instruction::CP_r(Register8::L),
            0xBE => Instruction::CP_HL,
            0xBF => Instruction::CP_r(Register8::A),
            0xC0 => Instruction::RET(Some(ConditionFlag::NZ)),
            0xC1 => Instruction::POP(Register16::BC),
            0xC2 => Instruction::JP(Some(ConditionFlag::NZ), self.fetch16()),
            0xC3 => Instruction::JP(None, self.fetch16()),
            0xC4 => Instruction::CALL(Some(ConditionFlag::NZ), self.fetch16()),
            0xC5 => Instruction::PUSH(Register16::BC),
            0xC6 => Instruction::ADD_A_d8(self.fetch8()),
            0xC7 => Instruction::RST(0x00),
            0xC8 => Instruction::RET(Some(ConditionFlag::Z)),
            0xC9 => Instruction::RET(None),
            0xCA => Instruction::JP(Some(ConditionFlag::Z), self.fetch16()),
            0xCB => self.decode_extended(),
            0xCC => Instruction::CALL(Some(ConditionFlag::Z), self.fetch16()),
            0xCD => Instruction::CALL(None, self.fetch16()),
            0xCE => Instruction::ADC_A_d8(self.fetch8()),
            0xCF => Instruction::RST(0x08),
            0xD0 => Instruction::RET(Some(ConditionFlag::NC)),
            0xD1 => Instruction::POP(Register16::DE),
            0xD2 => Instruction::JP(Some(ConditionFlag::NC), self.fetch16()),

            0xD4 => Instruction::CALL(Some(ConditionFlag::NC), self.fetch16()),
            0xD5 => Instruction::PUSH(Register16::DE),
            0xD6 => Instruction::SUB_d8(self.fetch8()),
            0xD7 => Instruction::RST(0x10),
            0xD8 => Instruction::RET(Some(ConditionFlag::C)),
            0xD9 => Instruction::RETI,
            0xDA => Instruction::JP(Some(ConditionFlag::C), self.fetch16()),

            0xDC => Instruction::CALL(Some(ConditionFlag::C), self.fetch16()),

            0xDE => Instruction::SBC_d8(self.fetch8()),
            0xDF => Instruction::RST(0x18),

            0xE0 => Instruction::LDH_n_A(self.fetch8()),
            0xE1 => Instruction::POP(Register16::HL),
            0xE2 => Instruction::LD_C_A,

            0xE5 => Instruction::PUSH(Register16::HL),
            0xE6 => Instruction::AND_d8(self.fetch8()),
            0xE7 => Instruction::RST(0x20),
            0xE8 => Instruction::ADD_SP_r8(self.fetch8() as i8),
            0xE9 => Instruction::JP_HL,

            0xEA => Instruction::LD_a16_A(self.fetch16()),

            0xEE => Instruction::XOR_d8(self.fetch8()),
            0xEF => Instruction::RST(0x28),

            0xF0 => Instruction::LDH_A_n(self.fetch8()),
            0xF1 => Instruction::POP(Register16::AF),
            0xF2 => Instruction::LD_A_C,
            0xF3 => Instruction::DI,

            0xF5 => Instruction::PUSH(Register16::AF),
            0xF6 => Instruction::OR_d8(self.fetch8()),
            0xF7 => Instruction::RST(0x30),
            0xF8 => Instruction::LD_HL_SP(self.fetch8() as i8),
            0xF9 => Instruction::LD_SP_HL,
            0xFA => Instruction::LD_A_a16(self.fetch16()),
            0xFB => Instruction::EI,
            0xFE => Instruction::CP_d8(self.fetch8()),
            0xFF => Instruction::RST(0x38),

            n => {
                panic!("Unexpected opcode 0x{:X} encountered", n);
            }
        }
    }

    // Decodes extended instruction following 0xCB instruction.
    fn decode_extended(&mut self) -> Instruction {
        match self.fetch8() {
            0x00 => Instruction::RLC_r(Register8::B),
            0x01 => Instruction::RLC_r(Register8::C),
            0x02 => Instruction::RLC_r(Register8::D),
            0x03 => Instruction::RLC_r(Register8::E),
            0x04 => Instruction::RLC_r(Register8::H),
            0x05 => Instruction::RLC_r(Register8::L),
            0x06 => Instruction::RLC_HL,
            0x07 => Instruction::RLC_r(Register8::A),
            0x08 => Instruction::RRC_r(Register8::B),
            0x09 => Instruction::RRC_r(Register8::C),
            0x0A => Instruction::RRC_r(Register8::D),
            0x0B => Instruction::RRC_r(Register8::E),
            0x0C => Instruction::RRC_r(Register8::H),
            0x0D => Instruction::RRC_r(Register8::L),
            0x0E => Instruction::RRC_HL,
            0x0F => Instruction::RRC_r(Register8::A),
            0x10 => Instruction::RL_r(Register8::B),
            0x11 => Instruction::RL_r(Register8::C),
            0x12 => Instruction::RL_r(Register8::D),
            0x13 => Instruction::RL_r(Register8::E),
            0x14 => Instruction::RL_r(Register8::H),
            0x15 => Instruction::RL_r(Register8::L),

            0x17 => Instruction::RL_r(Register8::A),
            0x18 => Instruction::RR_r(Register8::B),
            0x19 => Instruction::RR_r(Register8::C),
            0x1A => Instruction::RR_r(Register8::D),
            0x1B => Instruction::RR_r(Register8::E),
            0x1C => Instruction::RR_r(Register8::H),
            0x1D => Instruction::RR_r(Register8::L),
            0x1E => Instruction::RR_HL,
            0x1F => Instruction::RR_r(Register8::A),
            0x20 => Instruction::SLA_r(Register8::B),
            0x21 => Instruction::SLA_r(Register8::C),
            0x22 => Instruction::SLA_r(Register8::D),
            0x23 => Instruction::SLA_r(Register8::E),
            0x24 => Instruction::SLA_r(Register8::H),
            0x25 => Instruction::SLA_r(Register8::L),
            0x26 => Instruction::SLA_HL,
            0x27 => Instruction::SLA_r(Register8::A),
            0x28 => Instruction::SRA_r(Register8::B),
            0x29 => Instruction::SRA_r(Register8::C),
            0x2A => Instruction::SRA_r(Register8::D),
            0x2B => Instruction::SRA_r(Register8::E),
            0x2C => Instruction::SRA_r(Register8::H),
            0x2D => Instruction::SRA_r(Register8::L),
            0x2E => Instruction::SRA_HL,
            0x2F => Instruction::SRA_r(Register8::A),
            0x30 => Instruction::SWAP_r(Register8::B),
            0x31 => Instruction::SWAP_r(Register8::C),
            0x32 => Instruction::SWAP_r(Register8::D),
            0x33 => Instruction::SWAP_r(Register8::E),
            0x34 => Instruction::SWAP_r(Register8::H),
            0x35 => Instruction::SWAP_r(Register8::L),
            0x36 => Instruction::SWAP_HL,
            0x37 => Instruction::SWAP_r(Register8::A),
            0x38 => Instruction::SRL_r(Register8::B),
            0x39 => Instruction::SRL_r(Register8::C),
            0x3A => Instruction::SRL_r(Register8::D),
            0x3B => Instruction::SRL_r(Register8::E),
            0x3C => Instruction::SRL_r(Register8::H),
            0x3D => Instruction::SRL_r(Register8::L),
            0x3E => Instruction::SRL_HL,
            0x3F => Instruction::SRL_r(Register8::A),
            0x40 => Instruction::BIT_b_r(0, Register8::B),
            0x41 => Instruction::BIT_b_r(0, Register8::C),
            0x42 => Instruction::BIT_b_r(0, Register8::D),
            0x43 => Instruction::BIT_b_r(0, Register8::E),
            0x44 => Instruction::BIT_b_r(0, Register8::H),
            0x45 => Instruction::BIT_b_r(0, Register8::L),
            0x46 => Instruction::BIT_b_HL(0),
            0x47 => Instruction::BIT_b_r(0, Register8::A),
            0x48 => Instruction::BIT_b_r(1, Register8::B),
            0x49 => Instruction::BIT_b_r(1, Register8::C),
            0x4A => Instruction::BIT_b_r(1, Register8::D),
            0x4B => Instruction::BIT_b_r(1, Register8::E),
            0x4C => Instruction::BIT_b_r(1, Register8::H),
            0x4D => Instruction::BIT_b_r(1, Register8::L),
            0x4E => Instruction::BIT_b_HL(1),
            0x4F => Instruction::BIT_b_r(1, Register8::A),
            0x50 => Instruction::BIT_b_r(2, Register8::B),
            0x51 => Instruction::BIT_b_r(2, Register8::C),
            0x52 => Instruction::BIT_b_r(2, Register8::D),
            0x53 => Instruction::BIT_b_r(2, Register8::E),
            0x54 => Instruction::BIT_b_r(2, Register8::H),
            0x55 => Instruction::BIT_b_r(2, Register8::L),
            0x56 => Instruction::BIT_b_HL(2),
            0x57 => Instruction::BIT_b_r(2, Register8::A),
            0x58 => Instruction::BIT_b_r(3, Register8::B),
            0x59 => Instruction::BIT_b_r(3, Register8::C),
            0x5A => Instruction::BIT_b_r(3, Register8::D),
            0x5B => Instruction::BIT_b_r(3, Register8::E),
            0x5C => Instruction::BIT_b_r(3, Register8::H),
            0x5D => Instruction::BIT_b_r(3, Register8::L),
            0x5E => Instruction::BIT_b_HL(3),
            0x5F => Instruction::BIT_b_r(3, Register8::A),
            0x60 => Instruction::BIT_b_r(4, Register8::B),
            0x61 => Instruction::BIT_b_r(4, Register8::C),
            0x62 => Instruction::BIT_b_r(4, Register8::D),
            0x63 => Instruction::BIT_b_r(4, Register8::E),
            0x64 => Instruction::BIT_b_r(4, Register8::H),
            0x65 => Instruction::BIT_b_r(4, Register8::L),
            0x66 => Instruction::BIT_b_HL(4),
            0x67 => Instruction::BIT_b_r(4, Register8::A),
            0x68 => Instruction::BIT_b_r(5, Register8::B),
            0x69 => Instruction::BIT_b_r(5, Register8::C),
            0x6A => Instruction::BIT_b_r(5, Register8::D),
            0x6B => Instruction::BIT_b_r(5, Register8::E),
            0x6C => Instruction::BIT_b_r(5, Register8::H),
            0x6D => Instruction::BIT_b_r(5, Register8::L),
            0x6E => Instruction::BIT_b_HL(5),
            0x6F => Instruction::BIT_b_r(5, Register8::A),
            0x70 => Instruction::BIT_b_r(6, Register8::B),
            0x71 => Instruction::BIT_b_r(6, Register8::C),
            0x72 => Instruction::BIT_b_r(6, Register8::D),
            0x73 => Instruction::BIT_b_r(6, Register8::E),
            0x74 => Instruction::BIT_b_r(6, Register8::H),
            0x75 => Instruction::BIT_b_r(6, Register8::L),
            0x76 => Instruction::BIT_b_HL(6),
            0x77 => Instruction::BIT_b_r(6, Register8::A),
            0x78 => Instruction::BIT_b_r(7, Register8::B),
            0x79 => Instruction::BIT_b_r(7, Register8::C),
            0x7A => Instruction::BIT_b_r(7, Register8::D),
            0x7B => Instruction::BIT_b_r(7, Register8::E),
            0x7C => Instruction::BIT_b_r(7, Register8::H),
            0x7D => Instruction::BIT_b_r(7, Register8::L),
            0x7E => Instruction::BIT_b_HL(7),
            0x7F => Instruction::BIT_b_r(7, Register8::A),
            0x80 => Instruction::RES_b_r(0, Register8::B),
            0x81 => Instruction::RES_b_r(0, Register8::C),
            0x82 => Instruction::RES_b_r(0, Register8::D),
            0x83 => Instruction::RES_b_r(0, Register8::E),
            0x84 => Instruction::RES_b_r(0, Register8::H),
            0x85 => Instruction::RES_b_r(0, Register8::L),
            0x86 => Instruction::RES_b_HL(0),
            0x87 => Instruction::RES_b_r(0, Register8::A),
            0x88 => Instruction::RES_b_r(1, Register8::B),
            0x89 => Instruction::RES_b_r(1, Register8::C),
            0x8A => Instruction::RES_b_r(1, Register8::D),
            0x8B => Instruction::RES_b_r(1, Register8::E),
            0x8C => Instruction::RES_b_r(1, Register8::H),
            0x8D => Instruction::RES_b_r(1, Register8::L),
            0x8E => Instruction::RES_b_HL(1),
            0x8F => Instruction::RES_b_r(1, Register8::A),
            0x90 => Instruction::RES_b_r(2, Register8::B),
            0x91 => Instruction::RES_b_r(2, Register8::C),
            0x92 => Instruction::RES_b_r(2, Register8::D),
            0x93 => Instruction::RES_b_r(2, Register8::E),
            0x94 => Instruction::RES_b_r(2, Register8::H),
            0x95 => Instruction::RES_b_r(2, Register8::L),
            0x96 => Instruction::RES_b_HL(2),
            0x97 => Instruction::RES_b_r(2, Register8::A),
            0x98 => Instruction::RES_b_r(3, Register8::B),
            0x99 => Instruction::RES_b_r(3, Register8::C),
            0x9A => Instruction::RES_b_r(3, Register8::D),
            0x9B => Instruction::RES_b_r(3, Register8::E),
            0x9C => Instruction::RES_b_r(3, Register8::H),
            0x9D => Instruction::RES_b_r(3, Register8::L),
            0x9E => Instruction::RES_b_HL(3),
            0x9F => Instruction::RES_b_r(3, Register8::A),
            0xA0 => Instruction::RES_b_r(4, Register8::B),
            0xA1 => Instruction::RES_b_r(4, Register8::C),
            0xA2 => Instruction::RES_b_r(4, Register8::D),
            0xA3 => Instruction::RES_b_r(4, Register8::E),
            0xA4 => Instruction::RES_b_r(4, Register8::H),
            0xA5 => Instruction::RES_b_r(4, Register8::L),
            0xA6 => Instruction::RES_b_HL(4),
            0xA7 => Instruction::RES_b_r(4, Register8::A),
            0xA8 => Instruction::RES_b_r(5, Register8::B),
            0xA9 => Instruction::RES_b_r(5, Register8::C),
            0xAA => Instruction::RES_b_r(5, Register8::D),
            0xAB => Instruction::RES_b_r(5, Register8::E),
            0xAC => Instruction::RES_b_r(5, Register8::H),
            0xAD => Instruction::RES_b_r(5, Register8::L),
            0xAE => Instruction::RES_b_HL(5),
            0xAF => Instruction::RES_b_r(5, Register8::A),
            0xB0 => Instruction::RES_b_r(6, Register8::B),
            0xB1 => Instruction::RES_b_r(6, Register8::C),
            0xB2 => Instruction::RES_b_r(6, Register8::D),
            0xB3 => Instruction::RES_b_r(6, Register8::E),
            0xB4 => Instruction::RES_b_r(6, Register8::H),
            0xB5 => Instruction::RES_b_r(6, Register8::L),
            0xB6 => Instruction::RES_b_HL(6),
            0xB7 => Instruction::RES_b_r(6, Register8::A),
            0xB8 => Instruction::RES_b_r(7, Register8::B),
            0xB9 => Instruction::RES_b_r(7, Register8::C),
            0xBA => Instruction::RES_b_r(7, Register8::D),
            0xBB => Instruction::RES_b_r(7, Register8::E),
            0xBC => Instruction::RES_b_r(7, Register8::H),
            0xBD => Instruction::RES_b_r(7, Register8::L),
            0xBE => Instruction::RES_b_HL(7),
            0xBF => Instruction::RES_b_r(7, Register8::A),
            0xC0 => Instruction::SET_b_r(0, Register8::B),
            0xC1 => Instruction::SET_b_r(0, Register8::C),
            0xC2 => Instruction::SET_b_r(0, Register8::D),
            0xC3 => Instruction::SET_b_r(0, Register8::E),
            0xC4 => Instruction::SET_b_r(0, Register8::H),
            0xC5 => Instruction::SET_b_r(0, Register8::L),
            0xC6 => Instruction::SET_b_HL(0),
            0xC7 => Instruction::SET_b_r(0, Register8::A),
            0xC8 => Instruction::SET_b_r(1, Register8::B),
            0xC9 => Instruction::SET_b_r(1, Register8::C),
            0xCA => Instruction::SET_b_r(1, Register8::D),
            0xCB => Instruction::SET_b_r(1, Register8::E),
            0xCC => Instruction::SET_b_r(1, Register8::H),
            0xCD => Instruction::SET_b_r(1, Register8::L),
            0xCE => Instruction::SET_b_HL(1),
            0xCF => Instruction::SET_b_r(1, Register8::A),
            0xD0 => Instruction::SET_b_r(2, Register8::B),
            0xD1 => Instruction::SET_b_r(2, Register8::C),
            0xD2 => Instruction::SET_b_r(2, Register8::D),
            0xD3 => Instruction::SET_b_r(2, Register8::E),
            0xD4 => Instruction::SET_b_r(2, Register8::H),
            0xD5 => Instruction::SET_b_r(2, Register8::L),
            0xD6 => Instruction::SET_b_HL(2),
            0xD7 => Instruction::SET_b_r(2, Register8::A),
            0xD8 => Instruction::SET_b_r(3, Register8::B),
            0xD9 => Instruction::SET_b_r(3, Register8::C),
            0xDA => Instruction::SET_b_r(3, Register8::D),
            0xDB => Instruction::SET_b_r(3, Register8::E),
            0xDC => Instruction::SET_b_r(3, Register8::H),
            0xDD => Instruction::SET_b_r(3, Register8::L),
            0xDE => Instruction::SET_b_HL(3),
            0xDF => Instruction::SET_b_r(3, Register8::A),
            0xE0 => Instruction::SET_b_r(4, Register8::B),
            0xE1 => Instruction::SET_b_r(4, Register8::C),
            0xE2 => Instruction::SET_b_r(4, Register8::D),
            0xE3 => Instruction::SET_b_r(4, Register8::E),
            0xE4 => Instruction::SET_b_r(4, Register8::H),
            0xE5 => Instruction::SET_b_r(4, Register8::L),
            0xE6 => Instruction::SET_b_HL(4),
            0xE7 => Instruction::SET_b_r(4, Register8::A),
            0xE8 => Instruction::SET_b_r(5, Register8::B),
            0xE9 => Instruction::SET_b_r(5, Register8::C),
            0xEA => Instruction::SET_b_r(5, Register8::D),
            0xEB => Instruction::SET_b_r(5, Register8::E),
            0xEC => Instruction::SET_b_r(5, Register8::H),
            0xED => Instruction::SET_b_r(5, Register8::L),
            0xEE => Instruction::SET_b_HL(5),
            0xEF => Instruction::SET_b_r(5, Register8::A),
            0xF0 => Instruction::SET_b_r(6, Register8::B),
            0xF1 => Instruction::SET_b_r(6, Register8::C),
            0xF2 => Instruction::SET_b_r(6, Register8::D),
            0xF3 => Instruction::SET_b_r(6, Register8::E),
            0xF4 => Instruction::SET_b_r(6, Register8::H),
            0xF5 => Instruction::SET_b_r(6, Register8::L),
            0xF6 => Instruction::SET_b_HL(6),
            0xF7 => Instruction::SET_b_r(6, Register8::A),
            0xF8 => Instruction::SET_b_r(7, Register8::B),
            0xF9 => Instruction::SET_b_r(7, Register8::C),
            0xFA => Instruction::SET_b_r(7, Register8::D),
            0xFB => Instruction::SET_b_r(7, Register8::E),
            0xFC => Instruction::SET_b_r(7, Register8::H),
            0xFD => Instruction::SET_b_r(7, Register8::L),
            0xFE => Instruction::SET_b_HL(7),
            0xFF => Instruction::SET_b_r(7, Register8::A),

            n => {
                panic!("Unexpected extended opcode 0x{:X} encountered", n);
            }
        }
    }

    fn get_reg8(&self, reg: Register8) -> u8 {
        match reg {
            Register8::A => self.a,
            Register8::B => self.b,
            Register8::C => self.c,
            Register8::D => self.d,
            Register8::E => self.e,
            Register8::H => self.h,
            Register8::L => self.l,
        }
    }

    fn set_reg8(&mut self, reg: Register8, v: u8) {
        match reg {
            Register8::A => { self.a = v },
            Register8::B => { self.b = v },
            Register8::C => { self.c = v },
            Register8::D => { self.d = v },
            Register8::E => { self.e = v },
            Register8::H => { self.h = v },
            Register8::L => { self.l = v },
        };
    }

    fn get_reg16(&self, reg: Register16) -> u16 {
        let (hi, lo) = match reg {
            Register16::AF => { (self.a, self.f) },
            Register16::BC => { (self.b, self.c) },
            Register16::DE => { (self.d, self.e) },
            Register16::HL => { (self.h, self.l) },
            Register16::SP => { return self.sp },
        };

        (hi as u16) << 8 | (lo as u16)
    }

    fn set_reg16(&mut self, reg: Register16, v: u16) {
        let (hi, lo) = match reg {
            Register16::AF => { (&mut self.a, &mut self.f) },
            Register16::BC => { (&mut self.b, &mut self.c) },
            Register16::DE => { (&mut self.d, &mut self.e) },
            Register16::HL => { (&mut self.h, &mut self.l) },
            Register16::SP => { self.sp = v; return },
        };

        *hi = ((v & 0xFF00) >> 8) as u8;
        *lo = (v & 0xFF) as u8;
    }

    fn check_jmp_condition(&self, cc: ConditionFlag) -> bool {
        match cc {
            ConditionFlag::NZ => (self.f & FLAG_ZERO) == 0,
            ConditionFlag::Z  => (self.f & FLAG_ZERO) != 0,
            ConditionFlag::NC => (self.f & FLAG_CARRY) == 0,
            ConditionFlag::C  => (self.f & FLAG_CARRY) != 0,
        }
    }

    fn di(&mut self) -> u32 {
        self.ime_defer = Some(false);
        4
    }

    fn ei(&mut self) -> u32 {
        self.ime_defer = Some(true);
        4
    }

    fn reti(&mut self) -> u32 {
        self.ime_defer = Some(true);
        self.ret(None)
    }

    // STOP 
    fn stop(&mut self) -> u32 {
        // TODO:
        4
    }

    // INC r
    // Flags:
    // Z N H C
    // * 0 * -
    fn inc_r(&mut self, reg: Register8) -> u32 {
        let v = self.get_reg8(reg);
        let (v, _) = v.overflowing_add(1);
        self.set_reg8(reg, v);

        // Clear Z, N, H flags. If we overflowed top or bottom nibbles
        // we'll set the appropriate flags below.
        self.f &= !(FLAG_ZERO | FLAG_HALF_CARRY | FLAG_SUBTRACT);

        // Set zero flag if we overflowed.
        if v == 0 {
            self.f |= FLAG_ZERO;
        }

        // Set half carry flag if lower nibble overflowed.
        if (v & 0x0F) == 0 {
            self.f |= FLAG_HALF_CARRY;
        }

        4
    }

    // INC rr
    // Flags:
    // Z N H C
    // - - - -
    fn inc_rr(&mut self, r: Register16) -> u32 {
        let v = self.get_reg16(r);
        let (v, _) = v.overflowing_add(1);
        self.set_reg16(r, v);

        8
    }

    // DEC r
    // Flags:
    // Z N H C
    // * 1 * -
    fn dec(&mut self, v: u8) -> u8 {
        let v = v.wrapping_sub(1);
        self.f &= !(FLAG_ZERO | FLAG_HALF_CARRY);
        self.f |= FLAG_SUBTRACT;

        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        if (v & 0xF) == 0xF {
            self.f |= FLAG_HALF_CARRY;
        }

        v
    }

    fn dec_r(&mut self, r: Register8) -> u32 {
        let v = self.get_reg8(r);
        let v = self.dec(v);
        self.set_reg8(r, v);
        4
    }

    fn dec_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        let v = self.dec(v);
        self.mem_write8(addr, v);
        12
    }

    // DEC rr
    // Flags:
    // Z N H C
    // - - - -
    fn dec_rr(&mut self, r: Register16) -> u32 {
        let v = self.get_reg16(r);
        let (v, _) = v.overflowing_sub(1);
        self.set_reg16(r, v);

        8
    }

    // Raw ADD instruction used by both add_r and add_hl
    fn add(&mut self, v: u8, carry: bool) {
        let a = self.get_reg8(Register8::A);
        
        let carry = if carry && (self.f & FLAG_CARRY > 0) { 1 } else { 0 };
        self.f = 0;

        let new_a = a.wrapping_add(v).wrapping_add(carry);
        self.set_reg8(Register8::A, new_a);


        if new_a == 0 {
            self.f |= FLAG_ZERO;
        }

        if (a as u16) + (v as u16) + (carry as u16) > 0xFF {
            self.f |= FLAG_CARRY;
        }

        if (((a & 0xF) + (v & 0xF)) + carry) & 0x10 == 0x10 {
            self.f |= FLAG_HALF_CARRY;
        }
    }

    // Raw SUB/CP instruction used by both sub_r and sub_hl
    fn sub(&mut self, v: u8, carry: bool, store: bool) {
        let a = self.get_reg8(Register8::A);
        
        let carry = if carry && (self.f & FLAG_CARRY != 0) { 1 } else { 0 };
        let new_a = a.wrapping_sub(v).wrapping_sub(carry);

        if store {
            self.set_reg8(Register8::A, new_a);
        }

        self.f = FLAG_SUBTRACT;

        if new_a == 0 {
            self.f |= FLAG_ZERO;
        }

        if (a as u16) < (v as u16) + (carry as u16) {
            self.f |= FLAG_CARRY;
        }

        if ((a & 0xF) as u16) < ((v & 0xF) as u16) + (carry as u16) {
            self.f |= FLAG_HALF_CARRY;
        }
    }

    // DAA
    // Flags:
    // Z N H C
    // * - 0 *
    fn daa(&mut self) -> u32 {
        let mut a = self.a as i16;

        if self.f & FLAG_SUBTRACT > 0 {
            if self.f & FLAG_HALF_CARRY > 0 {
                a = (a - 6) & 0xFF;
            }
            if self.f & FLAG_CARRY > 0 {
                a = a - 0x60;
            }
        } else {
            if self.f & FLAG_HALF_CARRY > 0 || (self.a & 0xF) > 9 {
                a += 0x06;
            }
            if self.f & FLAG_CARRY > 0 || self.a > 0x9F {
                a += 0x60;
            }
        }

        self.a = (a & 0xFF) as u8;

        self.f &= !(FLAG_HALF_CARRY | FLAG_ZERO | FLAG_CARRY);
        if a & 0x100 > 0 {
            self.f |= FLAG_CARRY;
        }
        if self.a == 0 {
            self.f |= FLAG_ZERO;
        }

        4
    }

    // CPL
    // Flags:
    // Z N H C
    // - 1 1 -
    fn cpl(&mut self) -> u32 {
        self.a = !self.a;
        self.f |= FLAG_SUBTRACT | FLAG_HALF_CARRY;
        4
    }

    // CCF
    // Flags:
    // Z N H C
    // - 1 1 -
    fn ccf(&mut self) -> u32 {
        self.f &= !(FLAG_SUBTRACT | FLAG_HALF_CARRY);
        self.f ^= FLAG_CARRY;
        4
    }

    // SCF
    // Flags:
    // Z N H C
    // - 0 0 1
    fn scf(&mut self) -> u32 {
        self.f &= !(FLAG_SUBTRACT | FLAG_HALF_CARRY);
        self.f |= FLAG_CARRY;
        4
    }

    // ADD A, r
    // ADC A, r
    // Flags:
    // Z N H C
    // * 0 * *
    fn add_r(&mut self, r: Register8, carry: bool) -> u32 {
        let v = self.get_reg8(r);
        self.add(v, carry);
        4
    }

    fn add_d8(&mut self, d: u8, carry: bool) -> u32 {
        self.add(d, carry);
        4
    }

    // ADD SP, r8
    // Flags:
    // Z N H C
    // 0 0 * *
    fn add_sp_r8(&mut self, d: i8) -> u32 {
        let d = d as i16 as u16;
        let sp = self.sp;

        self.sp = sp.wrapping_add(d as i16 as u16);

        self.f = 0;
        if ((sp & 0xF) + (d & 0xF)) & 0x10 > 0 {
            self.f |= FLAG_HALF_CARRY;
        }
        if ((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0 {
            self.f |= FLAG_CARRY;
        }
        16
    }

    // ADD A, (HL)
    // ADC A, (HL)
    // Flags:
    // Z N H C
    // * 0 * *
    fn add_hl(&mut self, carry: bool) -> u32 {
        let v = self.mem_read8(self.get_reg16(Register16::HL));
        self.add(v, carry);
        8
    }

    // ADD HL, rr
    // Flags:
    // Z N H C
    // - 0 * *
    fn add_rr(&mut self, r: Register16) -> u32 {
        let hl = self.get_reg16(Register16::HL);
        let v = self.get_reg16(r);

        let (new_hl, overflow) = hl.overflowing_add(v);
        self.set_reg16(Register16::HL, new_hl);

        self.f &= FLAG_ZERO;

        if overflow {
            self.f |= FLAG_CARRY;
        }

        if ((hl & 0xFFF) + (v & 0xFFF)) & 0x1000 > 0 {
            self.f |= FLAG_HALF_CARRY;
        }
        8
    }

    // SUB r
    // SBC r
    // CP r
    // Flags:
    // * 1 * *
    fn sub_hl(&mut self, carry: bool, store: bool) -> u32 {
        let v = self.mem_read8(self.get_reg16(Register16::HL));
        self.sub(v, carry, store);
        4
    }

    // SUB (HL)
    // SBC (HL)
    // CP (HL)
    // Flags:
    // * 1 * *
    fn sub_r(&mut self, r: Register8, carry: bool, store: bool) -> u32 {
        let v = self.get_reg8(r);
        self.sub(v, carry, store);
        4
    }

    // SUB d8
    // SBC A, d8
    // CP d8
    // Flags:
    // * 1 * *
    fn sub_d8(&mut self, d: u8, carry: bool, store: bool) -> u32 {
        self.sub(d, carry, store);
        8
    }

    // LD r, r
    // Flags:
    // Z N H C
    // - - - -
    fn ld_r_r(&mut self, to: Register8, from: Register8) -> u32 {
        let v = self.get_reg8(from);
        self.set_reg8(to, v);
        4
    }

    // LD r, (rr)
    // Flags:
    // Z N H C
    // - - - -
    fn ld_r_rr(&mut self, r: Register8, rr: Register16) -> u32 {
        let v = self.mem_read8(self.get_reg16(rr));
        self.set_reg8(r, v);
        8
    }

    // LD (C), A
    // a.k.a LD ($FF00+C),A
    // Flags:
    // Z N H C
    // - - - -
    fn ldd_c_a(&mut self) -> u32 {
        let v = self.get_reg8(Register8::A);
        let addr = (self.get_reg8(Register8::C) as u16) + 0xFF00;
        self.mem_write8(addr, v);
        8
    }

    // LD A, (C)
    // a.k.a LD A, ($FF00+C)
    // Flags:
    // Z N H C
    // - - - -
    fn ldd_a_c(&mut self) -> u32 {
        let addr = (self.get_reg8(Register8::C) as u16) + 0xFF00;
        let v = self.mem_read8(addr);
        self.set_reg8(Register8::A, v);
        8
    }

    // LD r, d8
    // Flags:
    // Z N H C
    // - - - -
    fn ld_r_d8(&mut self, reg: Register8, d: u8) -> u32 {
        self.set_reg8(reg, d);

        8
    }

    // LD rr, d16
    // Flags:
    // Z N H C
    // - - - -
    fn ld_rr_d16(&mut self, reg: Register16, d: u16) -> u32 {
        self.set_reg16(reg, d);

        12
    }

    // LD (rr), r
    // Flags:
    // Z N H C
    // - - - -
    fn ld_r16_r(&mut self, r16: Register16, r: Register8) -> u32 {
        let addr = self.get_reg16(r16);
        let v = self.get_reg8(r);
        self.mem_write8(addr, v);
        8
    }

    // LD (HL-), A
    // Flags:
    // Z N H C
    // - - - -
    fn ld_hld_a(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.get_reg8(Register8::A);
        self.mem_write8(addr, v);
        self.set_reg16(Register16::HL, addr - 1);
        8
    }

    // LD (HL+), A
    // Flags:
    // Z N H C
    // - - - -
    fn ld_hli_a(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.get_reg8(Register8::A);
        self.mem_write8(addr, v);
        self.set_reg16(Register16::HL, addr + 1);
        8
    }


    // LD A, (HL-)
    // Flags:
    // Z N H C
    // - - - -
    fn ld_a_hld(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        self.set_reg8(Register8::A, v);
        self.set_reg16(Register16::HL, addr - 1);
        8
    }

    // LD A, (HL+)
    // Flags:
    // Z N H C
    // - - - -
    fn ld_a_hli(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        self.set_reg8(Register8::A, v);
        self.set_reg16(Register16::HL, addr + 1);
        8
    }

    // LD HL, SP+r8
    // Flags:
    // Z N H C
    // 0 0 * *
    fn ld_hl_sp(&mut self, d: i8) -> u32 {
        let sp = self.sp;
        let d = d as i16 as u16;
        let v = sp.wrapping_add(d);
        self.set_reg16(Register16::HL, v);

        self.f = 0;
        if (sp & 0xF) + (d & 0xF) & 0x10 > 0 {
            self.f |= FLAG_HALF_CARRY;
        }
        if (sp & 0xFF) + (d & 0xFF) & 0x100 > 0 {
            self.f |= FLAG_CARRY;
        }

        12
    }

    // LD (HL), d8
    // Flags:
    // Z N H C
    // - - - -
    fn ld_hl_d8(&mut self, d: u8) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        self.mem_write8(addr, d);
        12
    }

    // LD SP, HL
    // Flags:
    // Z N H C
    // - - - -
    fn ld_sp_hl(&mut self) -> u32 {
        self.sp = self.get_reg16(Register16::HL);
        8
    }    

    // LD ($FF00+n), A
    // Flags:
    // Z N H C
    // - - - -
    fn ldh_n_a(&mut self, n: u8) -> u32 {
        let addr = 0xFF00 + (n as u16);
        let v = self.get_reg8(Register8::A);
        self.mem_write8(addr, v);
        12
    }

    // LD A, ($FF00+n)
    // Flags:
    // Z N H C
    // - - - -
    fn ldh_a_n(&mut self, n: u8) -> u32 {
        let v = self.mem_read8(0xFF00 + (n as u16));
        self.set_reg8(Register8::A, v);
        12
    }

    // LD A, (a16)
    // Flags:
    // Z N H C
    // - - - -
    fn ld_a_a16(&mut self, a: u16) -> u32 {
        let v = self.mem_read8(a);
        self.set_reg8(Register8::A, v);
        16
    }

    // LD (a16), A
    // Flags:
    // Z N H C
    // - - - -
    fn ld_a16_a(&mut self, a: u16) -> u32 {
        let v = self.get_reg8(Register8::A);
        self.mem_write8(a, v);
        16
    }

    // LD (a16), SP
    // Flags:
    // Z N H C
    // - - - -
    fn ld_a16_sp(&mut self, a: u16) -> u32 {
        let v = self.get_reg16(Register16::SP);
        self.mem_write16(a, v);
        20
    }

    fn bitwise(&mut self, op: BitwiseOp, v: u8) {
        let a = self.get_reg8(Register8::A);
        let a = match op {
            BitwiseOp::AND => a & v,
            BitwiseOp::OR => a | v,
            BitwiseOp::XOR => a ^ v,
        };
        self.set_reg8(Register8::A, a);
        self.f = 0;
        if let BitwiseOp::AND = op {
            self.f |= FLAG_HALF_CARRY;
        }
        if a == 0 {
            self.f |= FLAG_ZERO;
        }
    }

    // AND r
    // AND r, (HL)
    // Flags:
    // Z N H C
    // * 0 1 0
    //
    // OR r
    // OR r, (HL)
    // Flags:
    // Z N H C
    // * 0 1 0
    //
    // XOR n
    // XOR n, (HL)
    // Flags:
    // Z N H C
    // * 0 0 0
    fn bitwise_r(&mut self, op: BitwiseOp, r: Register8) -> u32 {
        let v = self.get_reg8(r);
        self.bitwise(op, v);
        4
    }

    fn bitwise_d8(&mut self, op: BitwiseOp, d: u8) -> u32 {
        self.bitwise(op, d);
        4
    }

    fn bitwise_hl(&mut self, op: BitwiseOp) -> u32 {
        let v = self.mem_read8(self.get_reg16(Register16::HL));
        self.bitwise(op, v);
        8
    }

    // BIT b, r
    // Flags:
    // Z N H C
    // * 0 1 -
    fn bit_b_r(&mut self, b: u8, reg: Register8) -> u32 {
        let v = self.get_reg8(reg) & (1 << b);
        self.f &= !FLAG_SUBTRACT;
        self.f |= FLAG_HALF_CARRY;
        if v == 0 {
            self.f |= FLAG_ZERO;
        } else {
            self.f &= !FLAG_ZERO;
        }

        8
    }

    // BIT b, (HL)
    // Flags:
    // Z N H C
    // * 0 1 -
    fn bit_b_hl(&mut self, b: u8) -> u32 {
        let v = self.mem_read8(self.get_reg16(Register16::HL)) & (1 << b);
        self.f &= !FLAG_SUBTRACT;
        self.f |= FLAG_HALF_CARRY;
        if v == 0 {
            self.f |= FLAG_ZERO;
        } else {
            self.f &= !FLAG_ZERO;
        }

        16
    }

    // RES b, r
    // Flags:
    // Z N H C
    // - - - -
    fn res_b_r(&mut self, b: u8, r: Register8) -> u32 {
        let v = self.get_reg8(r) & !(1 << b);
        self.set_reg8(r, v);
        8
    }

    // RES b, (HL)
    // Flags:
    // Z N H C
    // - - - -
    fn res_b_hl(&mut self, b: u8) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr) & !(1 << b);
        self.mem_write8(addr, v);
        16
    }

    // SET b, r
    // Flags:
    // Z N H C
    // - - - -
    fn set_b_r(&mut self, b: u8, r: Register8) -> u32 {
        let v = self.get_reg8(r) | (1 << b);
        self.set_reg8(r, v);
        8
    }

    // SET b, (HL)
    // Flags:
    // Z N H C
    // - - - -
    fn set_b_hl(&mut self, b: u8) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr) | (1 << b);
        self.mem_write8(addr, v);
        16
    }

    // RL r
    // Flags:
    // Z N H C
    // * 0 0 *
    fn rl_r(&mut self, r: Register8, extended: bool) -> u32 {
        let v = self.get_reg8(r);
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
        self.set_reg8(r, v);

        if extended { 8 } else { 4 }
    }

    // RLC r
    // Flags:
    // Z N H C
    // * 0 0 *
    // RLCA
    // Flags:
    // Z N H C
    // 0 0 0 *
    fn rlc_r(&mut self, r: Register8, extended: bool) -> u32 {
        let mut v = self.get_reg8(r);
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
        self.set_reg8(r, v);

        if extended { 8 } else { 4 }
    }

    // RLC (HL)
    // Flags:
    // Z N H C
    // * 0 0 *
    fn rlc_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let mut v = self.mem_read8(addr);

        self.f = 0;
        if v & 0x80 > 0 {
            self.f |= FLAG_CARRY;
        }
        v <<= 1;
        if self.f & FLAG_CARRY > 0 {
            v |= 1;
        }
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        self.mem_write8(addr, v);
        8
    }

    // RR r
    // Flags:
    // Z N H C
    // * 0 0 *
    fn rr(&mut self, v: u8, set_zero: bool) -> u8 {
        let carry = self.f & FLAG_CARRY > 0;

        self.f = 0;
        if v & 0x1 > 0 {
            self.f |= FLAG_CARRY;
        }

        let mut v = v >> 1;

        if carry {
            v |= 0x80;
        }

        if set_zero && v == 0 {
            self.f |= FLAG_ZERO;
        }

        v
    }
    fn rr_r(&mut self, r: Register8, extended: bool) -> u32 {
        let v = self.get_reg8(r);
        let v = self.rr(v, extended);
        self.set_reg8(r, v);
        if extended { 8 } else { 4 }
    }
    fn rr_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        let v = self.rr(v, true);
        self.mem_write8(addr, v);
        16
    }

    // SLA r
    // Flags:
    // Z N H C
    // * 0 0 C
    fn sla_r(&mut self, r: Register8) -> u32 {
        let v = self.get_reg8(r);
        self.f = 0;
        if v & 0x80 > 0 {
            self.f |= FLAG_CARRY;
        }
        let v = v << 1;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        self.set_reg8(r, v);
        8
    }
    fn sla_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        self.f = 0;
        if v & 0x80 > 0 {
            self.f |= FLAG_CARRY;
        }
        let v = v << 1;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        self.mem_write8(addr, v);
        8
    }

    // SRA r
    // Flags:
    // Z N H C
    // * 0 0 *
    fn sra_r(&mut self, r: Register8) -> u32 {
        let v = self.get_reg8(r);
        self.f = 0;
        if v & 0x01 > 0 {
            self.f |= FLAG_CARRY;
        }
        let v = (v >> 1) | (v & 0x80);
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        self.set_reg8(r, v);
        8
    }
    fn sra_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        self.f = 0;
        if v & 0x01 > 0 {
            self.f |= FLAG_CARRY;
        }
        let v = (v >> 1) | (v & 0x80);
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        self.mem_write8(addr, v);
        8
    }

    // RRC r
    // Flags:
    // Z N H C
    // * 0 0 *
    fn rrc(&mut self, v: u8, set_zero: bool) -> u8 {
        self.f = 0;
        if v & 0x1 > 0 {
            self.f |= FLAG_CARRY;
        }

        let mut v = v >> 1;
        if self.f & FLAG_CARRY > 0 {
            v |= 0x80;
        }
        if set_zero && v == 0 {
            self.f |= FLAG_ZERO;
        }

        v
    }
    fn rrc_r(&mut self, r: Register8, extended: bool) -> u32 {
        let v = self.get_reg8(r);
        let v = self.rrc(v, extended);
        self.set_reg8(r, v);
        if extended { 8 } else { 4 }
    }
    fn rrc_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        let v = self.rrc(v, true);
        self.mem_write8(addr, v);
        16
    }

    // SWAP r
    // Flags:
    // Z N H C
    // * 0 0 0
    fn swap_r(&mut self, r: Register8) -> u32 {
        let v = self.get_reg8(r);
        let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
        self.f = 0;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        self.set_reg8(r, v);
        8
    }

    // SWAP (HL)
    // Flags:
    // Z N H C
    // * 0 0 0
    fn swap_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);
        let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
        self.f = 0;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        self.mem_write8(addr, v);
        16
    }

    // SRL r
    // SRL (HL)
    // Flags:
    // Z N H C
    // * 0 0 *
    fn srl_r(&mut self, r: Register8) -> u32 {
        let v = self.get_reg8(r);

        self.f = 0;
        if v & 0x1 == 1 {
            self.f |= FLAG_CARRY;
        }

        let v = v >> 1;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }

        self.set_reg8(r, v);

        8
    }
    fn srl_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        let v = self.mem_read8(addr);

        self.f = 0;
        if v & 0x1 == 1 {
            self.f |= FLAG_CARRY;
        }

        let v = v << 1;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }

        self.mem_write8(addr, v);

        16
    }

    fn call(&mut self, cc: Option<ConditionFlag>, addr: u16) -> u32 {
        if let Some(cc) = cc {
            if !self.check_jmp_condition(cc) {
                return 12;
            }
        }

        self.sp -= 2;
        let sp = self.sp;
        let pc = self.pc;
        self.mem_write16(sp, pc);
        self.pc = addr;
        24
    }

    fn ret(&mut self, cc: Option<ConditionFlag>) -> u32 {
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

    fn jp(&mut self, cc: Option<ConditionFlag>, a: u16) -> u32 {
        if let Some(cc) = cc {
            if !self.check_jmp_condition(cc) {
                return 12;
            }
        }

        self.pc = a;

        16
    }

    fn jp_hl(&mut self) -> u32 {
        let addr = self.get_reg16(Register16::HL);
        self.pc = addr;
        4
    }

    fn rst(&mut self, a: u8) -> u32 {
        self.sp -= 2;
        let sp = self.sp;
        let pc = self.pc;
        self.mem_write16(sp, pc);
        self.pc = a as u16;
        16
    }

    fn push(&mut self, r: Register16) -> u32 {
        let v = self.get_reg16(r);
        self.sp -= 2;
        let sp = self.sp;
        self.mem_write16(sp, v);
        16
    }

    fn pop(&mut self, r: Register16) -> u32 {
        let mut v = self.mem_read16(self.sp);
        if let Register16::AF = r {
            // Reset bits 0-3 in F.
            v &= 0xFFF0;
        }
        self.sp += 2;
        self.set_reg16(r, v);
        12
    }

    fn jr_n(&mut self, n: u8) -> u32 {
        let ns: i8 = n as i8;
        if ns < 0 {
            self.pc -= ns.abs() as u16;
        } else {
            self.pc += n as u16;
        }
        
        12
    }

    fn jr_cc_n(&mut self, cc: ConditionFlag, n: u8) -> u32 {
        if self.check_jmp_condition(cc) {
            return self.jr_n(n);
        }

        8
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
