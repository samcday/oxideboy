use std::fmt;

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

    ADD_A_r(Register8),
    ADD_A_HL,
    ADC_A_r(Register8),
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
    DEC_rr(Register16),
    AND_r(Register8),
    AND_HL,
    OR_r(Register8),
    OR_HL,
    XOR_r(Register8),
    XOR_HL,
    CP_r(Register8),
    CP_d8(u8),
    CP_HL,
    RLA,

    JP(Option<ConditionFlag>, u16),
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
    LDD_HL_A,
    LDI_HL_A,
    LDH_A_n(u8),
    LDH_n_A(u8),
    LD_a16_SP(u16),
    LD_a16_A(u16),
    LD_A_a16(u16),

    BIT_b_r(u8, Register8),
    BIT_b_HL(u8),
    RL_r(Register8),
}

impl ::fmt::Display for Instruction {
    fn fmt(&self, f: &mut ::fmt::Formatter) -> ::fmt::Result {
        match self {
            Instruction::NOP => write!(f,  "NOP"),
            Instruction::EI => write!(f,  "EI"),
            Instruction::DI => write!(f,  "DI"),
            Instruction::RETI => write!(f,  "RETI"),
            Instruction::ADD_A_r(r) => write!(f, "ADD A, {:?}", r),
            Instruction::ADD_A_HL => write!(f, "ADD A, (HL)"),
            Instruction::ADC_A_r(r) => write!(f, "ADC A, {:?}", r),
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
            Instruction::AND_r(r) => write!(f, "AND {:?}", r),
            Instruction::AND_HL => write!(f, "AND (HL)"),
            Instruction::OR_r(r) => write!(f, "OR {:?}", r),
            Instruction::OR_HL => write!(f, "OR (HL)"),
            Instruction::XOR_r(r) => write!(f, "XOR {:?}", r),
            Instruction::XOR_HL => write!(f, "XOR (HL)"),
            Instruction::CP_r(r) => write!(f, "CP {:?}", r),
            Instruction::CP_d8(d) => write!(f, "CP ${:X}", d),
            Instruction::CP_HL => write!(f, "CP (HL)"),
            Instruction::RLA => write!(f, "RLA"),
            Instruction::JP(None, addr) => write!(f, "JP ${:X}", addr),
            Instruction::JP(Some(cc), addr) => write!(f, "JP {:?}, ${:X}", cc, addr),
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
            Instruction::LDD_HL_A => write!(f, "LD (HL-), A"),
            Instruction::LDI_HL_A => write!(f, "LD (HL+), A"),
            Instruction::LDH_A_n(n) => write!(f, "LDH A, (0xFF00+${:X})", n),
            Instruction::LDH_n_A(n) => write!(f, "LDH (0xFF00+${:X}), A", n),
            Instruction::LD_a16_SP(a) => write!(f, "LD (${:X}), SP", a),
            Instruction::LD_a16_A(a) => write!(f, "LD (${:X}), A", a),
            Instruction::LD_A_a16(a) => write!(f, "LD A, (${:X})", a),
            Instruction::BIT_b_r(b, r) => write!(f, "BIT {}, {:?}", b, r),
            Instruction::BIT_b_HL(b) => write!(f, "BIT {}, (HL)", b),
            Instruction::RL_r(r) => write!(f, "RL {:?}", r),
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
    ime: bool,
    ime_defer: Option<bool>,
    wram: [u8; 0x2000],
    vram: [u8; 0x2000],
    sram: [u8; 0x7E],
    oam: [u8; 0x9F],
    mmu: &'a mut (MMU + 'a),
}

impl <'a> CPU<'a> {
    pub fn new(mmu: &'a mut (MMU + 'a)) -> CPU {
        CPU{
            a: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0, f: 0, sp: 0, pc: 0, ic: 0,
            ime: false, ime_defer: None,
            wram: [0; 0x2000],
            vram: [0; 0x2000],
            sram: [0; 0x7E],
            oam: [0; 0x9F],
            mmu}
    }

    // Runs the CPU for a single instruction.
    // This is the main "Fetch, decode, execute" cycle
    pub fn run(&mut self) {
        let addr = self.pc;
        let inst = self.decode();

        println!("Inst: {} ; ${:04X}", inst, addr);

        if self.ime {
            // Service interrupts.
        }

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

            Instruction::ADD_A_r(r) => self.add_r(r, false),
            Instruction::ADD_A_HL => self.add_hl(false),
            Instruction::ADC_A_r(r) => self.add_r(r, true),
            Instruction::ADC_A_HL => self.add_hl(true),
            Instruction::INC_r(r) => self.inc_r(r),
            Instruction::INC_rr(r) => self.inc_rr(r),
            Instruction::DEC_r(r) => self.dec_r(r),
            Instruction::DEC_rr(r) => self.dec_rr(r),
            Instruction::AND_r(r) => self.bitwise_r(BitwiseOp::AND, r),
            Instruction::XOR_r(r) => self.bitwise_r(BitwiseOp::XOR, r),
            Instruction::OR_r(r) => self.bitwise_r(BitwiseOp::OR, r),
            Instruction::AND_HL => self.bitwise_hl(BitwiseOp::AND),
            Instruction::XOR_HL => self.bitwise_hl(BitwiseOp::XOR),
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

            Instruction::LD_r_d8(r, v) => self.ld_r_d8(r, v),
            Instruction::LD_rr_d16(r, v) => self.ld_rr_d16(r, v),
            Instruction::LD_r16_r(r16, r) => self.ld_r16_r(r16, r),
            Instruction::LD_r_r(to, from) => self.ld_r_r(to, from),
            Instruction::LD_r_rr(r, rr) => self.ld_r_rr(r, rr),
            Instruction::LD_C_A => self.ldd_c_a(),
            Instruction::LDD_HL_A => self.ldd_hl_a(),
            Instruction::LDI_HL_A => self.ldi_hl_a(),
            Instruction::LDH_n_A(n) => self.ldh_n_a(n),
            Instruction::LDH_A_n(n) => self.ldh_a_n(n),
            Instruction::LD_A_a16(a) => self.ld_a_a16(a),
            Instruction::LD_a16_A(a) => self.ld_a16_a(a),
            Instruction::LD_a16_SP(a) => self.ld_a16_sp(a),

            Instruction::JP(cc, n) => self.jp(cc, n),
            Instruction::JR_n(n) => self.jr_n(n),
            Instruction::JR_cc_n(cc, n) => self.jr_cc_n(cc, n),
            Instruction::CALL(cc, addr) => self.call(cc, addr),
            Instruction::RET(cc) => self.ret(cc),
            Instruction::PUSH(r) => self.push(r),
            Instruction::POP(r) => self.pop(r),
            Instruction::RST(a) => self.rst(a),

            Instruction::BIT_b_r(b, r) => self.bit_b_r(b, r),
            Instruction::BIT_b_HL(b) => self.bit_b_hl(b),
            Instruction::RL_r(r) => self.rl_r(r, true),
        };
    }

    fn mem_read8(&self, addr: u16) -> u8 {
        match addr {
            0x0000 ... 0x7FFF => self.mmu.read8(addr),
            0x8000 ... 0x9FFF => self.vram[(addr - 0x8000) as usize],
            0xA000 ... 0xBFFF => self.mmu.read8(addr),
            0xC000 ... 0xDFFF => self.wram[(addr - 0xC000) as usize],
            0xE000 ... 0xFDFF => self.wram[(addr - 0xE000) as usize],
            0xFE00 ... 0xFEFF => self.oam[(addr - 0xFE00) as usize],
            0xFF44 => 0x90, // temp hack
            0xFF00 ... 0xFF79 => 0xFF, // TODO
            0xFF80 ... 0xFFFE => self.sram[(addr - 0xFF80) as usize],
            0xFFFF            => 0xFF, // TODO
            _ => panic!("Emulator not designed to run in an environment where the laws of physics no longer apply")
        }
    }

    fn mem_read16(&self, addr: u16) -> u16 {
        let mut v = self.mem_read8(addr) as u16;
        v |= (self.mem_read8(addr + 1) as u16) << 8;
        v
    }

    fn mem_write8(&mut self, addr: u16, v: u8) {
        match addr {
            0x0000 ... 0x7FFF => { self.mmu.write8(addr, v) },
            0x8000 ... 0x9FFF => { self.vram[(addr - 0x8000) as usize] = v },
            0xA000 ... 0xBFFF => { self.mmu.write8(addr, v) },
            0xC000 ... 0xDFFF => { self.wram[(addr - 0xC000) as usize] = v },
            0xE000 ... 0xFDFF => { self.wram[(addr - 0xE000) as usize] = v },
            0xFE00 ... 0xFEFF => { self.oam[(addr - 0xFE00) as usize] = v },
            0xFF00 ... 0xFF79 => { }, // TODO
            0xFF80 ... 0xFFFE => { self.sram[(addr - 0xFF80) as usize] = v },
            0xFFFF            => { }, // TODO
            _ => panic!("Emulator not designed to run in an environment where the laws of physics no longer apply")
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

            0x08 => Instruction::LD_a16_SP(self.fetch16()),

            0x0A => Instruction::LD_r_rr(Register8::A, Register16::BC),
            0x0B => Instruction::DEC_rr(Register16::BC),
            0x0C => Instruction::INC_r(Register8::C),
            0x0D => Instruction::DEC_r(Register8::C),
            0x0E => Instruction::LD_r_d8(Register8::C, self.fetch8()),

            0x11 => Instruction::LD_rr_d16(Register16::DE, self.fetch16()),
            0x12 => Instruction::LD_r16_r(Register16::DE, Register8::A),
            0x13 => Instruction::INC_rr(Register16::DE),
            0x14 => Instruction::INC_r(Register8::D),
            0x15 => Instruction::DEC_r(Register8::D),
            0x16 => Instruction::LD_r_d8(Register8::D, self.fetch8()), 
            0x17 => Instruction::RLA,
            0x18 => Instruction::JR_n(self.fetch8()),

            0x1A => Instruction::LD_r_rr(Register8::A, Register16::DE),
            0x1B => Instruction::DEC_rr(Register16::DE),
            0x1C => Instruction::INC_r(Register8::E),
            0x1D => Instruction::DEC_r(Register8::E),
            0x1E => Instruction::LD_r_d8(Register8::E, self.fetch8()),

            0x20 => Instruction::JR_cc_n(ConditionFlag::NZ, self.fetch8()),
            0x21 => Instruction::LD_rr_d16(Register16::HL, self.fetch16()),
            0x22 => Instruction::LDI_HL_A,
            0x23 => Instruction::INC_rr(Register16::HL),
            0x24 => Instruction::INC_r(Register8::H),
            0x25 => Instruction::DEC_r(Register8::H),
            0x26 => Instruction::LD_r_d8(Register8::H, self.fetch8()),

            0x28 => Instruction::JR_cc_n(ConditionFlag::Z, self.fetch8()),

            0x2B => Instruction::DEC_rr(Register16::HL),
            0x2C => Instruction::INC_r(Register8::L),
            0x2D => Instruction::DEC_r(Register8::L),
            0x2E => Instruction::LD_r_d8(Register8::L, self.fetch8()),

            0x30 => Instruction::JR_cc_n(ConditionFlag::NC, self.fetch8()),
            0x31 => Instruction::LD_rr_d16(Register16::SP, self.fetch16()),
            0x32 => Instruction::LDD_HL_A,
            0x33 => Instruction::INC_rr(Register16::SP),

            0x38 => Instruction::JR_cc_n(ConditionFlag::C, self.fetch8()),

            0x3B => Instruction::DEC_rr(Register16::SP),
            0x3C => Instruction::INC_r(Register8::A),
            0x3D => Instruction::DEC_r(Register8::A),
            0x3E => Instruction::LD_r_d8(Register8::A, self.fetch8()),

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
            0xA8 => Instruction::XOR_r(Register8::L),
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
            0xC8 => Instruction::RET(Some(ConditionFlag::Z)),
            0xC7 => Instruction::RST(0x00),
            0xC9 => Instruction::RET(None),
            0xCA => Instruction::JP(Some(ConditionFlag::Z), self.fetch16()),
            0xCB => self.decode_extended(),
            0xCC => Instruction::CALL(Some(ConditionFlag::Z), self.fetch16()),
            0xCD => Instruction::CALL(None, self.fetch16()),

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

            0xE7 => Instruction::RST(0x20),

            0xEA => Instruction::LD_a16_A(self.fetch16()),

            0xEF => Instruction::RST(0x28),

            0xF0 => Instruction::LDH_A_n(self.fetch8()),

            0xF3 => Instruction::DI,

            0xF7 => Instruction::RST(0x30),

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
            0x10 => Instruction::RL_r(Register8::B),
            0x11 => Instruction::RL_r(Register8::C),
            0x12 => Instruction::RL_r(Register8::D),
            0x13 => Instruction::RL_r(Register8::E),
            0x14 => Instruction::RL_r(Register8::H),
            0x15 => Instruction::RL_r(Register8::L),

            0x17 => Instruction::RL_r(Register8::A),

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
        match reg {
            Register16::BC => { (self.b as u16) << 8 | (self.c as u16) },
            Register16::DE => { (self.d as u16) << 8 | (self.e as u16) },
            Register16::HL => { (self.h as u16) << 8 | (self.l as u16) },
            Register16::SP => self.sp,
        }
    }

    fn set_reg16(&mut self, reg: Register16, v: u16) {
        match reg {
            Register16::BC => { self.b = ((v & 0xFF00) >> 8) as u8; self.c = (v & 0xFF) as u8; },
            Register16::DE => { self.d = ((v & 0xFF00) >> 8) as u8; self.e = (v & 0xFF) as u8; },
            Register16::HL => { self.h = ((v & 0xFF00) >> 8) as u8; self.l = (v & 0xFF) as u8; },
            Register16::SP => { self.sp = v; },
        };
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
    fn dec_r(&mut self, r: Register8) -> u32 {
        let v = self.get_reg8(r);
        let (v, _) = v.overflowing_sub(1);
        self.set_reg8(r, v);

        // Clear Z, N, H flags. If we overflowed top or bottom nibbles
        // we'll set the appropriate flags below.
        self.f &= !(FLAG_ZERO | FLAG_HALF_CARRY);
        self.f |= FLAG_SUBTRACT;

        if v == 0 {
            self.f |= FLAG_ZERO;
        }
        if (v & 0x0F) == 0 {
            self.f |= FLAG_HALF_CARRY;
        }

        4
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
        
        let carry = if carry && (self.f & FLAG_CARRY != 0) { 1 } else { 0 };

        let (new_a, overflow) = a.overflowing_add(v + carry);
        self.set_reg8(Register8::A, new_a);

        self.f = 0;

        if new_a == 0 {
            self.f |= FLAG_ZERO;
        }

        if overflow {
            self.f |= FLAG_CARRY;
        }

        if ((a & 0xF) + (v & 0xF)) & 0x10 == 0x10 {
            self.f |= FLAG_HALF_CARRY;
        }
    }

    // Raw SUB/CP instruction used by both sub_r and sub_hl
    fn sub(&mut self, v: u8, carry: bool, store: bool) {
        let a = self.get_reg8(Register8::A);
        
        let carry = if carry && (self.f & FLAG_CARRY != 0) { 1 } else { 0 };

        let (new_a, overflow) = a.overflowing_sub(v + carry);

        if store {
            self.set_reg8(Register8::A, new_a);
        }

        self.f = 0;

        if new_a == 0 {
            self.f |= FLAG_ZERO;
        }

        if overflow {
            self.f |= FLAG_CARRY;
        }

        if (a & 0xF) < (v & 0xF) {
            self.f |= FLAG_HALF_CARRY;
        }
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
    fn ldd_hl_a(&mut self) -> u32 {
        let mut addr = self.get_reg16(Register16::HL);
        let v = self.get_reg8(Register8::A);
        self.mem_write8(addr, v);
        addr -= 1;
        self.set_reg16(Register16::HL, addr);

        8
    }

    // LD (HL+), A
    // Flags:
    // Z N H C
    // - - - -
    fn ldi_hl_a(&mut self) -> u32 {
        let mut addr = self.get_reg16(Register16::HL);
        let v = self.get_reg8(Register8::A);
        self.mem_write8(addr, v);
        addr += 1;
        self.set_reg16(Register16::HL, addr);

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
        self.f = FLAG_HALF_CARRY;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }

        8
    }

    // BIT b, (HL)
    fn bit_b_hl(&mut self, b: u8) -> u32 {
        let v = self.mem_read8(self.get_reg16(Register16::HL)) & (1 << b);
        self.f = FLAG_HALF_CARRY;
        if v == 0 {
            self.f |= FLAG_ZERO;
        }

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
        // Only extended RL instructions set zero flag.
        if extended && v == 0 {
            self.f |= FLAG_ZERO;
        }

        if carry {
            v |= 1;
        }

        self.set_reg8(r, v);

        if extended { 8 } else { 4 }
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
        let v = self.mem_read16(self.sp);
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
