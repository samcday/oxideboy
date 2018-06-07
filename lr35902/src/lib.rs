#[allow(dead_code)]

pub mod lr35902 {
    const FLAG_ZERO: u8      = 0b10000000;
    const FLAG_SUBTRACT: u8  = 0b01000000;
    const FLAG_HALF_CARRY:u8 = 0b00100000;
    const FLAG_CARRY:u8      = 0b00010000;

    pub trait MMU {
        fn read8(&self, addr: u16) -> u8;
        fn write8(&self, addr: u16, v: u8);

        fn read16(&self, addr: u16) -> u16;
        fn write16(&self, addr: u16, v: u16);
    }

    #[derive(Copy, Clone)]
    enum Register8 {
        A,
        B,
        C,
        D,
        E,
        H,
        L,
    }

    #[derive(Copy, Clone)]
    enum Register16 {
        BC,
        DE,
        HL,
        SP,
    }

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

        ADD_A_r(Register8),
        ADD_A_HL,
        ADC_A_r(Register8),
        ADC_A_HL,
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

        JR_cc_n(ConditionFlag, u8),
        JR_n(u8),

        LD_r_d8(Register8, u8),
        LD_rr_d16(Register16, u16),
        LD_r16_r(Register16, Register8),
        LD_r_r(Register8, Register8),
        LD_r_HL(Register8),
        LDD_HL_A,
        LDI_HL_A,

        BIT_b_r(u8, Register8),
        BIT_b_HL(u8),
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
        mmu: &'a (MMU + 'a),
    }

    impl <'a> CPU<'a> {
        // Runs the CPU for a single instruction.
        // This is the main "Fetch, decode, execute" cycle
        pub fn run(&mut self) {
            let inst = self.decode();

            self.ic += match inst {
                Instruction::NOP => 4,

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

                Instruction::LD_r_d8(r, v) => self.ld_r_d8(r, v),
                Instruction::LD_rr_d16(r, v) => self.ld_rr_d16(r, v),
                Instruction::LD_r16_r(r16, r) => self.ld_r16_r(r16, r),
                Instruction::LD_r_r(to, from) => self.ld_r_r(to, from),
                Instruction::LD_r_HL(r) => self.ld_r_hl(r),
                Instruction::LDD_HL_A => self.ldd_hl_a(),
                Instruction::LDI_HL_A => self.ldi_hl_a(),

                Instruction::BIT_b_r(b, r) => self.bit_b_r(b, r),
                Instruction::BIT_b_HL(b) => self.bit_b_hl(b),

                Instruction::JR_n(n) => self.jr_n(n),
                Instruction::JR_cc_n(cc, n) => self.jr_cc_n(cc, n),
            };
        }

        // Fetches next byte from PC and increments PC.
        fn fetch8(&mut self) -> u8 {
            self.pc += 1;
            self.mmu.read8(self.pc - 1)
        }

        // Fetches next short from PC and increments PC by 2.
        fn fetch16(&mut self) -> u16 {
            self.pc += 2;
            self.mmu.read16(self.pc - 2)
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

                0x18 => Instruction::JR_n(self.fetch8()),

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

                0x40 => Instruction::LD_r_r(Register8::B, Register8::B),
                0x41 => Instruction::LD_r_r(Register8::B, Register8::C),
                0x42 => Instruction::LD_r_r(Register8::B, Register8::D),
                0x43 => Instruction::LD_r_r(Register8::B, Register8::E),
                0x44 => Instruction::LD_r_r(Register8::B, Register8::H),
                0x45 => Instruction::LD_r_r(Register8::B, Register8::L),
                0x46 => Instruction::LD_r_HL(Register8::B),
                0x47 => Instruction::LD_r_r(Register8::B, Register8::A),
                0x48 => Instruction::LD_r_r(Register8::C, Register8::B),
                0x49 => Instruction::LD_r_r(Register8::C, Register8::C),
                0x4A => Instruction::LD_r_r(Register8::C, Register8::D),
                0x4B => Instruction::LD_r_r(Register8::C, Register8::E),
                0x4C => Instruction::LD_r_r(Register8::C, Register8::H),
                0x4D => Instruction::LD_r_r(Register8::C, Register8::L),
                0x4E => Instruction::LD_r_HL(Register8::C),
                0x4F => Instruction::LD_r_r(Register8::C, Register8::A),
                0x50 => Instruction::LD_r_r(Register8::D, Register8::B),
                0x51 => Instruction::LD_r_r(Register8::D, Register8::C),
                0x52 => Instruction::LD_r_r(Register8::D, Register8::D),
                0x53 => Instruction::LD_r_r(Register8::D, Register8::E),
                0x54 => Instruction::LD_r_r(Register8::D, Register8::H),
                0x55 => Instruction::LD_r_r(Register8::D, Register8::L),
                0x56 => Instruction::LD_r_HL(Register8::D),
                0x57 => Instruction::LD_r_r(Register8::D, Register8::A),
                0x58 => Instruction::LD_r_r(Register8::E, Register8::B),
                0x59 => Instruction::LD_r_r(Register8::E, Register8::C),
                0x5A => Instruction::LD_r_r(Register8::E, Register8::D),
                0x5B => Instruction::LD_r_r(Register8::E, Register8::E),
                0x5C => Instruction::LD_r_r(Register8::E, Register8::H),
                0x5D => Instruction::LD_r_r(Register8::E, Register8::L),
                0x5E => Instruction::LD_r_HL(Register8::E),
                0x5F => Instruction::LD_r_r(Register8::E, Register8::A),
                0x60 => Instruction::LD_r_r(Register8::H, Register8::B),
                0x61 => Instruction::LD_r_r(Register8::H, Register8::C),
                0x62 => Instruction::LD_r_r(Register8::H, Register8::D),
                0x63 => Instruction::LD_r_r(Register8::H, Register8::E),
                0x64 => Instruction::LD_r_r(Register8::H, Register8::H),
                0x65 => Instruction::LD_r_r(Register8::H, Register8::L),
                0x66 => Instruction::LD_r_HL(Register8::H),
                0x67 => Instruction::LD_r_r(Register8::H, Register8::A),
                0x68 => Instruction::LD_r_r(Register8::L, Register8::B),
                0x69 => Instruction::LD_r_r(Register8::L, Register8::C),
                0x6A => Instruction::LD_r_r(Register8::L, Register8::D),
                0x6B => Instruction::LD_r_r(Register8::L, Register8::E),
                0x6C => Instruction::LD_r_r(Register8::L, Register8::H),
                0x6D => Instruction::LD_r_r(Register8::L, Register8::L),
                0x6E => Instruction::LD_r_HL(Register8::L),
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
                0x7E => Instruction::LD_r_HL(Register8::A),
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

                0xCB => self.decode_extended(),

                n => {
                    panic!("Unexpected opcode {} encountered", n);
                }
            }
        }

        // Decodes extended instruction following 0xCB instruction.
        fn decode_extended(&mut self) -> Instruction {
            match self.fetch8() {
                // BIT instructions.
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
                    panic!("Unexpected extended opcode {} encountered", n);
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
                Register16::BC => { self.b = (v & 0xFF00 >> 8) as u8; self.c = (v & 0xFF) as u8; },
                Register16::DE => { self.d = (v & 0xFF00 >> 8) as u8; self.e = (v & 0xFF) as u8; },
                Register16::HL => { self.h = (v & 0xFF00 >> 8) as u8; self.l = (v & 0xFF) as u8; },
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
            let v = self.mmu.read8(self.get_reg16(Register16::HL));
            self.add(v, carry);

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

        // LD r, (HL)
        // Flags:
        // Z N H C
        // - - - -
        fn ld_r_hl(&mut self, r: Register8) -> u32 {
            let v = self.mmu.read8(self.get_reg16(Register16::HL));
            self.set_reg8(r, v);
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
            self.mmu.write8(self.get_reg16(r16), self.get_reg8(r));
            8
        }

        // LD (HL-), A
        // Flags:
        // Z N H C
        // - - - -
        fn ldd_hl_a(&mut self) -> u32 {
            let mut addr = self.get_reg16(Register16::HL);
            self.mmu.write8(addr, self.get_reg8(Register8::A));
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
            self.mmu.write8(addr, self.get_reg8(Register8::A));
            addr += 1;
            self.set_reg16(Register16::HL, addr);

            8
        }

        // LD (C), A
        // LD (A), C
        // a.k.a LD A,($FF00+C) / LD ($FF00+C),A
        // Z N H C
        // - - - -
        fn ld_r2mem(&mut self) -> u32 {
            0
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
            let v = self.mmu.read8(self.get_reg16(Register16::HL));
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
            let v = self.mmu.read8(self.get_reg16(Register16::HL)) & (1 << b);
            self.f = FLAG_HALF_CARRY;
            if v == 0 {
                self.f |= FLAG_ZERO;
            }

            16
        }

        fn jr_n(&mut self, n: u8) -> u32 {
            let ns: i8 = n as i8;
            if ns < 0 {
                self.pc -= n as u16;
            } else {
                self.pc += n as u16;
            }
            return 12;
        }

        fn jr_cc_n(&mut self, cc: ConditionFlag, n: u8) -> u32 {
            if self.check_jmp_condition(cc) {
                return self.jr_n(n);
            }

            8
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
