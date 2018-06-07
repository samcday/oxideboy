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

    enum ConditionFlags {
        NZ,
        Z,
        NC,
        C,
    }

    #[allow(non_camel_case_types)]
    enum Instruction {
        NOP,
        INC_r(Register8),
        INC_rr(Register16),
        DEC_r(Register8),
        DEC_rr(Register16),
        LD_r_d8(Register8, u8),
        LD_rr_d16(Register16, u16),
        XOR_r(Register8),
        LDD_HL_A,
        LDI_HL_A,
        BIT_b_r(u8, Register8),
        BIT_b_HL(u8),
    }

    impl Instruction {
        fn num_cycles(&self) -> u8 {
            match self {
                Instruction::NOP => 4,
                Instruction::INC_r(_) => 4,
                Instruction::INC_rr(_) => 8,
                Instruction::DEC_r(_) => 4,
                Instruction::DEC_rr(_) => 8,
                Instruction::LD_r_d8(_, _) => 8,
                Instruction::LD_rr_d16(_, _) => 12,
                Instruction::XOR_r(_) => 4,
                Instruction::LDD_HL_A => 8,
                Instruction::LDI_HL_A => 8,

                Instruction::BIT_b_r(_, _) => 8,
                Instruction::BIT_b_HL(_) => 16,
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
        mmu: &'a (MMU + 'a),
    }

    impl <'a> CPU<'a> {
        // Runs the CPU for a single instruction.
        // This is the main "Fetch, decode, execute" cycle
        pub fn run(&mut self) {
            let inst = self.decode();
            self.ic += inst.num_cycles() as u32;

            match inst {
                Instruction::NOP => (),

                Instruction::INC_r(r) => self.inc_r(r),
                Instruction::INC_rr(r) => self.inc_rr(r),
                Instruction::DEC_r(r) => self.dec_r(r),
                Instruction::DEC_rr(r) => self.dec_rr(r),
                Instruction::XOR_r(r) => self.xor_r(r),

                Instruction::LD_r_d8(r, v) => self.ld_r_d8(r, v),
                Instruction::LD_rr_d16(r, v) => self.ld_rr_d16(r, v),
                Instruction::LDD_HL_A => self.ldd_hl_a(),
                Instruction::LDI_HL_A => self.ldi_hl_a(),

                Instruction::BIT_b_r(b, r) => self.bit_b_r(b, r),
                Instruction::BIT_b_HL(b) => self.bit_b_hl(b),
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
                // LD nn,d16 instructions
                0x01 => Instruction::LD_rr_d16(Register16::BC, self.fetch16()),
                0x11 => Instruction::LD_rr_d16(Register16::DE, self.fetch16()),
                0x21 => Instruction::LD_rr_d16(Register16::HL, self.fetch16()),
                0x31 => Instruction::LD_rr_d16(Register16::SP, self.fetch16()),
                // LD n, d8 instructions
                0x06 => Instruction::LD_r_d8(Register8::B, self.fetch8()),
                0x0E => Instruction::LD_r_d8(Register8::C, self.fetch8()),
                0x16 => Instruction::LD_r_d8(Register8::D, self.fetch8()),
                0x1E => Instruction::LD_r_d8(Register8::E, self.fetch8()),
                0x26 => Instruction::LD_r_d8(Register8::H, self.fetch8()),
                0x2E => Instruction::LD_r_d8(Register8::L, self.fetch8()),
                // INC n instructions
                0x04 => Instruction::INC_r(Register8::B),
                0x0C => Instruction::INC_r(Register8::C),
                0x14 => Instruction::INC_r(Register8::D),
                0x1C => Instruction::INC_r(Register8::E),
                0x24 => Instruction::INC_r(Register8::H),
                0x2C => Instruction::INC_r(Register8::L),
                0x3C => Instruction::INC_r(Register8::A),
                // INC nn instructions
                0x03 => Instruction::INC_rr(Register16::BC),
                0x13 => Instruction::INC_rr(Register16::DE),
                0x23 => Instruction::INC_rr(Register16::HL),
                0x33 => Instruction::INC_rr(Register16::SP),
                // DEC n instructions
                0x05 => Instruction::DEC_r(Register8::B),
                0x0D => Instruction::DEC_r(Register8::C),
                0x15 => Instruction::DEC_r(Register8::D),
                0x1D => Instruction::DEC_r(Register8::E),
                0x25 => Instruction::DEC_r(Register8::H),
                0x2D => Instruction::DEC_r(Register8::L),
                0x3D => Instruction::DEC_r(Register8::A),
                // DEC nn instructions
                0x0B => Instruction::DEC_rr(Register16::BC),
                0x1B => Instruction::DEC_rr(Register16::DE),
                0x2B => Instruction::DEC_rr(Register16::HL),
                0x3B => Instruction::DEC_rr(Register16::SP),
                // XOR instructions
                0xA8 => Instruction::XOR_r(Register8::B),
                0xA9 => Instruction::XOR_r(Register8::C),
                0xAA => Instruction::XOR_r(Register8::D),
                0xAB => Instruction::XOR_r(Register8::E),
                0xAC => Instruction::XOR_r(Register8::H),
                0xAD => Instruction::XOR_r(Register8::L),
                0xAF => Instruction::XOR_r(Register8::A),

                // LDD/LDI instructions
                0x22 => Instruction::LDI_HL_A,
                0x32 => Instruction::LDD_HL_A,

                // Extended instruction set.
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

        // INC r
        // Flags:
        // Z N H C
        // * 0 * -
        fn inc_r(&mut self, reg: Register8) {
            let mut v = self.get_reg8(reg);
            v += 1;
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
        }

        // INC rr
        // Flags:
        // Z N H C
        // - - - -
        fn inc_rr(&mut self, reg: Register16) {
            let v = self.get_reg16(reg);
            self.set_reg16(reg, v + 1);
        }

        // DEC r
        // Flags:
        // Z N H C
        // * 1 * -
        fn dec_r(&mut self, reg: Register8) {
            let mut v = self.get_reg8(reg);
            v -= 1;
            self.set_reg8(reg, v);

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
        }

        // DEC rr
        // Flags:
        // Z N H C
        // - - - -
        fn dec_rr(&mut self, reg: Register16) {
            let v = self.get_reg16(reg);
            self.set_reg16(reg, v - 1);
        }

        // LD r, d8
        // Flags:
        // Z N H C
        // - - - -
        fn ld_r_d8(&mut self, reg: Register8, d: u8) {
            self.set_reg8(reg, d);
        }

        // LD rr, d16
        // Flags:
        // Z N H C
        // - - - -
        fn ld_rr_d16(&mut self, reg: Register16, d: u16) {
            self.set_reg16(reg, d);
        }

        // LD (HL-), A
        // Flags:
        // Z N H C
        // - - - -
        fn ldd_hl_a(&mut self) {
            let mut addr = self.get_reg16(Register16::HL);
            self.mmu.write8(addr, self.get_reg8(Register8::A));
            addr -= 1;
            self.set_reg16(Register16::HL, addr);
        }

        // LD (HL+), A
        // Flags:
        // Z N H C
        // - - - -
        fn ldi_hl_a(&mut self) {
            let mut addr = self.get_reg16(Register16::HL);
            self.mmu.write8(addr, self.get_reg8(Register8::A));
            addr += 1;
            self.set_reg16(Register16::HL, addr);
        }

        // XOR n
        // Flags:
        // Z N H C
        // * 0 0 0
        fn xor_r(&mut self, reg: Register8) {
            let mut a = self.get_reg8(Register8::A);
            let v = self.get_reg8(reg);
            a ^= v;
            self.set_reg8(Register8::A, v);

            self.f = 0;
            if a == 0 {
                self.f |= FLAG_ZERO;
            }
        }

        // BIT b, r
        // Flags:
        // Z N H C
        // * 0 1 -
        fn bit_b_r(&mut self, b: u8, reg: Register8) {
            let v = self.get_reg8(reg) & (1 << b);
            self.f = FLAG_HALF_CARRY;
            if v == 0 {
                self.f |= FLAG_ZERO;
            }
        }

        // BIT b, (HL)
        fn bit_b_hl(&mut self, b: u8) {
            let v = self.mmu.read8(self.get_reg16(Register16::HL)) & (1 << b);
            self.f = FLAG_HALF_CARRY;
            if v == 0 {
                self.f |= FLAG_ZERO;
            }
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
