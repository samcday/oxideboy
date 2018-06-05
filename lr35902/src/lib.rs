pub mod lr35902 {
    const FLAG_ZERO: u8      = 0b1000000;
    const FLAG_SUBTRACT: u8  = 0b0100000;
    const FLAG_HALF_CARRY:u8 = 0b0010000;

    pub trait MMU {
        fn read8(&self, addr: u16) -> u8;
        fn write8(&self, addr: u16, v: u8);

        fn read16(&self, addr: u16) -> u16;
        fn write16(&self, addr: u16, v: u16);
    }

    pub struct Core<'a> {
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

        mmu: &'a (MMU + 'a),
    }

    impl <'a> Core<'a> {
        pub fn run(&mut self) {
            let op = self.mmu.read8(self.pc);
            self.pc += 1;

            match op {
                // NOP
                0x00 => (),
                // LD BC, d16
                0x01 => {
                    let d = self.mmu.read16(self.pc);
                    self.pc += 2;

                    self.b = (d >> 8 & 0xFF) as u8;
                    self.c = (d & 0xFF) as u8;
                },
                // TODO: LD (BC), A
                // INC BC
                0x03 => {
                    self.c += 1;
                    if self.c == 0 {
                        self.b += 1;
                    }
                }
                // INC B
                0x04 => inc8(&mut self.b, &mut self.f),
                // DEC B
                0x05 => dec8(&mut self.b, &mut self.f),
                _ => (),
            };
        }
    }

    // Performs INC instruction on given register r, and modifies appropriate flags in flag register f.
    fn inc8(r: &mut u8, f: &mut u8) {
        *r += 1;

        // Set zero flag if we overflowed.
        if *r == 0 {
            *f |= FLAG_ZERO;
        }
        // INC clears subtract flag.
        *f ^= FLAG_SUBTRACT;
        // Set half carry flag if lower nibble overflowed.
        if (*r | 0b1111) == 0 {
            *f |= FLAG_HALF_CARRY;
        }
    }

    // Performs DEC instruction on given register r, and modifies appropriate flags in flag register f.
    fn dec8(r: &mut u8, f: &mut u8) {
        *r -= 1;

        if *r == 0 {
            *f |= FLAG_ZERO;
        }
        // DEC always sets subtract flag, obvs.
        *f |= FLAG_SUBTRACT;
        // TODO: when would we set half carry flag?
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
