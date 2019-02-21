//! Gameboy CPU emulation.
//! The Gameboy CPU is essentially just a modified Z80 processor. The ALU has 7 8bit registers and a flag register.
//! Everything else (RAM, cartridge ROM, PPU, APU) is accessed via the 16-bit wide memory bus.

use crate::{Hardware};
use self::Register8::{*};
use self::Register16::{*};

/// The heart of a Gameboy. A modified Z80 processor with 7 8-bit registers, and a flag register. The main emulation
/// loop consists of running the Gameboy CPU fetch-decode-execute cycle over and over. The Gameboy CPU normally runs at
/// 4.194304Mhz. The Color Gameboy can run the CPU twice as fast. It should be noted that this speed is the raw 
/// "machine clock" speed. In most places in the codebase we instead deal with a different speed: the machine clock
/// speed divided by 4: 1.048576Mhz. This is because all instructions take some multiple of 4 machine clock pulses (or
/// T-cycles as they're often referred to as). So for example, if we were to just run NOP CPU instructions over and over
/// we'd be executing 1,048,576 of them per second, not 4,194,304.
/// The real Gameboy CPU is connected to a master crystal, which is oscillating at the 4.194304Mhz speed. Alongside the
/// CPU the PPU and APU are also connected to that same crystal and running in lockstep with the CPU. The way we choose
/// to emulate is by having each instruction cycle (4 machine cycles) pump the other hardware components. So for example
/// when we fetch a NOP instruction and execute it, we also run all the other hardware components for a single cycle.
pub struct Cpu<HW: Hardware> {
    // CPU registers
    pub a: u8, pub f: Flags,
    pub b: u8, pub c: u8,
    pub h: u8, pub l: u8,
    pub d: u8, pub e: u8,
    sp: u16,
    pub pc: u16,

    ime: bool,              // The IME register, master switch for turning all interrupts on/off.
    ime_defer: bool,        // Enabling interrupts is delayed by a cycle, we track that here.
    halted: bool,

    pub mooneye_breakpoint: bool,

    pub hw: HW,
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

/// The 8 available 8-bit CPU registers. The F register is not included here because
/// no instructions reference it directly, it's only modified / queried indirectly via various instructions.
#[derive(Clone, Copy, Debug)]
enum Register8 { A, B, C, D, E, H, L }

/// Enum for the 5 available 16-bit CPU registers. With the exception of HL and SP, these registers are just pairs of
/// the 8-bit ones viewed as 16 bits.
#[derive(Clone, Copy, Debug)]
enum Register16 { AF, BC, DE, HL, SP }

/// The 4 conditions available to branching instructions (CALL/JP/JR/RET).
#[derive(Clone, Copy, Debug)]
enum FlagCondition {
    NZ, // CPU flag Z is clear
    Z,  // CPU flag Z is set
    NC, // CPU flag C is clear
    C,  // CPU flag C is set
}

enum BitwiseOp { XOR, OR, AND }

/// Many of the CPU instructions do the same thing, and only differ by which register / memory location they operate on.
/// For example, you can ADD a value to the A register using either another register, the 8-bit value pointed to by HL,
/// or an immediate 8-bit value.
#[derive(Clone, Copy, Debug)]
enum Operand8 {
    Register(Register8),        // Get/set value of an 8-bit register.
    Immediate(u8),              // Use an immediate value (an 8-bit value that is embedded alongside the opcode).
    Address(Register16),        // Memory address pointed to by a 16-bit register.
    AddressInc(Register16),     // Memory address pointed to by a 16-bit register. Increment register after use.
    AddressDec(Register16),     // Memory address pointed to by a 16-bit register. Decrement register after use.
    ImmediateAddress(u16),      // Immediate 16-bit value interpreted as memory address.
    ImmediateAddressHigh(u8),   // Immediate 8-bit value interpreted as memory address offset from $FF00.
    AddressHigh(Register8),     // 8-bit register value interpreted as memory address offset from $FF00.
}

/// Similar to Operand8, but for the (few) 16-bit instructions the CPU provides.
#[derive(Clone, Copy, Debug)]
enum Operand16 {
    Register(Register16),   // Contents of a 16-bit register.
    Immediate(u16),         // Immediate value.
    ImmediateAddress(u16),  // Immediate 16-bit value interpreted as memory address.
}

impl Flags {
    /// Converts the CPU flags into an 8bit value. Used by instructions that operate on F register.
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

    /// Clears all CPU flags.
    fn reset(&mut self) -> &mut Self {
        self.z = false;
        self.n = false;
        self.h = false;
        self.c = false;
        self
    }

    /// Checks if current state of CPU flags satisfies a particular FlagCondition.
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

impl <HW: Hardware> Cpu<HW> {
    pub fn new(hw: HW) -> Cpu<HW> {
        Cpu {
            a: 0, f: Default::default(),
            b: 0, c: 0,
            h: 0, l: 0,
            d: 0, e: 0,
            sp: 0,
            pc: 0,
            halted: false,
            ime: false, ime_defer: false,
            mooneye_breakpoint: false,
            hw,
        }
    }

    pub fn skip_bootrom(&mut self) {
        self.pc = 0x100;
        self.sp = 0xFFFE;
        self.a = 0x01;
        self.b = 0xFF;
        self.c = 0x13;
        self.e = 0xC1;
        self.h = 0x84;
        self.l = 0x03;
        self.hw.skip_bootrom();
    }

    pub fn core_panic(&self, msg: String) -> ! {
        panic!(
            "{}\nIME: {},{}\nHalt: {}\nRegs:\n\tA=0x{:02X}\n\tB=0x{:02X}\n\tC=0x{:02X}\n\tD=0x{:02X}\n\tE=0x{:02X}\n\tF=0x{:02X}\n\tH=0x{:02X}\n\tL=0x{:02X}\n\tSP={:#04X}\n\tPC={:#04X}",
            msg, self.ime, self.ime_defer, self.halted, self.a, self.b, self.c, self.d, self.e, self.f.pack(), self.h, self.l, self.sp, self.pc);
    }

    /// Main entrypoint into the Cpu implementation. Fetches the next instruction, then decodes and executes it.
    pub fn fetch_decode_execute(&mut self) {
        // If the CPU is currently halted, we need to pump a clock cycle of the other hardware, so that if there's a new
        // interrupt available, we can wake up from HALT and continue on.
        if self.halted {
            self.hw.clock();
        }

        self.process_interrupts();

        // Apply deferred change to IME register.
        if self.ime_defer {
            self.ime = true;
            self.ime_defer = false;
        }

        // CPU is halted and there wasn't any pending interrupts to wake us up, we're done for now.
        if self.halted {
            return;
        }

        match self.fetch8() {
            0x00 /* NOP         */ => {},
            0x01 /* LD BC,d16   */ => { let v = self.fetch16(); self.ld16(Operand16::Register(BC), Operand16::Immediate(v), false); },
            0x02 /* LD (BC),A   */ => self.ld8(Operand8::Address(BC), Operand8::Register(A)),
            0x03 /* INC BC      */ => self.inc16(BC),
            0x04 /* INC B       */ => self.inc8(Operand8::Register(B)),
            0x05 /* DEC B       */ => self.dec8(Operand8::Register(B)),
            0x06 /* LD B,d8     */ => { let v = self.fetch8(); self.ld8(Operand8::Register(B), Operand8::Immediate(v)); }
            0x07 /* RLCA        */ => self.rlc(Operand8::Register(A), false),
            0x08 /* LD (a16),SP */ => { let v = self.fetch16(); self.ld16(Operand16::ImmediateAddress(v), Operand16::Register(SP), false) },
            0x09 /* ADD HL,BC   */ => self.add16(BC),
            0x0A /* LD A,(BC)   */ => self.ld8(Operand8::Register(A), Operand8::Address(BC)),
            0x0B /* DEC BC      */ => self.dec16(BC),
            0x0C /* INC C       */ => self.inc8(Operand8::Register(C)),
            0x0D /* DEC C       */ => self.dec8(Operand8::Register(C)),
            0x0E /* LD C,d8     */ => { let v = self.fetch8(); self.ld8(Operand8::Register(C), Operand8::Immediate(v)) },
            0x0F /* RRCA        */ => self.rrc(Operand8::Register(A), false),
            0x10 /* STOP        */ => self.stop(),
            0x11 /* LD DE,d16   */ => { let v = self.fetch16(); self.ld16(Operand16::Register(DE), Operand16::Immediate(v), false) },
            0x12 /* LD (DE),A   */ => self.ld8(Operand8::Address(DE), Operand8::Register(A)),
            0x13 /* INC DE      */ => self.inc16(DE),
            0x14 /* INC D       */ => self.inc8(Operand8::Register(D)),
            0x15 /* DEC D       */ => self.dec8(Operand8::Register(D)),
            0x16 /* LD D,d8     */ => { let v = self.fetch8(); self.ld8(Operand8::Register(D), Operand8::Immediate(v)) },
            0x17 /* RLA         */ => self.rl(Operand8::Register(A), false, true),
            0x18 /* JR r8       */ => { let v = self.fetch8(); self.jr(None, v) },
            0x19 /* ADD HL,DE   */ => self.add16(DE),
            0x1A /* LD A,(DE)   */ => self.ld8(Operand8::Register(A), Operand8::Address(DE)),
            0x1B /* DEC DE      */ => self.dec16(DE),
            0x1C /* INC E       */ => self.inc8(Operand8::Register(E)),
            0x1D /* DEC E       */ => self.dec8(Operand8::Register(E)),
            0x1E /* LD E,d8     */ => { let v = self.fetch8(); self.ld8(Operand8::Register(E), Operand8::Immediate(v)) },
            0x1F /* RRA         */ => self.rr(Operand8::Register(A), false),
            0x20 /* JR NZ,r8    */ => { let v = self.fetch8(); self.jr(Some(FlagCondition::NZ), v) },
            0x21 /* LD HL,d16   */ => { let v = self.fetch16(); self.ld16(Operand16::Register(HL), Operand16::Immediate(v), false) },
            0x22 /* LD (HL+),A  */ => self.ld8(Operand8::AddressInc(HL), Operand8::Register(A)),
            0x23 /* INC HL      */ => self.inc16(HL),
            0x24 /* INC H       */ => self.inc8(Operand8::Register(H)),
            0x25 /* DEC H       */ => self.dec8(Operand8::Register(H)),
            0x26 /* LD H,d8     */ => { let v = self.fetch8(); self.ld8(Operand8::Register(H), Operand8::Immediate(v)) },
            0x27 /* DAA         */ => self.daa(),
            0x28 /* JR Z,r8     */ => { let v = self.fetch8(); self.jr(Some(FlagCondition::Z), v) },
            0x29 /* ADD HL,HL   */ => self.add16(HL),
            0x2A /* LD A,(HL+)  */ => self.ld8(Operand8::Register(A), Operand8::AddressInc(HL)),
            0x2B /* DEC HL      */ => self.dec16(HL),
            0x2C /* INC L       */ => self.inc8(Operand8::Register(L)),
            0x2D /* DEC L       */ => self.dec8(Operand8::Register(L)),
            0x2E /* LD L,d8     */ => { let v = self.fetch8(); self.ld8(Operand8::Register(L), Operand8::Immediate(v)) },
            0x2F /* CPL         */ => self.cpl(),
            0x30 /* JR NC,r8    */ => { let v = self.fetch8(); self.jr(Some(FlagCondition::NC), v) },
            0x31 /* LD SP,d16   */ => { let v = self.fetch16(); self.ld16(Operand16::Register(SP), Operand16::Immediate(v), false) },
            0x32 /* LD (HL-), A */ => self.ld8(Operand8::AddressDec(HL), Operand8::Register(A)),
            0x33 /* INC SP      */ => self.inc16(SP),
            0x34 /* INC (HL)    */ => self.inc8(Operand8::Address(HL)),
            0x35 /* DEC (HL)    */ => self.dec8(Operand8::Address(HL)),
            0x36 /* LD (HL),d8  */ => { let v = self.fetch8(); self.ld8(Operand8::Address(HL), Operand8::Immediate(v)) },
            0x37 /* SCF         */ => self.scf(),
            0x38 /* JR C,r8     */ => { let v = self.fetch8(); self.jr(Some(FlagCondition::C), v) },
            0x39 /* ADD HL,SP   */ => self.add16(SP),
            0x3A /* LD A, (HL-) */ => self.ld8(Operand8::Register(A), Operand8::AddressDec(HL)),
            0x3B /* DEC SP      */ => self.dec16(SP),
            0x3C /* INC A       */ => self.inc8(Operand8::Register(A)),
            0x3D /* DEC A       */ => self.dec8(Operand8::Register(A)),
            0x3E /* LD A,d8     */ => { let v = self.fetch8(); self.ld8(Operand8::Register(A), Operand8::Immediate(v)) },
            0x3F /* CCF         */ => self.ccf(),
            0x40 /* LD B,B      */ => {
                self.mooneye_breakpoint = true;
                self.ld8(Operand8::Register(B), Operand8::Register(B))
            },
            0x41 /* LD B,C      */ => self.ld8(Operand8::Register(B), Operand8::Register(C)),
            0x42 /* LD B,D      */ => self.ld8(Operand8::Register(B), Operand8::Register(D)),
            0x43 /* LD B,E      */ => self.ld8(Operand8::Register(B), Operand8::Register(E)),
            0x44 /* LD B,H      */ => self.ld8(Operand8::Register(B), Operand8::Register(H)),
            0x45 /* LD B,L      */ => self.ld8(Operand8::Register(B), Operand8::Register(L)),
            0x46 /* LD B,(HL)   */ => self.ld8(Operand8::Register(B), Operand8::Address(HL)),
            0x47 /* LD B,A      */ => self.ld8(Operand8::Register(B), Operand8::Register(A)),
            0x48 /* LD C,B      */ => self.ld8(Operand8::Register(C), Operand8::Register(B)),
            0x49 /* LD C,C      */ => self.ld8(Operand8::Register(C), Operand8::Register(C)),
            0x4A /* LD C,D      */ => self.ld8(Operand8::Register(C), Operand8::Register(D)),
            0x4B /* LD C,E      */ => self.ld8(Operand8::Register(C), Operand8::Register(E)),
            0x4C /* LD C,H      */ => self.ld8(Operand8::Register(C), Operand8::Register(H)),
            0x4D /* LD C,L      */ => self.ld8(Operand8::Register(C), Operand8::Register(L)),
            0x4E /* LD C,(HL)   */ => self.ld8(Operand8::Register(C), Operand8::Address(HL)),
            0x4F /* LD C,A      */ => self.ld8(Operand8::Register(C), Operand8::Register(A)),
            0x50 /* LD D,B      */ => self.ld8(Operand8::Register(D), Operand8::Register(B)),
            0x51 /* LD D,C      */ => self.ld8(Operand8::Register(D), Operand8::Register(C)),
            0x52 /* LD D,D      */ => self.ld8(Operand8::Register(D), Operand8::Register(D)),
            0x53 /* LD D,E      */ => self.ld8(Operand8::Register(D), Operand8::Register(E)),
            0x54 /* LD D,H      */ => self.ld8(Operand8::Register(D), Operand8::Register(H)),
            0x55 /* LD D,L      */ => self.ld8(Operand8::Register(D), Operand8::Register(L)),
            0x56 /* LD D,(HL)   */ => self.ld8(Operand8::Register(D), Operand8::Address(HL)),
            0x57 /* LD D,A      */ => self.ld8(Operand8::Register(D), Operand8::Register(A)),
            0x58 /* LD E,B      */ => self.ld8(Operand8::Register(E), Operand8::Register(B)),
            0x59 /* LD E,C      */ => self.ld8(Operand8::Register(E), Operand8::Register(C)),
            0x5A /* LD E,D      */ => self.ld8(Operand8::Register(E), Operand8::Register(D)),
            0x5B /* LD E,E      */ => self.ld8(Operand8::Register(E), Operand8::Register(E)),
            0x5C /* LD E,H      */ => self.ld8(Operand8::Register(E), Operand8::Register(H)),
            0x5D /* LD E,L      */ => self.ld8(Operand8::Register(E), Operand8::Register(L)),
            0x5E /* LD E,(HL)   */ => self.ld8(Operand8::Register(E), Operand8::Address(HL)),
            0x5F /* LD E,A      */ => self.ld8(Operand8::Register(E), Operand8::Register(A)),
            0x60 /* LD H,B      */ => self.ld8(Operand8::Register(H), Operand8::Register(B)),
            0x61 /* LD H,C      */ => self.ld8(Operand8::Register(H), Operand8::Register(C)),
            0x62 /* LD H,D      */ => self.ld8(Operand8::Register(H), Operand8::Register(D)),
            0x63 /* LD H,E      */ => self.ld8(Operand8::Register(H), Operand8::Register(E)),
            0x64 /* LD H,H      */ => self.ld8(Operand8::Register(H), Operand8::Register(H)),
            0x65 /* LD H,L      */ => self.ld8(Operand8::Register(H), Operand8::Register(L)),
            0x66 /* LD H,(HL)   */ => self.ld8(Operand8::Register(H), Operand8::Address(HL)),
            0x67 /* LD H,A      */ => self.ld8(Operand8::Register(H), Operand8::Register(A)),
            0x68 /* LD L,B      */ => self.ld8(Operand8::Register(L), Operand8::Register(B)),
            0x69 /* LD L,C      */ => self.ld8(Operand8::Register(L), Operand8::Register(C)),
            0x6A /* LD L,D      */ => self.ld8(Operand8::Register(L), Operand8::Register(D)),
            0x6B /* LD L,E      */ => self.ld8(Operand8::Register(L), Operand8::Register(E)),
            0x6C /* LD L,H      */ => self.ld8(Operand8::Register(L), Operand8::Register(H)),
            0x6D /* LD L,L      */ => self.ld8(Operand8::Register(L), Operand8::Register(L)),
            0x6E /* LD L,(HL)   */ => self.ld8(Operand8::Register(L), Operand8::Address(HL)),
            0x6F /* LD L,A      */ => self.ld8(Operand8::Register(L), Operand8::Register(A)),
            0x70 /* LD (HL),B   */ => self.ld8(Operand8::Address(HL), Operand8::Register(B)),
            0x71 /* LD (HL),C   */ => self.ld8(Operand8::Address(HL), Operand8::Register(C)),
            0x72 /* LD (HL),D   */ => self.ld8(Operand8::Address(HL), Operand8::Register(D)),
            0x73 /* LD (HL),E   */ => self.ld8(Operand8::Address(HL), Operand8::Register(E)),
            0x74 /* LD (HL),H   */ => self.ld8(Operand8::Address(HL), Operand8::Register(H)),
            0x75 /* LD (HL),L   */ => self.ld8(Operand8::Address(HL), Operand8::Register(L)),
            0x76 /* HALT        */ => self.halt(),
            0x77 /* LD (HL),A   */ => self.ld8(Operand8::Address(HL), Operand8::Register(A)),
            0x78 /* LD A,B      */ => self.ld8(Operand8::Register(A), Operand8::Register(B)),
            0x79 /* LD A,C      */ => self.ld8(Operand8::Register(A), Operand8::Register(C)),
            0x7A /* LD A,D      */ => self.ld8(Operand8::Register(A), Operand8::Register(D)),
            0x7B /* LD A,E      */ => self.ld8(Operand8::Register(A), Operand8::Register(E)),
            0x7C /* LD A,H      */ => self.ld8(Operand8::Register(A), Operand8::Register(H)),
            0x7D /* LD A,L      */ => self.ld8(Operand8::Register(A), Operand8::Register(L)),
            0x7E /* LD A,(HL)   */ => self.ld8(Operand8::Register(A), Operand8::Address(HL)),
            0x7F /* LD A,A      */ => self.ld8(Operand8::Register(A), Operand8::Register(A)),
            0x80 /* ADD A,B     */ => self.add8(Operand8::Register(B), false),
            0x81 /* ADD A,C     */ => self.add8(Operand8::Register(C), false),
            0x82 /* ADD A,D     */ => self.add8(Operand8::Register(D), false),
            0x83 /* ADD A,E     */ => self.add8(Operand8::Register(E), false),
            0x84 /* ADD A,H     */ => self.add8(Operand8::Register(H), false),
            0x85 /* ADD A,L     */ => self.add8(Operand8::Register(L), false),
            0x86 /* ADD A,(HL)  */ => self.add8(Operand8::Address(HL), false),
            0x87 /* ADD A,A     */ => self.add8(Operand8::Register(A), false),
            0x88 /* ADC A,B     */ => self.add8(Operand8::Register(B), true),
            0x89 /* ADC A,C     */ => self.add8(Operand8::Register(C), true),
            0x8A /* ADC A,D     */ => self.add8(Operand8::Register(D), true),
            0x8B /* ADC A,E     */ => self.add8(Operand8::Register(E), true),
            0x8C /* ADC A,H     */ => self.add8(Operand8::Register(H), true),
            0x8D /* ADC A,L     */ => self.add8(Operand8::Register(L), true),
            0x8E /* ADC A,(HL)  */ => self.add8(Operand8::Address(HL), true),
            0x8F /* ADC A,A     */ => self.add8(Operand8::Register(A), true),
            0x90 /* SUB B       */ => self.sub8(Operand8::Register(B), false, true),
            0x91 /* SUB C       */ => self.sub8(Operand8::Register(C), false, true),
            0x92 /* SUB D       */ => self.sub8(Operand8::Register(D), false, true),
            0x93 /* SUB E       */ => self.sub8(Operand8::Register(E), false, true),
            0x94 /* SUB H       */ => self.sub8(Operand8::Register(H), false, true),
            0x95 /* SUB L       */ => self.sub8(Operand8::Register(L), false, true),
            0x96 /* SUB (HL)    */ => self.sub8(Operand8::Address(HL), false, true),
            0x97 /* SUB A       */ => self.sub8(Operand8::Register(A), false, true),
            0x98 /* SBC A,B     */ => self.sub8(Operand8::Register(B), true, true),
            0x99 /* SBC A,C     */ => self.sub8(Operand8::Register(C), true, true),
            0x9A /* SBC A,D     */ => self.sub8(Operand8::Register(D), true, true),
            0x9B /* SBC A,E     */ => self.sub8(Operand8::Register(E), true, true),
            0x9C /* SBC A,H     */ => self.sub8(Operand8::Register(H), true, true),
            0x9D /* SBC A,L     */ => self.sub8(Operand8::Register(L), true, true),
            0x9E /* SBC A,(HL)  */ => self.sub8(Operand8::Address(HL), true, true),
            0x9F /* SBC A,A     */ => self.sub8(Operand8::Register(A), true, true),
            0xA0 /* AND B       */ => self.bitwise(BitwiseOp::AND, Operand8::Register(B)),
            0xA1 /* AND C       */ => self.bitwise(BitwiseOp::AND, Operand8::Register(C)),
            0xA2 /* AND D       */ => self.bitwise(BitwiseOp::AND, Operand8::Register(D)),
            0xA3 /* AND E       */ => self.bitwise(BitwiseOp::AND, Operand8::Register(E)),
            0xA4 /* AND H       */ => self.bitwise(BitwiseOp::AND, Operand8::Register(H)),
            0xA5 /* AND L       */ => self.bitwise(BitwiseOp::AND, Operand8::Register(L)),
            0xA6 /* AND (HL)    */ => self.bitwise(BitwiseOp::AND, Operand8::Address(HL)),
            0xA7 /* AND A       */ => self.bitwise(BitwiseOp::AND, Operand8::Register(A)),
            0xA8 /* XOR B       */ => self.bitwise(BitwiseOp::XOR, Operand8::Register(B)),
            0xA9 /* XOR C       */ => self.bitwise(BitwiseOp::XOR, Operand8::Register(C)),
            0xAA /* XOR D       */ => self.bitwise(BitwiseOp::XOR, Operand8::Register(D)),
            0xAB /* XOR E       */ => self.bitwise(BitwiseOp::XOR, Operand8::Register(E)),
            0xAC /* XOR H       */ => self.bitwise(BitwiseOp::XOR, Operand8::Register(H)),
            0xAD /* XOR L       */ => self.bitwise(BitwiseOp::XOR, Operand8::Register(L)),
            0xAE /* XOR (HL)    */ => self.bitwise(BitwiseOp::XOR, Operand8::Address(HL)),
            0xAF /* XOR A       */ => self.bitwise(BitwiseOp::XOR, Operand8::Register(A)),
            0xB0 /* OR B        */ => self.bitwise(BitwiseOp::OR,  Operand8::Register(B)),
            0xB1 /* OR C        */ => self.bitwise(BitwiseOp::OR,  Operand8::Register(C)),
            0xB2 /* OR D        */ => self.bitwise(BitwiseOp::OR,  Operand8::Register(D)),
            0xB3 /* OR E        */ => self.bitwise(BitwiseOp::OR,  Operand8::Register(E)),
            0xB4 /* OR H        */ => self.bitwise(BitwiseOp::OR,  Operand8::Register(H)),
            0xB5 /* OR L        */ => self.bitwise(BitwiseOp::OR,  Operand8::Register(L)),
            0xB6 /* OR (HL)     */ => self.bitwise(BitwiseOp::OR,  Operand8::Address(HL)),
            0xB7 /* OR A        */ => self.bitwise(BitwiseOp::OR,  Operand8::Register(A)),
            0xB8 /* CP B        */ => self.sub8(Operand8::Register(B), false, false),
            0xB9 /* CP C        */ => self.sub8(Operand8::Register(C), false, false),
            0xBA /* CP D        */ => self.sub8(Operand8::Register(D), false, false),
            0xBB /* CP E        */ => self.sub8(Operand8::Register(E), false, false),
            0xBC /* CP H        */ => self.sub8(Operand8::Register(H), false, false),
            0xBD /* CP L        */ => self.sub8(Operand8::Register(L), false, false),
            0xBE /* CP (HL)     */ => self.sub8(Operand8::Address(HL), false, false),
            0xBF /* CP A        */ => self.sub8(Operand8::Register(A), false, false),
            0xC0 /* RET NZ      */ => self.ret(Some(FlagCondition::NZ), false),
            0xC1 /* POP BC      */ => self.pop(BC),
            0xC2 /* JP NZ,a16   */ => { let v = self.fetch16(); self.jp(Some(FlagCondition::NZ), Operand16::Immediate(v)) },
            0xC3 /* JP a16      */ => { let v = self.fetch16(); self.jp(None, Operand16::Immediate(v)) },
            0xC4 /* CALL NZ,a16 */ => { let v = self.fetch16(); self.call(Some(FlagCondition::NZ), v) },
            0xC5 /* PUSH BC     */ => self.push(BC),
            0xC6 /* ADD A,d8    */ => { let v = self.fetch8(); self.add8(Operand8::Immediate(v), false) },
            0xC7 /* RST 00H     */ => self.rst(0x00),
            0xC8 /* RET Z       */ => self.ret(Some(FlagCondition::Z), false),
            0xC9 /* RET         */ => self.ret(None, false),
            0xCA /* JP Z,a16    */ => { let v = self.fetch16(); self.jp(Some(FlagCondition::Z), Operand16::Immediate(v)) },
            0xCB /* PREFIX CB   */ => self.fetch_decode_execute_extended(),
            0xCC /* CALL Z,a16  */ => { let v = self.fetch16(); self.call(Some(FlagCondition::Z), v) },
            0xCD /* CALL a16    */ => { let v = self.fetch16(); self.call(None, v) },
            0xCE /* ADC A,d8    */ => { let v = self.fetch8(); self.add8(Operand8::Immediate(v), true) },
            0xCF /* RST 08H     */ => self.rst(0x08),
            0xD0 /* RET NC      */ => self.ret(Some(FlagCondition::NC), false),
            0xD1 /* POP DE      */ => self.pop(DE),
            0xD2 /* JP NC,a16   */ => { let v = self.fetch16(); self.jp(Some(FlagCondition::NC), Operand16::Immediate(v)) },
            0xD4 /* CALL NC,a16 */ => { let v = self.fetch16(); self.call(Some(FlagCondition::NC), v) },
            0xD5 /* PUSH DE     */ => self.push(DE),
            0xD6 /* SUB d8      */ => { let v = self.fetch8(); self.sub8(Operand8::Immediate(v), false, true) },
            0xD7 /* RST 10H     */ => self.rst(0x10),
            0xD8 /* RET C       */ => self.ret(Some(FlagCondition::C), false),
            0xD9 /* RETI        */ => self.ret(None, true),
            0xDA /* JP C,a16    */ => { let v = self.fetch16(); self.jp(Some(FlagCondition::C), Operand16::Immediate(v)) },
            0xDC /* CALL C,a16  */ => { let v = self.fetch16(); self.call(Some(FlagCondition::C), v) },
            0xDE /* SBC A,d8    */ => { let v = self.fetch8(); self.sub8(Operand8::Immediate(v), true, true) },
            0xDF /* RST 18H     */ => self.rst(0x18),
            0xE0 /* LDH (a8),A  */ => { let v = self.fetch8(); self.ld8(Operand8::ImmediateAddressHigh(v), Operand8::Register(A)) },
            0xE1 /* POP HL      */ => self.pop(HL),
            0xE2 /* LD (C),A    */ => self.ld8(Operand8::AddressHigh(C), Operand8::Register(A)),
            0xE5 /* PUSH HL     */ => self.push(HL),
            0xE6 /* ADD d8      */ => { let v = self.fetch8(); self.bitwise(BitwiseOp::AND, Operand8::Immediate(v)) },
            0xE7 /* RST 20H     */ => self.rst(0x20),
            0xE8 /* ADD SP,r8   */ => { let v = self.fetch8(); self.add_sp_r8(v as i8) },
            0xE9 /* JP (HL)     */ => self.jp(None, Operand16::Register(HL)),
            0xEA /* LD (a16),A  */ => { let v = self.fetch16(); self.ld8(Operand8::ImmediateAddress(v), Operand8::Register(A)) },
            0xEE /* XOR d8      */ => { let v = self.fetch8(); self.bitwise(BitwiseOp::XOR, Operand8::Immediate(v)) },
            0xEF /* RST 28H     */ => self.rst(0x28),
            0xF0 /* LDH A,(a8)  */ => { let v = self.fetch8(); self.ld8(Operand8::Register(A), Operand8::ImmediateAddressHigh(v)) },
            0xF1 /* POP AF      */ => self.pop(AF),
            0xF2 /* LD A,(C)    */ => self.ld8(Operand8::Register(A), Operand8::AddressHigh(C)),
            0xF3 /* DI          */ => self.set_ime(false),
            0xF5 /* PUSH AF     */ => self.push(AF),
            0xF6 /* OR d8       */ => { let v = self.fetch8(); self.bitwise(BitwiseOp::OR, Operand8::Immediate(v)) },
            0xF7 /* RST 30H     */ => self.rst(0x30),
            0xF8 /* LD HL,SP+r8 */ => { let v = self.fetch8(); self.ld_hl_sp(v as i8) },
            0xF9 /* LD SP,HL    */ => self.ld16(Operand16::Register(SP), Operand16::Register(HL), true),
            0xFA /* LD A,(a16)  */ => { let v = self.fetch16(); self.ld8(Operand8::Register(A), Operand8::ImmediateAddress(v)) },
            0xFB /* EI          */ => self.set_ime(true),
            0xFE /* CP d8       */ => { let v = self.fetch8(); self.sub8(Operand8::Immediate(v), false, false) },
            0xFF /* RST 38H     */ => self.rst(0x38),
            v => panic!("Unhandled opcode ${:X}", v),
        }
    }

    /// An extension to fetch_decode_execute for the $CB extended opcode.
    fn fetch_decode_execute_extended(&mut self) {
        match self.fetch8() {
            0x00 /* RLC B      */ => self.rlc(Operand8::Register(B), true),
            0x01 /* RLC C      */ => self.rlc(Operand8::Register(C), true),
            0x02 /* RLC D      */ => self.rlc(Operand8::Register(D), true),
            0x03 /* RLC E      */ => self.rlc(Operand8::Register(E), true),
            0x04 /* RLC H      */ => self.rlc(Operand8::Register(H), true),
            0x05 /* RLC L      */ => self.rlc(Operand8::Register(L), true),
            0x06 /* RLC (HL)   */ => self.rlc(Operand8::Address(HL), true),
            0x07 /* RLC A      */ => self.rlc(Operand8::Register(A), true),
            0x08 /* RRC B      */ => self.rrc(Operand8::Register(B), true),
            0x09 /* RRC C      */ => self.rrc(Operand8::Register(C), true),
            0x0A /* RRC D      */ => self.rrc(Operand8::Register(D), true),
            0x0B /* RRC E      */ => self.rrc(Operand8::Register(E), true),
            0x0C /* RRC H      */ => self.rrc(Operand8::Register(H), true),
            0x0D /* RRC L      */ => self.rrc(Operand8::Register(L), true),
            0x0E /* RRC (HL)   */ => self.rrc(Operand8::Address(HL), true),
            0x0F /* RRC A      */ => self.rrc(Operand8::Register(A), true),
            0x10 /* RL B       */ => self.rl(Operand8::Register(B), true, true),
            0x11 /* RL C       */ => self.rl(Operand8::Register(C), true, true),
            0x12 /* RL D       */ => self.rl(Operand8::Register(D), true, true),
            0x13 /* RL E       */ => self.rl(Operand8::Register(E), true, true),
            0x14 /* RL H       */ => self.rl(Operand8::Register(H), true, true),
            0x15 /* RL L       */ => self.rl(Operand8::Register(L), true, true),
            0x16 /* RL (HL)    */ => self.rl(Operand8::Address(HL), true, true),
            0x17 /* RL A       */ => self.rl(Operand8::Register(A), true, true),
            0x18 /* RR B       */ => self.rr(Operand8::Register(B), true),
            0x19 /* RR C       */ => self.rr(Operand8::Register(C), true),
            0x1A /* RR D       */ => self.rr(Operand8::Register(D), true),
            0x1B /* RR E       */ => self.rr(Operand8::Register(E), true),
            0x1C /* RR H       */ => self.rr(Operand8::Register(H), true),
            0x1D /* RR L       */ => self.rr(Operand8::Register(L), true),
            0x1E /* RR (HL)    */ => self.rr(Operand8::Address(HL), true),
            0x1F /* RR A       */ => self.rr(Operand8::Register(A), true),
            0x20 /* SLA B      */ => self.rl(Operand8::Register(B), true, false),
            0x21 /* SLA C      */ => self.rl(Operand8::Register(C), true, false),
            0x22 /* SLA D      */ => self.rl(Operand8::Register(D), true, false),
            0x23 /* SLA E      */ => self.rl(Operand8::Register(E), true, false),
            0x24 /* SLA H      */ => self.rl(Operand8::Register(H), true, false),
            0x25 /* SLA L      */ => self.rl(Operand8::Register(L), true, false),
            0x26 /* SLA (HL)   */ => self.rl(Operand8::Address(HL), true, false),
            0x27 /* SLA A      */ => self.rl(Operand8::Register(A), true, false),
            0x28 /* SRA B      */ => self.shift_right(Operand8::Register(B), true),
            0x29 /* SRA C      */ => self.shift_right(Operand8::Register(C), true),
            0x2A /* SRA D      */ => self.shift_right(Operand8::Register(D), true),
            0x2B /* SRA E      */ => self.shift_right(Operand8::Register(E), true),
            0x2C /* SRA H      */ => self.shift_right(Operand8::Register(H), true),
            0x2D /* SRA L      */ => self.shift_right(Operand8::Register(L), true),
            0x2E /* SRA (HL)   */ => self.shift_right(Operand8::Address(HL), true),
            0x2F /* SRA A      */ => self.shift_right(Operand8::Register(A), true),
            0x30 /* SWAP B     */ => self.swap(Operand8::Register(B)),
            0x31 /* SWAP C     */ => self.swap(Operand8::Register(C)),
            0x32 /* SWAP D     */ => self.swap(Operand8::Register(D)),
            0x33 /* SWAP E     */ => self.swap(Operand8::Register(E)),
            0x34 /* SWAP H     */ => self.swap(Operand8::Register(H)),
            0x35 /* SWAP L     */ => self.swap(Operand8::Register(L)),
            0x36 /* SWAP (HL)  */ => self.swap(Operand8::Address(HL)),
            0x37 /* SWAP A     */ => self.swap(Operand8::Register(A)),
            0x38 /* SRL B      */ => self.shift_right(Operand8::Register(B), false),
            0x39 /* SRL C      */ => self.shift_right(Operand8::Register(C), false),
            0x3A /* SRL D      */ => self.shift_right(Operand8::Register(D), false),
            0x3B /* SRL E      */ => self.shift_right(Operand8::Register(E), false),
            0x3C /* SRL H      */ => self.shift_right(Operand8::Register(H), false),
            0x3D /* SRL L      */ => self.shift_right(Operand8::Register(L), false),
            0x3E /* SRL (HL)   */ => self.shift_right(Operand8::Address(HL), false),
            0x3F /* SRL A      */ => self.shift_right(Operand8::Register(A), false),
            0x40 /* BIT 0,B    */ => self.bit(0, Operand8::Register(B)),
            0x41 /* BIT 0,C    */ => self.bit(0, Operand8::Register(C)),
            0x42 /* BIT 0,D    */ => self.bit(0, Operand8::Register(D)),
            0x43 /* BIT 0,E    */ => self.bit(0, Operand8::Register(E)),
            0x44 /* BIT 0,H    */ => self.bit(0, Operand8::Register(H)),
            0x45 /* BIT 0,L    */ => self.bit(0, Operand8::Register(L)),
            0x46 /* BIT 0,(HL) */ => self.bit(0, Operand8::Address(HL)),
            0x47 /* BIT 0,A    */ => self.bit(0, Operand8::Register(A)),
            0x48 /* BIT 1,B    */ => self.bit(1, Operand8::Register(B)),
            0x49 /* BIT 1,C    */ => self.bit(1, Operand8::Register(C)),
            0x4A /* BIT 1,D    */ => self.bit(1, Operand8::Register(D)),
            0x4B /* BIT 1,E    */ => self.bit(1, Operand8::Register(E)),
            0x4C /* BIT 1,H    */ => self.bit(1, Operand8::Register(H)),
            0x4D /* BIT 1,L    */ => self.bit(1, Operand8::Register(L)),
            0x4E /* BIT 1,(HL) */ => self.bit(1, Operand8::Address(HL)),
            0x4F /* BIT 1,A    */ => self.bit(1, Operand8::Register(A)),
            0x50 /* BIT 2,B    */ => self.bit(2, Operand8::Register(B)),
            0x51 /* BIT 2,C    */ => self.bit(2, Operand8::Register(C)),
            0x52 /* BIT 2,D    */ => self.bit(2, Operand8::Register(D)),
            0x53 /* BIT 2,E    */ => self.bit(2, Operand8::Register(E)),
            0x54 /* BIT 2,H    */ => self.bit(2, Operand8::Register(H)),
            0x55 /* BIT 2,L    */ => self.bit(2, Operand8::Register(L)),
            0x56 /* BIT 2,(HL) */ => self.bit(2, Operand8::Address(HL)),
            0x57 /* BIT 2,A    */ => self.bit(2, Operand8::Register(A)),
            0x58 /* BIT 3,B    */ => self.bit(3, Operand8::Register(B)),
            0x59 /* BIT 3,C    */ => self.bit(3, Operand8::Register(C)),
            0x5A /* BIT 3,D    */ => self.bit(3, Operand8::Register(D)),
            0x5B /* BIT 3,E    */ => self.bit(3, Operand8::Register(E)),
            0x5C /* BIT 3,H    */ => self.bit(3, Operand8::Register(H)),
            0x5D /* BIT 3,L    */ => self.bit(3, Operand8::Register(L)),
            0x5E /* BIT 3,(HL) */ => self.bit(3, Operand8::Address(HL)),
            0x5F /* BIT 3,A    */ => self.bit(3, Operand8::Register(A)),
            0x60 /* BIT 4,B    */ => self.bit(4, Operand8::Register(B)),
            0x61 /* BIT 4,C    */ => self.bit(4, Operand8::Register(C)),
            0x62 /* BIT 4,D    */ => self.bit(4, Operand8::Register(D)),
            0x63 /* BIT 4,E    */ => self.bit(4, Operand8::Register(E)),
            0x64 /* BIT 4,H    */ => self.bit(4, Operand8::Register(H)),
            0x65 /* BIT 4,L    */ => self.bit(4, Operand8::Register(L)),
            0x66 /* BIT 4,(HL) */ => self.bit(4, Operand8::Address(HL)),
            0x67 /* BIT 4,A    */ => self.bit(4, Operand8::Register(A)),
            0x68 /* BIT 5,B    */ => self.bit(5, Operand8::Register(B)),
            0x69 /* BIT 5,C    */ => self.bit(5, Operand8::Register(C)),
            0x6A /* BIT 5,D    */ => self.bit(5, Operand8::Register(D)),
            0x6B /* BIT 5,E    */ => self.bit(5, Operand8::Register(E)),
            0x6C /* BIT 5,H    */ => self.bit(5, Operand8::Register(H)),
            0x6D /* BIT 5,L    */ => self.bit(5, Operand8::Register(L)),
            0x6E /* BIT 5,(HL) */ => self.bit(5, Operand8::Address(HL)),
            0x6F /* BIT 5,A    */ => self.bit(5, Operand8::Register(A)),
            0x70 /* BIT 6,B    */ => self.bit(6, Operand8::Register(B)),
            0x71 /* BIT 6,C    */ => self.bit(6, Operand8::Register(C)),
            0x72 /* BIT 6,D    */ => self.bit(6, Operand8::Register(D)),
            0x73 /* BIT 6,E    */ => self.bit(6, Operand8::Register(E)),
            0x74 /* BIT 6,H    */ => self.bit(6, Operand8::Register(H)),
            0x75 /* BIT 6,L    */ => self.bit(6, Operand8::Register(L)),
            0x76 /* BIT 6,(HL) */ => self.bit(6, Operand8::Address(HL)),
            0x77 /* BIT 6,A    */ => self.bit(6, Operand8::Register(A)),
            0x78 /* BIT 7,B    */ => self.bit(7, Operand8::Register(B)),
            0x79 /* BIT 7,C    */ => self.bit(7, Operand8::Register(C)),
            0x7A /* BIT 7,D    */ => self.bit(7, Operand8::Register(D)),
            0x7B /* BIT 7,E    */ => self.bit(7, Operand8::Register(E)),
            0x7C /* BIT 7,H    */ => self.bit(7, Operand8::Register(H)),
            0x7D /* BIT 7,L    */ => self.bit(7, Operand8::Register(L)),
            0x7E /* BIT 7,(HL) */ => self.bit(7, Operand8::Address(HL)),
            0x7F /* BIT 7,A    */ => self.bit(7, Operand8::Register(A)),
            0x80 /* RES 0,B    */ => self.setbit(0, Operand8::Register(B), false),
            0x81 /* RES 0,C    */ => self.setbit(0, Operand8::Register(C), false),
            0x82 /* RES 0,D    */ => self.setbit(0, Operand8::Register(D), false),
            0x83 /* RES 0,E    */ => self.setbit(0, Operand8::Register(E), false),
            0x84 /* RES 0,H    */ => self.setbit(0, Operand8::Register(H), false),
            0x85 /* RES 0,L    */ => self.setbit(0, Operand8::Register(L), false),
            0x86 /* RES 0,(HL) */ => self.setbit(0, Operand8::Address(HL), false),
            0x87 /* RES 0,A    */ => self.setbit(0, Operand8::Register(A), false),
            0x88 /* RES 1,B    */ => self.setbit(1, Operand8::Register(B), false),
            0x89 /* RES 1,C    */ => self.setbit(1, Operand8::Register(C), false),
            0x8A /* RES 1,D    */ => self.setbit(1, Operand8::Register(D), false),
            0x8B /* RES 1,E    */ => self.setbit(1, Operand8::Register(E), false),
            0x8C /* RES 1,H    */ => self.setbit(1, Operand8::Register(H), false),
            0x8D /* RES 1,L    */ => self.setbit(1, Operand8::Register(L), false),
            0x8E /* RES 1,(HL) */ => self.setbit(1, Operand8::Address(HL), false),
            0x8F /* RES 1,A    */ => self.setbit(1, Operand8::Register(A), false),
            0x90 /* RES 2,B    */ => self.setbit(2, Operand8::Register(B), false),
            0x91 /* RES 2,C    */ => self.setbit(2, Operand8::Register(C), false),
            0x92 /* RES 2,D    */ => self.setbit(2, Operand8::Register(D), false),
            0x93 /* RES 2,E    */ => self.setbit(2, Operand8::Register(E), false),
            0x94 /* RES 2,H    */ => self.setbit(2, Operand8::Register(H), false),
            0x95 /* RES 2,L    */ => self.setbit(2, Operand8::Register(L), false),
            0x96 /* RES 2,(HL) */ => self.setbit(2, Operand8::Address(HL), false),
            0x97 /* RES 2,A    */ => self.setbit(2, Operand8::Register(A), false),
            0x98 /* RES 3,B    */ => self.setbit(3, Operand8::Register(B), false),
            0x99 /* RES 3,C    */ => self.setbit(3, Operand8::Register(C), false),
            0x9A /* RES 3,D    */ => self.setbit(3, Operand8::Register(D), false),
            0x9B /* RES 3,E    */ => self.setbit(3, Operand8::Register(E), false),
            0x9C /* RES 3,H    */ => self.setbit(3, Operand8::Register(H), false),
            0x9D /* RES 3,L    */ => self.setbit(3, Operand8::Register(L), false),
            0x9E /* RES 3,(HL) */ => self.setbit(3, Operand8::Address(HL), false),
            0x9F /* RES 3,A    */ => self.setbit(3, Operand8::Register(A), false),
            0xA0 /* RES 4,B    */ => self.setbit(4, Operand8::Register(B), false),
            0xA1 /* RES 4,C    */ => self.setbit(4, Operand8::Register(C), false),
            0xA2 /* RES 4,D    */ => self.setbit(4, Operand8::Register(D), false),
            0xA3 /* RES 4,E    */ => self.setbit(4, Operand8::Register(E), false),
            0xA4 /* RES 4,H    */ => self.setbit(4, Operand8::Register(H), false),
            0xA5 /* RES 4,L    */ => self.setbit(4, Operand8::Register(L), false),
            0xA6 /* RES 4,(HL) */ => self.setbit(4, Operand8::Address(HL), false),
            0xA7 /* RES 4,A    */ => self.setbit(4, Operand8::Register(A), false),
            0xA8 /* RES 5,B    */ => self.setbit(5, Operand8::Register(B), false),
            0xA9 /* RES 5,C    */ => self.setbit(5, Operand8::Register(C), false),
            0xAA /* RES 5,D    */ => self.setbit(5, Operand8::Register(D), false),
            0xAB /* RES 5,E    */ => self.setbit(5, Operand8::Register(E), false),
            0xAC /* RES 5,H    */ => self.setbit(5, Operand8::Register(H), false),
            0xAD /* RES 5,L    */ => self.setbit(5, Operand8::Register(L), false),
            0xAE /* RES 5,(HL) */ => self.setbit(5, Operand8::Address(HL), false),
            0xAF /* RES 5,A    */ => self.setbit(5, Operand8::Register(A), false),
            0xB0 /* RES 6,B    */ => self.setbit(6, Operand8::Register(B), false),
            0xB1 /* RES 6,C    */ => self.setbit(6, Operand8::Register(C), false),
            0xB2 /* RES 6,D    */ => self.setbit(6, Operand8::Register(D), false),
            0xB3 /* RES 6,E    */ => self.setbit(6, Operand8::Register(E), false),
            0xB4 /* RES 6,H    */ => self.setbit(6, Operand8::Register(H), false),
            0xB5 /* RES 6,L    */ => self.setbit(6, Operand8::Register(L), false),
            0xB6 /* RES 6,(HL) */ => self.setbit(6, Operand8::Address(HL), false),
            0xB7 /* RES 6,A    */ => self.setbit(6, Operand8::Register(A), false),
            0xB8 /* RES 7,B    */ => self.setbit(7, Operand8::Register(B), false),
            0xB9 /* RES 7,C    */ => self.setbit(7, Operand8::Register(C), false),
            0xBA /* RES 7,D    */ => self.setbit(7, Operand8::Register(D), false),
            0xBB /* RES 7,E    */ => self.setbit(7, Operand8::Register(E), false),
            0xBC /* RES 7,H    */ => self.setbit(7, Operand8::Register(H), false),
            0xBD /* RES 7,L    */ => self.setbit(7, Operand8::Register(L), false),
            0xBE /* RES 7,(HL) */ => self.setbit(7, Operand8::Address(HL), false),
            0xBF /* RES 7,A    */ => self.setbit(7, Operand8::Register(A), false),
            0xC0 /* SET 0,B    */ => self.setbit(0, Operand8::Register(B), true),
            0xC1 /* SET 0,C    */ => self.setbit(0, Operand8::Register(C), true),
            0xC2 /* SET 0,D    */ => self.setbit(0, Operand8::Register(D), true),
            0xC3 /* SET 0,E    */ => self.setbit(0, Operand8::Register(E), true),
            0xC4 /* SET 0,H    */ => self.setbit(0, Operand8::Register(H), true),
            0xC5 /* SET 0,L    */ => self.setbit(0, Operand8::Register(L), true),
            0xC6 /* SET 0,(HL) */ => self.setbit(0, Operand8::Address(HL), true),
            0xC7 /* SET 0,A    */ => self.setbit(0, Operand8::Register(A), true),
            0xC8 /* SET 1,B    */ => self.setbit(1, Operand8::Register(B), true),
            0xC9 /* SET 1,C    */ => self.setbit(1, Operand8::Register(C), true),
            0xCA /* SET 1,D    */ => self.setbit(1, Operand8::Register(D), true),
            0xCB /* SET 1,E    */ => self.setbit(1, Operand8::Register(E), true),
            0xCC /* SET 1,H    */ => self.setbit(1, Operand8::Register(H), true),
            0xCD /* SET 1,L    */ => self.setbit(1, Operand8::Register(L), true),
            0xCE /* SET 1,(HL) */ => self.setbit(1, Operand8::Address(HL), true),
            0xCF /* SET 1,A    */ => self.setbit(1, Operand8::Register(A), true),
            0xD0 /* SET 2,B    */ => self.setbit(2, Operand8::Register(B), true),
            0xD1 /* SET 2,C    */ => self.setbit(2, Operand8::Register(C), true),
            0xD2 /* SET 2,D    */ => self.setbit(2, Operand8::Register(D), true),
            0xD3 /* SET 2,E    */ => self.setbit(2, Operand8::Register(E), true),
            0xD4 /* SET 2,H    */ => self.setbit(2, Operand8::Register(H), true),
            0xD5 /* SET 2,L    */ => self.setbit(2, Operand8::Register(L), true),
            0xD6 /* SET 2,(HL) */ => self.setbit(2, Operand8::Address(HL), true),
            0xD7 /* SET 2,A    */ => self.setbit(2, Operand8::Register(A), true),
            0xD8 /* SET 3,B    */ => self.setbit(3, Operand8::Register(B), true),
            0xD9 /* SET 3,C    */ => self.setbit(3, Operand8::Register(C), true),
            0xDA /* SET 3,D    */ => self.setbit(3, Operand8::Register(D), true),
            0xDB /* SET 3,E    */ => self.setbit(3, Operand8::Register(E), true),
            0xDC /* SET 3,H    */ => self.setbit(3, Operand8::Register(H), true),
            0xDD /* SET 3,L    */ => self.setbit(3, Operand8::Register(L), true),
            0xDE /* SET 3,(HL) */ => self.setbit(3, Operand8::Address(HL), true),
            0xDF /* SET 3,A    */ => self.setbit(3, Operand8::Register(A), true),
            0xE0 /* SET 4,B    */ => self.setbit(4, Operand8::Register(B), true),
            0xE1 /* SET 4,C    */ => self.setbit(4, Operand8::Register(C), true),
            0xE2 /* SET 4,D    */ => self.setbit(4, Operand8::Register(D), true),
            0xE3 /* SET 4,E    */ => self.setbit(4, Operand8::Register(E), true),
            0xE4 /* SET 4,H    */ => self.setbit(4, Operand8::Register(H), true),
            0xE5 /* SET 4,L    */ => self.setbit(4, Operand8::Register(L), true),
            0xE6 /* SET 4,(HL) */ => self.setbit(4, Operand8::Address(HL), true),
            0xE7 /* SET 4,A    */ => self.setbit(4, Operand8::Register(A), true),
            0xE8 /* SET 5,B    */ => self.setbit(5, Operand8::Register(B), true),
            0xE9 /* SET 5,C    */ => self.setbit(5, Operand8::Register(C), true),
            0xEA /* SET 5,D    */ => self.setbit(5, Operand8::Register(D), true),
            0xEB /* SET 5,E    */ => self.setbit(5, Operand8::Register(E), true),
            0xEC /* SET 5,H    */ => self.setbit(5, Operand8::Register(H), true),
            0xED /* SET 5,L    */ => self.setbit(5, Operand8::Register(L), true),
            0xEE /* SET 5,(HL) */ => self.setbit(5, Operand8::Address(HL), true),
            0xEF /* SET 5,A    */ => self.setbit(5, Operand8::Register(A), true),
            0xF0 /* SET 6,B    */ => self.setbit(6, Operand8::Register(B), true),
            0xF1 /* SET 6,C    */ => self.setbit(6, Operand8::Register(C), true),
            0xF2 /* SET 6,D    */ => self.setbit(6, Operand8::Register(D), true),
            0xF3 /* SET 6,E    */ => self.setbit(6, Operand8::Register(E), true),
            0xF4 /* SET 6,H    */ => self.setbit(6, Operand8::Register(H), true),
            0xF5 /* SET 6,L    */ => self.setbit(6, Operand8::Register(L), true),
            0xF6 /* SET 6,(HL) */ => self.setbit(6, Operand8::Address(HL), true),
            0xF7 /* SET 6,A    */ => self.setbit(6, Operand8::Register(A), true),
            0xF8 /* SET 7,B    */ => self.setbit(7, Operand8::Register(B), true),
            0xF9 /* SET 7,C    */ => self.setbit(7, Operand8::Register(C), true),
            0xFA /* SET 7,D    */ => self.setbit(7, Operand8::Register(D), true),
            0xFB /* SET 7,E    */ => self.setbit(7, Operand8::Register(E), true),
            0xFC /* SET 7,H    */ => self.setbit(7, Operand8::Register(H), true),
            0xFD /* SET 7,L    */ => self.setbit(7, Operand8::Register(L), true),
            0xFE /* SET 7,(HL) */ => self.setbit(7, Operand8::Address(HL), true),
            0xFF /* SET 7,A    */ => self.setbit(7, Operand8::Register(A), true),
            _ => unreachable!("All u8 values handled"),
        }
    }

    /// Process any pending interrupts. Called before the CPU fetches the next instruction to execute.
    fn process_interrupts(&mut self) {
        let next_interrupt = self.hw.next_interrupt();
        // We can bail quickly if there's no interrupts to process.
        if next_interrupt.is_none() {
            return;
        }

        // If there are interrupts to process, we clear HALT state, even if IME is disabled.
        self.halted = false;

        if !self.ime {
            // If IME isn't enabled though, we don't actually process any interrupts.
            return;
        }

        // Interrupt handling needs 3 internal cycles to do interrupt-y stuff.
        self.hw.clock();
        self.hw.clock();
        self.hw.clock();

        // Here's an interesting quirk. If the stack pointer was set to 0000 or 0001, then the push we just did
        // above would have overwritten IE. If the new IE value no longer matches the interrupt we were processing,
        // then we cancel that interrupt and set PC to 0. We then try and find another interrupt.
        // If there isn't one, we end up running code from 0000. Crazy.
        let pc = self.pc;
        let mut sp = self.sp;
        sp = sp.wrapping_sub(1);
        self.hw.mem_write8(sp, ((pc & 0xFF00) >> 8) as u8);
        // This is where we capture what IE is after pushing the upper byte. Pushing the lower byte might
        // also overwrite IE, but in that case we ignore that occurring.
        let still_pending = self.hw.next_interrupt();
        sp = sp.wrapping_sub(1);
        self.hw.mem_write8(sp, pc as u8);
        self.sp = self.sp.wrapping_sub(2);

        if next_interrupt != still_pending {
            self.pc = 0;
            // Okay so this interrupt didn't go so good. Let's see if there's another one.
            self.process_interrupts();
            // Regardless of what happens in the next try, IME needs to be disabled.
            self.ime = false;
            return;
        }

        let intr = next_interrupt.unwrap();
        self.pc = intr.handler_addr();
        self.hw.clear_interrupt(intr);
        self.ime = false;
    }

    /// Fetches next byte from PC location, and bumps PC.
    fn fetch8(&mut self) -> u8 {
        let addr = self.pc;
        self.pc = self.pc.wrapping_add(1);
        self.hw.mem_read8(addr)
    }

    /// Fetches next word from PC location, and bumps PC.
    fn fetch16(&mut self) -> u16 {
        let addr = self.pc;
        self.pc = self.pc.wrapping_add(2);
        self.hw.mem_read16(addr)
    }

    /// Returns the current value of an 8-bit CPU register.
    fn register8_get(&self, r: Register8) -> u8 {
        match r {
            Register8::A => self.a,
            Register8::B => self.b,
            Register8::C => self.c,
            Register8::D => self.d,
            Register8::E => self.e,
            Register8::H => self.h,
            Register8::L => self.l,
        }
    }

    /// Sets a new value for an 8-bit CPU register.
    fn register8_set(&mut self, r: Register8, v: u8) {
        match r {
            Register8::A => self.a = v,
            Register8::B => self.b = v,
            Register8::C => self.c = v,
            Register8::D => self.d = v,
            Register8::E => self.e = v,
            Register8::H => self.h = v,
            Register8::L => self.l = v,
        }
    }

    /// Returns the current value of a 16-bit CPU register.
    fn register16_get(&self, reg: Register16) -> u16 {
        let (hi, lo) = match reg {
            AF => { (self.a, self.f.pack()) },
            BC => { (self.b, self.c) },
            DE => { (self.d, self.e) },
            HL => { (self.h, self.l) },
            SP => { return self.sp },
        };

        (hi as u16) << 8 | (lo as u16)
    }

    /// Sets a new value for a 16-bit CPU register.
    fn register16_set(&mut self, reg: Register16, v: u16) {
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

    /// Resolves the value for a given 8-bit instruction operand.
    fn operand8_get(&mut self, o: Operand8) -> u8 {
        match o {
            Operand8::Register(r) => self.register8_get(r),
            Operand8::Immediate(d) => d,
            Operand8::Address(rr) => { let addr = self.register16_get(rr); self.hw.mem_read8(addr) },
            Operand8::AddressInc(rr) => {
                let addr = self.register16_get(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    // TODO: this potentially triggers OAM corruption bug.
                }
                self.register16_set(rr, addr.wrapping_add(1));
                self.hw.mem_read8(addr)
            },
            Operand8::AddressDec(rr) => {
                let addr = self.register16_get(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    // TODO: this potentially triggers OAM corruption bug.
                }
                self.register16_set(rr, addr.wrapping_sub(1));
                self.hw.mem_read8(addr)
            },
            Operand8::ImmediateAddress(addr) => self.hw.mem_read8(addr),
            Operand8::ImmediateAddressHigh(addr) => self.hw.mem_read8(0xFF00 + (addr as u16)),
            Operand8::AddressHigh(r) => { let addr = 0xFF00 + (self.register8_get(r) as u16); self.hw.mem_read8(addr) },
        }
    }

    /// Writes a new value to the target of an 8-bit instruction operand.
    fn operand8_set(&mut self, o: Operand8, v: u8) -> u8 {
        match o {
            Operand8::Register(r) => self.register8_set(r, v),
            Operand8::Immediate(_) => { panic!("Attempted to write to immediate operand") },
            Operand8::Address(rr) => {
                let addr = self.register16_get(rr);
                self.hw.mem_write8(addr, v)
            },
            Operand8::AddressInc(rr) => {
                let addr = self.register16_get(rr);
                self.register16_set(rr, addr.wrapping_add(1));
                self.hw.mem_write8(addr, v)
            },
            Operand8::AddressDec(rr) => {
                let addr = self.register16_get(rr);
                self.register16_set(rr, addr.wrapping_sub(1));
                self.hw.mem_write8(addr, v)
            },
            Operand8::ImmediateAddress(addr) => self.hw.mem_write8(addr, v),
            Operand8::ImmediateAddressHigh(addr) => self.hw.mem_write8(0xFF00 + (addr as u16), v),
            Operand8::AddressHigh(r) => { let addr = self.register8_get(r); self.hw.mem_write8(0xFF00 + (addr as u16), v) },
        };
        v
    }

    /// Resolves the value for a given 16-bit instruction operand.
    fn operand16_get(&mut self, o: Operand16) -> u16 {
        match o {
            Operand16::Register(r) => self.register16_get(r),
            Operand16::Immediate(d) => d,
            Operand16::ImmediateAddress(addr) => self.hw.mem_read16(addr),
        }
    }

    /// Writes a new value to the target of a 16-bit instruction operand.
    fn operand16_set(&mut self, o: Operand16, v: u16) {
        match o {
            Operand16::Register(r) => self.register16_set(r, v),
            Operand16::Immediate(_) => panic!("Attempted to write to immediate operand"),
            Operand16::ImmediateAddress(addr) => self.hw.mem_write16(addr, v),
        }
    }

    fn ld8(&mut self, lhs: Operand8, rhs: Operand8) {
        let v = self.operand8_get(rhs);
        self.operand8_set(lhs, v);
    }

    fn ld16(&mut self, lhs: Operand16, rhs: Operand16, extra_clock: bool) {
        let v = self.operand16_get(rhs);
        self.operand16_set(lhs, v);

        // In the specific case of loading a 16bit reg into another 16bit reg, this consumes another CPU cycle. I don't
        // really understand why, since in every other case the cost of reading/writing a 16bit reg appears to be free.
        if extra_clock {
            self.hw.clock();
        }
    }

    fn ld_hl_sp(&mut self, d: i8) {
        let sp = self.sp;
        let d = d as i16 as u16;
        let v = sp.wrapping_add(d);
        self.register16_set(HL, v);
        self.f.reset();
        self.f.h = (sp & 0xF) + (d & 0xF) & 0x10 > 0;
        self.f.c = (sp & 0xFF) + (d & 0xFF) & 0x100 > 0;
        self.hw.clock();
    }

    fn inc8(&mut self, o: Operand8) {
        let v = self.operand8_get(o).wrapping_add(1);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = v & 0x0F == 0;
        self.operand8_set(o, v);
    }

    fn dec8(&mut self, o: Operand8) {
        let v = self.operand8_get(o).wrapping_sub(1);
        self.f.z = v == 0;
        self.f.n = true;
        self.f.h = v & 0x0F == 0x0F;
        self.operand8_set(o, v);
    }

    fn dec16(&mut self, r: Register16) {
        let v = self.register16_get(r);
        if v >= 0xFE00 && v <= 0xFEFF {
            // TODO: this potentially triggers OAM corruption bug.
        }
        self.register16_set(r, v.wrapping_sub(1));
        self.hw.clock();
    }

    fn inc16(&mut self, reg: Register16) {
        let v = self.register16_get(reg);
        if v >= 0xFE00 && v <= 0xFEFF {
            // TODO: this potentially triggers OAM corruption bug.
        }
        self.register16_set(reg, v.wrapping_add(1));
        self.hw.clock();
    }

    fn add8(&mut self, o: Operand8, carry: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let old = self.a;
        let v = self.operand8_get(o);
        let new = old.wrapping_add(v).wrapping_add(carry);
        self.a = new;
        self.f.z = new == 0;
        self.f.n = false;
        self.f.h = (((old & 0xF) + (v & 0xF)) + carry) & 0x10 == 0x10;
        self.f.c = (old as u16) + (v as u16) + (carry as u16) > 0xFF;
    }

    fn add16(&mut self, r: Register16) {
        let hl = self.register16_get(HL);
        let v = self.register16_get(r);
        let (new_hl, overflow) = hl.overflowing_add(v);
        self.register16_set(HL, new_hl);

        self.hw.clock();

        self.f.n = false;
        self.f.h = ((hl & 0xFFF) + (v & 0xFFF)) & 0x1000 > 0;
        self.f.c = overflow;
    }

    fn add_sp_r8(&mut self, d: i8) {
        let d = d as i16 as u16;
        let sp = self.sp;

        self.sp = sp.wrapping_add(d as i16 as u16);

        self.hw.clock();
        self.hw.clock();

        self.f.z = false;
        self.f.n = false;
        self.f.h = ((sp & 0xF) + (d & 0xF)) & 0x10 > 0;
        self.f.c = ((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0;
    }


    fn sub8(&mut self, o: Operand8, carry: bool, store: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let a = self.a;
        let v = self.operand8_get(o);
        let new_a = a.wrapping_sub(v).wrapping_sub(carry);
        if store {
            self.a = new_a;
        }

        self.f.z = new_a == 0;
        self.f.n = true;
        self.f.h = ((a & 0xF) as u16) < ((v & 0xF) as u16) + (carry as u16);
        self.f.c = (a as u16) < (v as u16) + (carry as u16);
    }

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

    fn cpl(&mut self) {
        self.a = !self.a;
        self.f.n = true;
        self.f.h = true;
    }

    fn ccf(&mut self) {
        self.f.n = false;
        self.f.h = false;
        self.f.c = !self.f.c;
    }

    fn scf(&mut self) {
        self.f.n = false;
        self.f.h = false;
        self.f.c = true;
    }

    fn bitwise(&mut self, op: BitwiseOp, o: Operand8) {
        let a = self.a;
        let v = self.operand8_get(o);
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

    fn halt(&mut self) {
        // TODO: pretty sure I need to check interrupt states here.
        self.halted = true;
    }

    fn stop(&mut self) {
        // TODO: this should be more than a noop.
        self.pc = self.pc.wrapping_add(1);
    }

    fn rl(&mut self, o: Operand8, set_zero: bool, preserve_lsb: bool) {
        let v = self.operand8_get(o);
        let lsb = if preserve_lsb && self.f.c { 1 } else { 0 };
        let carry = v & 0x80 > 0;
        let v = self.operand8_set(o, v << 1 | lsb);
        self.f.reset();
        self.f.z = set_zero && v == 0;
        self.f.c = carry;
    }

    fn rlc(&mut self, o: Operand8, extended: bool) {
        let v = self.operand8_get(o);
        let carry = v & 0x80 > 0;
        let lsb = if carry { 1 } else { 0 };
        let v = self.operand8_set(o, v << 1 | lsb);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn rr(&mut self, o: Operand8, extended: bool) {
        let v = self.operand8_get(o);
        let msb = if self.f.c { 0x80 } else { 0 };
        let carry = v & 0x1 > 0;
        let v = self.operand8_set(o, v >> 1 | msb);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }


    fn rrc(&mut self, o: Operand8, extended: bool) {
        let v = self.operand8_get(o);
        let carry = v & 0x1 > 0;
        let msb = if carry { 0x80 } else { 0 };
        let v = self.operand8_set(o, v >> 1 | msb);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn shift_right(&mut self, o: Operand8, preserve_msb: bool) {
        let v = self.operand8_get(o);
        let carry = v & 0x01 > 0;
        let preserve = if preserve_msb { v & 0x80 } else { 0 };
        let v = self.operand8_set(o, v >> 1 | preserve);
        self.f.reset();
        self.f.z = v == 0;
        self.f.c = carry;
    }

    fn swap(&mut self, o: Operand8) {
        let v = self.operand8_get(o);
        let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
        self.operand8_set(o, v);
        self.f.reset();
        self.f.z = v == 0;
    }


    fn bit(&mut self, b: u8, o: Operand8) {
        let v = self.operand8_get(o) & (1 << b);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = true;
    }

    fn setbit(&mut self, b: u8, o: Operand8, on: bool) {
        let v = self.operand8_get(o);
        self.operand8_set(o, if on { v | 1 << b } else { v & !(1 << b) });
    }

    fn stack_push(&mut self, v: u16) {
        if self.sp >= 0xFE00 && self.sp <= 0xFEFF {
            // TODO: this potentially triggers OAM corruption bug.
        }

        self.sp = self.sp.wrapping_sub(2);
        let sp = self.sp;
        self.hw.mem_write16(sp, v);
    }

    fn stack_pop(&mut self) -> u16 {
        if self.sp >= 0xFDFF && self.sp <= 0xFEFE {
            // TODO: this potentially triggers OAM corruption bug.
        }

        let addr = self.sp;
        let v = self.hw.mem_read16(addr);
        self.sp = self.sp.wrapping_add(2);

        v
    }

    fn push(&mut self, r: Register16) {
        let v = self.register16_get(r);
        self.hw.clock();
        self.stack_push(v);
    }

    fn pop(&mut self, r: Register16) {
        let mut v = self.stack_pop();
        if let AF = r {
            // Reset bits 0-3 in F.
            v &= 0xFFF0;
        }
        self.register16_set(r, v);
    }

    fn push_and_jump(&mut self, addr: u16) {
        let pc = self.pc;
        self.stack_push(pc);
        self.pc = addr;
    }

    fn call(&mut self, cc: Option<FlagCondition>, addr: u16) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }
        self.hw.clock();
        self.push_and_jump(addr);
    }

    fn jp(&mut self, cc: Option<FlagCondition>, o: Operand16) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }

        let addr = self.operand16_get(o);
        self.pc = addr;

        if let Operand16::Register(HL) = o {
            // For some reason, JP (HL) doesn't cause an extra clock cycle. Very mysterious.
        } else {
            self.hw.clock();
        }
    }

    fn jr(&mut self, cc: Option<FlagCondition>, n: u8) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }
        self.hw.clock();
        self.pc = self.pc.wrapping_add(n as i8 as i16 as u16);
    }

    fn ret(&mut self, cc: Option<FlagCondition>, ei: bool) {
        if !self.f.check_jmp_condition(cc) {
            self.hw.clock();
            return;
        }
        if cc.is_some() {
            self.hw.clock();
        }
        if ei {
            // RETI immediately enables IME, it's not deferred like eith an EI or DI call.
            self.ime = true;
        }
        let pc = self.stack_pop();
        self.hw.clock();
        self.pc = pc;
    }

    fn rst(&mut self, a: u8) {
        self.hw.clock();
        self.push_and_jump(a as u16);
    }

    pub fn set_ime(&mut self, v: bool) {
        if v {
            // Enabling interrupts happens on the next cycle ...
            self.ime_defer = true;
        } else {
            // ... However, disabling interrupts is immediate.
            self.ime = false;
        }
    }
}
