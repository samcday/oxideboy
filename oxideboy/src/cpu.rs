//! The Gameboy and all its variants have the same CPU at their heart: an 8-bit Z80-like processor produced by Sharp and
//! codenamed LR25902. This processor has 8 8-bit registers (1 is reserved for ALU flags) and a 16-bit memory bus.
//! The CPU is mostly 8-bit, but does sport a handful of 16-bit instructions. For example, pairs of the 8-bit registers
//! can be viewed and manipulated.
//! This module contains everything needed to decode and execute instructions for this CPU.

// TODO: remove me when refactor is complete.
#![allow(dead_code)]

use crate::EventListener;
use crate::GameboyHardware;

/// The main Cpu struct, containing all the CPU registers and core CPU state. Many of the CPU instructions modify the
/// registers contained here. Anything else is modified by reading/writing from the external memory bus.
pub struct Cpu {
    // CPU registers
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub f: Flags,
    pub h: u8,
    pub l: u8,
    pub pc: u16,
    pub sp: u16,

    pub ime: bool,
    pub ime_defer: bool,
    pub halted: bool,
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

/// The 8 available 8-bit CPU registers. The F register is not included here because no instructions reference it
/// directly, it's only modified / queried indirectly.
#[derive(Clone, Copy, Debug)]
#[rustfmt::skip]
pub enum Register { A, B, C, D, E, H, L }
use Register::*;

/// The 5 available 16-bit CPU registers. With the exception of HL and SP, these registers are just pairs of
/// the 8-bit ones viewed as 16 bits.
#[derive(Clone, Copy, Debug)]
#[rustfmt::skip]
pub enum Register16 { AF, BC, DE, HL, SP }
use Register16::*;

/// The 4 conditions available to branching instructions (CALL/JP/JR/RET).
#[derive(Clone, Copy, Debug)]
pub enum FlagCondition {
    NZ, // CPU flag Z is clear
    Z,  // CPU flag Z is set
    NC, // CPU flag C is clear
    C,  // CPU flag C is set
}

/// An enum of the available instructions for the Gameboy CPU. Many instructions do the same thing but just on different
/// operands. i.e LD A, B and LD A, C are similar - the former copies the B register into A, and the latter copies C.
/// We don't specialize these two different instructions, instead we have a single "LD" enum variant that accepts the
/// appropriate operand.
#[allow(non_camel_case_types)]
#[derive(Debug)]
pub enum Instruction {
    ADC(Operand),
    ADD(Operand),
    ADD16(Register16),
    ADD_SP_r8(i8),
    AND(Operand),
    BIT(u8, Operand),
    CALL(Option<FlagCondition>, u16),
    CCF,
    CP(Operand),
    CPL,
    DAA,
    DEC(Operand),
    DEC16(Register16),
    DI,
    EI,
    HALT,
    INC(Operand),
    INC16(Register16),
    JP(Option<FlagCondition>, Operand16),
    JR(Option<FlagCondition>, u8),
    LD(Operand, Operand),
    LD16(Operand16, Operand16),
    LD_HL_SP(i8),
    NOP,
    OR(Operand),
    POP(Register16),
    PUSH(Register16),
    RES(u8, Operand),
    RET(Option<FlagCondition>),
    RETI,
    RL(Operand),
    RLA,
    RLC(Operand),
    RLCA,
    RR(Operand),
    RRA,
    RRC(Operand),
    RRCA,
    RST(u8),
    SBC(Operand),
    SCF,
    SET(u8, Operand),
    SLA(Operand),
    SRA(Operand),
    SRL(Operand),
    STOP,
    SUB(Operand),
    SWAP(Operand),
    XOR(Operand),
    Invalid(u8),
}
use Instruction::*;

#[derive(Clone, Copy, Debug)]
pub enum Operand {
    Register(Register),       // Get/set an 8-bit register.
    Immediate(u8),            // An immediate 8-bit value embedded alongside the opcode.
    Address(Register16),      // Value at memory address specified by 16-bit register.
    AddressInc(Register16),   // Memory address pointed to by a 16-bit register. Increment register after use.
    AddressDec(Register16),   // Memory address pointed to by a 16-bit register. Decrement register after use.
    ImmediateAddress(u16),    // Immediate 16-bit value interpreted as memory address.
    ImmediateAddressHigh(u8), // Immediate 8-bit value interpreted as memory address offset from $FF00.
    AddressHigh(Register),    // 8-bit register value interpreted as memory address offset from $FF00.
}

#[derive(Clone, Copy, Debug)]
pub enum Operand16 {
    Register(Register16),  // Get/set a 16-bit register.
    Immediate(u16),        // An immediate 16-bit value embedded alongside the opcode.
    ImmediateAddress(u16), // Immediate 16-bit value interpreted as memory address.
}

enum BitwiseOp {
    XOR,
    OR,
    AND,
}

impl Cpu {
    pub fn new() -> Cpu {
        Cpu {
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            f: Default::default(),
            h: 0,
            l: 0,
            pc: 0,
            sp: 0,
            ime: false,
            ime_defer: false,
            halted: false,
        }
    }

    /// Runs the CPU for a single fetch-decode-execute step. The actual number of cycles this will take depends on which
    /// instruction is executed.
    pub fn step<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>) {
        // If the CPU is currently halted, we need to pump a clock cycle of the other hardware, so that if there's a new
        // interrupt available, we can wake up from HALT and continue on.
        if self.halted {
            hw.clock();
        }

        self.process_interrupts(hw);

        // Apply deferred change to IME register.
        if self.ime_defer {
            self.ime = true;
            self.ime_defer = false;
        }

        if self.halted {
            // CPU is halted and there wasn't any pending interrupts to wake us up, we're done for now.
            return;
        }

        // Decode the next instruction from memory location pointed to by PC.
        let instruction = decode_instruction(|| {
            let v = hw.mem_read(self.pc);
            self.pc = self.pc.wrapping_add(1);
            v
        });

        match instruction {
            ADC(o) => self.add(hw, o, true),
            ADD(o) => self.add(hw, o, false),
            ADD16(rr) => self.add16(hw, rr),
            ADD_SP_r8(r8) => self.add_sp_r8(hw, r8),
            AND(o) => self.bitwise(hw, BitwiseOp::AND, o),
            BIT(b, o) => self.bit(hw, b, o),
            CALL(cc, addr) => self.call(hw, cc, addr),
            CCF => self.ccf(),
            CP(o) => self.sub(hw, o, false, false),
            CPL => self.cpl(),
            DAA => self.daa(),
            DEC(o) => self.dec(hw, o),
            DEC16(rr) => self.dec16(hw, rr),
            DI => self.set_ime(false),
            EI => self.set_ime(true),
            HALT => self.halt(),
            INC(o) => self.inc(hw, o),
            INC16(rr) => self.inc16(hw, rr),
            JP(cc, o) => self.jp(hw, cc, o),
            JR(cc, r8) => self.jr(hw, cc, r8),
            LD(lhs @ Operand::Register(B), rhs @ Operand::Register(B)) => {
                hw.listener.on_debug_breakpoint();
                self.ld(hw, lhs, rhs);
            }
            LD(lhs, rhs) => self.ld(hw, lhs, rhs),
            LD16(lhs @ Operand16::Register(SP), rhs @ Operand16::Register(HL)) => self.ld16(hw, lhs, rhs, true),
            LD16(lhs, rhs) => self.ld16(hw, lhs, rhs, false),
            LD_HL_SP(d) => self.ld_hl_sp(hw, d),
            NOP => {}
            OR(o) => self.bitwise(hw, BitwiseOp::OR, o),
            POP(rr) => self.pop(hw, rr),
            PUSH(rr) => self.push(hw, rr),
            RES(b, o) => self.setbit(hw, b, o, false),
            RET(cc) => self.ret(hw, cc, false),
            RETI => self.ret(hw, None, true),
            RL(o) => self.rl(hw, o, true, true),
            RLA => self.rl(hw, Operand::Register(A), false, true),
            RLC(o) => self.rlc(hw, o, true),
            RLCA => self.rlc(hw, Operand::Register(A), false),
            RR(o) => self.rr(hw, o, true),
            RRA => self.rr(hw, Operand::Register(A), false),
            RRC(o) => self.rrc(hw, o, true),
            RRCA => self.rrc(hw, Operand::Register(A), false),
            RST(vec) => self.rst(hw, vec),
            SBC(o) => self.sub(hw, o, true, true),
            SCF => self.scf(),
            SET(b, o) => self.setbit(hw, b, o, true),
            SLA(o) => self.rl(hw, o, true, false),
            SRA(o) => self.shift_right(hw, o, true),
            SRL(o) => self.shift_right(hw, o, false),
            STOP => self.stop(),
            SUB(o) => self.sub(hw, o, false, true),
            SWAP(o) => self.swap(hw, o),
            XOR(o) => self.bitwise(hw, BitwiseOp::XOR, o),
            Invalid(_) => panic!("Unhandled instruction: {:?}", instruction),
        }
    }

    /// Process any pending interrupts. Called before the CPU fetches the next instruction to execute.
    fn process_interrupts<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>) {
        // We can bail quickly if there's no interrupts to process.
        if !hw.interrupts.pending {
            return;
        }

        let interrupt = hw.interrupts.next_interrupt();

        // If there are interrupts to process, we clear HALT state, even if IME is disabled.
        self.halted = false;

        if !self.ime {
            // If IME isn't enabled though, we don't actually process any interrupts.
            return;
        }

        // Interrupt handling needs 3 internal cycles to do interrupt-y stuff.
        hw.clock();
        hw.clock();
        hw.clock();

        // Here's an interesting quirk. If the stack pointer was set to 0000 or 0001, then the push we just did
        // above would have overwritten IE. If the new IE value no longer matches the interrupt we were processing,
        // then we cancel that interrupt and set PC to 0. We then try and find another interrupt.
        // If there isn't one, we end up running code from 0000. Crazy.
        let pc = self.pc;
        let mut sp = self.sp;
        sp = sp.wrapping_sub(1);
        hw.mem_write(sp, ((pc & 0xFF00) >> 8) as u8);
        // This is where we capture what IE is after pushing the upper byte. Pushing the lower byte might
        // also overwrite IE, but in that case we ignore that occurring.
        let still_pending = hw.interrupts.pending && hw.interrupts.next_interrupt() == interrupt;
        sp = sp.wrapping_sub(1);
        hw.mem_write(sp, pc as u8);
        self.sp = self.sp.wrapping_sub(2);

        if !still_pending {
            self.pc = 0;
            // Okay so this interrupt didn't go so good. Let's see if there's another one.
            self.process_interrupts(hw);
            // Regardless of what happens in the next try, IME needs to be disabled.
            self.ime = false;
            return;
        }

        self.pc = interrupt.handler_addr();
        hw.interrupts.clear(interrupt);
        self.ime = false;
    }

    /// Returns the current value of an 8-bit CPU register.
    fn register_get(&self, r: Register) -> u8 {
        match r {
            Register::A => self.a,
            Register::B => self.b,
            Register::C => self.c,
            Register::D => self.d,
            Register::E => self.e,
            Register::H => self.h,
            Register::L => self.l,
        }
    }

    /// Sets a new value for an 8-bit CPU register.
    fn register_set(&mut self, r: Register, v: u8) {
        match r {
            Register::A => self.a = v,
            Register::B => self.b = v,
            Register::C => self.c = v,
            Register::D => self.d = v,
            Register::E => self.e = v,
            Register::H => self.h = v,
            Register::L => self.l = v,
        }
    }

    /// Returns the current value of a 16-bit CPU register.
    pub fn register16_get(&self, reg: Register16) -> u16 {
        let (hi, lo) = match reg {
            AF => (self.a, self.f.pack()),
            BC => (self.b, self.c),
            DE => (self.d, self.e),
            HL => (self.h, self.l),
            SP => return self.sp,
        };

        (hi as u16) << 8 | (lo as u16)
    }

    /// Sets a new value for a 16-bit CPU register.
    pub fn register16_set(&mut self, reg: Register16, v: u16) {
        let (hi, lo) = match reg {
            AF => {
                self.a = ((v & 0xFF00) >> 8) as u8;
                self.f.unpack(v as u8);
                return;
            }
            BC => (&mut self.b, &mut self.c),
            DE => (&mut self.d, &mut self.e),
            HL => (&mut self.h, &mut self.l),
            SP => {
                self.sp = v;
                return;
            }
        };

        *hi = ((v & 0xFF00) >> 8) as u8;
        *lo = v as u8;
    }

    /// Resolves this Operand into a concrete 8-bit value.
    pub fn operand_get<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand) -> u8 {
        match o {
            Operand::Register(r) => self.register_get(r),
            Operand::Immediate(d) => d,
            Operand::Address(rr) => {
                let addr = self.register16_get(rr);
                hw.mem_read(addr)
            }
            Operand::AddressInc(rr) => {
                let addr = self.register16_get(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    // TODO: this potentially triggers OAM corruption bug.
                }
                self.register16_set(rr, addr.wrapping_add(1));
                hw.mem_read(addr)
            }
            Operand::AddressDec(rr) => {
                let addr = self.register16_get(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    // TODO: this potentially triggers OAM corruption bug.
                }
                self.register16_set(rr, addr.wrapping_sub(1));
                hw.mem_read(addr)
            }
            Operand::ImmediateAddress(addr) => hw.mem_read(addr),
            Operand::ImmediateAddressHigh(addr) => hw.mem_read(0xFF00 + (addr as u16)),
            Operand::AddressHigh(r) => {
                let addr = 0xFF00 + (self.register_get(r) as u16);
                hw.mem_read(addr)
            }
        }
    }

    /// Saves provided 8-bit value into Operand destination.
    pub fn operand_set<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, v: u8) {
        match o {
            Operand::Register(r) => self.register_set(r, v),
            Operand::Immediate(_) => panic!("Attempted to write to immediate operand"),
            Operand::Address(rr) => {
                let addr = self.register16_get(rr);
                hw.mem_write(addr, v)
            }
            Operand::AddressInc(rr) => {
                let addr = self.register16_get(rr);
                self.register16_set(rr, addr.wrapping_add(1));
                hw.mem_write(addr, v)
            }
            Operand::AddressDec(rr) => {
                let addr = self.register16_get(rr);
                self.register16_set(rr, addr.wrapping_sub(1));
                hw.mem_write(addr, v)
            }
            Operand::ImmediateAddress(addr) => hw.mem_write(addr, v),
            Operand::ImmediateAddressHigh(addr) => hw.mem_write(0xFF00 + (addr as u16), v),
            Operand::AddressHigh(r) => {
                let addr = self.register_get(r);
                hw.mem_write(0xFF00 + (addr as u16), v)
            }
        }
    }

    /// Resolves the value for a given 16-bit instruction operand.
    fn operand16_get<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand16) -> u16 {
        match o {
            Operand16::Register(r) => self.register16_get(r),
            Operand16::Immediate(d) => d,
            Operand16::ImmediateAddress(addr) => hw.mem_read16(addr),
        }
    }

    /// Writes a new value to the target of a 16-bit instruction operand.
    fn operand16_set<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand16, v: u16) {
        match o {
            Operand16::Register(r) => self.register16_set(r, v),
            Operand16::Immediate(_) => panic!("Attempted to write to immediate operand"),
            Operand16::ImmediateAddress(addr) => hw.mem_write16(addr, v),
        }
    }

    fn halt(&mut self) {
        // TODO: pretty sure I need to check interrupt states here.
        self.halted = true;
    }

    fn stop(&mut self) {
        // TODO: this should be more than a noop.
        self.pc = self.pc.wrapping_add(1);
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

    fn inc<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand) {
        let v = self.operand_get(hw, o).wrapping_add(1);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = v & 0x0F == 0;
        self.operand_set(hw, o, v);
    }

    fn inc16<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, reg: Register16) {
        let v = self.register16_get(reg);
        if v >= 0xFE00 && v <= 0xFEFF {
            // TODO: this potentially triggers OAM corruption bug.
        }
        self.register16_set(reg, v.wrapping_add(1));
        hw.clock();
    }

    fn dec<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand) {
        let v = self.operand_get(hw, o).wrapping_sub(1);
        self.f.z = v == 0;
        self.f.n = true;
        self.f.h = v & 0x0F == 0x0F;
        self.operand_set(hw, o, v);
    }

    fn dec16<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, r: Register16) {
        let v = self.register16_get(r);
        if v >= 0xFE00 && v <= 0xFEFF {
            // TODO: this potentially triggers OAM corruption bug.
        }
        self.register16_set(r, v.wrapping_sub(1));
        hw.clock();
    }

    fn add<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, carry: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let old = self.a;
        let v = self.operand_get(hw, o);
        let new = old.wrapping_add(v).wrapping_add(carry);
        self.a = new;
        self.f.z = new == 0;
        self.f.n = false;
        self.f.h = (((old & 0xF) + (v & 0xF)) + carry) & 0x10 == 0x10;
        self.f.c = (old as u16) + (v as u16) + (carry as u16) > 0xFF;
    }

    fn add16<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, r: Register16) {
        let hl = self.register16_get(HL);
        let v = self.register16_get(r);
        let (new_hl, overflow) = hl.overflowing_add(v);
        self.register16_set(HL, new_hl);

        hw.clock();

        self.f.n = false;
        self.f.h = ((hl & 0xFFF) + (v & 0xFFF)) & 0x1000 > 0;
        self.f.c = overflow;
    }

    fn add_sp_r8<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, d: i8) {
        let d = d as i16 as u16;
        let sp = self.sp;

        self.sp = sp.wrapping_add(d as i16 as u16);

        hw.clock();
        hw.clock();

        self.f.z = false;
        self.f.n = false;
        self.f.h = ((sp & 0xF) + (d & 0xF)) & 0x10 > 0;
        self.f.c = ((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0;
    }

    fn sub<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, carry: bool, store: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let a = self.a;
        let v = self.operand_get(hw, o);
        let new_a = a.wrapping_sub(v).wrapping_sub(carry);
        if store {
            self.a = new_a;
        }

        self.f.z = new_a == 0;
        self.f.n = true;
        self.f.h = ((a & 0xF) as u16) < ((v & 0xF) as u16) + (carry as u16);
        self.f.c = (a as u16) < (v as u16) + (carry as u16);
    }

    fn ld<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, lhs: Operand, rhs: Operand) {
        let v = self.operand_get(hw, rhs);
        self.operand_set(hw, lhs, v);
    }

    fn ld16<T: EventListener>(
        &mut self,
        hw: &mut GameboyHardware<T>,
        lhs: Operand16,
        rhs: Operand16,
        extra_clock: bool,
    ) {
        let v = self.operand16_get(hw, rhs);
        self.operand16_set(hw, lhs, v);

        // In the specific case of loading a 16bit reg into another 16bit reg, this consumes another CPU cycle. I don't
        // really understand why, since in every other case the cost of reading/writing a 16bit reg appears to be free.
        if extra_clock {
            hw.clock();
        }
    }

    fn ld_hl_sp<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, d: i8) {
        let sp = self.sp;
        let d = d as i16 as u16;
        let v = sp.wrapping_add(d);
        self.register16_set(HL, v);
        self.f.reset();
        self.f.h = (sp & 0xF) + (d & 0xF) & 0x10 > 0;
        self.f.c = (sp & 0xFF) + (d & 0xFF) & 0x100 > 0;
        hw.clock();
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

    fn bitwise<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, op: BitwiseOp, o: Operand) {
        let a = self.a;
        let v = self.operand_get(hw, o);
        let mut hc = false;
        self.a = match op {
            BitwiseOp::AND => {
                hc = true;
                a & v
            }
            BitwiseOp::OR => a | v,
            BitwiseOp::XOR => a ^ v,
        };

        self.f.reset();
        self.f.z = self.a == 0;
        self.f.h = hc;
    }

    fn bit<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, b: u8, o: Operand) {
        let v = self.operand_get(hw, o) & (1 << b);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = true;
    }

    fn setbit<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, b: u8, o: Operand, on: bool) {
        let v = self.operand_get(hw, o);
        self.operand_set(hw, o, if on { v | 1 << b } else { v & !(1 << b) });
    }

    fn rl<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, set_zero: bool, preserve_lsb: bool) {
        let v = self.operand_get(hw, o);
        let lsb = if preserve_lsb && self.f.c { 1 } else { 0 };
        let carry = v & 0x80 > 0;
        let v = v << 1 | lsb;
        self.operand_set(hw, o, v);
        self.f.reset();
        self.f.z = set_zero && v == 0;
        self.f.c = carry;
    }

    fn rlc<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, extended: bool) {
        let v = self.operand_get(hw, o);
        let carry = v & 0x80 > 0;
        let lsb = if carry { 1 } else { 0 };
        let v = v << 1 | lsb;
        self.operand_set(hw, o, v);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn rr<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, extended: bool) {
        let v = self.operand_get(hw, o);
        let msb = if self.f.c { 0x80 } else { 0 };
        let carry = v & 0x1 > 0;
        let v = v >> 1 | msb;
        self.operand_set(hw, o, v);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn rrc<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, extended: bool) {
        let v = self.operand_get(hw, o);
        let carry = v & 0x1 > 0;
        let msb = if carry { 0x80 } else { 0 };
        let v = v >> 1 | msb;
        self.operand_set(hw, o, v);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn shift_right<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand, preserve_msb: bool) {
        let v = self.operand_get(hw, o);
        let carry = v & 0x01 > 0;
        let preserve = if preserve_msb { v & 0x80 } else { 0 };
        let v = v >> 1 | preserve;
        self.operand_set(hw, o, v);
        self.f.reset();
        self.f.z = v == 0;
        self.f.c = carry;
    }

    fn swap<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, o: Operand) {
        let v = self.operand_get(hw, o);
        let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
        self.operand_set(hw, o, v);
        self.f.reset();
        self.f.z = v == 0;
    }

    fn stack_push<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, v: u16) {
        if self.sp >= 0xFE00 && self.sp <= 0xFEFF {
            // TODO: this potentially triggers OAM corruption bug.
        }

        self.sp = self.sp.wrapping_sub(2);
        let sp = self.sp;
        hw.mem_write16(sp, v);
    }

    fn stack_pop<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>) -> u16 {
        if self.sp >= 0xFDFF && self.sp <= 0xFEFE {
            // TODO: this potentially triggers OAM corruption bug.
        }

        let addr = self.sp;
        let v = hw.mem_read16(addr);
        self.sp = self.sp.wrapping_add(2);

        v
    }

    fn push<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, r: Register16) {
        let v = self.register16_get(r);
        hw.clock();
        self.stack_push(hw, v);
    }

    fn pop<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, r: Register16) {
        let mut v = self.stack_pop(hw);
        if let AF = r {
            // Reset bits 0-3 in F.
            v &= 0xFFF0;
        }
        self.register16_set(r, v);
    }

    fn push_and_jump<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, addr: u16) {
        let pc = self.pc;
        self.stack_push(hw, pc);
        self.pc = addr;
    }

    fn call<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, cc: Option<FlagCondition>, addr: u16) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }
        hw.clock();
        self.push_and_jump(hw, addr);
    }

    fn jp<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, cc: Option<FlagCondition>, o: Operand16) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }

        let addr = self.operand16_get(hw, o);
        self.pc = addr;

        if let Operand16::Register(HL) = o {
            // For some reason, JP (HL) doesn't cause an extra clock cycle. Very mysterious.
        } else {
            hw.clock();
        }
    }

    fn jr<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, cc: Option<FlagCondition>, n: u8) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }
        hw.clock();
        self.pc = self.pc.wrapping_add(n as i8 as i16 as u16);
    }

    fn ret<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, cc: Option<FlagCondition>, ei: bool) {
        if !self.f.check_jmp_condition(cc) {
            hw.clock();
            return;
        }
        if cc.is_some() {
            hw.clock();
        }
        if ei {
            // RETI immediately enables IME, it's not deferred like eith an EI or DI call.
            self.ime = true;
        }
        let pc = self.stack_pop(hw);
        hw.clock();
        self.pc = pc;
    }

    fn rst<T: EventListener>(&mut self, hw: &mut GameboyHardware<T>, a: u8) {
        hw.clock();
        self.push_and_jump(hw, a as u16);
    }
}

impl Flags {
    /// Converts the CPU flags into an 8bit value. Used by instructions that operate on F register.
    pub fn pack(&self) -> u8 {
        0 | if self.z { 0b1000_0000 } else { 0 }
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
            Some(FlagCondition::Z) => self.z,
            Some(FlagCondition::NC) => !self.c,
            Some(FlagCondition::C) => self.c,
        }
    }
}

impl std::fmt::Display for Register {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            A => write!(f, "a"),
            B => write!(f, "b"),
            C => write!(f, "c"),
            D => write!(f, "d"),
            E => write!(f, "e"),
            H => write!(f, "h"),
            L => write!(f, "l"),
        }
    }
}

impl std::fmt::Display for Register16 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            AF => write!(f, "af"),
            BC => write!(f, "bc"),
            DE => write!(f, "de"),
            HL => write!(f, "hl"),
            SP => write!(f, "sp"),
        }
    }
}

impl std::fmt::Display for Operand {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Operand::Register(r) => write!(f, "{:?}", r),
            Operand::Immediate(n8) => write!(f, "${:2x}", n8),
            Operand::Address(rr) => write!(f, "[{:?}]", rr),
            Operand::AddressInc(rr) => write!(f, "[{:?}+]", rr),
            Operand::AddressDec(rr) => write!(f, "[{:?}-]", rr),
            Operand::ImmediateAddress(n16) => write!(f, "[${:4x}]", n16),
            Operand::ImmediateAddressHigh(n8) => write!(f, "[$ff00+${:2x}]", n8),
            Operand::AddressHigh(r) => write!(f, "[$ff00+{:?}]", r),
        }
    }
}

impl std::fmt::Display for Operand16 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Operand16::Register(r) => write!(f, "{:?}", r),
            Operand16::Immediate(n16) => write!(f, "${:4x}", n16),
            Operand16::ImmediateAddress(addr) => write!(f, "[${:4x}]", addr),
        }
    }
}

impl std::fmt::Display for FlagCondition {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            FlagCondition::NZ => write!(f, "nz"),
            FlagCondition::Z => write!(f, "z"),
            FlagCondition::NC => write!(f, "nc"),
            FlagCondition::C => write!(f, "c"),
        }
    }
}

impl std::fmt::Display for Instruction {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            ADC(o) => write!(f, "adc a, {}", o),
            ADD(o) => write!(f, "add a, {}", o),
            ADD16(rr) => write!(f, "add hl, {:?}", rr),
            ADD_SP_r8(r8) => write!(f, "add sp, ${:2x}", r8),
            AND(o) => write!(f, "and {}", o),
            BIT(b, o) => write!(f, "bit {}, {}", b, o),
            CALL(None, addr) => write!(f, "call {}", addr),
            CALL(cc, addr) => write!(f, "call {}, {}", cc.unwrap(), addr),
            CCF => write!(f, "ccf"),
            CP(o) => write!(f, "cp {}", o),
            CPL => write!(f, "cpl"),
            DAA => write!(f, "daa"),
            DEC(o) => write!(f, "dec {}", o),
            DEC16(rr) => write!(f, "dec {}", rr),
            DI => write!(f, "di"),
            EI => write!(f, "ei"),
            HALT => write!(f, "halt"),
            INC(o) => write!(f, "inc {}", o),
            INC16(rr) => write!(f, "inc {}", rr),
            JP(None, o) => write!(f, "jp {}", o),
            JP(cc, o) => write!(f, "jp {}, {}", cc.unwrap(), o),
            JR(None, r8) => write!(f, "jr {}", r8),
            JR(cc, r8) => write!(f, "jr {}, {}", cc.unwrap(), r8),
            LD(lhs, rhs) => write!(f, "ld {}, {}", lhs, rhs),
            LD16(lhs, rhs) => write!(f, "ld {}, {}", lhs, rhs),
            LD_HL_SP(e8) => write!(f, "ld hl, sp+${:2x}", e8),
            NOP => write!(f, "nop"),
            OR(o) => write!(f, "or {}", o),
            POP(rr) => write!(f, "pop {}", rr),
            PUSH(rr) => write!(f, "push {}", rr),
            RES(b, o) => write!(f, "res {}, {}", b, o),
            RET(None) => write!(f, "ret"),
            RET(cc) => write!(f, "ret {}", cc.unwrap()),
            RETI => write!(f, "reti"),
            RL(o) => write!(f, "rl {}", o),
            RLA => write!(f, "rla"),
            RLC(o) => write!(f, "rlc {}", o),
            RLCA => write!(f, "rlca"),
            RR(o) => write!(f, "rr {}", o),
            RRA => write!(f, "rra"),
            RRC(o) => write!(f, "rrc {}", o),
            RRCA => write!(f, "rrca"),
            RST(vec) => write!(f, "rst ${:2x}", vec),
            SBC(o) => write!(f, "sbc a, {}", o),
            SCF => write!(f, "scf"),
            SET(b, o) => write!(f, "set {}, {}", b, o),
            SLA(o) => write!(f, "sla {}", o),
            SRA(o) => write!(f, "sra {}", o),
            SRL(o) => write!(f, "srl {}", o),
            STOP => write!(f, "stop"),
            SUB(o) => write!(f, "sub a, {}", o),
            SWAP(o) => write!(f, "swap {}", o),
            XOR(o) => write!(f, "xor a, {}", o),
            Invalid(op) => write!(f, "unknown opcode ${:2x}", op),
        }
    }
}

impl Instruction {
    /// Indicates if current instruction affects program execution flow (CALL, JP, JR, RET, RETI, RST)
    pub fn is_flow_control(&self) -> bool {
        match self {
            Instruction::CALL(_, _)
            | Instruction::JP(_, _)
            | Instruction::JR(_, _)
            | Instruction::RET(_)
            | Instruction::RETI
            | Instruction::RST(_) => true,
            _ => false,
        }
    }
}

/// The instruction decoder. Fetches the next 8-bit opcode from an arbitrary source, decodes it into an instruction, and
/// then possibly fetches another 1-2 bytes used by that instruction. The "arbitrary source" in the main case will be
/// the CPU fetching the next byte from the memory address pointed to by the PC register (and then bumping it), but this
/// decode logic is also appropriate for a disassembler.
pub fn decode_instruction<T: FnMut() -> u8>(mut fetch: T) -> Instruction {
    macro_rules! fetch16 {
        () => {
            u16::from(fetch()) | (u16::from(fetch()) << 8)
        };
    }

    match fetch() {
        0x00 /* NOP         */ => Instruction::NOP,
        0x01 /* LD BC,d16   */ => Instruction::LD16(Operand16::Register(BC), Operand16::Immediate(fetch16!())),
        0x02 /* LD (BC),A   */ => Instruction::LD(Operand::Address(BC), Operand::Register(A)),
        0x03 /* INC BC      */ => Instruction::INC16(BC),
        0x04 /* INC B       */ => Instruction::INC(Operand::Register(B)),
        0x05 /* DEC B       */ => Instruction::DEC(Operand::Register(B)),
        0x06 /* LD B,d8     */ => Instruction::LD(Operand::Register(B), Operand::Immediate(fetch())),
        0x07 /* RLCA        */ => Instruction::RLCA,
        0x08 /* LD (a16),SP */ => Instruction::LD16(Operand16::ImmediateAddress(fetch16!()), Operand16::Register(SP)),
        0x09 /* ADD HL,BC   */ => Instruction::ADD16(BC),
        0x0A /* LD A,(BC)   */ => Instruction::LD(Operand::Register(A), Operand::Address(BC)),
        0x0B /* DEC BC      */ => Instruction::DEC16(BC),
        0x0C /* INC C       */ => Instruction::INC(Operand::Register(C)),
        0x0D /* DEC C       */ => Instruction::DEC(Operand::Register(C)),
        0x0E /* LD C,d8     */ => Instruction::LD(Operand::Register(C), Operand::Immediate(fetch())),
        0x0F /* RRCA        */ => Instruction::RRCA,
        0x10 /* STOP        */ => Instruction::STOP,
        0x11 /* LD DE,d16   */ => Instruction::LD16(Operand16::Register(DE), Operand16::Immediate(fetch16!())),
        0x12 /* LD (DE),A   */ => Instruction::LD(Operand::Address(DE), Operand::Register(A)),
        0x13 /* INC DE      */ => Instruction::INC16(DE),
        0x14 /* INC D       */ => Instruction::INC(Operand::Register(D)),
        0x15 /* DEC D       */ => Instruction::DEC(Operand::Register(D)),
        0x16 /* LD D,d8     */ => Instruction::LD(Operand::Register(D), Operand::Immediate(fetch())),
        0x17 /* RLA         */ => Instruction::RLA,
        0x18 /* JR r8       */ => Instruction::JR(None, fetch()),
        0x19 /* ADD HL,DE   */ => Instruction::ADD16(DE),
        0x1A /* LD A,(DE)   */ => Instruction::LD(Operand::Register(A), Operand::Address(DE)),
        0x1B /* DEC DE      */ => Instruction::DEC16(DE),
        0x1C /* INC E       */ => Instruction::INC(Operand::Register(E)),
        0x1D /* DEC E       */ => Instruction::DEC(Operand::Register(E)),
        0x1E /* LD E,d8     */ => Instruction::LD(Operand::Register(E), Operand::Immediate(fetch())),
        0x1F /* RRA         */ => Instruction::RRA,
        0x20 /* JR NZ,r8    */ => Instruction::JR(Some(FlagCondition::NZ), fetch()),
        0x21 /* LD HL,d16   */ => Instruction::LD16(Operand16::Register(HL), Operand16::Immediate(fetch16!())),
        0x22 /* LD (HL+),A  */ => Instruction::LD(Operand::AddressInc(HL), Operand::Register(A)),
        0x23 /* INC HL      */ => Instruction::INC16(HL),
        0x24 /* INC H       */ => Instruction::INC(Operand::Register(H)),
        0x25 /* DEC H       */ => Instruction::DEC(Operand::Register(H)),
        0x26 /* LD H,d8     */ => Instruction::LD(Operand::Register(H), Operand::Immediate(fetch())),
        0x27 /* DAA         */ => Instruction::DAA,
        0x28 /* JR Z,r8     */ => Instruction::JR(Some(FlagCondition::Z), fetch()),
        0x29 /* ADD HL,HL   */ => Instruction::ADD16(HL),
        0x2A /* LD A,(HL+)  */ => Instruction::LD(Operand::Register(A), Operand::AddressInc(HL)),
        0x2B /* DEC HL      */ => Instruction::DEC16(HL),
        0x2C /* INC L       */ => Instruction::INC(Operand::Register(L)),
        0x2D /* DEC L       */ => Instruction::DEC(Operand::Register(L)),
        0x2E /* LD L,d8     */ => Instruction::LD(Operand::Register(L), Operand::Immediate(fetch())),
        0x2F /* CPL         */ => Instruction::CPL,
        0x30 /* JR NC,r8    */ => Instruction::JR(Some(FlagCondition::NC), fetch()),
        0x31 /* LD SP,d16   */ => Instruction::LD16(Operand16::Register(SP), Operand16::Immediate(fetch16!())),
        0x32 /* LD (HL-), A */ => Instruction::LD(Operand::AddressDec(HL), Operand::Register(A)),
        0x33 /* INC SP      */ => Instruction::INC16(SP),
        0x34 /* INC (HL)    */ => Instruction::INC(Operand::Address(HL)),
        0x35 /* DEC (HL)    */ => Instruction::DEC(Operand::Address(HL)),
        0x36 /* LD (HL),d8  */ => Instruction::LD(Operand::Address(HL), Operand::Immediate(fetch())),
        0x37 /* SCF         */ => Instruction::SCF,
        0x38 /* JR C,r8     */ => Instruction::JR(Some(FlagCondition::C), fetch()),
        0x39 /* ADD HL,SP   */ => Instruction::ADD16(SP),
        0x3A /* LD A, (HL-) */ => Instruction::LD(Operand::Register(A), Operand::AddressDec(HL)),
        0x3B /* DEC SP      */ => Instruction::DEC16(SP),
        0x3C /* INC A       */ => Instruction::INC(Operand::Register(A)),
        0x3D /* DEC A       */ => Instruction::DEC(Operand::Register(A)),
        0x3E /* LD A,d8     */ => Instruction::LD(Operand::Register(A), Operand::Immediate(fetch())),
        0x3F /* CCF         */ => Instruction::CCF,
        0x40 /* LD B,B      */ => Instruction::LD(Operand::Register(B), Operand::Register(B)),
        0x41 /* LD B,C      */ => Instruction::LD(Operand::Register(B), Operand::Register(C)),
        0x42 /* LD B,D      */ => Instruction::LD(Operand::Register(B), Operand::Register(D)),
        0x43 /* LD B,E      */ => Instruction::LD(Operand::Register(B), Operand::Register(E)),
        0x44 /* LD B,H      */ => Instruction::LD(Operand::Register(B), Operand::Register(H)),
        0x45 /* LD B,L      */ => Instruction::LD(Operand::Register(B), Operand::Register(L)),
        0x46 /* LD B,(HL)   */ => Instruction::LD(Operand::Register(B), Operand::Address(HL)),
        0x47 /* LD B,A      */ => Instruction::LD(Operand::Register(B), Operand::Register(A)),
        0x48 /* LD C,B      */ => Instruction::LD(Operand::Register(C), Operand::Register(B)),
        0x49 /* LD C,C      */ => Instruction::LD(Operand::Register(C), Operand::Register(C)),
        0x4A /* LD C,D      */ => Instruction::LD(Operand::Register(C), Operand::Register(D)),
        0x4B /* LD C,E      */ => Instruction::LD(Operand::Register(C), Operand::Register(E)),
        0x4C /* LD C,H      */ => Instruction::LD(Operand::Register(C), Operand::Register(H)),
        0x4D /* LD C,L      */ => Instruction::LD(Operand::Register(C), Operand::Register(L)),
        0x4E /* LD C,(HL)   */ => Instruction::LD(Operand::Register(C), Operand::Address(HL)),
        0x4F /* LD C,A      */ => Instruction::LD(Operand::Register(C), Operand::Register(A)),
        0x50 /* LD D,B      */ => Instruction::LD(Operand::Register(D), Operand::Register(B)),
        0x51 /* LD D,C      */ => Instruction::LD(Operand::Register(D), Operand::Register(C)),
        0x52 /* LD D,D      */ => Instruction::LD(Operand::Register(D), Operand::Register(D)),
        0x53 /* LD D,E      */ => Instruction::LD(Operand::Register(D), Operand::Register(E)),
        0x54 /* LD D,H      */ => Instruction::LD(Operand::Register(D), Operand::Register(H)),
        0x55 /* LD D,L      */ => Instruction::LD(Operand::Register(D), Operand::Register(L)),
        0x56 /* LD D,(HL)   */ => Instruction::LD(Operand::Register(D), Operand::Address(HL)),
        0x57 /* LD D,A      */ => Instruction::LD(Operand::Register(D), Operand::Register(A)),
        0x58 /* LD E,B      */ => Instruction::LD(Operand::Register(E), Operand::Register(B)),
        0x59 /* LD E,C      */ => Instruction::LD(Operand::Register(E), Operand::Register(C)),
        0x5A /* LD E,D      */ => Instruction::LD(Operand::Register(E), Operand::Register(D)),
        0x5B /* LD E,E      */ => Instruction::LD(Operand::Register(E), Operand::Register(E)),
        0x5C /* LD E,H      */ => Instruction::LD(Operand::Register(E), Operand::Register(H)),
        0x5D /* LD E,L      */ => Instruction::LD(Operand::Register(E), Operand::Register(L)),
        0x5E /* LD E,(HL)   */ => Instruction::LD(Operand::Register(E), Operand::Address(HL)),
        0x5F /* LD E,A      */ => Instruction::LD(Operand::Register(E), Operand::Register(A)),
        0x60 /* LD H,B      */ => Instruction::LD(Operand::Register(H), Operand::Register(B)),
        0x61 /* LD H,C      */ => Instruction::LD(Operand::Register(H), Operand::Register(C)),
        0x62 /* LD H,D      */ => Instruction::LD(Operand::Register(H), Operand::Register(D)),
        0x63 /* LD H,E      */ => Instruction::LD(Operand::Register(H), Operand::Register(E)),
        0x64 /* LD H,H      */ => Instruction::LD(Operand::Register(H), Operand::Register(H)),
        0x65 /* LD H,L      */ => Instruction::LD(Operand::Register(H), Operand::Register(L)),
        0x66 /* LD H,(HL)   */ => Instruction::LD(Operand::Register(H), Operand::Address(HL)),
        0x67 /* LD H,A      */ => Instruction::LD(Operand::Register(H), Operand::Register(A)),
        0x68 /* LD L,B      */ => Instruction::LD(Operand::Register(L), Operand::Register(B)),
        0x69 /* LD L,C      */ => Instruction::LD(Operand::Register(L), Operand::Register(C)),
        0x6A /* LD L,D      */ => Instruction::LD(Operand::Register(L), Operand::Register(D)),
        0x6B /* LD L,E      */ => Instruction::LD(Operand::Register(L), Operand::Register(E)),
        0x6C /* LD L,H      */ => Instruction::LD(Operand::Register(L), Operand::Register(H)),
        0x6D /* LD L,L      */ => Instruction::LD(Operand::Register(L), Operand::Register(L)),
        0x6E /* LD L,(HL)   */ => Instruction::LD(Operand::Register(L), Operand::Address(HL)),
        0x6F /* LD L,A      */ => Instruction::LD(Operand::Register(L), Operand::Register(A)),
        0x70 /* LD (HL),B   */ => Instruction::LD(Operand::Address(HL), Operand::Register(B)),
        0x71 /* LD (HL),C   */ => Instruction::LD(Operand::Address(HL), Operand::Register(C)),
        0x72 /* LD (HL),D   */ => Instruction::LD(Operand::Address(HL), Operand::Register(D)),
        0x73 /* LD (HL),E   */ => Instruction::LD(Operand::Address(HL), Operand::Register(E)),
        0x74 /* LD (HL),H   */ => Instruction::LD(Operand::Address(HL), Operand::Register(H)),
        0x75 /* LD (HL),L   */ => Instruction::LD(Operand::Address(HL), Operand::Register(L)),
        0x76 /* HALT        */ => Instruction::HALT,
        0x77 /* LD (HL),A   */ => Instruction::LD(Operand::Address(HL), Operand::Register(A)),
        0x78 /* LD A,B      */ => Instruction::LD(Operand::Register(A), Operand::Register(B)),
        0x79 /* LD A,C      */ => Instruction::LD(Operand::Register(A), Operand::Register(C)),
        0x7A /* LD A,D      */ => Instruction::LD(Operand::Register(A), Operand::Register(D)),
        0x7B /* LD A,E      */ => Instruction::LD(Operand::Register(A), Operand::Register(E)),
        0x7C /* LD A,H      */ => Instruction::LD(Operand::Register(A), Operand::Register(H)),
        0x7D /* LD A,L      */ => Instruction::LD(Operand::Register(A), Operand::Register(L)),
        0x7E /* LD A,(HL)   */ => Instruction::LD(Operand::Register(A), Operand::Address(HL)),
        0x7F /* LD A,A      */ => Instruction::LD(Operand::Register(A), Operand::Register(A)),
        0x80 /* ADD A,B     */ => Instruction::ADD(Operand::Register(B)),
        0x81 /* ADD A,C     */ => Instruction::ADD(Operand::Register(C)),
        0x82 /* ADD A,D     */ => Instruction::ADD(Operand::Register(D)),
        0x83 /* ADD A,E     */ => Instruction::ADD(Operand::Register(E)),
        0x84 /* ADD A,H     */ => Instruction::ADD(Operand::Register(H)),
        0x85 /* ADD A,L     */ => Instruction::ADD(Operand::Register(L)),
        0x86 /* ADD A,(HL)  */ => Instruction::ADD(Operand::Address(HL)),
        0x87 /* ADD A,A     */ => Instruction::ADD(Operand::Register(A)),
        0x88 /* ADC A,B     */ => Instruction::ADC(Operand::Register(B)),
        0x89 /* ADC A,C     */ => Instruction::ADC(Operand::Register(C)),
        0x8A /* ADC A,D     */ => Instruction::ADC(Operand::Register(D)),
        0x8B /* ADC A,E     */ => Instruction::ADC(Operand::Register(E)),
        0x8C /* ADC A,H     */ => Instruction::ADC(Operand::Register(H)),
        0x8D /* ADC A,L     */ => Instruction::ADC(Operand::Register(L)),
        0x8E /* ADC A,(HL)  */ => Instruction::ADC(Operand::Address(HL)),
        0x8F /* ADC A,A     */ => Instruction::ADC(Operand::Register(A)),
        0x90 /* SUB B       */ => Instruction::SUB(Operand::Register(B)),
        0x91 /* SUB C       */ => Instruction::SUB(Operand::Register(C)),
        0x92 /* SUB D       */ => Instruction::SUB(Operand::Register(D)),
        0x93 /* SUB E       */ => Instruction::SUB(Operand::Register(E)),
        0x94 /* SUB H       */ => Instruction::SUB(Operand::Register(H)),
        0x95 /* SUB L       */ => Instruction::SUB(Operand::Register(L)),
        0x96 /* SUB (HL)    */ => Instruction::SUB(Operand::Address(HL)),
        0x97 /* SUB A       */ => Instruction::SUB(Operand::Register(A)),
        0x98 /* SBC A,B     */ => Instruction::SBC(Operand::Register(B)),
        0x99 /* SBC A,C     */ => Instruction::SBC(Operand::Register(C)),
        0x9A /* SBC A,D     */ => Instruction::SBC(Operand::Register(D)),
        0x9B /* SBC A,E     */ => Instruction::SBC(Operand::Register(E)),
        0x9C /* SBC A,H     */ => Instruction::SBC(Operand::Register(H)),
        0x9D /* SBC A,L     */ => Instruction::SBC(Operand::Register(L)),
        0x9E /* SBC A,(HL)  */ => Instruction::SBC(Operand::Address(HL)),
        0x9F /* SBC A,A     */ => Instruction::SBC(Operand::Register(A)),
        0xA0 /* AND B       */ => Instruction::AND(Operand::Register(B)),
        0xA1 /* AND C       */ => Instruction::AND(Operand::Register(C)),
        0xA2 /* AND D       */ => Instruction::AND(Operand::Register(D)),
        0xA3 /* AND E       */ => Instruction::AND(Operand::Register(E)),
        0xA4 /* AND H       */ => Instruction::AND(Operand::Register(H)),
        0xA5 /* AND L       */ => Instruction::AND(Operand::Register(L)),
        0xA6 /* AND (HL)    */ => Instruction::AND(Operand::Address(HL)),
        0xA7 /* AND A       */ => Instruction::AND(Operand::Register(A)),
        0xA8 /* XOR B       */ => Instruction::XOR(Operand::Register(B)),
        0xA9 /* XOR C       */ => Instruction::XOR(Operand::Register(C)),
        0xAA /* XOR D       */ => Instruction::XOR(Operand::Register(D)),
        0xAB /* XOR E       */ => Instruction::XOR(Operand::Register(E)),
        0xAC /* XOR H       */ => Instruction::XOR(Operand::Register(H)),
        0xAD /* XOR L       */ => Instruction::XOR(Operand::Register(L)),
        0xAE /* XOR (HL)    */ => Instruction::XOR(Operand::Address(HL)),
        0xAF /* XOR A       */ => Instruction::XOR(Operand::Register(A)),
        0xB0 /* OR B        */ => Instruction::OR (Operand::Register(B)),
        0xB1 /* OR C        */ => Instruction::OR (Operand::Register(C)),
        0xB2 /* OR D        */ => Instruction::OR (Operand::Register(D)),
        0xB3 /* OR E        */ => Instruction::OR (Operand::Register(E)),
        0xB4 /* OR H        */ => Instruction::OR (Operand::Register(H)),
        0xB5 /* OR L        */ => Instruction::OR (Operand::Register(L)),
        0xB6 /* OR (HL)     */ => Instruction::OR (Operand::Address(HL)),
        0xB7 /* OR A        */ => Instruction::OR (Operand::Register(A)),
        0xB8 /* CP B        */ => Instruction::CP (Operand::Register(B)),
        0xB9 /* CP C        */ => Instruction::CP (Operand::Register(C)),
        0xBA /* CP D        */ => Instruction::CP (Operand::Register(D)),
        0xBB /* CP E        */ => Instruction::CP (Operand::Register(E)),
        0xBC /* CP H        */ => Instruction::CP (Operand::Register(H)),
        0xBD /* CP L        */ => Instruction::CP (Operand::Register(L)),
        0xBE /* CP (HL)     */ => Instruction::CP (Operand::Address(HL)),
        0xBF /* CP A        */ => Instruction::CP (Operand::Register(A)),
        0xC0 /* RET NZ      */ => Instruction::RET(Some(FlagCondition::NZ)),
        0xC1 /* POP BC      */ => Instruction::POP(BC),
        0xC2 /* JP NZ,a16   */ => Instruction::JP(Some(FlagCondition::NZ), Operand16::Immediate(fetch16!())),
        0xC3 /* JP a16      */ => Instruction::JP(None, Operand16::Immediate(fetch16!())),
        0xC4 /* CALL NZ,a16 */ => Instruction::CALL(Some(FlagCondition::NZ), fetch16!()),
        0xC5 /* PUSH BC     */ => Instruction::PUSH(BC),
        0xC6 /* ADD A,d8    */ => Instruction::ADD(Operand::Immediate(fetch())),
        0xC7 /* RST 00H     */ => Instruction::RST(0x00),
        0xC8 /* RET Z       */ => Instruction::RET(Some(FlagCondition::Z)),
        0xC9 /* RET         */ => Instruction::RET(None),
        0xCA /* JP Z,a16    */ => Instruction::JP(Some(FlagCondition::Z), Operand16::Immediate(fetch16!())),
        0xCB /* PREFIX CB   */ => decode_extended_instruction(fetch),
        0xCC /* CALL Z,a16  */ => Instruction::CALL(Some(FlagCondition::Z), fetch16!()),
        0xCD /* CALL a16    */ => Instruction::CALL(None, fetch16!()),
        0xCE /* ADC A,d8    */ => Instruction::ADC(Operand::Immediate(fetch())),
        0xCF /* RST 08H     */ => Instruction::RST(0x08),
        0xD0 /* RET NC      */ => Instruction::RET(Some(FlagCondition::NC)),
        0xD1 /* POP DE      */ => Instruction::POP(DE),
        0xD2 /* JP NC,a16   */ => Instruction::JP(Some(FlagCondition::NC), Operand16::Immediate(fetch16!())),
        0xD4 /* CALL NC,a16 */ => Instruction::CALL(Some(FlagCondition::NC), fetch16!()),
        0xD5 /* PUSH DE     */ => Instruction::PUSH(DE),
        0xD6 /* SUB d8      */ => Instruction::SUB(Operand::Immediate(fetch())),
        0xD7 /* RST 10H     */ => Instruction::RST(0x10),
        0xD8 /* RET C       */ => Instruction::RET(Some(FlagCondition::C)),
        0xD9 /* RETI        */ => Instruction::RETI,
        0xDA /* JP C,a16    */ => Instruction::JP(Some(FlagCondition::C), Operand16::Immediate(fetch16!())),
        0xDC /* CALL C,a16  */ => Instruction::CALL(Some(FlagCondition::C), fetch16!()),
        0xDE /* SBC A,d8    */ => Instruction::SBC(Operand::Immediate(fetch())),
        0xDF /* RST 18H     */ => Instruction::RST(0x18),
        0xE0 /* LDH (a8),A  */ => Instruction::LD(Operand::ImmediateAddressHigh(fetch()), Operand::Register(A)),
        0xE1 /* POP HL      */ => Instruction::POP(HL),
        0xE2 /* LD (C),A    */ => Instruction::LD(Operand::AddressHigh(C), Operand::Register(A)),
        0xE5 /* PUSH HL     */ => Instruction::PUSH(HL),
        0xE6 /* AND d8      */ => Instruction::AND(Operand::Immediate(fetch())),
        0xE7 /* RST 20H     */ => Instruction::RST(0x20),
        0xE8 /* ADD SP,r8   */ => Instruction::ADD_SP_r8(fetch() as i8),
        0xE9 /* JP (HL)     */ => Instruction::JP(None, Operand16::Register(HL)),
        0xEA /* LD (a16),A  */ => Instruction::LD(Operand::ImmediateAddress(fetch16!()), Operand::Register(A)),
        0xEE /* XOR d8      */ => Instruction::XOR(Operand::Immediate(fetch())),
        0xEF /* RST 28H     */ => Instruction::RST(0x28),
        0xF0 /* LDH A,(a8)  */ => Instruction::LD(Operand::Register(A), Operand::ImmediateAddressHigh(fetch())),
        0xF1 /* POP AF      */ => Instruction::POP(AF),
        0xF2 /* LD A,(C)    */ => Instruction::LD(Operand::Register(A), Operand::AddressHigh(C)),
        0xF3 /* DI          */ => Instruction::DI,
        0xF5 /* PUSH AF     */ => Instruction::PUSH(AF),
        0xF6 /* OR d8       */ => Instruction::OR(Operand::Immediate(fetch())),
        0xF7 /* RST 30H     */ => Instruction::RST(0x30),
        0xF8 /* LD HL,SP+r8 */ => Instruction::LD_HL_SP(fetch() as i8),
        0xF9 /* LD SP,HL    */ => Instruction::LD16(Operand16::Register(SP), Operand16::Register(HL)),
        0xFA /* LD A,(a16)  */ => Instruction::LD(Operand::Register(A), Operand::ImmediateAddress(fetch16!())),
        0xFB /* EI          */ => Instruction::EI,
        0xFE /* CP d8       */ => Instruction::CP(Operand::Immediate(fetch())),
        0xFF /* RST 38H     */ => Instruction::RST(0x38),
        op => Instruction::Invalid(op)
    }
}

pub fn decode_extended_instruction<T: FnMut() -> u8>(mut fetch: T) -> Instruction {
    match fetch() {
        0x00 /* RLC B       */ => Instruction::RLC(Operand::Register(B)),
        0x01 /* RLC C       */ => Instruction::RLC(Operand::Register(C)),
        0x02 /* RLC D       */ => Instruction::RLC(Operand::Register(D)),
        0x03 /* RLC E       */ => Instruction::RLC(Operand::Register(E)),
        0x04 /* RLC H       */ => Instruction::RLC(Operand::Register(H)),
        0x05 /* RLC L       */ => Instruction::RLC(Operand::Register(L)),
        0x06 /* RLC (HL)    */ => Instruction::RLC(Operand::Address(HL)),
        0x07 /* RLC A       */ => Instruction::RLC(Operand::Register(A)),
        0x08 /* RRC B       */ => Instruction::RRC(Operand::Register(B)),
        0x09 /* RRC C       */ => Instruction::RRC(Operand::Register(C)),
        0x0A /* RRC D       */ => Instruction::RRC(Operand::Register(D)),
        0x0B /* RRC E       */ => Instruction::RRC(Operand::Register(E)),
        0x0C /* RRC H       */ => Instruction::RRC(Operand::Register(H)),
        0x0D /* RRC L       */ => Instruction::RRC(Operand::Register(L)),
        0x0E /* RRC (HL)    */ => Instruction::RRC(Operand::Address(HL)),
        0x0F /* RRC A       */ => Instruction::RRC(Operand::Register(A)),
        0x10 /* RL B        */ => Instruction::RL (Operand::Register(B)),
        0x11 /* RL C        */ => Instruction::RL (Operand::Register(C)),
        0x12 /* RL D        */ => Instruction::RL (Operand::Register(D)),
        0x13 /* RL E        */ => Instruction::RL (Operand::Register(E)),
        0x14 /* RL H        */ => Instruction::RL (Operand::Register(H)),
        0x15 /* RL L        */ => Instruction::RL (Operand::Register(L)),
        0x16 /* RL (HL)     */ => Instruction::RL (Operand::Address(HL)),
        0x17 /* RL A        */ => Instruction::RL (Operand::Register(A)),
        0x18 /* RR B        */ => Instruction::RR (Operand::Register(B)),
        0x19 /* RR C        */ => Instruction::RR (Operand::Register(C)),
        0x1A /* RR D        */ => Instruction::RR (Operand::Register(D)),
        0x1B /* RR E        */ => Instruction::RR (Operand::Register(E)),
        0x1C /* RR H        */ => Instruction::RR (Operand::Register(H)),
        0x1D /* RR L        */ => Instruction::RR (Operand::Register(L)),
        0x1E /* RR (HL)     */ => Instruction::RR (Operand::Address(HL)),
        0x1F /* RR A        */ => Instruction::RR (Operand::Register(A)),
        0x20 /* SLA B       */ => Instruction::SLA(Operand::Register(B)),
        0x21 /* SLA C       */ => Instruction::SLA(Operand::Register(C)),
        0x22 /* SLA D       */ => Instruction::SLA(Operand::Register(D)),
        0x23 /* SLA E       */ => Instruction::SLA(Operand::Register(E)),
        0x24 /* SLA H       */ => Instruction::SLA(Operand::Register(H)),
        0x25 /* SLA L       */ => Instruction::SLA(Operand::Register(L)),
        0x26 /* SLA (HL)    */ => Instruction::SLA(Operand::Address(HL)),
        0x27 /* SLA A       */ => Instruction::SLA(Operand::Register(A)),
        0x28 /* SRA B       */ => Instruction::SRA(Operand::Register(B)),
        0x29 /* SRA C       */ => Instruction::SRA(Operand::Register(C)),
        0x2A /* SRA D       */ => Instruction::SRA(Operand::Register(D)),
        0x2B /* SRA E       */ => Instruction::SRA(Operand::Register(E)),
        0x2C /* SRA H       */ => Instruction::SRA(Operand::Register(H)),
        0x2D /* SRA L       */ => Instruction::SRA(Operand::Register(L)),
        0x2E /* SRA (HL)    */ => Instruction::SRA(Operand::Address(HL)),
        0x2F /* SRA A       */ => Instruction::SRA(Operand::Register(A)),
        0x30 /* SWAP B      */ => Instruction::SWAP(Operand::Register(B)),
        0x31 /* SWAP C      */ => Instruction::SWAP(Operand::Register(C)),
        0x32 /* SWAP D      */ => Instruction::SWAP(Operand::Register(D)),
        0x33 /* SWAP E      */ => Instruction::SWAP(Operand::Register(E)),
        0x34 /* SWAP H      */ => Instruction::SWAP(Operand::Register(H)),
        0x35 /* SWAP L      */ => Instruction::SWAP(Operand::Register(L)),
        0x36 /* SWAP (HL)   */ => Instruction::SWAP(Operand::Address(HL)),
        0x37 /* SWAP A      */ => Instruction::SWAP(Operand::Register(A)),
        0x38 /* SRL B       */ => Instruction::SRL(Operand::Register(B)),
        0x39 /* SRL C       */ => Instruction::SRL(Operand::Register(C)),
        0x3A /* SRL D       */ => Instruction::SRL(Operand::Register(D)),
        0x3B /* SRL E       */ => Instruction::SRL(Operand::Register(E)),
        0x3C /* SRL H       */ => Instruction::SRL(Operand::Register(H)),
        0x3D /* SRL L       */ => Instruction::SRL(Operand::Register(L)),
        0x3E /* SRL (HL)    */ => Instruction::SRL(Operand::Address(HL)),
        0x3F /* SRL A       */ => Instruction::SRL(Operand::Register(A)),
        0x40 /* BIT 0,B     */ => Instruction::BIT(0, Operand::Register(B)),
        0x41 /* BIT 0,C     */ => Instruction::BIT(0, Operand::Register(C)),
        0x42 /* BIT 0,D     */ => Instruction::BIT(0, Operand::Register(D)),
        0x43 /* BIT 0,E     */ => Instruction::BIT(0, Operand::Register(E)),
        0x44 /* BIT 0,H     */ => Instruction::BIT(0, Operand::Register(H)),
        0x45 /* BIT 0,L     */ => Instruction::BIT(0, Operand::Register(L)),
        0x46 /* BIT 0,(HL)  */ => Instruction::BIT(0, Operand::Address(HL)),
        0x47 /* BIT 0,A     */ => Instruction::BIT(0, Operand::Register(A)),
        0x48 /* BIT 1,B     */ => Instruction::BIT(1, Operand::Register(B)),
        0x49 /* BIT 1,C     */ => Instruction::BIT(1, Operand::Register(C)),
        0x4A /* BIT 1,D     */ => Instruction::BIT(1, Operand::Register(D)),
        0x4B /* BIT 1,E     */ => Instruction::BIT(1, Operand::Register(E)),
        0x4C /* BIT 1,H     */ => Instruction::BIT(1, Operand::Register(H)),
        0x4D /* BIT 1,L     */ => Instruction::BIT(1, Operand::Register(L)),
        0x4E /* BIT 1,(HL)  */ => Instruction::BIT(1, Operand::Address(HL)),
        0x4F /* BIT 1,A     */ => Instruction::BIT(1, Operand::Register(A)),
        0x50 /* BIT 2,B     */ => Instruction::BIT(2, Operand::Register(B)),
        0x51 /* BIT 2,C     */ => Instruction::BIT(2, Operand::Register(C)),
        0x52 /* BIT 2,D     */ => Instruction::BIT(2, Operand::Register(D)),
        0x53 /* BIT 2,E     */ => Instruction::BIT(2, Operand::Register(E)),
        0x54 /* BIT 2,H     */ => Instruction::BIT(2, Operand::Register(H)),
        0x55 /* BIT 2,L     */ => Instruction::BIT(2, Operand::Register(L)),
        0x56 /* BIT 2,(HL)  */ => Instruction::BIT(2, Operand::Address(HL)),
        0x57 /* BIT 2,A     */ => Instruction::BIT(2, Operand::Register(A)),
        0x58 /* BIT 3,B     */ => Instruction::BIT(3, Operand::Register(B)),
        0x59 /* BIT 3,C     */ => Instruction::BIT(3, Operand::Register(C)),
        0x5A /* BIT 3,D     */ => Instruction::BIT(3, Operand::Register(D)),
        0x5B /* BIT 3,E     */ => Instruction::BIT(3, Operand::Register(E)),
        0x5C /* BIT 3,H     */ => Instruction::BIT(3, Operand::Register(H)),
        0x5D /* BIT 3,L     */ => Instruction::BIT(3, Operand::Register(L)),
        0x5E /* BIT 3,(HL)  */ => Instruction::BIT(3, Operand::Address(HL)),
        0x5F /* BIT 3,A     */ => Instruction::BIT(3, Operand::Register(A)),
        0x60 /* BIT 4,B     */ => Instruction::BIT(4, Operand::Register(B)),
        0x61 /* BIT 4,C     */ => Instruction::BIT(4, Operand::Register(C)),
        0x62 /* BIT 4,D     */ => Instruction::BIT(4, Operand::Register(D)),
        0x63 /* BIT 4,E     */ => Instruction::BIT(4, Operand::Register(E)),
        0x64 /* BIT 4,H     */ => Instruction::BIT(4, Operand::Register(H)),
        0x65 /* BIT 4,L     */ => Instruction::BIT(4, Operand::Register(L)),
        0x66 /* BIT 4,(HL)  */ => Instruction::BIT(4, Operand::Address(HL)),
        0x67 /* BIT 4,A     */ => Instruction::BIT(4, Operand::Register(A)),
        0x68 /* BIT 5,B     */ => Instruction::BIT(5, Operand::Register(B)),
        0x69 /* BIT 5,C     */ => Instruction::BIT(5, Operand::Register(C)),
        0x6A /* BIT 5,D     */ => Instruction::BIT(5, Operand::Register(D)),
        0x6B /* BIT 5,E     */ => Instruction::BIT(5, Operand::Register(E)),
        0x6C /* BIT 5,H     */ => Instruction::BIT(5, Operand::Register(H)),
        0x6D /* BIT 5,L     */ => Instruction::BIT(5, Operand::Register(L)),
        0x6E /* BIT 5,(HL)  */ => Instruction::BIT(5, Operand::Address(HL)),
        0x6F /* BIT 5,A     */ => Instruction::BIT(5, Operand::Register(A)),
        0x70 /* BIT 6,B     */ => Instruction::BIT(6, Operand::Register(B)),
        0x71 /* BIT 6,C     */ => Instruction::BIT(6, Operand::Register(C)),
        0x72 /* BIT 6,D     */ => Instruction::BIT(6, Operand::Register(D)),
        0x73 /* BIT 6,E     */ => Instruction::BIT(6, Operand::Register(E)),
        0x74 /* BIT 6,H     */ => Instruction::BIT(6, Operand::Register(H)),
        0x75 /* BIT 6,L     */ => Instruction::BIT(6, Operand::Register(L)),
        0x76 /* BIT 6,(HL)  */ => Instruction::BIT(6, Operand::Address(HL)),
        0x77 /* BIT 6,A     */ => Instruction::BIT(6, Operand::Register(A)),
        0x78 /* BIT 7,B     */ => Instruction::BIT(7, Operand::Register(B)),
        0x79 /* BIT 7,C     */ => Instruction::BIT(7, Operand::Register(C)),
        0x7A /* BIT 7,D     */ => Instruction::BIT(7, Operand::Register(D)),
        0x7B /* BIT 7,E     */ => Instruction::BIT(7, Operand::Register(E)),
        0x7C /* BIT 7,H     */ => Instruction::BIT(7, Operand::Register(H)),
        0x7D /* BIT 7,L     */ => Instruction::BIT(7, Operand::Register(L)),
        0x7E /* BIT 7,(HL)  */ => Instruction::BIT(7, Operand::Address(HL)),
        0x7F /* BIT 7,A     */ => Instruction::BIT(7, Operand::Register(A)),
        0x80 /* RES 0,B     */ => Instruction::RES(0, Operand::Register(B)),
        0x81 /* RES 0,C     */ => Instruction::RES(0, Operand::Register(C)),
        0x82 /* RES 0,D     */ => Instruction::RES(0, Operand::Register(D)),
        0x83 /* RES 0,E     */ => Instruction::RES(0, Operand::Register(E)),
        0x84 /* RES 0,H     */ => Instruction::RES(0, Operand::Register(H)),
        0x85 /* RES 0,L     */ => Instruction::RES(0, Operand::Register(L)),
        0x86 /* RES 0,(HL)  */ => Instruction::RES(0, Operand::Address(HL)),
        0x87 /* RES 0,A     */ => Instruction::RES(0, Operand::Register(A)),
        0x88 /* RES 1,B     */ => Instruction::RES(1, Operand::Register(B)),
        0x89 /* RES 1,C     */ => Instruction::RES(1, Operand::Register(C)),
        0x8A /* RES 1,D     */ => Instruction::RES(1, Operand::Register(D)),
        0x8B /* RES 1,E     */ => Instruction::RES(1, Operand::Register(E)),
        0x8C /* RES 1,H     */ => Instruction::RES(1, Operand::Register(H)),
        0x8D /* RES 1,L     */ => Instruction::RES(1, Operand::Register(L)),
        0x8E /* RES 1,(HL)  */ => Instruction::RES(1, Operand::Address(HL)),
        0x8F /* RES 1,A     */ => Instruction::RES(1, Operand::Register(A)),
        0x90 /* RES 2,B     */ => Instruction::RES(2, Operand::Register(B)),
        0x91 /* RES 2,C     */ => Instruction::RES(2, Operand::Register(C)),
        0x92 /* RES 2,D     */ => Instruction::RES(2, Operand::Register(D)),
        0x93 /* RES 2,E     */ => Instruction::RES(2, Operand::Register(E)),
        0x94 /* RES 2,H     */ => Instruction::RES(2, Operand::Register(H)),
        0x95 /* RES 2,L     */ => Instruction::RES(2, Operand::Register(L)),
        0x96 /* RES 2,(HL)  */ => Instruction::RES(2, Operand::Address(HL)),
        0x97 /* RES 2,A     */ => Instruction::RES(2, Operand::Register(A)),
        0x98 /* RES 3,B     */ => Instruction::RES(3, Operand::Register(B)),
        0x99 /* RES 3,C     */ => Instruction::RES(3, Operand::Register(C)),
        0x9A /* RES 3,D     */ => Instruction::RES(3, Operand::Register(D)),
        0x9B /* RES 3,E     */ => Instruction::RES(3, Operand::Register(E)),
        0x9C /* RES 3,H     */ => Instruction::RES(3, Operand::Register(H)),
        0x9D /* RES 3,L     */ => Instruction::RES(3, Operand::Register(L)),
        0x9E /* RES 3,(HL)  */ => Instruction::RES(3, Operand::Address(HL)),
        0x9F /* RES 3,A     */ => Instruction::RES(3, Operand::Register(A)),
        0xA0 /* RES 4,B     */ => Instruction::RES(4, Operand::Register(B)),
        0xA1 /* RES 4,C     */ => Instruction::RES(4, Operand::Register(C)),
        0xA2 /* RES 4,D     */ => Instruction::RES(4, Operand::Register(D)),
        0xA3 /* RES 4,E     */ => Instruction::RES(4, Operand::Register(E)),
        0xA4 /* RES 4,H     */ => Instruction::RES(4, Operand::Register(H)),
        0xA5 /* RES 4,L     */ => Instruction::RES(4, Operand::Register(L)),
        0xA6 /* RES 4,(HL)  */ => Instruction::RES(4, Operand::Address(HL)),
        0xA7 /* RES 4,A     */ => Instruction::RES(4, Operand::Register(A)),
        0xA8 /* RES 5,B     */ => Instruction::RES(5, Operand::Register(B)),
        0xA9 /* RES 5,C     */ => Instruction::RES(5, Operand::Register(C)),
        0xAA /* RES 5,D     */ => Instruction::RES(5, Operand::Register(D)),
        0xAB /* RES 5,E     */ => Instruction::RES(5, Operand::Register(E)),
        0xAC /* RES 5,H     */ => Instruction::RES(5, Operand::Register(H)),
        0xAD /* RES 5,L     */ => Instruction::RES(5, Operand::Register(L)),
        0xAE /* RES 5,(HL)  */ => Instruction::RES(5, Operand::Address(HL)),
        0xAF /* RES 5,A     */ => Instruction::RES(5, Operand::Register(A)),
        0xB0 /* RES 6,B     */ => Instruction::RES(6, Operand::Register(B)),
        0xB1 /* RES 6,C     */ => Instruction::RES(6, Operand::Register(C)),
        0xB2 /* RES 6,D     */ => Instruction::RES(6, Operand::Register(D)),
        0xB3 /* RES 6,E     */ => Instruction::RES(6, Operand::Register(E)),
        0xB4 /* RES 6,H     */ => Instruction::RES(6, Operand::Register(H)),
        0xB5 /* RES 6,L     */ => Instruction::RES(6, Operand::Register(L)),
        0xB6 /* RES 6,(HL)  */ => Instruction::RES(6, Operand::Address(HL)),
        0xB7 /* RES 6,A     */ => Instruction::RES(6, Operand::Register(A)),
        0xB8 /* RES 7,B     */ => Instruction::RES(7, Operand::Register(B)),
        0xB9 /* RES 7,C     */ => Instruction::RES(7, Operand::Register(C)),
        0xBA /* RES 7,D     */ => Instruction::RES(7, Operand::Register(D)),
        0xBB /* RES 7,E     */ => Instruction::RES(7, Operand::Register(E)),
        0xBC /* RES 7,H     */ => Instruction::RES(7, Operand::Register(H)),
        0xBD /* RES 7,L     */ => Instruction::RES(7, Operand::Register(L)),
        0xBE /* RES 7,(HL)  */ => Instruction::RES(7, Operand::Address(HL)),
        0xBF /* RES 7,A     */ => Instruction::RES(7, Operand::Register(A)),
        0xC0 /* SET 0,B     */ => Instruction::SET(0, Operand::Register(B)),
        0xC1 /* SET 0,C     */ => Instruction::SET(0, Operand::Register(C)),
        0xC2 /* SET 0,D     */ => Instruction::SET(0, Operand::Register(D)),
        0xC3 /* SET 0,E     */ => Instruction::SET(0, Operand::Register(E)),
        0xC4 /* SET 0,H     */ => Instruction::SET(0, Operand::Register(H)),
        0xC5 /* SET 0,L     */ => Instruction::SET(0, Operand::Register(L)),
        0xC6 /* SET 0,(HL)  */ => Instruction::SET(0, Operand::Address(HL)),
        0xC7 /* SET 0,A     */ => Instruction::SET(0, Operand::Register(A)),
        0xC8 /* SET 1,B     */ => Instruction::SET(1, Operand::Register(B)),
        0xC9 /* SET 1,C     */ => Instruction::SET(1, Operand::Register(C)),
        0xCA /* SET 1,D     */ => Instruction::SET(1, Operand::Register(D)),
        0xCB /* SET 1,E     */ => Instruction::SET(1, Operand::Register(E)),
        0xCC /* SET 1,H     */ => Instruction::SET(1, Operand::Register(H)),
        0xCD /* SET 1,L     */ => Instruction::SET(1, Operand::Register(L)),
        0xCE /* SET 1,(HL)  */ => Instruction::SET(1, Operand::Address(HL)),
        0xCF /* SET 1,A     */ => Instruction::SET(1, Operand::Register(A)),
        0xD0 /* SET 2,B     */ => Instruction::SET(2, Operand::Register(B)),
        0xD1 /* SET 2,C     */ => Instruction::SET(2, Operand::Register(C)),
        0xD2 /* SET 2,D     */ => Instruction::SET(2, Operand::Register(D)),
        0xD3 /* SET 2,E     */ => Instruction::SET(2, Operand::Register(E)),
        0xD4 /* SET 2,H     */ => Instruction::SET(2, Operand::Register(H)),
        0xD5 /* SET 2,L     */ => Instruction::SET(2, Operand::Register(L)),
        0xD6 /* SET 2,(HL)  */ => Instruction::SET(2, Operand::Address(HL)),
        0xD7 /* SET 2,A     */ => Instruction::SET(2, Operand::Register(A)),
        0xD8 /* SET 3,B     */ => Instruction::SET(3, Operand::Register(B)),
        0xD9 /* SET 3,C     */ => Instruction::SET(3, Operand::Register(C)),
        0xDA /* SET 3,D     */ => Instruction::SET(3, Operand::Register(D)),
        0xDB /* SET 3,E     */ => Instruction::SET(3, Operand::Register(E)),
        0xDC /* SET 3,H     */ => Instruction::SET(3, Operand::Register(H)),
        0xDD /* SET 3,L     */ => Instruction::SET(3, Operand::Register(L)),
        0xDE /* SET 3,(HL)  */ => Instruction::SET(3, Operand::Address(HL)),
        0xDF /* SET 3,A     */ => Instruction::SET(3, Operand::Register(A)),
        0xE0 /* SET 4,B     */ => Instruction::SET(4, Operand::Register(B)),
        0xE1 /* SET 4,C     */ => Instruction::SET(4, Operand::Register(C)),
        0xE2 /* SET 4,D     */ => Instruction::SET(4, Operand::Register(D)),
        0xE3 /* SET 4,E     */ => Instruction::SET(4, Operand::Register(E)),
        0xE4 /* SET 4,H     */ => Instruction::SET(4, Operand::Register(H)),
        0xE5 /* SET 4,L     */ => Instruction::SET(4, Operand::Register(L)),
        0xE6 /* SET 4,(HL)  */ => Instruction::SET(4, Operand::Address(HL)),
        0xE7 /* SET 4,A     */ => Instruction::SET(4, Operand::Register(A)),
        0xE8 /* SET 5,B     */ => Instruction::SET(5, Operand::Register(B)),
        0xE9 /* SET 5,C     */ => Instruction::SET(5, Operand::Register(C)),
        0xEA /* SET 5,D     */ => Instruction::SET(5, Operand::Register(D)),
        0xEB /* SET 5,E     */ => Instruction::SET(5, Operand::Register(E)),
        0xEC /* SET 5,H     */ => Instruction::SET(5, Operand::Register(H)),
        0xED /* SET 5,L     */ => Instruction::SET(5, Operand::Register(L)),
        0xEE /* SET 5,(HL)  */ => Instruction::SET(5, Operand::Address(HL)),
        0xEF /* SET 5,A     */ => Instruction::SET(5, Operand::Register(A)),
        0xF0 /* SET 6,B     */ => Instruction::SET(6, Operand::Register(B)),
        0xF1 /* SET 6,C     */ => Instruction::SET(6, Operand::Register(C)),
        0xF2 /* SET 6,D     */ => Instruction::SET(6, Operand::Register(D)),
        0xF3 /* SET 6,E     */ => Instruction::SET(6, Operand::Register(E)),
        0xF4 /* SET 6,H     */ => Instruction::SET(6, Operand::Register(H)),
        0xF5 /* SET 6,L     */ => Instruction::SET(6, Operand::Register(L)),
        0xF6 /* SET 6,(HL)  */ => Instruction::SET(6, Operand::Address(HL)),
        0xF7 /* SET 6,A     */ => Instruction::SET(6, Operand::Register(A)),
        0xF8 /* SET 7,B     */ => Instruction::SET(7, Operand::Register(B)),
        0xF9 /* SET 7,C     */ => Instruction::SET(7, Operand::Register(C)),
        0xFA /* SET 7,D     */ => Instruction::SET(7, Operand::Register(D)),
        0xFB /* SET 7,E     */ => Instruction::SET(7, Operand::Register(E)),
        0xFC /* SET 7,H     */ => Instruction::SET(7, Operand::Register(H)),
        0xFD /* SET 7,L     */ => Instruction::SET(7, Operand::Register(L)),
        0xFE /* SET 7,(HL)  */ => Instruction::SET(7, Operand::Address(HL)),
        0xFF /* SET 7,A     */ => Instruction::SET(7, Operand::Register(A)),
        _ => unreachable!("All u8 values handled"),
    }
}
