//! The Gameboy and all its variants have the same CPU at their heart: an 8-bit Z80-like processor produced by Sharp and
//! codenamed LR25902. This processor has 8 8-bit registers (1 is reserved for ALU flags) and a 16-bit memory bus.
//! The CPU is mostly 8-bit, but does sport a handful of 16-bit instructions. For example, pairs of the 8-bit registers
//! can be viewed and manipulated.
//! This module contains everything needed to decode and execute instructions for this CPU.

use crate::interrupt::InterruptController;
use crate::ppu::OamCorruptionType;
use serde::{Deserialize, Serialize};

/// The main Cpu struct, containing all the CPU registers and core CPU state. Many of the CPU instructions modify the
/// registers contained here. Anything else is modified by reading/writing from the external memory bus.
#[derive(Default, Deserialize, Serialize)]
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

/// The instruction set and behaviour of the CPU remained more or less the same from the very first Gameboy until the
/// last Gameboy Color iteration. Sure, the CGB can run in double speed mode, and the memory bus behaves differently
/// across the different models (different RAM banking, registers, SGB functionality, etc). But all the CPU cares about
/// is being able to read/write memory to fetch+decode instructions and execute them.
pub trait Bus {
    fn clock(&mut self);
    fn memory_read(&mut self, addr: u16) -> u8;
    fn memory_write(&mut self, addr: u16, v: u8);
    fn trigger_oam_glitch(&mut self, kind: OamCorruptionType);
    fn interrupt_controller(&mut self) -> &mut InterruptController;

    fn memory_read16(&mut self, addr: u16) -> u16 {
        let mut v = u16::from(self.memory_read(addr));
        v |= u16::from(self.memory_read(addr + 1)) << 8;
        v
    }
    fn memory_write16(&mut self, addr: u16, v: u16) {
        self.memory_write(addr + 1, ((v & 0xFF00) >> 8) as u8);
        self.memory_write(addr, (v & 0xFF) as u8);
    }
}

/// CPU flags contained in the "F" register:
/// Z: Zero flag, N: subtract flag, H: half carry flag, C: carry flag
#[derive(Default, Deserialize, Serialize)]
pub struct Flags {
    pub z: bool, // Bit 7
    pub n: bool, // Bit 6
    pub h: bool, // Bit 5
    pub c: bool, // Bit 4
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
#[derive(Clone, Copy, Debug)]
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
        Default::default()
    }

    /// Returns the current value of an 8-bit CPU register.
    fn register_get(&self, r: Register) -> u8 {
        match r {
            A => self.a,
            B => self.b,
            C => self.c,
            D => self.d,
            E => self.e,
            H => self.h,
            L => self.l,
        }
    }

    /// Sets a new value for an 8-bit CPU register.
    fn register_set(&mut self, r: Register, v: u8) {
        match r {
            A => self.a = v,
            B => self.b = v,
            C => self.c = v,
            D => self.d = v,
            E => self.e = v,
            H => self.h = v,
            L => self.l = v,
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

        u16::from(hi) << 8 | u16::from(lo)
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
    pub fn operand_get<T: Bus>(&mut self, bus: &mut T, o: Operand) -> u8 {
        match o {
            Operand::Register(r) => self.register_get(r),
            Operand::Immediate(d) => d,
            Operand::Address(rr) => {
                let addr = self.register16_get(rr);
                bus.memory_read(addr)
            }
            Operand::AddressInc(rr) => {
                let addr = self.register16_get(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    bus.trigger_oam_glitch(OamCorruptionType::LDI);
                }
                self.register16_set(rr, addr.wrapping_add(1));
                bus.memory_read(addr)
            }
            Operand::AddressDec(rr) => {
                let addr = self.register16_get(rr);
                if addr >= 0xFE00 && addr <= 0xFEFF {
                    bus.trigger_oam_glitch(OamCorruptionType::LDD);
                }
                self.register16_set(rr, addr.wrapping_sub(1));
                bus.memory_read(addr)
            }
            Operand::ImmediateAddress(addr) => bus.memory_read(addr),
            Operand::ImmediateAddressHigh(addr) => bus.memory_read(0xFF00 + u16::from(addr)),
            Operand::AddressHigh(r) => {
                let addr = 0xFF00 + u16::from(self.register_get(r));
                bus.memory_read(addr)
            }
        }
    }

    /// Saves provided 8-bit value into Operand destination.
    pub fn operand_set<T: Bus>(&mut self, bus: &mut T, o: Operand, v: u8) {
        match o {
            Operand::Register(r) => self.register_set(r, v),
            Operand::Immediate(_) => panic!("Attempted to write to immediate operand"),
            Operand::Address(rr) => {
                let addr = self.register16_get(rr);
                bus.memory_write(addr, v)
            }
            Operand::AddressInc(rr) => {
                let addr = self.register16_get(rr);
                self.register16_set(rr, addr.wrapping_add(1));
                bus.memory_write(addr, v)
            }
            Operand::AddressDec(rr) => {
                let addr = self.register16_get(rr);
                self.register16_set(rr, addr.wrapping_sub(1));
                bus.memory_write(addr, v)
            }
            Operand::ImmediateAddress(addr) => bus.memory_write(addr, v),
            Operand::ImmediateAddressHigh(addr) => bus.memory_write(0xFF00 + u16::from(addr), v),
            Operand::AddressHigh(r) => {
                let addr = self.register_get(r);
                bus.memory_write(0xFF00 + u16::from(addr), v)
            }
        }
    }

    /// Resolves the value for a given 16-bit instruction operand.
    fn operand_get16<T: Bus>(&mut self, bus: &mut T, o: Operand16) -> u16 {
        match o {
            Operand16::Register(r) => self.register16_get(r),
            Operand16::Immediate(d) => d,
            Operand16::ImmediateAddress(addr) => bus.memory_read16(addr),
        }
    }

    /// Writes a new value to the target of a 16-bit instruction operand.
    fn operand_set16<T: Bus>(&mut self, bus: &mut T, o: Operand16, v: u16) {
        match o {
            Operand16::Register(r) => self.register16_set(r, v),
            Operand16::Immediate(_) => panic!("Attempted to write to immediate operand"),
            Operand16::ImmediateAddress(addr) => bus.memory_write16(addr, v),
        }
    }

    /// Runs the CPU for a single fetch-decode-execute step. The actual number of cycles this will take depends on which
    /// instruction is executed.
    pub fn step<T: Bus>(&mut self, bus: &mut T) {
        // If the CPU is currently halted, we need to pump a clock cycle of the other hardware, so that if there's a new
        // interrupt available, we can wake up from HALT and continue on.
        if self.halted {
            bus.clock();
        }

        // Are there any pending interrupts to service?
        if self.process_interrupts(bus) {
            // We processed an interrupt. We return now before decoding and executing the next instruction.
            // This gives debuggers a chance to examine the new PC location and handle any potential breakpoints.
            return;
        }

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
            let v = bus.memory_read(self.pc);
            self.pc = self.pc.wrapping_add(1);
            v
        });

        match instruction {
            ADC(o) => self.add(bus, o, true),
            ADD(o) => self.add(bus, o, false),
            ADD16(rr) => self.add16(bus, rr),
            ADD_SP_r8(r8) => self.add_sp_r8(bus, r8),
            AND(o) => self.bitwise(bus, BitwiseOp::AND, o),
            BIT(b, o) => self.bit(bus, b, o),
            CALL(cc, addr) => self.call(bus, cc, addr),
            CCF => self.ccf(),
            CP(o) => self.sub(bus, o, false, false),
            CPL => self.cpl(),
            DAA => self.daa(),
            DEC(o) => self.dec(bus, o),
            DEC16(rr) => self.dec16(bus, rr),
            DI => self.set_ime(false),
            EI => self.set_ime(true),
            HALT => self.halt(),
            INC(o) => self.inc(bus, o),
            INC16(rr) => self.inc16(bus, rr),
            JP(cc, o) => self.jp(bus, cc, o),
            JR(cc, r8) => self.jr(bus, cc, r8),
            LD(lhs, rhs) => self.ld(bus, lhs, rhs),
            LD16(lhs @ Operand16::Register(SP), rhs @ Operand16::Register(HL)) => self.ld16(bus, lhs, rhs, true),
            LD16(lhs, rhs) => self.ld16(bus, lhs, rhs, false),
            LD_HL_SP(d) => self.ld_hl_sp(bus, d),
            NOP => {}
            OR(o) => self.bitwise(bus, BitwiseOp::OR, o),
            POP(rr) => self.pop(bus, rr),
            PUSH(rr) => self.push(bus, rr),
            RES(b, o) => self.setbit(bus, b, o, false),
            RET(cc) => self.ret(bus, cc, false),
            RETI => self.ret(bus, None, true),
            RL(o) => self.rl(bus, o, true, true),
            RLA => self.rl(bus, Operand::Register(A), false, true),
            RLC(o) => self.rlc(bus, o, true),
            RLCA => self.rlc(bus, Operand::Register(A), false),
            RR(o) => self.rr(bus, o, true),
            RRA => self.rr(bus, Operand::Register(A), false),
            RRC(o) => self.rrc(bus, o, true),
            RRCA => self.rrc(bus, Operand::Register(A), false),
            RST(vec) => self.rst(bus, vec),
            SBC(o) => self.sub(bus, o, true, true),
            SCF => self.scf(),
            SET(b, o) => self.setbit(bus, b, o, true),
            SLA(o) => self.rl(bus, o, true, false),
            SRA(o) => self.shift_right(bus, o, true),
            SRL(o) => self.shift_right(bus, o, false),
            STOP => self.stop(),
            SUB(o) => self.sub(bus, o, false, true),
            SWAP(o) => self.swap(bus, o),
            XOR(o) => self.bitwise(bus, BitwiseOp::XOR, o),
            Invalid(_) => panic!("Unhandled instruction: {:?}", instruction),
        }
    }

    /// Process any pending interrupts. Called before the CPU fetches the next instruction to execute.
    /// Returns true if an interrupt was successfully serviced.
    fn process_interrupts<T: Bus>(&mut self, bus: &mut T) -> bool {
        // We can bail quickly if there's no interrupts to process.
        if !bus.interrupt_controller().pending {
            return false;
        }

        let interrupt = bus.interrupt_controller().next_interrupt();

        // If there are interrupts to process, we clear HALT state, even if IME is disabled.
        self.halted = false;

        if !self.ime {
            // If IME isn't enabled though, we don't actually process any interrupts.
            return false;
        }

        // Interrupt handling needs 3 internal cycles to do interrupt-y stuff.
        bus.clock();
        bus.clock();
        bus.clock();

        // Here's an interesting quirk. If the stack pointer was set to 0000 or 0001, then the push we just did
        // above would have overwritten IE. If the new IE value no longer matches the interrupt we were processing,
        // then we cancel that interrupt and set PC to 0. We then try and find another interrupt.
        // If there isn't one, we end up running code from 0000. Crazy.
        let pc = self.pc;
        let mut sp = self.sp;
        sp = sp.wrapping_sub(1);
        bus.memory_write(sp, ((pc & 0xFF00) >> 8) as u8);
        // This is where we capture what IE is after pushing the upper byte. Pushing the lower byte might
        // also overwrite IE, but in that case we ignore that occurring.
        let still_pending = {
            let interrupts = bus.interrupt_controller();
            interrupts.pending && interrupts.next_interrupt() == interrupt
        };
        sp = sp.wrapping_sub(1);
        bus.memory_write(sp, pc as u8);
        self.sp = self.sp.wrapping_sub(2);

        if !still_pending {
            self.pc = 0;
            // Okay so this interrupt didn't go so good. Let's see if there's another one.
            let res = self.process_interrupts(bus);
            // Regardless of what happens in the next try, IME needs to be disabled.
            self.ime = false;
            return res;
        }

        self.pc = interrupt.handler_addr();
        bus.interrupt_controller().clear(interrupt);
        self.ime = false;
        true
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

    fn halt(&mut self) {
        // TODO: pretty sure I need to check interrupt states here.
        self.halted = true;
    }

    fn stop(&mut self) {
        // TODO: this should be more than a noop.
        self.pc = self.pc.wrapping_add(1);
    }

    fn inc<T: Bus>(&mut self, bus: &mut T, o: Operand) {
        let v = self.operand_get(bus, o).wrapping_add(1);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = v.trailing_zeros() >= 4;
        self.operand_set(bus, o, v);
    }

    fn inc16<T: Bus>(&mut self, bus: &mut T, reg: Register16) {
        let v = self.register16_get(reg);
        if v >= 0xFE00 && v <= 0xFEFF {
            bus.trigger_oam_glitch(OamCorruptionType::READ);
        }
        self.register16_set(reg, v.wrapping_add(1));
        bus.clock();
    }

    fn dec<T: Bus>(&mut self, bus: &mut T, o: Operand) {
        let v = self.operand_get(bus, o).wrapping_sub(1);
        self.f.z = v == 0;
        self.f.n = true;
        self.f.h = v & 0x0F == 0x0F;
        self.operand_set(bus, o, v);
    }

    fn dec16<T: Bus>(&mut self, bus: &mut T, r: Register16) {
        let v = self.register16_get(r);
        if v >= 0xFE00 && v <= 0xFEFF {
            bus.trigger_oam_glitch(OamCorruptionType::READ);
        }
        self.register16_set(r, v.wrapping_sub(1));
        bus.clock();
    }

    fn add<T: Bus>(&mut self, bus: &mut T, o: Operand, carry: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let old = self.a;
        let v = self.operand_get(bus, o);
        let new = old.wrapping_add(v).wrapping_add(carry);
        self.a = new;
        self.f.z = new == 0;
        self.f.n = false;
        self.f.h = (((old & 0xF) + (v & 0xF)) + carry) & 0x10 == 0x10;
        self.f.c = u16::from(old) + u16::from(v) + u16::from(carry) > 0xFF;
    }

    fn add16<T: Bus>(&mut self, bus: &mut T, r: Register16) {
        let hl = self.register16_get(HL);
        let v = self.register16_get(r);
        let (new_hl, overflow) = hl.overflowing_add(v);
        self.register16_set(HL, new_hl);

        bus.clock();

        self.f.n = false;
        self.f.h = ((hl & 0xFFF) + (v & 0xFFF)) & 0x1000 > 0;
        self.f.c = overflow;
    }

    fn add_sp_r8<T: Bus>(&mut self, bus: &mut T, d: i8) {
        let d = i16::from(d) as u16;
        let sp = self.sp;

        self.sp = sp.wrapping_add(d as i16 as u16);

        bus.clock();
        bus.clock();

        self.f.z = false;
        self.f.n = false;
        self.f.h = ((sp & 0xF) + (d & 0xF)) & 0x10 > 0;
        self.f.c = ((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0;
    }

    fn sub<T: Bus>(&mut self, bus: &mut T, o: Operand, carry: bool, store: bool) {
        let carry = if carry && self.f.c { 1 } else { 0 };

        let a = self.a;
        let v = self.operand_get(bus, o);
        let new_a = a.wrapping_sub(v).wrapping_sub(carry);
        if store {
            self.a = new_a;
        }

        self.f.z = new_a == 0;
        self.f.n = true;
        self.f.h = u16::from(a & 0xF) < u16::from(v & 0xF) + u16::from(carry);
        self.f.c = u16::from(a) < u16::from(v) + u16::from(carry);
    }

    fn ld<T: Bus>(&mut self, bus: &mut T, lhs: Operand, rhs: Operand) {
        let v = self.operand_get(bus, rhs);
        self.operand_set(bus, lhs, v);
    }

    fn ld16<T: Bus>(&mut self, bus: &mut T, lhs: Operand16, rhs: Operand16, extra_clock: bool) {
        let v = self.operand_get16(bus, rhs);
        self.operand_set16(bus, lhs, v);

        // In the specific case of loading a 16bit reg into another 16bit reg, this consumes another CPU cycle. I don't
        // really understand why, since in every other case the cost of reading/writing a 16bit reg appears to be free.
        if extra_clock {
            bus.clock();
        }
    }

    fn ld_hl_sp<T: Bus>(&mut self, bus: &mut T, d: i8) {
        let sp = self.sp;
        let d = i16::from(d) as u16;
        let v = sp.wrapping_add(d);
        self.register16_set(HL, v);
        self.f.reset();
        self.f.h = ((sp & 0xF) + (d & 0xF)) & 0x10 > 0;
        self.f.c = ((sp & 0xFF) + (d & 0xFF)) & 0x100 > 0;
        bus.clock();
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

    fn bitwise<T: Bus>(&mut self, bus: &mut T, op: BitwiseOp, o: Operand) {
        let a = self.a;
        let v = self.operand_get(bus, o);
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

    fn bit<T: Bus>(&mut self, bus: &mut T, b: u8, o: Operand) {
        let v = self.operand_get(bus, o) & (1 << b);
        self.f.z = v == 0;
        self.f.n = false;
        self.f.h = true;
    }

    fn setbit<T: Bus>(&mut self, bus: &mut T, b: u8, o: Operand, on: bool) {
        let v = self.operand_get(bus, o);
        self.operand_set(bus, o, if on { v | 1 << b } else { v & !(1 << b) });
    }

    fn rl<T: Bus>(&mut self, bus: &mut T, o: Operand, set_zero: bool, preserve_lsb: bool) {
        let v = self.operand_get(bus, o);
        let lsb = if preserve_lsb && self.f.c { 1 } else { 0 };
        let carry = v & 0x80 > 0;
        let v = v << 1 | lsb;
        self.operand_set(bus, o, v);
        self.f.reset();
        self.f.z = set_zero && v == 0;
        self.f.c = carry;
    }

    fn rlc<T: Bus>(&mut self, bus: &mut T, o: Operand, extended: bool) {
        let v = self.operand_get(bus, o);
        let carry = v & 0x80 > 0;
        let lsb = if carry { 1 } else { 0 };
        let v = v << 1 | lsb;
        self.operand_set(bus, o, v);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn rr<T: Bus>(&mut self, bus: &mut T, o: Operand, extended: bool) {
        let v = self.operand_get(bus, o);
        let msb = if self.f.c { 0x80 } else { 0 };
        let carry = v & 0x1 > 0;
        let v = v >> 1 | msb;
        self.operand_set(bus, o, v);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn rrc<T: Bus>(&mut self, bus: &mut T, o: Operand, extended: bool) {
        let v = self.operand_get(bus, o);
        let carry = v & 0x1 > 0;
        let msb = if carry { 0x80 } else { 0 };
        let v = v >> 1 | msb;
        self.operand_set(bus, o, v);
        self.f.reset();
        self.f.z = extended && v == 0;
        self.f.c = carry;
    }

    fn shift_right<T: Bus>(&mut self, bus: &mut T, o: Operand, preserve_msb: bool) {
        let v = self.operand_get(bus, o);
        let carry = v & 0x01 > 0;
        let preserve = if preserve_msb { v & 0x80 } else { 0 };
        let v = v >> 1 | preserve;
        self.operand_set(bus, o, v);
        self.f.reset();
        self.f.z = v == 0;
        self.f.c = carry;
    }

    fn swap<T: Bus>(&mut self, bus: &mut T, o: Operand) {
        let v = self.operand_get(bus, o);
        let v = ((v & 0xF) << 4) | ((v & 0xF0) >> 4);
        self.operand_set(bus, o, v);
        self.f.reset();
        self.f.z = v == 0;
    }

    fn stack_push<T: Bus>(&mut self, bus: &mut T, v: u16) {
        if self.sp >= 0xFE00 && self.sp <= 0xFEFF {
            bus.trigger_oam_glitch(OamCorruptionType::PUSH);
        }

        self.sp = self.sp.wrapping_sub(2);
        let sp = self.sp;
        bus.memory_write16(sp, v);
    }

    fn stack_pop<T: Bus>(&mut self, bus: &mut T) -> u16 {
        if self.sp >= 0xFDFF && self.sp <= 0xFEFE {
            bus.trigger_oam_glitch(OamCorruptionType::POP);
        }

        let addr = self.sp;
        let v = bus.memory_read16(addr);
        self.sp = self.sp.wrapping_add(2);

        v
    }

    fn push<T: Bus>(&mut self, bus: &mut T, r: Register16) {
        let v = self.register16_get(r);
        bus.clock();
        self.stack_push(bus, v);
    }

    fn pop<T: Bus>(&mut self, bus: &mut T, r: Register16) {
        let mut v = self.stack_pop(bus);
        if let AF = r {
            // Reset bits 0-3 in F.
            v &= 0xFFF0;
        }
        self.register16_set(r, v);
    }

    fn push_and_jump<T: Bus>(&mut self, bus: &mut T, addr: u16) {
        let pc = self.pc;
        self.stack_push(bus, pc);
        self.pc = addr;
    }

    fn call<T: Bus>(&mut self, bus: &mut T, cc: Option<FlagCondition>, addr: u16) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }
        bus.clock();
        self.push_and_jump(bus, addr);
    }

    fn jp<T: Bus>(&mut self, bus: &mut T, cc: Option<FlagCondition>, o: Operand16) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }

        let addr = self.operand_get16(bus, o);
        self.pc = addr;

        if let Operand16::Register(HL) = o {
            // For some reason, JP (HL) doesn't cause an extra clock cycle. Very mysterious.
        } else {
            bus.clock();
        }
    }

    fn jr<T: Bus>(&mut self, bus: &mut T, cc: Option<FlagCondition>, n: u8) {
        if !self.f.check_jmp_condition(cc) {
            return;
        }
        bus.clock();
        self.pc = self.pc.wrapping_add(i16::from(n as i8) as u16);
    }

    fn ret<T: Bus>(&mut self, bus: &mut T, cc: Option<FlagCondition>, ei: bool) {
        if !self.f.check_jmp_condition(cc) {
            bus.clock();
            return;
        }
        if cc.is_some() {
            bus.clock();
        }
        if ei {
            // RETI immediately enables IME, it's not deferred like eith an EI or DI call.
            self.ime = true;
        }
        let pc = self.stack_pop(bus);
        bus.clock();
        self.pc = pc;
    }

    fn rst<T: Bus>(&mut self, bus: &mut T, a: u8) {
        bus.clock();
        self.push_and_jump(bus, u16::from(a));
    }
}

impl Flags {
    /// Converts the CPU flags into an 8bit value. Used by instructions that operate on F register.
    pub fn pack(&self) -> u8 {
        (if self.z { 0b1000_0000 } else { 0 })
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
    fn reset(&mut self) {
        self.z = false;
        self.n = false;
        self.h = false;
        self.c = false;
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
            Operand::Immediate(n8) => write!(f, "${:02x}", n8),
            Operand::Address(rr) => write!(f, "[{:?}]", rr),
            Operand::AddressInc(rr) => write!(f, "[{:?}+]", rr),
            Operand::AddressDec(rr) => write!(f, "[{:?}-]", rr),
            Operand::ImmediateAddress(n16) => write!(f, "[${:04x}]", n16),
            Operand::ImmediateAddressHigh(n8) => write!(f, "[$ff00+${:02x}]", n8),
            Operand::AddressHigh(r) => write!(f, "[$ff00+{:?}]", r),
        }
    }
}

impl std::fmt::Display for Operand16 {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            Operand16::Register(r) => write!(f, "{:?}", r),
            Operand16::Immediate(n16) => write!(f, "${:04x}", n16),
            Operand16::ImmediateAddress(addr) => write!(f, "[${:04x}]", addr),
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
            ADD_SP_r8(r8) => write!(f, "add sp, ${:02x}", r8),
            AND(o) => write!(f, "and {}", o),
            BIT(b, o) => write!(f, "bit {}, {}", b, o),
            CALL(None, addr) => write!(f, "call ${:04x}", addr),
            CALL(cc, addr) => write!(f, "call {}, ${:04x}", cc.unwrap(), addr),
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
            LD_HL_SP(e8) => write!(f, "ld hl, sp+${:02x}", e8),
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
            RST(vec) => write!(f, "rst ${:02x}", vec),
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
            Invalid(op) => write!(f, "unknown opcode ${:02x}", op),
        }
    }
}

impl Operand {
    /// Returns how many bytes this operand is encoded in, between 0 and 2.
    pub fn size(self) -> u8 {
        match self {
            Operand::Register(_) => 0,
            Operand::Immediate(_) => 1,
            Operand::Address(_) => 0,
            Operand::AddressInc(_) => 0,
            Operand::AddressDec(_) => 0,
            Operand::ImmediateAddress(_) => 2,
            Operand::ImmediateAddressHigh(_) => 1,
            Operand::AddressHigh(_) => 0,
        }
    }
}

impl Operand16 {
    /// Returns how many bytes this operand is encoded in, between 0 and 2.
    pub fn size(self) -> u8 {
        match self {
            Operand16::Register(_) => 0,
            Operand16::Immediate(_) => 2,
            Operand16::ImmediateAddress(_) => 2,
        }
    }
}

impl Instruction {
    /// Returns true if this instruction is the LD B,B instruction
    pub fn is_debug_breakpoint(&self) -> bool {
        if let LD(Operand::Register(B), Operand::Register(B)) = self {
            true
        } else {
            false
        }
    }
    /// Indicates if current instruction affects program execution flow (CALL, JP, JR, RET, RETI, RST)
    pub fn is_flow_control(&self) -> bool {
        match self {
            CALL(_, _) | JP(_, _) | JR(_, _) | RET(_) | RETI | RST(_) => true,
            _ => false,
        }
    }

    /// Returns how many bytes of memory this instruction is encoded in.
    pub fn size(&self) -> u8 {
        1 + // all instructions are at least 1 byte for the opcode.
            match self {
                // Beyond that, some instruction sizes depend on the operand.
                ADC(o) => o.size(),
                ADD(o) => o.size(),
                AND(o) => o.size(),
                CP(o) => o.size(),
                JP(_, o) => o.size(),
                LD(lhs, rhs) => lhs.size() + rhs.size(),
                LD16(lhs, rhs) => lhs.size() + rhs.size(),
                OR(o) => o.size(),
                SBC(o) => o.size(),
                SUB(o) => o.size(),
                XOR(o) => o.size(),

                // Some instructions are implicitly bigger
                ADD_SP_r8(_) => 1,
                CALL(_, _) => 2,
                JR(_, _) => 1,
                LD_HL_SP(_) => 1,

                // And all the PREFIX CB commands are one byte larger for the $CB prefix opcode.
                BIT(_, _)       => 1,
                RES(_, _)   => 1,
                RL(_)       => 1,
                RLC(_) => 1,
                RR(_) => 1,
                RRC(_) => 1,
                SET(_,_) => 1,
                SLA(_) => 1,
                SRA(_) => 1,
                SRL(_) => 1,
                SWAP(_) => 1,

                // Everything else is just the 1 byte.
                _ => 0,
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
        0x00 /* NOP         */ => NOP,
        0x01 /* LD BC,d16   */ => LD16(Operand16::Register(BC), Operand16::Immediate(fetch16!())),
        0x02 /* LD (BC),A   */ => LD(Operand::Address(BC), Operand::Register(A)),
        0x03 /* INC BC      */ => INC16(BC),
        0x04 /* INC B       */ => INC(Operand::Register(B)),
        0x05 /* DEC B       */ => DEC(Operand::Register(B)),
        0x06 /* LD B,d8     */ => LD(Operand::Register(B), Operand::Immediate(fetch())),
        0x07 /* RLCA        */ => RLCA,
        0x08 /* LD (a16),SP */ => LD16(Operand16::ImmediateAddress(fetch16!()), Operand16::Register(SP)),
        0x09 /* ADD HL,BC   */ => ADD16(BC),
        0x0A /* LD A,(BC)   */ => LD(Operand::Register(A), Operand::Address(BC)),
        0x0B /* DEC BC      */ => DEC16(BC),
        0x0C /* INC C       */ => INC(Operand::Register(C)),
        0x0D /* DEC C       */ => DEC(Operand::Register(C)),
        0x0E /* LD C,d8     */ => LD(Operand::Register(C), Operand::Immediate(fetch())),
        0x0F /* RRCA        */ => RRCA,
        0x10 /* STOP        */ => STOP,
        0x11 /* LD DE,d16   */ => LD16(Operand16::Register(DE), Operand16::Immediate(fetch16!())),
        0x12 /* LD (DE),A   */ => LD(Operand::Address(DE), Operand::Register(A)),
        0x13 /* INC DE      */ => INC16(DE),
        0x14 /* INC D       */ => INC(Operand::Register(D)),
        0x15 /* DEC D       */ => DEC(Operand::Register(D)),
        0x16 /* LD D,d8     */ => LD(Operand::Register(D), Operand::Immediate(fetch())),
        0x17 /* RLA         */ => RLA,
        0x18 /* JR r8       */ => JR(None, fetch()),
        0x19 /* ADD HL,DE   */ => ADD16(DE),
        0x1A /* LD A,(DE)   */ => LD(Operand::Register(A), Operand::Address(DE)),
        0x1B /* DEC DE      */ => DEC16(DE),
        0x1C /* INC E       */ => INC(Operand::Register(E)),
        0x1D /* DEC E       */ => DEC(Operand::Register(E)),
        0x1E /* LD E,d8     */ => LD(Operand::Register(E), Operand::Immediate(fetch())),
        0x1F /* RRA         */ => RRA,
        0x20 /* JR NZ,r8    */ => JR(Some(FlagCondition::NZ), fetch()),
        0x21 /* LD HL,d16   */ => LD16(Operand16::Register(HL), Operand16::Immediate(fetch16!())),
        0x22 /* LD (HL+),A  */ => LD(Operand::AddressInc(HL), Operand::Register(A)),
        0x23 /* INC HL      */ => INC16(HL),
        0x24 /* INC H       */ => INC(Operand::Register(H)),
        0x25 /* DEC H       */ => DEC(Operand::Register(H)),
        0x26 /* LD H,d8     */ => LD(Operand::Register(H), Operand::Immediate(fetch())),
        0x27 /* DAA         */ => DAA,
        0x28 /* JR Z,r8     */ => JR(Some(FlagCondition::Z), fetch()),
        0x29 /* ADD HL,HL   */ => ADD16(HL),
        0x2A /* LD A,(HL+)  */ => LD(Operand::Register(A), Operand::AddressInc(HL)),
        0x2B /* DEC HL      */ => DEC16(HL),
        0x2C /* INC L       */ => INC(Operand::Register(L)),
        0x2D /* DEC L       */ => DEC(Operand::Register(L)),
        0x2E /* LD L,d8     */ => LD(Operand::Register(L), Operand::Immediate(fetch())),
        0x2F /* CPL         */ => CPL,
        0x30 /* JR NC,r8    */ => JR(Some(FlagCondition::NC), fetch()),
        0x31 /* LD SP,d16   */ => LD16(Operand16::Register(SP), Operand16::Immediate(fetch16!())),
        0x32 /* LD (HL-), A */ => LD(Operand::AddressDec(HL), Operand::Register(A)),
        0x33 /* INC SP      */ => INC16(SP),
        0x34 /* INC (HL)    */ => INC(Operand::Address(HL)),
        0x35 /* DEC (HL)    */ => DEC(Operand::Address(HL)),
        0x36 /* LD (HL),d8  */ => LD(Operand::Address(HL), Operand::Immediate(fetch())),
        0x37 /* SCF         */ => SCF,
        0x38 /* JR C,r8     */ => JR(Some(FlagCondition::C), fetch()),
        0x39 /* ADD HL,SP   */ => ADD16(SP),
        0x3A /* LD A, (HL-) */ => LD(Operand::Register(A), Operand::AddressDec(HL)),
        0x3B /* DEC SP      */ => DEC16(SP),
        0x3C /* INC A       */ => INC(Operand::Register(A)),
        0x3D /* DEC A       */ => DEC(Operand::Register(A)),
        0x3E /* LD A,d8     */ => LD(Operand::Register(A), Operand::Immediate(fetch())),
        0x3F /* CCF         */ => CCF,
        0x40 /* LD B,B      */ => LD(Operand::Register(B), Operand::Register(B)),
        0x41 /* LD B,C      */ => LD(Operand::Register(B), Operand::Register(C)),
        0x42 /* LD B,D      */ => LD(Operand::Register(B), Operand::Register(D)),
        0x43 /* LD B,E      */ => LD(Operand::Register(B), Operand::Register(E)),
        0x44 /* LD B,H      */ => LD(Operand::Register(B), Operand::Register(H)),
        0x45 /* LD B,L      */ => LD(Operand::Register(B), Operand::Register(L)),
        0x46 /* LD B,(HL)   */ => LD(Operand::Register(B), Operand::Address(HL)),
        0x47 /* LD B,A      */ => LD(Operand::Register(B), Operand::Register(A)),
        0x48 /* LD C,B      */ => LD(Operand::Register(C), Operand::Register(B)),
        0x49 /* LD C,C      */ => LD(Operand::Register(C), Operand::Register(C)),
        0x4A /* LD C,D      */ => LD(Operand::Register(C), Operand::Register(D)),
        0x4B /* LD C,E      */ => LD(Operand::Register(C), Operand::Register(E)),
        0x4C /* LD C,H      */ => LD(Operand::Register(C), Operand::Register(H)),
        0x4D /* LD C,L      */ => LD(Operand::Register(C), Operand::Register(L)),
        0x4E /* LD C,(HL)   */ => LD(Operand::Register(C), Operand::Address(HL)),
        0x4F /* LD C,A      */ => LD(Operand::Register(C), Operand::Register(A)),
        0x50 /* LD D,B      */ => LD(Operand::Register(D), Operand::Register(B)),
        0x51 /* LD D,C      */ => LD(Operand::Register(D), Operand::Register(C)),
        0x52 /* LD D,D      */ => LD(Operand::Register(D), Operand::Register(D)),
        0x53 /* LD D,E      */ => LD(Operand::Register(D), Operand::Register(E)),
        0x54 /* LD D,H      */ => LD(Operand::Register(D), Operand::Register(H)),
        0x55 /* LD D,L      */ => LD(Operand::Register(D), Operand::Register(L)),
        0x56 /* LD D,(HL)   */ => LD(Operand::Register(D), Operand::Address(HL)),
        0x57 /* LD D,A      */ => LD(Operand::Register(D), Operand::Register(A)),
        0x58 /* LD E,B      */ => LD(Operand::Register(E), Operand::Register(B)),
        0x59 /* LD E,C      */ => LD(Operand::Register(E), Operand::Register(C)),
        0x5A /* LD E,D      */ => LD(Operand::Register(E), Operand::Register(D)),
        0x5B /* LD E,E      */ => LD(Operand::Register(E), Operand::Register(E)),
        0x5C /* LD E,H      */ => LD(Operand::Register(E), Operand::Register(H)),
        0x5D /* LD E,L      */ => LD(Operand::Register(E), Operand::Register(L)),
        0x5E /* LD E,(HL)   */ => LD(Operand::Register(E), Operand::Address(HL)),
        0x5F /* LD E,A      */ => LD(Operand::Register(E), Operand::Register(A)),
        0x60 /* LD H,B      */ => LD(Operand::Register(H), Operand::Register(B)),
        0x61 /* LD H,C      */ => LD(Operand::Register(H), Operand::Register(C)),
        0x62 /* LD H,D      */ => LD(Operand::Register(H), Operand::Register(D)),
        0x63 /* LD H,E      */ => LD(Operand::Register(H), Operand::Register(E)),
        0x64 /* LD H,H      */ => LD(Operand::Register(H), Operand::Register(H)),
        0x65 /* LD H,L      */ => LD(Operand::Register(H), Operand::Register(L)),
        0x66 /* LD H,(HL)   */ => LD(Operand::Register(H), Operand::Address(HL)),
        0x67 /* LD H,A      */ => LD(Operand::Register(H), Operand::Register(A)),
        0x68 /* LD L,B      */ => LD(Operand::Register(L), Operand::Register(B)),
        0x69 /* LD L,C      */ => LD(Operand::Register(L), Operand::Register(C)),
        0x6A /* LD L,D      */ => LD(Operand::Register(L), Operand::Register(D)),
        0x6B /* LD L,E      */ => LD(Operand::Register(L), Operand::Register(E)),
        0x6C /* LD L,H      */ => LD(Operand::Register(L), Operand::Register(H)),
        0x6D /* LD L,L      */ => LD(Operand::Register(L), Operand::Register(L)),
        0x6E /* LD L,(HL)   */ => LD(Operand::Register(L), Operand::Address(HL)),
        0x6F /* LD L,A      */ => LD(Operand::Register(L), Operand::Register(A)),
        0x70 /* LD (HL),B   */ => LD(Operand::Address(HL), Operand::Register(B)),
        0x71 /* LD (HL),C   */ => LD(Operand::Address(HL), Operand::Register(C)),
        0x72 /* LD (HL),D   */ => LD(Operand::Address(HL), Operand::Register(D)),
        0x73 /* LD (HL),E   */ => LD(Operand::Address(HL), Operand::Register(E)),
        0x74 /* LD (HL),H   */ => LD(Operand::Address(HL), Operand::Register(H)),
        0x75 /* LD (HL),L   */ => LD(Operand::Address(HL), Operand::Register(L)),
        0x76 /* HALT        */ => HALT,
        0x77 /* LD (HL),A   */ => LD(Operand::Address(HL), Operand::Register(A)),
        0x78 /* LD A,B      */ => LD(Operand::Register(A), Operand::Register(B)),
        0x79 /* LD A,C      */ => LD(Operand::Register(A), Operand::Register(C)),
        0x7A /* LD A,D      */ => LD(Operand::Register(A), Operand::Register(D)),
        0x7B /* LD A,E      */ => LD(Operand::Register(A), Operand::Register(E)),
        0x7C /* LD A,H      */ => LD(Operand::Register(A), Operand::Register(H)),
        0x7D /* LD A,L      */ => LD(Operand::Register(A), Operand::Register(L)),
        0x7E /* LD A,(HL)   */ => LD(Operand::Register(A), Operand::Address(HL)),
        0x7F /* LD A,A      */ => LD(Operand::Register(A), Operand::Register(A)),
        0x80 /* ADD A,B     */ => ADD(Operand::Register(B)),
        0x81 /* ADD A,C     */ => ADD(Operand::Register(C)),
        0x82 /* ADD A,D     */ => ADD(Operand::Register(D)),
        0x83 /* ADD A,E     */ => ADD(Operand::Register(E)),
        0x84 /* ADD A,H     */ => ADD(Operand::Register(H)),
        0x85 /* ADD A,L     */ => ADD(Operand::Register(L)),
        0x86 /* ADD A,(HL)  */ => ADD(Operand::Address(HL)),
        0x87 /* ADD A,A     */ => ADD(Operand::Register(A)),
        0x88 /* ADC A,B     */ => ADC(Operand::Register(B)),
        0x89 /* ADC A,C     */ => ADC(Operand::Register(C)),
        0x8A /* ADC A,D     */ => ADC(Operand::Register(D)),
        0x8B /* ADC A,E     */ => ADC(Operand::Register(E)),
        0x8C /* ADC A,H     */ => ADC(Operand::Register(H)),
        0x8D /* ADC A,L     */ => ADC(Operand::Register(L)),
        0x8E /* ADC A,(HL)  */ => ADC(Operand::Address(HL)),
        0x8F /* ADC A,A     */ => ADC(Operand::Register(A)),
        0x90 /* SUB B       */ => SUB(Operand::Register(B)),
        0x91 /* SUB C       */ => SUB(Operand::Register(C)),
        0x92 /* SUB D       */ => SUB(Operand::Register(D)),
        0x93 /* SUB E       */ => SUB(Operand::Register(E)),
        0x94 /* SUB H       */ => SUB(Operand::Register(H)),
        0x95 /* SUB L       */ => SUB(Operand::Register(L)),
        0x96 /* SUB (HL)    */ => SUB(Operand::Address(HL)),
        0x97 /* SUB A       */ => SUB(Operand::Register(A)),
        0x98 /* SBC A,B     */ => SBC(Operand::Register(B)),
        0x99 /* SBC A,C     */ => SBC(Operand::Register(C)),
        0x9A /* SBC A,D     */ => SBC(Operand::Register(D)),
        0x9B /* SBC A,E     */ => SBC(Operand::Register(E)),
        0x9C /* SBC A,H     */ => SBC(Operand::Register(H)),
        0x9D /* SBC A,L     */ => SBC(Operand::Register(L)),
        0x9E /* SBC A,(HL)  */ => SBC(Operand::Address(HL)),
        0x9F /* SBC A,A     */ => SBC(Operand::Register(A)),
        0xA0 /* AND B       */ => AND(Operand::Register(B)),
        0xA1 /* AND C       */ => AND(Operand::Register(C)),
        0xA2 /* AND D       */ => AND(Operand::Register(D)),
        0xA3 /* AND E       */ => AND(Operand::Register(E)),
        0xA4 /* AND H       */ => AND(Operand::Register(H)),
        0xA5 /* AND L       */ => AND(Operand::Register(L)),
        0xA6 /* AND (HL)    */ => AND(Operand::Address(HL)),
        0xA7 /* AND A       */ => AND(Operand::Register(A)),
        0xA8 /* XOR B       */ => XOR(Operand::Register(B)),
        0xA9 /* XOR C       */ => XOR(Operand::Register(C)),
        0xAA /* XOR D       */ => XOR(Operand::Register(D)),
        0xAB /* XOR E       */ => XOR(Operand::Register(E)),
        0xAC /* XOR H       */ => XOR(Operand::Register(H)),
        0xAD /* XOR L       */ => XOR(Operand::Register(L)),
        0xAE /* XOR (HL)    */ => XOR(Operand::Address(HL)),
        0xAF /* XOR A       */ => XOR(Operand::Register(A)),
        0xB0 /* OR B        */ => OR (Operand::Register(B)),
        0xB1 /* OR C        */ => OR (Operand::Register(C)),
        0xB2 /* OR D        */ => OR (Operand::Register(D)),
        0xB3 /* OR E        */ => OR (Operand::Register(E)),
        0xB4 /* OR H        */ => OR (Operand::Register(H)),
        0xB5 /* OR L        */ => OR (Operand::Register(L)),
        0xB6 /* OR (HL)     */ => OR (Operand::Address(HL)),
        0xB7 /* OR A        */ => OR (Operand::Register(A)),
        0xB8 /* CP B        */ => CP (Operand::Register(B)),
        0xB9 /* CP C        */ => CP (Operand::Register(C)),
        0xBA /* CP D        */ => CP (Operand::Register(D)),
        0xBB /* CP E        */ => CP (Operand::Register(E)),
        0xBC /* CP H        */ => CP (Operand::Register(H)),
        0xBD /* CP L        */ => CP (Operand::Register(L)),
        0xBE /* CP (HL)     */ => CP (Operand::Address(HL)),
        0xBF /* CP A        */ => CP (Operand::Register(A)),
        0xC0 /* RET NZ      */ => RET(Some(FlagCondition::NZ)),
        0xC1 /* POP BC      */ => POP(BC),
        0xC2 /* JP NZ,a16   */ => JP(Some(FlagCondition::NZ), Operand16::Immediate(fetch16!())),
        0xC3 /* JP a16      */ => JP(None, Operand16::Immediate(fetch16!())),
        0xC4 /* CALL NZ,a16 */ => CALL(Some(FlagCondition::NZ), fetch16!()),
        0xC5 /* PUSH BC     */ => PUSH(BC),
        0xC6 /* ADD A,d8    */ => ADD(Operand::Immediate(fetch())),
        0xC7 /* RST 00H     */ => RST(0x00),
        0xC8 /* RET Z       */ => RET(Some(FlagCondition::Z)),
        0xC9 /* RET         */ => RET(None),
        0xCA /* JP Z,a16    */ => JP(Some(FlagCondition::Z), Operand16::Immediate(fetch16!())),
        0xCB /* PREFIX CB   */ => decode_extended_instruction(fetch),
        0xCC /* CALL Z,a16  */ => CALL(Some(FlagCondition::Z), fetch16!()),
        0xCD /* CALL a16    */ => CALL(None, fetch16!()),
        0xCE /* ADC A,d8    */ => ADC(Operand::Immediate(fetch())),
        0xCF /* RST 08H     */ => RST(0x08),
        0xD0 /* RET NC      */ => RET(Some(FlagCondition::NC)),
        0xD1 /* POP DE      */ => POP(DE),
        0xD2 /* JP NC,a16   */ => JP(Some(FlagCondition::NC), Operand16::Immediate(fetch16!())),
        0xD4 /* CALL NC,a16 */ => CALL(Some(FlagCondition::NC), fetch16!()),
        0xD5 /* PUSH DE     */ => PUSH(DE),
        0xD6 /* SUB d8      */ => SUB(Operand::Immediate(fetch())),
        0xD7 /* RST 10H     */ => RST(0x10),
        0xD8 /* RET C       */ => RET(Some(FlagCondition::C)),
        0xD9 /* RETI        */ => RETI,
        0xDA /* JP C,a16    */ => JP(Some(FlagCondition::C), Operand16::Immediate(fetch16!())),
        0xDC /* CALL C,a16  */ => CALL(Some(FlagCondition::C), fetch16!()),
        0xDE /* SBC A,d8    */ => SBC(Operand::Immediate(fetch())),
        0xDF /* RST 18H     */ => RST(0x18),
        0xE0 /* LDH (a8),A  */ => LD(Operand::ImmediateAddressHigh(fetch()), Operand::Register(A)),
        0xE1 /* POP HL      */ => POP(HL),
        0xE2 /* LD (C),A    */ => LD(Operand::AddressHigh(C), Operand::Register(A)),
        0xE5 /* PUSH HL     */ => PUSH(HL),
        0xE6 /* AND d8      */ => AND(Operand::Immediate(fetch())),
        0xE7 /* RST 20H     */ => RST(0x20),
        0xE8 /* ADD SP,r8   */ => ADD_SP_r8(fetch() as i8),
        0xE9 /* JP (HL)     */ => JP(None, Operand16::Register(HL)),
        0xEA /* LD (a16),A  */ => LD(Operand::ImmediateAddress(fetch16!()), Operand::Register(A)),
        0xEE /* XOR d8      */ => XOR(Operand::Immediate(fetch())),
        0xEF /* RST 28H     */ => RST(0x28),
        0xF0 /* LDH A,(a8)  */ => LD(Operand::Register(A), Operand::ImmediateAddressHigh(fetch())),
        0xF1 /* POP AF      */ => POP(AF),
        0xF2 /* LD A,(C)    */ => LD(Operand::Register(A), Operand::AddressHigh(C)),
        0xF3 /* DI          */ => DI,
        0xF5 /* PUSH AF     */ => PUSH(AF),
        0xF6 /* OR d8       */ => OR(Operand::Immediate(fetch())),
        0xF7 /* RST 30H     */ => RST(0x30),
        0xF8 /* LD HL,SP+r8 */ => LD_HL_SP(fetch() as i8),
        0xF9 /* LD SP,HL    */ => LD16(Operand16::Register(SP), Operand16::Register(HL)),
        0xFA /* LD A,(a16)  */ => LD(Operand::Register(A), Operand::ImmediateAddress(fetch16!())),
        0xFB /* EI          */ => EI,
        0xFE /* CP d8       */ => CP(Operand::Immediate(fetch())),
        0xFF /* RST 38H     */ => RST(0x38),
        op => Invalid(op)
    }
}

pub fn decode_extended_instruction<T: FnMut() -> u8>(mut fetch: T) -> Instruction {
    match fetch() {
        0x00 /* RLC B       */ => RLC(Operand::Register(B)),
        0x01 /* RLC C       */ => RLC(Operand::Register(C)),
        0x02 /* RLC D       */ => RLC(Operand::Register(D)),
        0x03 /* RLC E       */ => RLC(Operand::Register(E)),
        0x04 /* RLC H       */ => RLC(Operand::Register(H)),
        0x05 /* RLC L       */ => RLC(Operand::Register(L)),
        0x06 /* RLC (HL)    */ => RLC(Operand::Address(HL)),
        0x07 /* RLC A       */ => RLC(Operand::Register(A)),
        0x08 /* RRC B       */ => RRC(Operand::Register(B)),
        0x09 /* RRC C       */ => RRC(Operand::Register(C)),
        0x0A /* RRC D       */ => RRC(Operand::Register(D)),
        0x0B /* RRC E       */ => RRC(Operand::Register(E)),
        0x0C /* RRC H       */ => RRC(Operand::Register(H)),
        0x0D /* RRC L       */ => RRC(Operand::Register(L)),
        0x0E /* RRC (HL)    */ => RRC(Operand::Address(HL)),
        0x0F /* RRC A       */ => RRC(Operand::Register(A)),
        0x10 /* RL B        */ => RL (Operand::Register(B)),
        0x11 /* RL C        */ => RL (Operand::Register(C)),
        0x12 /* RL D        */ => RL (Operand::Register(D)),
        0x13 /* RL E        */ => RL (Operand::Register(E)),
        0x14 /* RL H        */ => RL (Operand::Register(H)),
        0x15 /* RL L        */ => RL (Operand::Register(L)),
        0x16 /* RL (HL)     */ => RL (Operand::Address(HL)),
        0x17 /* RL A        */ => RL (Operand::Register(A)),
        0x18 /* RR B        */ => RR (Operand::Register(B)),
        0x19 /* RR C        */ => RR (Operand::Register(C)),
        0x1A /* RR D        */ => RR (Operand::Register(D)),
        0x1B /* RR E        */ => RR (Operand::Register(E)),
        0x1C /* RR H        */ => RR (Operand::Register(H)),
        0x1D /* RR L        */ => RR (Operand::Register(L)),
        0x1E /* RR (HL)     */ => RR (Operand::Address(HL)),
        0x1F /* RR A        */ => RR (Operand::Register(A)),
        0x20 /* SLA B       */ => SLA(Operand::Register(B)),
        0x21 /* SLA C       */ => SLA(Operand::Register(C)),
        0x22 /* SLA D       */ => SLA(Operand::Register(D)),
        0x23 /* SLA E       */ => SLA(Operand::Register(E)),
        0x24 /* SLA H       */ => SLA(Operand::Register(H)),
        0x25 /* SLA L       */ => SLA(Operand::Register(L)),
        0x26 /* SLA (HL)    */ => SLA(Operand::Address(HL)),
        0x27 /* SLA A       */ => SLA(Operand::Register(A)),
        0x28 /* SRA B       */ => SRA(Operand::Register(B)),
        0x29 /* SRA C       */ => SRA(Operand::Register(C)),
        0x2A /* SRA D       */ => SRA(Operand::Register(D)),
        0x2B /* SRA E       */ => SRA(Operand::Register(E)),
        0x2C /* SRA H       */ => SRA(Operand::Register(H)),
        0x2D /* SRA L       */ => SRA(Operand::Register(L)),
        0x2E /* SRA (HL)    */ => SRA(Operand::Address(HL)),
        0x2F /* SRA A       */ => SRA(Operand::Register(A)),
        0x30 /* SWAP B      */ => SWAP(Operand::Register(B)),
        0x31 /* SWAP C      */ => SWAP(Operand::Register(C)),
        0x32 /* SWAP D      */ => SWAP(Operand::Register(D)),
        0x33 /* SWAP E      */ => SWAP(Operand::Register(E)),
        0x34 /* SWAP H      */ => SWAP(Operand::Register(H)),
        0x35 /* SWAP L      */ => SWAP(Operand::Register(L)),
        0x36 /* SWAP (HL)   */ => SWAP(Operand::Address(HL)),
        0x37 /* SWAP A      */ => SWAP(Operand::Register(A)),
        0x38 /* SRL B       */ => SRL(Operand::Register(B)),
        0x39 /* SRL C       */ => SRL(Operand::Register(C)),
        0x3A /* SRL D       */ => SRL(Operand::Register(D)),
        0x3B /* SRL E       */ => SRL(Operand::Register(E)),
        0x3C /* SRL H       */ => SRL(Operand::Register(H)),
        0x3D /* SRL L       */ => SRL(Operand::Register(L)),
        0x3E /* SRL (HL)    */ => SRL(Operand::Address(HL)),
        0x3F /* SRL A       */ => SRL(Operand::Register(A)),
        0x40 /* BIT 0,B     */ => BIT(0, Operand::Register(B)),
        0x41 /* BIT 0,C     */ => BIT(0, Operand::Register(C)),
        0x42 /* BIT 0,D     */ => BIT(0, Operand::Register(D)),
        0x43 /* BIT 0,E     */ => BIT(0, Operand::Register(E)),
        0x44 /* BIT 0,H     */ => BIT(0, Operand::Register(H)),
        0x45 /* BIT 0,L     */ => BIT(0, Operand::Register(L)),
        0x46 /* BIT 0,(HL)  */ => BIT(0, Operand::Address(HL)),
        0x47 /* BIT 0,A     */ => BIT(0, Operand::Register(A)),
        0x48 /* BIT 1,B     */ => BIT(1, Operand::Register(B)),
        0x49 /* BIT 1,C     */ => BIT(1, Operand::Register(C)),
        0x4A /* BIT 1,D     */ => BIT(1, Operand::Register(D)),
        0x4B /* BIT 1,E     */ => BIT(1, Operand::Register(E)),
        0x4C /* BIT 1,H     */ => BIT(1, Operand::Register(H)),
        0x4D /* BIT 1,L     */ => BIT(1, Operand::Register(L)),
        0x4E /* BIT 1,(HL)  */ => BIT(1, Operand::Address(HL)),
        0x4F /* BIT 1,A     */ => BIT(1, Operand::Register(A)),
        0x50 /* BIT 2,B     */ => BIT(2, Operand::Register(B)),
        0x51 /* BIT 2,C     */ => BIT(2, Operand::Register(C)),
        0x52 /* BIT 2,D     */ => BIT(2, Operand::Register(D)),
        0x53 /* BIT 2,E     */ => BIT(2, Operand::Register(E)),
        0x54 /* BIT 2,H     */ => BIT(2, Operand::Register(H)),
        0x55 /* BIT 2,L     */ => BIT(2, Operand::Register(L)),
        0x56 /* BIT 2,(HL)  */ => BIT(2, Operand::Address(HL)),
        0x57 /* BIT 2,A     */ => BIT(2, Operand::Register(A)),
        0x58 /* BIT 3,B     */ => BIT(3, Operand::Register(B)),
        0x59 /* BIT 3,C     */ => BIT(3, Operand::Register(C)),
        0x5A /* BIT 3,D     */ => BIT(3, Operand::Register(D)),
        0x5B /* BIT 3,E     */ => BIT(3, Operand::Register(E)),
        0x5C /* BIT 3,H     */ => BIT(3, Operand::Register(H)),
        0x5D /* BIT 3,L     */ => BIT(3, Operand::Register(L)),
        0x5E /* BIT 3,(HL)  */ => BIT(3, Operand::Address(HL)),
        0x5F /* BIT 3,A     */ => BIT(3, Operand::Register(A)),
        0x60 /* BIT 4,B     */ => BIT(4, Operand::Register(B)),
        0x61 /* BIT 4,C     */ => BIT(4, Operand::Register(C)),
        0x62 /* BIT 4,D     */ => BIT(4, Operand::Register(D)),
        0x63 /* BIT 4,E     */ => BIT(4, Operand::Register(E)),
        0x64 /* BIT 4,H     */ => BIT(4, Operand::Register(H)),
        0x65 /* BIT 4,L     */ => BIT(4, Operand::Register(L)),
        0x66 /* BIT 4,(HL)  */ => BIT(4, Operand::Address(HL)),
        0x67 /* BIT 4,A     */ => BIT(4, Operand::Register(A)),
        0x68 /* BIT 5,B     */ => BIT(5, Operand::Register(B)),
        0x69 /* BIT 5,C     */ => BIT(5, Operand::Register(C)),
        0x6A /* BIT 5,D     */ => BIT(5, Operand::Register(D)),
        0x6B /* BIT 5,E     */ => BIT(5, Operand::Register(E)),
        0x6C /* BIT 5,H     */ => BIT(5, Operand::Register(H)),
        0x6D /* BIT 5,L     */ => BIT(5, Operand::Register(L)),
        0x6E /* BIT 5,(HL)  */ => BIT(5, Operand::Address(HL)),
        0x6F /* BIT 5,A     */ => BIT(5, Operand::Register(A)),
        0x70 /* BIT 6,B     */ => BIT(6, Operand::Register(B)),
        0x71 /* BIT 6,C     */ => BIT(6, Operand::Register(C)),
        0x72 /* BIT 6,D     */ => BIT(6, Operand::Register(D)),
        0x73 /* BIT 6,E     */ => BIT(6, Operand::Register(E)),
        0x74 /* BIT 6,H     */ => BIT(6, Operand::Register(H)),
        0x75 /* BIT 6,L     */ => BIT(6, Operand::Register(L)),
        0x76 /* BIT 6,(HL)  */ => BIT(6, Operand::Address(HL)),
        0x77 /* BIT 6,A     */ => BIT(6, Operand::Register(A)),
        0x78 /* BIT 7,B     */ => BIT(7, Operand::Register(B)),
        0x79 /* BIT 7,C     */ => BIT(7, Operand::Register(C)),
        0x7A /* BIT 7,D     */ => BIT(7, Operand::Register(D)),
        0x7B /* BIT 7,E     */ => BIT(7, Operand::Register(E)),
        0x7C /* BIT 7,H     */ => BIT(7, Operand::Register(H)),
        0x7D /* BIT 7,L     */ => BIT(7, Operand::Register(L)),
        0x7E /* BIT 7,(HL)  */ => BIT(7, Operand::Address(HL)),
        0x7F /* BIT 7,A     */ => BIT(7, Operand::Register(A)),
        0x80 /* RES 0,B     */ => RES(0, Operand::Register(B)),
        0x81 /* RES 0,C     */ => RES(0, Operand::Register(C)),
        0x82 /* RES 0,D     */ => RES(0, Operand::Register(D)),
        0x83 /* RES 0,E     */ => RES(0, Operand::Register(E)),
        0x84 /* RES 0,H     */ => RES(0, Operand::Register(H)),
        0x85 /* RES 0,L     */ => RES(0, Operand::Register(L)),
        0x86 /* RES 0,(HL)  */ => RES(0, Operand::Address(HL)),
        0x87 /* RES 0,A     */ => RES(0, Operand::Register(A)),
        0x88 /* RES 1,B     */ => RES(1, Operand::Register(B)),
        0x89 /* RES 1,C     */ => RES(1, Operand::Register(C)),
        0x8A /* RES 1,D     */ => RES(1, Operand::Register(D)),
        0x8B /* RES 1,E     */ => RES(1, Operand::Register(E)),
        0x8C /* RES 1,H     */ => RES(1, Operand::Register(H)),
        0x8D /* RES 1,L     */ => RES(1, Operand::Register(L)),
        0x8E /* RES 1,(HL)  */ => RES(1, Operand::Address(HL)),
        0x8F /* RES 1,A     */ => RES(1, Operand::Register(A)),
        0x90 /* RES 2,B     */ => RES(2, Operand::Register(B)),
        0x91 /* RES 2,C     */ => RES(2, Operand::Register(C)),
        0x92 /* RES 2,D     */ => RES(2, Operand::Register(D)),
        0x93 /* RES 2,E     */ => RES(2, Operand::Register(E)),
        0x94 /* RES 2,H     */ => RES(2, Operand::Register(H)),
        0x95 /* RES 2,L     */ => RES(2, Operand::Register(L)),
        0x96 /* RES 2,(HL)  */ => RES(2, Operand::Address(HL)),
        0x97 /* RES 2,A     */ => RES(2, Operand::Register(A)),
        0x98 /* RES 3,B     */ => RES(3, Operand::Register(B)),
        0x99 /* RES 3,C     */ => RES(3, Operand::Register(C)),
        0x9A /* RES 3,D     */ => RES(3, Operand::Register(D)),
        0x9B /* RES 3,E     */ => RES(3, Operand::Register(E)),
        0x9C /* RES 3,H     */ => RES(3, Operand::Register(H)),
        0x9D /* RES 3,L     */ => RES(3, Operand::Register(L)),
        0x9E /* RES 3,(HL)  */ => RES(3, Operand::Address(HL)),
        0x9F /* RES 3,A     */ => RES(3, Operand::Register(A)),
        0xA0 /* RES 4,B     */ => RES(4, Operand::Register(B)),
        0xA1 /* RES 4,C     */ => RES(4, Operand::Register(C)),
        0xA2 /* RES 4,D     */ => RES(4, Operand::Register(D)),
        0xA3 /* RES 4,E     */ => RES(4, Operand::Register(E)),
        0xA4 /* RES 4,H     */ => RES(4, Operand::Register(H)),
        0xA5 /* RES 4,L     */ => RES(4, Operand::Register(L)),
        0xA6 /* RES 4,(HL)  */ => RES(4, Operand::Address(HL)),
        0xA7 /* RES 4,A     */ => RES(4, Operand::Register(A)),
        0xA8 /* RES 5,B     */ => RES(5, Operand::Register(B)),
        0xA9 /* RES 5,C     */ => RES(5, Operand::Register(C)),
        0xAA /* RES 5,D     */ => RES(5, Operand::Register(D)),
        0xAB /* RES 5,E     */ => RES(5, Operand::Register(E)),
        0xAC /* RES 5,H     */ => RES(5, Operand::Register(H)),
        0xAD /* RES 5,L     */ => RES(5, Operand::Register(L)),
        0xAE /* RES 5,(HL)  */ => RES(5, Operand::Address(HL)),
        0xAF /* RES 5,A     */ => RES(5, Operand::Register(A)),
        0xB0 /* RES 6,B     */ => RES(6, Operand::Register(B)),
        0xB1 /* RES 6,C     */ => RES(6, Operand::Register(C)),
        0xB2 /* RES 6,D     */ => RES(6, Operand::Register(D)),
        0xB3 /* RES 6,E     */ => RES(6, Operand::Register(E)),
        0xB4 /* RES 6,H     */ => RES(6, Operand::Register(H)),
        0xB5 /* RES 6,L     */ => RES(6, Operand::Register(L)),
        0xB6 /* RES 6,(HL)  */ => RES(6, Operand::Address(HL)),
        0xB7 /* RES 6,A     */ => RES(6, Operand::Register(A)),
        0xB8 /* RES 7,B     */ => RES(7, Operand::Register(B)),
        0xB9 /* RES 7,C     */ => RES(7, Operand::Register(C)),
        0xBA /* RES 7,D     */ => RES(7, Operand::Register(D)),
        0xBB /* RES 7,E     */ => RES(7, Operand::Register(E)),
        0xBC /* RES 7,H     */ => RES(7, Operand::Register(H)),
        0xBD /* RES 7,L     */ => RES(7, Operand::Register(L)),
        0xBE /* RES 7,(HL)  */ => RES(7, Operand::Address(HL)),
        0xBF /* RES 7,A     */ => RES(7, Operand::Register(A)),
        0xC0 /* SET 0,B     */ => SET(0, Operand::Register(B)),
        0xC1 /* SET 0,C     */ => SET(0, Operand::Register(C)),
        0xC2 /* SET 0,D     */ => SET(0, Operand::Register(D)),
        0xC3 /* SET 0,E     */ => SET(0, Operand::Register(E)),
        0xC4 /* SET 0,H     */ => SET(0, Operand::Register(H)),
        0xC5 /* SET 0,L     */ => SET(0, Operand::Register(L)),
        0xC6 /* SET 0,(HL)  */ => SET(0, Operand::Address(HL)),
        0xC7 /* SET 0,A     */ => SET(0, Operand::Register(A)),
        0xC8 /* SET 1,B     */ => SET(1, Operand::Register(B)),
        0xC9 /* SET 1,C     */ => SET(1, Operand::Register(C)),
        0xCA /* SET 1,D     */ => SET(1, Operand::Register(D)),
        0xCB /* SET 1,E     */ => SET(1, Operand::Register(E)),
        0xCC /* SET 1,H     */ => SET(1, Operand::Register(H)),
        0xCD /* SET 1,L     */ => SET(1, Operand::Register(L)),
        0xCE /* SET 1,(HL)  */ => SET(1, Operand::Address(HL)),
        0xCF /* SET 1,A     */ => SET(1, Operand::Register(A)),
        0xD0 /* SET 2,B     */ => SET(2, Operand::Register(B)),
        0xD1 /* SET 2,C     */ => SET(2, Operand::Register(C)),
        0xD2 /* SET 2,D     */ => SET(2, Operand::Register(D)),
        0xD3 /* SET 2,E     */ => SET(2, Operand::Register(E)),
        0xD4 /* SET 2,H     */ => SET(2, Operand::Register(H)),
        0xD5 /* SET 2,L     */ => SET(2, Operand::Register(L)),
        0xD6 /* SET 2,(HL)  */ => SET(2, Operand::Address(HL)),
        0xD7 /* SET 2,A     */ => SET(2, Operand::Register(A)),
        0xD8 /* SET 3,B     */ => SET(3, Operand::Register(B)),
        0xD9 /* SET 3,C     */ => SET(3, Operand::Register(C)),
        0xDA /* SET 3,D     */ => SET(3, Operand::Register(D)),
        0xDB /* SET 3,E     */ => SET(3, Operand::Register(E)),
        0xDC /* SET 3,H     */ => SET(3, Operand::Register(H)),
        0xDD /* SET 3,L     */ => SET(3, Operand::Register(L)),
        0xDE /* SET 3,(HL)  */ => SET(3, Operand::Address(HL)),
        0xDF /* SET 3,A     */ => SET(3, Operand::Register(A)),
        0xE0 /* SET 4,B     */ => SET(4, Operand::Register(B)),
        0xE1 /* SET 4,C     */ => SET(4, Operand::Register(C)),
        0xE2 /* SET 4,D     */ => SET(4, Operand::Register(D)),
        0xE3 /* SET 4,E     */ => SET(4, Operand::Register(E)),
        0xE4 /* SET 4,H     */ => SET(4, Operand::Register(H)),
        0xE5 /* SET 4,L     */ => SET(4, Operand::Register(L)),
        0xE6 /* SET 4,(HL)  */ => SET(4, Operand::Address(HL)),
        0xE7 /* SET 4,A     */ => SET(4, Operand::Register(A)),
        0xE8 /* SET 5,B     */ => SET(5, Operand::Register(B)),
        0xE9 /* SET 5,C     */ => SET(5, Operand::Register(C)),
        0xEA /* SET 5,D     */ => SET(5, Operand::Register(D)),
        0xEB /* SET 5,E     */ => SET(5, Operand::Register(E)),
        0xEC /* SET 5,H     */ => SET(5, Operand::Register(H)),
        0xED /* SET 5,L     */ => SET(5, Operand::Register(L)),
        0xEE /* SET 5,(HL)  */ => SET(5, Operand::Address(HL)),
        0xEF /* SET 5,A     */ => SET(5, Operand::Register(A)),
        0xF0 /* SET 6,B     */ => SET(6, Operand::Register(B)),
        0xF1 /* SET 6,C     */ => SET(6, Operand::Register(C)),
        0xF2 /* SET 6,D     */ => SET(6, Operand::Register(D)),
        0xF3 /* SET 6,E     */ => SET(6, Operand::Register(E)),
        0xF4 /* SET 6,H     */ => SET(6, Operand::Register(H)),
        0xF5 /* SET 6,L     */ => SET(6, Operand::Register(L)),
        0xF6 /* SET 6,(HL)  */ => SET(6, Operand::Address(HL)),
        0xF7 /* SET 6,A     */ => SET(6, Operand::Register(A)),
        0xF8 /* SET 7,B     */ => SET(7, Operand::Register(B)),
        0xF9 /* SET 7,C     */ => SET(7, Operand::Register(C)),
        0xFA /* SET 7,D     */ => SET(7, Operand::Register(D)),
        0xFB /* SET 7,E     */ => SET(7, Operand::Register(E)),
        0xFC /* SET 7,H     */ => SET(7, Operand::Register(H)),
        0xFD /* SET 7,L     */ => SET(7, Operand::Register(L)),
        0xFE /* SET 7,(HL)  */ => SET(7, Operand::Address(HL)),
        0xFF /* SET 7,A     */ => SET(7, Operand::Register(A)),
    }
}
