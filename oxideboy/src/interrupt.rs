//! Gameboy interrupt controller.
//! Interrupts on the Gameboy are pretty straightforward. There's only a small handful of events that can trigger
//! interrupts: serial port activity, events in the PPU (VBlank, HBlank, etc), Joypad input, and Timer increments.
//! The CPU checks for any pending interrupts on each instruction cycle. If there's a pending interrupt, and interrupts
//! are enabled, then the CPU stops executing where it was up to, pushes the current PC register value onto the stack,
//! and jumps to a fixed location in memory, depending on which interrupt was processed.

#[derive(Default)]
pub struct InterruptController {
    pub request: u8, // 0xFF0F IF register
    pub enable: u8,  // 0xFFFF IE register

    pub pending: bool, // Efficient dirty check to determine if there's any pending interrupts to process.
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Interrupt {
    VBlank = 0b0_0001,
    Stat = 0b0_0010,
    Timer = 0b0_0100,
    Serial = 0b0_1000,
    Joypad = 0b1_0000,
}
use Interrupt::*;

impl Interrupt {
    /// Determines the appropriate memory location to jump to when processing an interrupt.
    pub fn handler_addr(&self) -> u16 {
        match self {
            VBlank => 0x40,
            Stat => 0x48,
            Timer => 0x50,
            Serial => 0x58,
            Joypad => 0x60,
        }
    }
}

impl InterruptController {
    pub fn new() -> InterruptController {
        Default::default()
    }

    /// Returns the next interrupt to process. Should only be called if pending flag is true, otherwise this will panic.
    pub fn next_interrupt(&self) -> Interrupt {
        // Interrupts follow a priority order, in case there's multiple pending requested interrupts.
        let pending = self.enable & self.request;
        if pending & (VBlank as u8) > 0 {
            VBlank
        } else if pending & (Stat as u8) > 0 {
            Stat
        } else if pending & (Timer as u8) > 0 {
            Timer
        } else if pending & (Serial as u8) > 0 {
            Serial
        } else if pending & (Joypad as u8) > 0 {
            Joypad
        } else {
            panic!("next_interrupt called with no pending interrupts");
        }
    }

    /// Request an interrupt.
    pub fn request(&mut self, intr: Interrupt) {
        self.request |= intr as u8;
        self.pending = (self.request & self.enable) > 0;
    }

    /// Clear an interrupt request.
    pub fn clear(&mut self, intr: Interrupt) {
        self.request &= !(intr as u8);
        self.pending = (self.request & self.enable) > 0;
    }

    // Write to the 0xFF0F IF register.
    pub fn reg_if_write(&mut self, v: u8) {
        self.request = v;
        self.pending = (self.request & self.enable) > 0;
    }

    // Write to the 0xFFFF IE register.
    pub fn reg_ie_write(&mut self, v: u8) {
        self.enable = v;
        self.pending = (self.request & self.enable) > 0;
    }
}
