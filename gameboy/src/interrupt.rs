use ::GameboyContext;
use self::Interrupt::*;

#[derive(Clone, Copy, Debug)]
pub enum Interrupt {
    VBlank = 0b00001,
    Stat   = 0b00010,
    Timer  = 0b00100,
    Serial = 0b01000,
    Joypad = 0b10000,
}

#[derive(Serialize, Deserialize, Debug, Default)]
pub struct InterruptState {
    pending_interrupts: bool,
    pub enabled: u8,
    pub requested: u8,
    pub ime: bool,              // The IME register, master switch for turning all interrupts on/off.
    pub ime_defer: bool,        // Enabling interrupts is delayed by a cycle, we track that here.
}

impl Interrupt {
    fn handler_addr(&self) -> u16 {
        match self {
            VBlank => 0x40,
            Stat   => 0x48,
            Timer  => 0x50,
            Serial => 0x58,
            Joypad => 0x60,
        }
    }
}

impl InterruptState {
    /// Request an interrupt.
    pub fn request(&mut self, intr: Interrupt) {
        self.requested |= intr as u8;
        self.pending_interrupts = self.requested & self.enabled > 0;
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

    pub fn reg_if_write(&mut self, v: u8) {
        self.requested = v;
        self.pending_interrupts = self.requested & self.enabled > 0;
    }

    pub fn reg_ie_write(&mut self, v: u8) {
        self.enabled = v;
        self.pending_interrupts = self.requested & self.enabled > 0;
    }

    pub fn clear(&mut self, intr: Interrupt) {
        self.requested &= !(intr as u8);
        self.pending_interrupts = self.requested & self.enabled > 0;
    }
}

/// Checks if there's any interrupts to process. If there is, interrupt state is updated
/// and the address to jump to is returned.
pub fn process(ctx: &mut GameboyContext) {
    // We can bail quickly if there's no interrupts to process.
    if !ctx.state.int.pending_interrupts {
        return;
    }

    // If there are interrupts to process, we clear HALT state, even if IME is disabled.
    ctx.state.cpu.halted = false;

    if !ctx.state.int.ime {
        // If IME isn't enabled though, we don't actually process any interrupts.
        return;
    }

    // Now let's find which interrupt we're processing...
    let pending = ctx.state.int.enabled & ctx.state.int.requested;
    let intr = if pending & (VBlank as u8) > 0 {
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
        unreachable!("There must be at least one pending interrupt at this point");
    };

    // Interrupt handling needs 3 internal cycles to do interrupt-y stuff.
    ctx.clock();
    ctx.clock();
    ctx.clock();

    // Here's an interesting quirk. If the stack pointer was set to 0000 or 0001, then the push we just did
    // above would have overwritten IE. If the new IE value no longer matches the interrupt we were processing,
    // then we cancel that interrupt and set PC to 0. We then try and find another interrupt.
    // If there isn't one, we end up running code from 0000. Crazy.
    let pc = ctx.state.cpu.pc;
    let mut sp = ctx.state.cpu.sp;
    sp = sp.wrapping_sub(1);
    ctx.mem_write8(sp, ((pc & 0xFF00) >> 8) as u8);
    // This is where we capture what IE is after pushing the upper byte. Pushing the lower byte might
    // also overwrite IE, but in that case we ignore that occurring.
    let new_enabled = ctx.state.int.enabled;
    sp = sp.wrapping_sub(1);
    ctx.mem_write8(sp, pc as u8);
    ctx.state.cpu.sp = ctx.state.cpu.sp.wrapping_sub(2);

    if new_enabled & (intr as u8) == 0 {
        ctx.state.cpu.pc = 0;
        // Okay so this interrupt didn't go so good. Let's see if there's another one.
        process(ctx);
        // Regardless of what happens in the next try, IME needs to be disabled.
        ctx.state.int.ime = false;
        return;
    }

    ctx.state.cpu.pc = intr.handler_addr();
    ctx.state.int.clear(intr);
    ctx.state.int.ime = false;
}
