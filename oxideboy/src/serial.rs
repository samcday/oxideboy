//! Implements the serial (link cable) port for the Gameboy.
//! http://gbdev.gg8.se/wiki/articles/Serial_Data_Transfer_(Link_Cable)

use crate::interrupt::{Interrupt, InterruptController};

#[derive(Default)]
pub struct Serial {
    pub serial_out: Option<u8>, // When serial port is active, SB register will get shifted to here to be read out.
    serial_in: Option<u8>,      // When serial port is active, this incoming value will be read into the SB register.
    sb: u8,                     // Value of the SB register (0xFF01)
    internal_clock: bool,       // Are we using the internal clock to read/write the serial port?
    transfer_fast: bool,        // Are we transferring serial data at "normal" speed or fast (32x faster) speed?
    transfer_speed: u8,         // The number of clock cycles it takes to shift a bit in/out when using internal clock.
    pub transfer_clock: u8,     // The number of clock cycles since we last shifted a bit.
    transfer_countdown: u8,     // Tracks how many more clock cycles we need until next transfer takes place.
}

impl Serial {
    pub fn new() -> Serial {
        Serial {
            transfer_speed: 128, // Default transfer speed is 8192Hz, or 1 bit every 128 clock cycles

            ..Default::default()
        }
    }

    /// Advances the serial port by a single CPU clock step.
    pub fn clock(&mut self, interrupts: &mut InterruptController) {
        // No matter what, the serial port is running its clock to determine when it's time to shift out another bit.
        // When the clock triggers, we won't actually shift anything out unless the internal clock mode is enabled and
        // there's a transfer requested.
        self.transfer_clock += 1;
        let clock_edge = self.transfer_clock > self.transfer_speed;
        if clock_edge {
            self.transfer_clock = 0;
        }

        // Were we provided some data from the remote end?
        if self.serial_in.is_some() {
            // TODO: this isn't really correct. At the very least I think it needs to be checking if the external
            // clock is selected, and triggering a Serial interrupt if so.
            self.sb = self.serial_in.take().unwrap();
            return;
        }

        // If internal clock isn't enabled, or there's no pending transfer, we're done for now.
        if !self.internal_clock || self.transfer_countdown == 0 {
            return;
        }

        if clock_edge {
            self.transfer_countdown -= 1;
        }

        if self.transfer_countdown == 0 {
            // We've "shifted out" all 8 bits, so the transfer is done. Trigger interrupt and make the byte
            // available externally.
            interrupts.request(Interrupt::Serial);
            self.serial_out = Some(self.sb);
            self.sb = 0;
        }
    }

    /// Externally called by whoever is controlling the remote end of the serial port. Provides a byte to be received
    /// into the SB register. We don't emulate the external clock speed here since it's kinda pointless. Whatever is
    /// provided here will be made immediately available to the CPU on the next clock cycle.
    pub fn receive(&mut self, v: u8) {
        self.serial_in = Some(v);
    }

    pub fn reg_sb_read(&self) -> u8 {
        self.sb
    }

    pub fn reg_sb_write(&mut self, v: u8) {
        self.sb = v;
    }

    pub fn reg_sc_read(&self) -> u8 {
        // TODO: mooneye/acceptance/bits/unused_hwio_gs test expects bit 1 to be set, but that bit is used for
        // serial fast mode according to docs.
        // 0b0111_1110 |  // Unused bits are set to 1
        0b0111_1110 |  // Unused bits are set to 1
        (if self.internal_clock { 0b0000_0001 } else { 0 }) |
        (if self.transfer_fast  { 0b0000_0010 } else { 0 }) |
        (if self.transfer_countdown > 0 { 0b1000_0000 } else { 0 })
    }

    pub fn reg_sc_write(&mut self, v: u8) {
        let transfer_request = v & 0b1000_0000 > 0;
        self.transfer_fast = v & 0b0000_0010 > 0;
        self.internal_clock = v & 0b0000_0001 > 0;

        // At "normal" speed we transfer at 8192Hz, or 1 bit every 128 CPU clock cycles.
        // At "fast" speed we transfer at 262144Hz, or 1 bit every 4 CPU clock cyles.
        // Note that when in CGB double speed mode, these numbers still apply, but the CPU is simply running twice as
        // fast, which has the effect of doubling the serial transfer speed.
        self.transfer_speed = if self.transfer_fast { 4 } else { 128 };

        if self.internal_clock && transfer_request {
            // A transfer was just started by this CPU. Start the countdown.
            // Every self.transfer_speed clock cycles, we decrement this counter by 1, until we've "shifted" out all
            // 8 bits, at which point we make the serial data available externally, and trigger the interrupt.
            self.transfer_countdown = 8;
        } else {
            // Not really sure if this is correct, but I'm assuming if the internal clock was deselected, or the
            // transfer start bit was cleared, then we should stop any serial transfer we might have been doing.
            self.transfer_countdown = 0;
        }
    }
}
