//! Implements the serial (link cable) port for the Gameboy.
//! http://gbdev.gg8.se/wiki/articles/Serial_Data_Transfer_(Link_Cable)

pub struct Serial {
    pub serial_out: Option<u8>, // When serial port is active, SB register will get shifted to here to be read out.
    serial_in: Option<u8>,      // When serial port is active, this incoming value will be read into the SB register.

    sb: u8,               // Value of the SB register (0xFF01)
    internal_clock: bool, // Are we using the internal clock to read/write the serial port?
    transfer_start: bool, // Are we performing a transfer right now?
    transfer_fast: bool,  // Are we transferring serial data at "normal" speed or fast (32x faster) speed?

    transfer_countdown: u16, // Tracks how many more clock cycles we need until next transfer takes place.
}

impl Serial {
    pub fn new() -> Serial {
        Serial {
            serial_out: None,
            serial_in: None,

            sb: 0,
            internal_clock: false,
            transfer_start: false,
            transfer_fast: false,
            transfer_countdown: 0,
        }
    }

    /// Advances the serial port by a single CPU clock step.
    pub fn clock(&mut self) {
        // Were we provided some data from the remote end?
        if self.serial_in.is_some() {
            self.sb = self.serial_in.take().unwrap();
            return;
        }

        // If no remote data and we're not trying to send anything, then we're done here.
        if !self.transfer_start {
            return;
        }

        self.transfer_countdown -= 1;
        if self.transfer_countdown == 0 {
            self.serial_out = Some(self.sb);
            self.sb = 0;
            self.transfer_start = false;
        }
    }

    /// Externally called by whoever is controlling the remote end of the serial port. Provides a byte to be received
    /// into the SB register. We don't emulate the external clock speed here since it's kinda pointless. Whatever is
    /// provided here will be made immediately available to the CPU on the next clock cycle.
    pub fn receive(&mut self, v: u8) {
        self.serial_in = Some(v);
    }

    pub fn reg_sb_read(&self) -> u8 {
        return self.sb;
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
        (if self.transfer_start { 0b1000_0000 } else { 0 })
    }

    pub fn reg_sc_write(&mut self, v: u8) {
        self.transfer_start = if (v & 0b1000_0000) > 0 { true } else { false };
        self.transfer_fast = if (v & 0b0000_0010) > 0 { true } else { false };
        self.internal_clock = if (v & 0b0000_0001) > 0 { true } else { false };

        if self.internal_clock && self.transfer_start {
            // A transfer was just started by this CPU. Start the countdown.
            // If we're in "fast" mode (32KB/s) it'll only take 32 CPU clocks to have shifted everything out.
            // Otherwise, normal mode is 1KB/s and takes 1024 CPU clocks.
            // Math: "fast" mode clock runs at 262144Hz and normal clock runs at 8192Hz. In both cases each clock pulse
            // shifts 1 bit out. So for fast mode we determine cycle delay as 1048576÷8192*8.
            self.transfer_countdown = if self.transfer_fast { 32 } else { 1024 };
        }
    }
}
