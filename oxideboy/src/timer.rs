//! Implementation of the timer unit in the Gameboy LR35902 chip.

pub struct Timer {
    enabled: bool,       // Controls whether the timer unit is running.
    freq: u16,           // Expressed as a divisor of the DIV register.
    pub tima: u8,        // Every time the DIV register ticks `freq` times, TIMA is incremented by 1.
    pub tma: u8,         // When TIMA overflows 8-bits, the timer interrupt is set and TIMA is reset to this value.
    tima_overflow: bool, // The reloading of TIMA with TMA is delayed by 1 instruction cycle.
    tima_reloaded: bool, // If we just reloaded TIMA, this flag will be set for that cycle.
    pub div: u16,        // Increments every CPU clock cycle (4.194304Mhz). Only top 8 bits are visible to the CPU.
}

impl Timer {
    pub fn new() -> Timer {
        Timer {
            enabled: false,
            freq: 1024,
            tima: 0,
            tma: 0,
            tima_overflow: false,
            tima_reloaded: false,
            div: 0,
        }
    }

    /// Advances the timer by a single CPU clock step. Returning true means the timer interrupt request should be set.
    pub fn clock(&mut self) -> bool {
        let mut timer_interrupt = false;

        // DIV is a 16 bit register (of which only the upper 8 bits are addressable) that increments
        // at the same speed as the CPU clock - 4.194304Mhz.
        self.div = self.div.wrapping_add(4);
        self.tima_reloaded = false;

        // If TIMA overflowed on the previous cycle, this is where we reload it to TMA and request timer interrupt.
        // See the code below here for when we set tima_overflow.
        if self.tima_overflow {
            self.tima = self.tma;
            timer_interrupt = true;
            self.tima_overflow = false;
            self.tima_reloaded = true;
        }

        if self.enabled && self.div % self.freq == 0 {
            self.tima = self.tima.wrapping_add(1);
            if self.tima == 0 {
                // When TIMA overflows, we don't actually reload it with TMA and request timer interrupt immediately.
                // That happens on the next cycle.
                self.tima_overflow = true;
            }
        }

        timer_interrupt
    }

    /// Write to the 0xFF04 DIV register.
    pub fn reg_div_write(&mut self) {
        // Writes to DIV are sort-of ignored. They just reset DIV to 0.
        // HOWEVER, there's also a side effect to note ...
        let old_div = self.div;
        self.div = 0;

        // ... If the timer is currently running, then depending on the frequency and which bits in DIV just went
        // from 1 to 0, we may trigger another timer increment. This is because of how the actual timer
        // circuitry works.
        // See http://gbdev.gg8.se/wiki/articles/Timer_Obscure_Behaviour
        if self.enabled {
            let should_inc = old_div
                & match self.freq {
                    1024 => 0b0000_0010_0000_0000,
                    16 => 0b0000_0000_0000_1000,
                    64 => 0b0000_0000_0010_0000,
                    256 => 0b0000_0000_1000_0000,
                    _ => unreachable!("All timer frequencies covered"),
                }
                > 0;
            if should_inc {
                self.tima = self.tima.wrapping_add(1);
                if self.tima == 0 {
                    self.tima_overflow = true;
                }
            }
        }
    }

    /// Write to the 0xFF05 TIMA register.
    pub fn reg_tima_write(&mut self, v: u8) {
        // If TIMA is written to during the same cycle it overflowed, then the interrupt and reloading of TIMA
        // with TMA is cancelled.
        // See tima_write_reloading mooneye test.
        self.tima_overflow = false;

        // Converse to above, if we're in the next cycle where TIMA was reloaded with TMA, we actually swallow
        // this write to TIMA.
        if !self.tima_reloaded {
            self.tima = v;
        }
    }

    /// Write to the 0xFF06 TMA register.
    pub fn reg_tma_write(&mut self, v: u8) {
        self.tma = v;

        // Another timer quirk. If TMA is written to in the same cycle that we refreshed TIMA, then
        // we actually write this new value into TIMA immediately.
        if self.tima_reloaded {
            self.tima = self.tma;
        }
    }

    /// Read from the 0xFF07 TAC register.
    pub fn reg_tac_read(&mut self) -> u8 {
        0b1111_1000 // Unused bits.
            | (if self.enabled { 0b100 } else { 0 })
            | match self.freq {
                0    => 0,
                1024 => 0,
                16   => 1,
                64   => 2,
                256  => 3,
                _    => unreachable!("freq has fixed set of values")
            }
    }

    /// Write to the 0xFF07 TAC register.
    /// Sometimes writes to the TAC register can cause a glitchy spurious timer interrupt request. In those cases we
    /// return true here to signal that Timer interrupt request should be set.
    pub fn reg_tac_write(&mut self, v: u8) -> bool {
        let mut timer_interrupt = false;
        let was_enabled = self.enabled;
        let orig_freq = self.freq;

        self.enabled = v & 0b100 > 0;
        if self.enabled {
            self.freq = match v & 0b11 {
                0b00 => 1024,
                0b01 => 16,
                0b10 => 64,
                0b11 => 256,
                _ => unreachable!("Matched all possible 2 bit values"),
            }
        }

        // The original DMG had a glitch that could cause the TIMA register to sometimes spuriously increment.
        // We emulate that quirk here.
        // More info: http://gbdev.gg8.se/wiki/articles/Timer_Obscure_Behaviour
        // TODO: which models should we emulate this for?
        let glitch = if was_enabled {
            if !self.enabled {
                self.div & (orig_freq / 2) != 0
            } else {
                (self.div & (orig_freq / 2) != 0) && (self.div & (self.freq / 2) == 0)
            }
        } else {
            false
        };
        if glitch {
            // A cut down version of what we do in advance_timer.
            // Note how here we're not delaying the TIMA reset or interrupt request to the next cycle.
            // We do it immediately in the glitch case. Hardware is weird.
            self.tima = self.tima.wrapping_add(1);
            if self.tima == 0 {
                self.tima = self.tma;
                timer_interrupt = true;
            }
        }

        timer_interrupt
    }
}
