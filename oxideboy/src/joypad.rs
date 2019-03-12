//! The Gameboy joypad is a pretty simple affair. It has 8 buttons. The interesting thing to note is how the key state
//! is accessed. You would think, given that there's 8 buttons and 8 bits in a byte, that the up/down state of each
//! button would simply be a bit in a register. You would be wrong. For whatever reason, the buttons are divided into
//! two groups of 4, and you have to set the upper bits to select which button group state will appear in the register.

use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Joypad {
    select_btn: bool,
    select_dir: bool,

    pub up: bool,
    pub down: bool,
    pub left: bool,
    pub right: bool,
    pub a: bool,
    pub b: bool,
    pub select: bool,
    pub start: bool,
}

impl Joypad {
    /// Read from the 0xFF00 P1 register
    pub fn reg_p1_read(&self) -> u8 {
        let mut v = 0xCF;

        if self.select_btn {
            v ^= 0x20;
            if self.a {
                v ^= 1
            }
            if self.b {
                v ^= 2
            }
            if self.select {
                v ^= 4
            }
            if self.start {
                v ^= 8
            }
        } else if self.select_dir {
            v ^= 0x10;
            if self.right {
                v ^= 1
            }
            if self.left {
                v ^= 2
            }
            if self.up {
                v ^= 4
            }
            if self.down {
                v ^= 8
            }
        }

        v
    }

    /// Write to the 0xFF00 P1 register
    pub fn reg_p1_write(&mut self, v: u8) {
        self.select_btn = v & 0x20 == 0;
        self.select_dir = v & 0x10 == 0;
    }
}
