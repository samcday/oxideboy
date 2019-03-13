//! The Gameboy joypad is a pretty simple affair. It has 8 buttons. The interesting thing to note is how the key state
//! is accessed. You would think, given that there's 8 buttons and 8 bits in a byte, that the up/down state of each
//! button would simply be a bit in a register. You would be wrong. For whatever reason, the buttons are divided into
//! two groups of 4, and you have to set the upper bits to select which button group state will appear in the register.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Default, Deserialize, Serialize)]
pub struct Joypad {
    select_btn: bool,
    select_dir: bool,

    pub state: JoypadState,
}

/// there's only 8 buttons on the joypad, so we can encode the state of all of them in a single u8.
/// The Gameboy treats a set bit as unpressed, and a cleared bit as pressed, so we store it like that too.
#[derive(Clone, Copy, Deserialize, Serialize)]
pub struct JoypadState(u8);

#[rustfmt::skip]
pub enum Button {
    Down    = 0b1000_0000,
    Up      = 0b0100_0000,
    Left    = 0b0010_0000,
    Right   = 0b0001_0000,
    Start   = 0b0000_1000,
    Select  = 0b0000_0100,
    B       = 0b0000_0010,
    A       = 0b0000_0001,
}

impl Joypad {
    /// Read from the 0xFF00 P1 register
    pub fn reg_p1_read(&self) -> u8 {
        0xC0 | if self.select_btn {
            0x20 | self.state.btn()
        } else if self.select_dir {
            0x10 | self.state.dir()
        } else {
            0xF
        }
    }

    /// Write to the 0xFF00 P1 register
    pub fn reg_p1_write(&mut self, v: u8) {
        self.select_btn = v & 0x20 == 0;
        self.select_dir = v & 0x10 == 0;
    }
}

impl Default for JoypadState {
    fn default() -> JoypadState {
        // By default all buttons are unpressed, which means all bits are set.
        JoypadState(0xFF)
    }
}

impl JoypadState {
    pub fn set_button(&mut self, key: Button, pressed: bool) {
        if pressed {
            self.0 ^= key as u8;
        } else {
            // Note that if a button bit is set it means it's *not* pressed, because Gameboy logic.
            self.0 |= key as u8;
        }
    }

    pub fn clear(&mut self) {
        self.0 = 0xFF;
    }

    fn dir(&self) -> u8 {
        self.0 >> 4
    }

    fn btn(&self) -> u8 {
        self.0 & 0b1111
    }
}
