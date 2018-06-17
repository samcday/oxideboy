#[derive(Default)]
pub struct SoundController {
  sound_enabled: bool,
  left_vol: u8,
  right_vol: u8,

  sound1_on: bool,
  sound1_left: bool,
  sound1_right: bool,

  sound2_on: bool,
  sound2_left: bool,
  sound2_right: bool,

  sound3_on: bool,
  sound3_left: bool,
  sound3_right: bool,

  sound4_on: bool,
  sound4_left: bool,
  sound4_right: bool,
}

impl SoundController {
    pub fn advance(&mut self) {

    }

    pub fn read_nr50(&self) -> u8 {
        0
            | (self.left_vol & 3)
            | ((self.right_vol & 3) << 4)
    }

    pub fn write_nr50(&mut self, v: u8) {
        self.left_vol  = v & 3;
        self.right_vol = (v >> 4) & 3;
    }

    pub fn read_nr51(&self) -> u8 {
        0
            | if self.sound1_left  { 0b00000001 } else { 0 }
            | if self.sound2_left  { 0b00000010 } else { 0 }
            | if self.sound3_left  { 0b00000100 } else { 0 }
            | if self.sound4_left  { 0b00001000 } else { 0 }
            | if self.sound1_right { 0b00010000 } else { 0 }
            | if self.sound2_right { 0b00100000 } else { 0 }
            | if self.sound3_right { 0b01000000 } else { 0 }
            | if self.sound4_right { 0b10000000 } else { 0 }
    }

    pub fn write_nr51(&mut self, v: u8) {
        self.sound1_left  = v & 0b00000001 > 0;
        self.sound2_left  = v & 0b00000010 > 0;
        self.sound3_left  = v & 0b00000100 > 0;
        self.sound4_left  = v & 0b00001000 > 0;
        self.sound1_right = v & 0b00010000 > 0;
        self.sound2_right = v & 0b00100000 > 0;
        self.sound3_right = v & 0b01000000 > 0;
        self.sound4_right = v & 0b10000000 > 0;
    }

    pub fn read_nr52(&self) -> u8 {
        0
            | if self.sound1_on     { 0b00000001 } else { 0 }
            | if self.sound2_on     { 0b00000010 } else { 0 }
            | if self.sound3_on     { 0b00000100 } else { 0 }
            | if self.sound4_on     { 0b00001000 } else { 0 }
            | if self.sound_enabled { 0b10000000 } else { 0 }
    }

    pub fn write_nr52(&mut self, v: u8) {
        self.sound1_on     = v & 0b00000001 > 0;
        self.sound2_on     = v & 0b00000010 > 0;
        self.sound3_on     = v & 0b00000100 > 0;
        self.sound4_on     = v & 0b00001000 > 0;
        self.sound_enabled = v & 0b10000000 > 0;
    }
}
