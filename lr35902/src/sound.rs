const SAMPLE_RATE: f64 = 44100.0;  // TODO: configurable?

#[derive(Default)]
pub struct WaveRam {
    pub data: [u8; 16],
}

impl WaveRam {
    fn get_step(&self, n: u8) -> f32 {
        (self.data[(n / 2) as usize] >> (n % 2 * 4) & 0b1111) as f32
    }
}

#[derive(Default)]
pub struct SoundController {
    sample_cycles: f64,

    timer: u16,
    frame_seq_timer: u8,         // Main timer to drive other clocks. 512hz (2048 CPU cycles)

    enabled: bool,
    left_vol: u8,
    right_vol: u8,

    // Tone & Sweep voice
    sound1_on: bool,
    sound1_left: bool,
    sound1_right: bool,
    sound1_sweep_num: u8,
    sound1_sweep_sub: bool,
    sound1_sweep_time: u8,
    sound1_length: u8,
    sound1_duty: u8,
    sound1_env_init: u8,
    sound1_env_inc: bool,
    sound1_env_num: u8,

    // Tone voice
    sound2_on: bool,
    sound2_left: bool,
    sound2_right: bool,

    // Wave voice
    sound3_on: bool,
    sound3_left: bool,
    sound3_right: bool,
    sound3_length: u8,
    sound3_level: u8,
    sound3_freq: u16,
    sound3_counter: bool,
    pub sound3_wave_ram: WaveRam,
    sound3_pos: u8,
    sound3_timer: u16,

    // Noise voice
    sound4_on: bool,
    sound4_left: bool,
    sound4_right: bool,

    samples: Vec<f32>,
}

impl SoundController {
    pub fn get_samples(&self) -> &[f32] {
        &self.samples
    }

    pub fn clear_samples(&mut self) {
        self.samples.clear();
    }

    pub fn advance(&mut self) {
        if !self.enabled {
            return;
        }

        self.timer += 1;
        if self.timer == 2048 {
            self.timer = 0;
            self.frame_seq_timer += 1;
            if self.frame_seq_timer % 2 == 1 {
                self.length_clock();
            }
            if self.frame_seq_timer == 8 {
                self.vol_env_clock();
                self.frame_seq_timer = 0;
            }
            if self.frame_seq_timer % 4 == 3 {
                self.sweep_clock();
            }
        }

        if self.sound3_on {
            self.sound3_timer += 1;
            if (self.sound3_timer / 16) == (2048 - self.sound3_freq) {
                self.sound3_pos = (self.sound3_pos + 1) % 32;
            }
        }

        self.sample_cycles += 1.0;
        let cycles_per_sample = 1048576.0 / SAMPLE_RATE;
        if self.sample_cycles >= cycles_per_sample {
            self.sample_cycles -= cycles_per_sample;
            self.generate_sample();
        }
    }

    fn generate_sample(&mut self) {
        let mut l = 0.0;
        let mut r = 0.0;

        if self.sound3_on {
            l = self.sound3_wave_ram.get_step(self.sound3_pos) / 15.0 * 0.5;
            r = self.sound3_wave_ram.get_step(self.sound3_pos) / 15.0 * 0.5;
        }

        self.samples.push(l);
        self.samples.push(r);
    }

    fn length_clock(&mut self) {
        if self.sound3_on && self.sound3_counter {
            self.sound3_length -= 1;
            if self.sound3_length == 0 {
                self.sound3_on = false;
            }
        }
    }

    fn vol_env_clock(&self) {

    }

    fn sweep_clock(&self) {

    }

    pub fn read_nr10(&self) -> u8 {
        if !self.enabled {
            return 0;
        }
        0
            | self.sound1_sweep_num
            | if self.sound1_sweep_sub { 0b0000_1000 } else { 0 }
            | self.sound1_sweep_time << 5
    }

    pub fn write_nr10(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.sound1_sweep_num  = v & 0x07;
        self.sound1_sweep_sub  = v & 0b0000_1000 > 0;
        self.sound1_sweep_time = v & 0b0111_0000 >> 5;
    }

    pub fn read_nr11(&self) -> u8 {
        if self.enabled { self.sound1_duty << 6 } else { 0 }
    }

    pub fn write_nr11(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.sound1_length = v & 0b0011_1111;
        self.sound1_duty   = v & 0b1100_0000 >> 6;
    }

    pub fn read_nr30(&self) -> u8 {
        return if self.enabled && self.sound3_on { 0b1000_0000 } else { 0 };
    }

    pub fn write_nr30(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.sound3_on = v & 0b1000_0000 > 0;
    }

    pub fn read_nr31(&self) -> u8 {
        if !self.enabled {
            return 0;
        }
        self.sound3_length
    }

    pub fn write_nr31(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.sound3_length = v;
    }

    pub fn read_nr32(&self) -> u8 {
        if !self.enabled {
            return 0;
        }
        self.sound3_level << 5
    }

    pub fn write_nr32(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.sound3_level = v >> 5;
    }

    pub fn read_nr33(&self) -> u8 {
        // Manual says this is write only.
        0
    }

    pub fn write_nr33(&mut self, v: u8) {
        self.sound3_freq = (self.sound3_freq & 0x700) | (v as u16);
    }

    pub fn read_nr34(&self) -> u8 {
        // Manual says only readable bit is continuous selection flag.
        return if self.enabled && !self.sound3_counter { 0b0100_0000 } else { 0 };
    }

    pub fn write_nr34(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.sound3_freq = (self.sound3_freq & 0xFF) | (((v & 7) as u16) << 8);
        self.sound3_counter = v & 0b0100_0000 > 0;
        
        if v & 0b1000_0000 > 0 {
            self.sound3_on = true;
            self.sound3_timer = 0;
            self.sound3_pos = 0;
        }
    }

    pub fn read_nr50(&self) -> u8 {
        if !self.enabled {
            return 0;
        }
        0
            | (self.left_vol & 3)
            | ((self.right_vol & 3) << 4)
    }

    pub fn write_nr50(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.left_vol  = v & 3;
        self.right_vol = (v >> 4) & 3;
    }

    pub fn read_nr51(&self) -> u8 {
        if !self.enabled {
            return 0;
        }
        0
            | if self.sound1_right { 0b0000_0001 } else { 0 }
            | if self.sound2_right { 0b0000_0010 } else { 0 }
            | if self.sound3_right { 0b0000_0100 } else { 0 }
            | if self.sound4_right { 0b0000_1000 } else { 0 }
            | if self.sound1_left  { 0b0001_0000 } else { 0 }
            | if self.sound2_left  { 0b0010_0000 } else { 0 }
            | if self.sound3_left  { 0b0100_0000 } else { 0 }
            | if self.sound4_left  { 0b1000_0000 } else { 0 }
    }

    pub fn write_nr51(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.sound1_right = v & 0b0000_0001 > 0;
        self.sound2_right = v & 0b0000_0010 > 0;
        self.sound3_right = v & 0b0000_0100 > 0;
        self.sound4_right = v & 0b0000_1000 > 0;
        self.sound1_left  = v & 0b0001_0000 > 0;
        self.sound2_left  = v & 0b0010_0000 > 0;
        self.sound3_left  = v & 0b0100_0000 > 0;
        self.sound4_left  = v & 0b1000_0000 > 0;
    }

    pub fn read_nr52(&self) -> u8 {
        0
            | if self.sound1_on     { 0b0000_0001 } else { 0 }
            | if self.sound2_on     { 0b0000_0010 } else { 0 }
            | if self.sound3_on     { 0b0000_0100 } else { 0 }
            | if self.sound4_on     { 0b0000_1000 } else { 0 }
            | if self.enabled       { 0b1000_0000 } else { 0 }
    }

    pub fn write_nr52(&mut self, v: u8) {
        self.enabled = v & 0b1000_0000 > 0;
        // Turning off sound controller turns off all voices.
        if !self.enabled {
            self.sample_cycles = 0.0;
            self.timer = 0;
            self.frame_seq_timer = 0;
            self.sound1_on = false;
            self.sound2_on = false;
            self.sound3_on = false;
            self.sound4_on = false;
        }
    }
}
