const SAMPLE_RATE: f64 = 44100.0;

const DUTY_CYCLES: [[f32; 8]; 4] = [
    [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0],
    [1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0],
    [1.0, -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
    [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0],
];

pub struct SoundController<'cb> {
    sample_cycles: f64,

    timer: u16,
    frame_seq_timer: u8,         // Main timer to drive other clocks. Increments @ 512hz (2048 CPU cycles)

    enabled: bool,
    left_vol: u8,
    right_vol: u8,
    left_vin: bool,
    right_vin: bool,

    channel1: Channel1, // Tone & Sweep voice
    channel2: Channel2, // Tone voice
    channel3: Channel3, // Wave voice
    channel4: Channel4, // Noise voice

    pub wave_ram: WaveRam,

    cb: &'cb mut FnMut((f32, f32)),
}

#[derive(Default)]
struct VolumeEnvelope {
    val: u8,
    inc: bool,
    steps: u8,
}

impl VolumeEnvelope {
    fn from_u8(&mut self, v: u8) {
        self.steps =  v & 0b0000_0111;
        self.inc   =  v & 0b0000_1000 > 0;
        self.val   = (v & 0b1111_0000) >> 4;
    }

    fn to_u8(&self) -> u8 {
        0
            | self.steps
            | if self.inc { 0b0000_1000 } else { 0 }
            | (self.val << 4)
    }
}

#[derive(Default)]
struct Channel1 {
    on: bool,
    left: bool, right: bool,
    sweep_num: u8,
    sweep_sub: bool,
    sweep_time: u8,
    vol_env: VolumeEnvelope,
    length: u8,
    duty: u8,
    freq: u16,
    counter: bool,
    timer: u16,
    pos: u8,
}

#[derive(Default)]
struct Channel2 {
    on: bool,
    left: bool,
    right: bool,
    length: u8,
    duty: u8,
    vol_env: VolumeEnvelope,
    freq: u16,
    counter: bool,
}

#[derive(Default)]
struct Channel3 {
    on: bool,
    left: bool,
    right: bool,
    length: u16,
    level: u8,
    freq: u16,
    counter: bool,
    pos: u8,
    timer: u16,
}

#[derive(Default)]
struct Channel4 {
    on: bool,
    left: bool,
    right: bool,
    length: u8,
    vol_env: VolumeEnvelope,
    div_ratio: u8,
    poly_steps: bool,
    poly_freq: u8,
    counter: bool,
}

pub struct WaveRam {
    pub data: [u8; 16],
}

impl WaveRam {
    fn get_step(&self, n: u8) -> f32 {
        (self.data[(n / 2) as usize] >> (n % 2 * 4) & 0b1111) as f32
    }
}

impl <'cb> SoundController<'cb> {
    pub fn new(cb: &'cb mut FnMut((f32, f32))) -> SoundController {
        SoundController{
            sample_cycles: 0.0,
            timer: 0, frame_seq_timer: 0,

            enabled: true,
            left_vol: 0, right_vol: 0,
            left_vin: false, right_vin: false,

            channel1: Default::default(),
            channel2: Default::default(),
            channel3: Default::default(),
            channel4: Default::default(),
            wave_ram: WaveRam{data: [0; 16]},

            cb,
        }
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
            if self.frame_seq_timer % 4 == 3 {
                self.sweep_clock();
            }
            if self.frame_seq_timer == 8 {
                self.vol_env_clock();
                self.frame_seq_timer = 0;
            }
        }

        if self.channel1.on {
            self.channel1.timer += 4;
            if (self.channel1.timer / 16) >= (2048 - self.channel1.freq) {
                self.channel1.pos = self.channel1.pos.wrapping_add(1);
                self.channel1.timer = 0;
            }
        }

        if self.channel3.on {
            self.channel3.timer += 1;
            if (self.channel3.timer / 16) == (2048 - self.channel3.freq) {
                self.channel3.pos = (self.channel3.pos + 1) % 32;
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

        if self.channel1.on {
            let val = DUTY_CYCLES[self.channel1.duty as usize][(self.channel1.pos & 7) as usize] * 0.25;
            l = val;
            r = val;
        }

        // if self.channel3.on {
        //     l = self.channel3.wave_ram.get_step(self.channel3.pos) / 15.0 * 0.5;
        //     r = self.channel3.wave_ram.get_step(self.channel3.pos) / 15.0 * 0.5;
        // }

        (self.cb)((l, r));
        // self.samples.push(l);
        // self.samples.push(r);
    }

    fn length_clock(&mut self) {
        if self.channel1.counter {
            self.channel1.length += 1;
            if self.channel1.length == 64 {
                self.channel1.on = false;
                self.channel1.length = 0;
            }
        }

        if self.channel2.counter {
            self.channel2.length += 1;
            if self.channel2.length == 64 {
                self.channel2.on = false;
                self.channel2.length = 0;
            }
        }

        if self.channel3.counter {
            self.channel3.length += 1;
            if self.channel3.length == 255 {
                self.channel3.on = false;
                self.channel3.length = 0;
            }
        }

        if self.channel4.counter {
            self.channel4.length += 1;
            if self.channel4.length == 64 {
                self.channel4.on = false;
                self.channel4.length = 0;
            }
        }
    }

    fn vol_env_clock(&self) {

    }

    fn sweep_clock(&self) {

    }

    pub fn read_nr10(&self) -> u8 {
        0x80
            | self.channel1.sweep_num
            | if self.channel1.sweep_sub { 0b0000_1000 } else { 0 }
            | (self.channel1.sweep_time << 4)
    }

    pub fn write_nr10(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel1.sweep_num  =  v & 0x07;
        self.channel1.sweep_sub  =  v & 0b0000_1000 > 0;
        self.channel1.sweep_time = (v & 0b0111_0000) >> 4;
    }

    pub fn read_nr11(&self) -> u8 {
        0x3F | (self.channel1.duty << 6)
    }

    pub fn write_nr11(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel1.length =  v & 0b0011_1111;
        self.channel1.duty   = (v & 0b1100_0000) >> 6;
    }

    pub fn read_nr12(&self) -> u8 {
        self.channel1.vol_env.to_u8()
    }

    pub fn write_nr12(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel1.vol_env.from_u8(v);
    }

    pub fn read_nr13(&self) -> u8 {
        0xFF
    }

    pub fn write_nr13(&mut self, v: u8) {
        self.channel1.freq = (self.channel1.freq & 0x700) | (v as u16);
    }

    pub fn read_nr14(&self) -> u8 {
        0xBF | (if self.channel1.counter { 0b0100_0000 } else { 0 })
    }

    pub fn write_nr14(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel1.freq = (self.channel1.freq & 0xFF) | (((v & 0b111) as u16) << 8);
        self.channel1.counter = v & 0b0100_0000 > 0;
        if v & 0b1000_0000 > 0 {
            self.channel1.on = true;
        }
    }

    pub fn read_nr21(&self) -> u8 {
        0x3F | (self.channel2.duty << 6)
    }

    pub fn write_nr21(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel2.length =  v & 0b0011_1111;
        self.channel2.duty   = (v & 0b1100_0000) >> 6;
    }

    pub fn read_nr22(&self) -> u8 {
        self.channel2.vol_env.to_u8()
    }

    pub fn write_nr22(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel2.vol_env.from_u8(v);
    }

    pub fn read_nr23(&self) -> u8 {
        0xFF
    }

    pub fn write_nr23(&mut self, v: u8) {
        self.channel2.freq = (self.channel2.freq & 0x700) | (v as u16);
    }

    pub fn read_nr24(&self) -> u8 {
        0xBF | (if self.channel2.counter { 0b0100_0000 } else { 0 })
    }

    pub fn write_nr24(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel2.freq = (self.channel2.freq & 0xFF) | (((v & 0b111) as u16) << 8);
        self.channel2.counter = v & 0b0100_0000 > 0;

        if v & 0b1000_0000 > 0 {
            self.channel2.on = true;
        }
    }

    pub fn read_nr30(&self) -> u8 {
        0x7F | (if self.channel3.on { 0b1000_0000 } else { 0 })
    }

    pub fn write_nr30(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        if v & 0b1000_0000 > 0 {
            self.channel3.on = true;
        }
    }

    pub fn read_nr31(&self) -> u8 {
        0xFF
    }

    pub fn write_nr31(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel3.length = v as u16;
    }

    pub fn read_nr32(&self) -> u8 {
        0x9F | (self.channel3.level << 5)
    }

    pub fn write_nr32(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel3.level = v >> 5;
    }

    pub fn read_nr33(&self) -> u8 {
        0xFF
    }

    pub fn write_nr33(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel3.freq = (self.channel3.freq & 0x700) | (v as u16);
    }

    pub fn read_nr34(&self) -> u8 {
        // Manual says only readable bit is continuous selection flag.
        0xBF | (if self.channel3.counter { 0b0100_0000 } else { 0 })
    }

    pub fn write_nr34(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel3.freq = (self.channel3.freq & 0xFF) | (((v & 7) as u16) << 8);
        self.channel3.counter = v & 0b0100_0000 > 0;
        
        if v & 0b1000_0000 > 0 {
            self.channel3.on = true;
            self.channel3.timer = 0;
            self.channel3.pos = 0;
        }
    }

    pub fn read_nr41(&self) -> u8 {
        0xFF
    }

    pub fn write_nr41(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel4.length = v & 0b0011_1111;
    }

    pub fn read_nr42(&self) -> u8 {
        self.channel4.vol_env.to_u8()
    }

    pub fn write_nr42(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel4.vol_env.from_u8(v);
    }

    pub fn read_nr43(&self) -> u8 {
        0
            | self.channel4.div_ratio
            | if self.channel4.poly_steps { 0b0000_1000 } else { 0 }
            | (self.channel4.poly_freq << 4)
    }

    pub fn write_nr43(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel4.div_ratio  = v & 0b0000_0111;
        self.channel4.poly_steps = v & 0b0000_1000 > 0;
        self.channel4.poly_freq = (v & 0b1111_0000) >> 4;
    }

    pub fn read_nr44(&self) -> u8 {
        0xBF | (if self.channel4.counter { 0b0100_0000 } else { 0 })
    }

    pub fn write_nr44(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel4.counter = v & 0b0100_0000 > 0;
        if v & 0b1000_0000 > 0 {
            self.channel4.on = true;
        }
    }

    pub fn read_nr50(&self) -> u8 {
        0
            | self.left_vol
            | if self.left_vin { 0b0000_1000 } else { 0 }
            | (self.right_vol << 4)
            | if self.right_vin { 0b1000_0000 } else { 0 }
    }

    pub fn write_nr50(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.left_vol  = v & 7;
        self.left_vin  = v & 0b0000_1000 > 0;
        self.right_vol = (v >> 4) & 7;
        self.right_vin = v & 0b1000_0000 > 0;
    }

    pub fn read_nr51(&self) -> u8 {
        0
            | if self.channel1.right { 0b0000_0001 } else { 0 }
            | if self.channel2.right { 0b0000_0010 } else { 0 }
            | if self.channel3.right { 0b0000_0100 } else { 0 }
            | if self.channel4.right { 0b0000_1000 } else { 0 }
            | if self.channel1.left  { 0b0001_0000 } else { 0 }
            | if self.channel2.left  { 0b0010_0000 } else { 0 }
            | if self.channel3.left  { 0b0100_0000 } else { 0 }
            | if self.channel4.left  { 0b1000_0000 } else { 0 }
    }

    pub fn write_nr51(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.channel1.right = v & 0b0000_0001 > 0;
        self.channel2.right = v & 0b0000_0010 > 0;
        self.channel3.right = v & 0b0000_0100 > 0;
        self.channel4.right = v & 0b0000_1000 > 0;
        self.channel1.left  = v & 0b0001_0000 > 0;
        self.channel2.left  = v & 0b0010_0000 > 0;
        self.channel3.left  = v & 0b0100_0000 > 0;
        self.channel4.left  = v & 0b1000_0000 > 0;
    }

    pub fn read_nr52(&self) -> u8 {
        0x70
            | if self.channel1.on     { 0b0000_0001 } else { 0 }
            | if self.channel2.on     { 0b0000_0010 } else { 0 }
            | if self.channel3.on     { 0b0000_0100 } else { 0 }
            | if self.channel4.on     { 0b0000_1000 } else { 0 }
            | if self.enabled       { 0b1000_0000 } else { 0 }
    }

    pub fn write_nr52(&mut self, v: u8) {
        self.enabled = v & 0b1000_0000 > 0;
        // Turning off sound controller turns off all voices.
        if !self.enabled {
            self.sample_cycles = 0.0;
            self.timer = 0;
            self.frame_seq_timer = 0;
            self.channel1 = Default::default();
            self.channel2 = Default::default();
            self.channel3 = Default::default();
            self.channel4 = Default::default();
            self.left_vol = 0;
            self.left_vin = false;
            self.right_vol = 0;
            self.right_vin = false;
        }
    }
}
