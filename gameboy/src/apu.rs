const SAMPLE_RATE: f64 = 44100.0;

const DUTY_CYCLES: [[f32; 8]; 4] = [
    [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,  1.0],
    [ 1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,  1.0],
    [ 1.0, -1.0, -1.0, -1.0, -1.0,  1.0,  1.0,  1.0],
    [-1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0, -1.0],
];
const NOISE_DIVISORS: [u16; 8] = [8, 16, 32, 48, 64, 80, 96, 112];

pub struct APUState {
    sample_cycles: f64,

    timer: u16,
    frame_seq_timer: u8,         // Main timer to drive other clocks. Increments @ 512hz (2048 CPU cycles)

    enabled: bool,
    left_vol: u8,
    right_vol: u8,
    left_vin: bool,
    right_vin: bool,

    pub chan1: Channel1, // Tone & Sweep voice
    chan2: Channel2, // Tone voice
    chan3: Channel3, // Wave voice
    chan4: Channel4, // Noise voice

    wave_ram: [u8; 32],

    pub sample_queue: Vec<f32>,
}

/// Runs APU for a single CPU (1MHz) clock cycle.
pub fn clock(state: &mut APUState) {
    if !state.enabled {
        return;
    }

    state.timer += 1;
    state.chan3.wav_read_clocks = state.chan3.wav_read_clocks.saturating_sub(1);

    if state.timer == 2048 {
        state.timer = 0;
        state.frame_seq_timer = state.frame_seq_timer.wrapping_add(1);

        if state.frame_seq_timer % 2 == 1 {
            if state.chan1.counter { length_clock(64,  &mut state.chan1.length, &mut state.chan1.on); }
            if state.chan2.counter { length_clock(64,  &mut state.chan2.length, &mut state.chan2.on); }
            if state.chan3.counter { length_clock(256, &mut state.chan3.length, &mut state.chan3.on); }
            if state.chan4.counter { length_clock(64,  &mut state.chan4.length, &mut state.chan4.on); }
        }
        if state.frame_seq_timer % 4 == 3 {
            state.chan1.sweep_clock();
        }
        if state.frame_seq_timer == 8 {
            state.chan1.vol_env.clock();
            state.chan2.vol_env.clock();
            state.chan4.vol_env.clock();
            state.frame_seq_timer = 0;
        }
    }

    if state.chan1.on {
        state.chan1.freq_timer += 4;
        if (state.chan1.freq_timer / 4) >= (2048 - state.chan1.freq) {
            state.chan1.pos = state.chan1.pos.wrapping_add(1);
            state.chan1.freq_timer = 0;
        }
    }

    if state.chan2.on {
        state.chan2.freq_timer += 4;
        if (state.chan2.freq_timer / 4) >= (2048 - state.chan2.freq) {
            state.chan2.pos = state.chan2.pos.wrapping_add(1);
            state.chan2.freq_timer = 0;
        }
    }

    if state.chan3.on {
        state.chan3.freq_timer += 4;
        if (state.chan3.freq_timer / 2) >= (2048 - state.chan3.freq) {
            state.chan3.pos = (state.chan3.pos + 1) % 32;
            state.chan3.wav_read_clocks = 2;
            state.chan3.freq_timer = 0;
        }
    }

    if state.chan4.on {
        state.chan4.freq_timer = state.chan4.freq_timer.saturating_add(4);
        let freq = NOISE_DIVISORS[state.chan4.div_ratio as usize] << state.chan4.poly_freq;
        if (state.chan4.freq_timer / 8) >= freq {
            state.chan4.noise_clock();
            state.chan4.freq_timer = 0;
        }
    }

    state.sample_cycles += 1.0;
    let cycles_per_sample = 1048576.0 / SAMPLE_RATE;
    if state.sample_cycles >= cycles_per_sample {
        state.sample_cycles -= cycles_per_sample;
        state.generate_sample();
    }
}

fn length_clock(max: u16, len: &mut u16, on: &mut bool) {
    if *len < max {
        *len += 1;
        if *len == max {
            *on = false;
        }
    }
}

#[derive(Debug, Default)]
pub struct VolumeEnvelope {
    pub default: u8,
    pub inc: bool,
    pub steps: u8,
    pub val: u8,
    pub timer: u8,
}

impl VolumeEnvelope {
    fn unpack(&mut self, v: u8) {
        self.steps   =  v & 0b0000_0111;
        self.inc     =  v & 0b0000_1000 > 0;
        self.default = (v & 0b1111_0000) >> 4;

        if self.steps == 0 {
            self.val = self.default;
        }
    }

    fn pack(&self) -> u8 {
        0
            | self.steps
            | if self.inc { 0b0000_1000 } else { 0 }
            | (self.default << 4)
    }

    fn is_zero(&self) -> bool {
        !self.inc && self.val == 0
    }

    fn reload(&mut self) {
        self.val = self.default;
        self.timer = self.steps;
    }

    fn clock(&mut self) {
        if self.val > 0 {
            self.timer = self.timer.saturating_sub(1);
            if self.timer == 0 {
                self.val = if self.inc {
                     (self.val + 1).min(15)
                } else {
                    self.val.saturating_sub(1)
                };
                self.timer = self.steps;
            }
        }
    }
}

#[derive(Debug, Default)]
pub struct Channel1 {
    pub on: bool,
    left: bool, right: bool,
    sweep_shift: u8,
    sweep_sub: bool,
    sweep_period: u8,
    sweep_timer: u8,
    sweep_shadow_freq: u16,
    sweep_enabled: bool,
    sweep_has_negated: bool,
    pub vol_env: VolumeEnvelope,
    length: u16,
    duty: u8,
    freq: u16,
    counter: bool,
    freq_timer: u16,
    pos: u8,
}

impl Channel1 {
    fn sweep_next_freq(&mut self) -> u16 {
        self.sweep_has_negated = self.sweep_sub;
        if self.sweep_sub {
            self.sweep_shadow_freq.saturating_sub(self.sweep_shadow_freq >> self.sweep_shift)
        } else {
            self.sweep_shadow_freq + (self.sweep_shadow_freq >> self.sweep_shift)
        }
    }

    fn sweep_clock(&mut self) {
        if !self.sweep_enabled {
            return;
        }

        self.sweep_timer = self.sweep_timer.saturating_sub(1);
        if self.sweep_timer == 0 {
            self.sweep_timer = if self.sweep_period > 0 { self.sweep_period } else { 8 };

            if self.sweep_period > 0 {
                let new_freq = self.sweep_next_freq();
                if new_freq > 2047 {
                    self.on = false;
                } else if self.sweep_shift > 0 {
                    self.sweep_shadow_freq = new_freq;
                    self.freq = self.sweep_shadow_freq;

                    if self.sweep_next_freq() > 2047 {
                        self.on = false;
                    }
                }
            }
        }
    }

    fn reload_sweep(&mut self) -> bool {
        self.sweep_has_negated = false;
        self.sweep_enabled = self.sweep_period > 0 || self.sweep_shift > 0;
        self.sweep_shadow_freq = self.freq;
        self.sweep_timer = if self.sweep_period > 0 { self.sweep_period } else { 8 };

        if self.sweep_shift > 0 {
            self.sweep_next_freq() <= 2047
        } else { true }
    }

    fn get_output(&self, l: f32, r: f32) -> (f32, f32) {
        if !self.on {
            return (l, r);
        }
        let val = DUTY_CYCLES[self.duty as usize][(self.pos & 7) as usize]
            * (self.vol_env.val as f32) / 15.0;
        (
            l + if self.left  { val } else { 0.0 },
            r + if self.right { val } else { 0.0 }
        )
    }
}

#[derive(Default)]
struct Channel2 {
    on: bool,
    left: bool,
    right: bool,
    length: u16,
    duty: u8,
    vol_env: VolumeEnvelope,
    freq: u16,
    counter: bool,
    freq_timer: u16,
    pos: u8,
}

impl Channel2 {
    fn get_output(&self, l: f32, r: f32) -> (f32, f32) {
        if !self.on {
            return (l, r);
        }
        let val = DUTY_CYCLES[self.duty as usize][(self.pos & 7) as usize]
            * (self.vol_env.val as f32) / 15.0;
        (
            l + if self.left  { val } else { 0.0 },
            r + if self.right { val } else { 0.0 }
        )
    }
}

#[derive(Default)]
struct Channel3 {
    enabled: bool,
    on: bool,
    left: bool,
    right: bool,
    length: u16,
    level: u8,
    freq: u16,
    counter: bool,
    pos: usize,
    freq_timer: u16,
    wav_read_clocks: u16,
}

impl Channel3 {
    fn get_output(&self, l: f32, r: f32, wav: u8) -> (f32, f32) {
        if !self.on {
            return (l, r);
        }
        let wav = (if self.level == 0 { 0. }
                   else if self.level == 1 { (wav as f32) - 7. }
                   else { ((wav >> (self.level - 1)) as f32) - 7. }) / 7.;
        (
            l + if self.left  { wav } else { 0.0 },
            r + if self.right { wav } else { 0.0 }
        )
    }
}

#[derive(Default)]
struct Channel4 {
    on: bool,
    left: bool,
    right: bool,
    length: u16,
    vol_env: VolumeEnvelope,
    div_ratio: u8,
    poly_7bit: bool,
    poly_freq: u8,
    counter: bool,
    lfsr: u16,
    freq_timer: u16,
}

impl Channel4 {
    fn noise_clock(&mut self) {
        let new_bit = (self.lfsr & 1) ^ ((self.lfsr & 2) >> 1);
        self.lfsr >>= 1;
        self.lfsr |= new_bit << 14;
        if self.poly_7bit { self.lfsr |= new_bit << 6; }
    }

    fn get_output(&self, l: f32, r: f32) -> (f32, f32) {
        if !self.on {
            return (l, r);
        }
        let val = if (self.lfsr & 1) ^ 1 == 1 { 1.0 } else { -1.0 }
            * (self.vol_env.val as f32) / 15.0;;
        (
            l + if self.left  { val } else { 0.0 },
            r + if self.right { val } else { 0.0 }
        )
    }
}

impl APUState {
    pub fn new() -> APUState {
        APUState{
            sample_cycles: 0.0,
            timer: 0, frame_seq_timer: 0,

            enabled: true,
            left_vol: 0, right_vol: 0,
            left_vin: false, right_vin: false,

            chan1: Default::default(),
            chan2: Default::default(),
            chan3: Default::default(),
            chan4: Default::default(),
            wave_ram: [0; 32],

            sample_queue: Vec::new(),
        }
    }

    fn generate_sample(&mut self) {
        let l = 0.0;
        let r = 0.0;

        let (l, r) = self.chan1.get_output(l, r);
        let (l, r) = self.chan2.get_output(l, r);
        let (l, r) = self.chan3.get_output(l, r, self.wave_ram[self.chan3.pos]);
        let (l, r) = self.chan4.get_output(l, r);

        // Divide each value by the number of channels.
        let l = l / 4.; 
        let r = r / 4.;

        // Master volume control.
        let l = l * (self.left_vol as f32) / 7.;
        let r = r * (self.right_vol as f32) / 7.;

        if cfg!(test) {
            return;
        }

        self.sample_queue.push(l);
        self.sample_queue.push(r);
    }

    pub fn reg_nr10_read(&self) -> u8 {
        0b1000_0000 // Unreadable bits
            | self.chan1.sweep_shift
            | if self.chan1.sweep_sub { 0b0000_1000 } else { 0 }
            | (self.chan1.sweep_period << 4)
    }

    pub fn reg_nr10_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        let was_sub = self.chan1.sweep_sub;
        self.chan1.sweep_shift  =  v & 0b0000_0111;
        self.chan1.sweep_sub    =  v & 0b0000_1000 > 0;
        self.chan1.sweep_period = (v & 0b0111_0000) >> 4;

        // If, since enabling sweep, we've subtracted freqency, then switching from subtraction mode to
        // addition mode causes the channel to be instantly disabled.
        if self.chan1.sweep_has_negated && was_sub && !self.chan1.sweep_sub {
            self.chan1.on = false;
        }
    }

    pub fn reg_nr11_read(&self) -> u8 {
        /* Unreadable bits */ 0b0011_1111 | (self.chan1.duty << 6)
    }

    pub fn reg_nr11_write(&mut self, v: u8) {
        self.chan1.length = v as u16 & 0b0011_1111;
        if self.enabled {
            self.chan1.duty = (v & 0b1100_0000) >> 6;
        }
    }

    pub fn reg_nr12_read(&self) -> u8 {
        self.chan1.vol_env.pack()
    }

    pub fn reg_nr12_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan1.vol_env.unpack(v);
        if self.chan1.vol_env.is_zero() {
            self.chan1.on = false;
        }
    }

    pub fn reg_nr13_read(&self) -> u8 {
        0b1111_1111 // NR13 has no readable bits
    }

    pub fn reg_nr13_write(&mut self, v: u8) {
        self.chan1.freq = (self.chan1.freq & 0x700) | (v as u16);
    }

    pub fn reg_nr14_read(&self) -> u8 {
        /* Unreadable bits */ 0b1011_1111 | (if self.chan1.counter { 0b0100_0000 } else { 0 })
    }

    pub fn reg_nr14_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan1.freq = (self.chan1.freq & 0xFF) | (((v & 0b111) as u16) << 8);
        let prev_counter = self.chan1.counter;
        self.chan1.counter = v & 0b0100_0000 > 0;

        // If length counter was previously disabled and is now being enabled, we force an extra
        // clock of the length immediately. This is a hardware quirk.
        if self.frame_seq_timer % 2 == 1 && !prev_counter && self.chan1.counter {
            self.chan1.length = (self.chan1.length + 1).min(64);
            if self.chan1.length == 64 {
                self.chan1.on = false;
            }
        }

        if v & 0b1000_0000 > 0 {
            self.chan1.on = true;

            self.chan1.vol_env.reload();
            if self.chan1.vol_env.is_zero() {
                self.chan1.on = false;
            }

            if !self.chan1.reload_sweep() {
                self.chan1.on = false;
            }

            if self.chan1.length == 64 {
                self.chan1.length = if self.chan1.counter && self.frame_seq_timer % 2 == 1 { 1 } else { 0 };
            }
        }
    }

    pub fn reg_nr21_read(&self) -> u8 {
        /* Unreadable bits */ 0b0011_1111 | (self.chan2.duty << 6)
    }

    pub fn reg_nr21_write(&mut self, v: u8) {
        self.chan2.length = v as u16 & 0b0011_1111;
        if self.enabled {
            self.chan2.duty = (v & 0b1100_0000) >> 6;
        }
    }

    pub fn reg_nr22_read(&self) -> u8 {
        self.chan2.vol_env.pack()
    }

    pub fn reg_nr22_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan2.vol_env.unpack(v);
        if self.chan2.vol_env.is_zero() {
            self.chan2.on = false;
        }
    }

    pub fn reg_nr23_read(&self) -> u8 {
        0b1111_1111 // NR23 has no readable bits
    }

    pub fn reg_nr23_write(&mut self, v: u8) {
        self.chan2.freq = (self.chan2.freq & 0x700) | (v as u16);
    }

    pub fn reg_nr24_read(&self) -> u8 {
        /* Unreadable bits */ 0b1011_1111 | (if self.chan2.counter { 0b0100_0000 } else { 0 })
    }

    pub fn reg_nr24_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan2.freq = (self.chan2.freq & 0xFF) | (((v & 0b111) as u16) << 8);
        let prev_counter = self.chan2.counter;
        self.chan2.counter = v & 0b0100_0000 > 0;

        // If length counter was previously disabled and is now being enabled, we force an extra
        // clock of the length immediately. This is a hardware quirk.
        if self.frame_seq_timer % 2 == 1 && !prev_counter && self.chan2.counter {
            self.chan2.length = (self.chan2.length + 1).min(64);
            if self.chan2.length == 64 {
                self.chan2.on = false;
            }
        }

        if v & 0b1000_0000 > 0 {
            self.chan2.vol_env.reload();
            if !self.chan2.vol_env.is_zero() {
                self.chan2.on = true;
            }
            if self.chan2.length == 64 {
                self.chan2.length = if self.chan2.counter && self.frame_seq_timer % 2 == 1 { 1 } else { 0 };
            }
        }
    }

    pub fn reg_nr30_read(&self) -> u8 {
        /* Unreadable bits */ 0b0111_1111 | (if self.chan3.enabled { 0b1000_0000 } else { 0 })
    }

    pub fn reg_nr30_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan3.enabled = v & 0b1000_0000 > 0;
        if !self.chan3.enabled {
            self.chan3.on = false;
        }
    }

    pub fn reg_nr31_read(&self) -> u8 {
        0b1111_1111 // NR31 has no readable bits
    }

    pub fn reg_nr31_write(&mut self, v: u8) {
        self.chan3.length = v as u16;
    }

    pub fn reg_nr32_read(&self) -> u8 {
        /* Unreadable bits */ 0b1001_1111 | (self.chan3.level << 5)
    }

    pub fn reg_nr32_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan3.level = v >> 5;

        // TODO: this looks wrong so I commented it out, why was it here?
        // if self.chan3.level == 0 {
        //     self.chan3.on = false;
        // }
    }

    pub fn reg_nr33_read(&self) -> u8 {
        0b1111_1111 // NR33 has no readable bits
    }

    pub fn reg_nr33_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan3.freq = (self.chan3.freq & 0x700) | (v as u16);
    }

    pub fn reg_nr34_read(&self) -> u8 {
        /* Unreadable bits */ 0b1011_1111 | (if self.chan3.counter { 0b0100_0000 } else { 0 })
    }

    pub fn reg_nr34_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan3.freq = (self.chan3.freq & 0xFF) | (((v & 7) as u16) << 8);
        let prev_counter = self.chan3.counter;
        self.chan3.counter = v & 0b0100_0000 > 0;

        // If length counter was previously disabled and is now being enabled, we force an extra
        // clock of the length immediately. This is a hardware quirk.
        if self.frame_seq_timer % 2 == 1 && !prev_counter && self.chan3.counter {
            self.chan3.length = (self.chan3.length + 1).min(256);
            if self.chan3.length == 256 {
                self.chan3.on = false;
            }
        }

        if v & 0b1000_0000 > 0 {
            if self.chan3.enabled {
                self.chan3.on = true;
                self.chan3.pos = 0;
            }

            if self.chan3.length == 256 {
                self.chan3.length = if self.chan3.counter && self.frame_seq_timer % 2 == 1 { 1 } else { 0 };
            }
        }
    }

    pub fn reg_nr41_read(&self) -> u8 {
        0b1111_1111 // NR41 has no readable bits
    }

    pub fn reg_nr41_write(&mut self, v: u8) {
        self.chan4.length = v as u16 & 0b0011_1111;
    }

    pub fn reg_nr42_read(&self) -> u8 {
        self.chan4.vol_env.pack()
    }

    pub fn reg_nr42_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan4.vol_env.unpack(v);
        if self.chan4.vol_env.is_zero() {
            self.chan4.on = false;
        }
    }

    pub fn reg_nr43_read(&self) -> u8 {
        0
            | self.chan4.div_ratio
            | if self.chan4.poly_7bit { 0b0000_1000 } else { 0 }
            | (self.chan4.poly_freq << 4)
    }

    pub fn reg_nr43_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.chan4.div_ratio  = v & 0b0000_0111;
        self.chan4.poly_7bit  = v & 0b0000_1000 > 0;
        self.chan4.poly_freq = (v & 0b1111_0000) >> 4;
    }

    pub fn reg_nr44_read(&self) -> u8 {
        /* Unreadable bits */ 0b1011_1111 | (if self.chan4.counter { 0b0100_0000 } else { 0 })
    }

    pub fn reg_nr44_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        let prev_counter = self.chan4.counter;
        self.chan4.counter = v & 0b0100_0000 > 0;

        // If length counter was previously disabled and is now being enabled, we force an extra
        // clock of the length immediately. This is a hardware quirk.
        if self.frame_seq_timer % 2 == 1 && !prev_counter && self.chan4.counter {
            self.chan4.length = (self.chan4.length + 1).min(64);
            if self.chan4.length == 64 {
                self.chan4.on = false;
            }
        }

        if v & 0b1000_0000 > 0 {
            self.chan4.on = true;
            self.chan4.lfsr = 0b0111_1111_1111_1111;

            self.chan4.vol_env.reload();
            if self.chan4.vol_env.is_zero() {
                self.chan4.on = false;
            }

            if self.chan4.length == 64 {
                self.chan4.length = if self.chan4.counter && self.frame_seq_timer % 2 == 1 { 1 } else { 0 };
            }
        }
    }

    pub fn reg_nr50_read(&self) -> u8 {
        0
            | self.left_vol
            | if self.left_vin { 0b0000_1000 } else { 0 }
            | (self.right_vol << 4)
            | if self.right_vin { 0b1000_0000 } else { 0 }
    }

    pub fn reg_nr50_write(&mut self, v: u8) {
        if !self.enabled {
            return;
        }
        self.left_vol  = v & 7;
        self.left_vin  = v & 0b0000_1000 > 0;
        self.right_vol = (v >> 4) & 7;
        self.right_vin = v & 0b1000_0000 > 0;
    }

    pub fn reg_nr51_read(&self) -> u8 {
        0
            | if self.chan1.right { 0b0000_0001 } else { 0 }
            | if self.chan2.right { 0b0000_0010 } else { 0 }
            | if self.chan3.right { 0b0000_0100 } else { 0 }
            | if self.chan4.right { 0b0000_1000 } else { 0 }
            | if self.chan1.left  { 0b0001_0000 } else { 0 }
            | if self.chan2.left  { 0b0010_0000 } else { 0 }
            | if self.chan3.left  { 0b0100_0000 } else { 0 }
            | if self.chan4.left  { 0b1000_0000 } else { 0 }
    }

    pub fn reg_nr51_write(&mut self, v: u8) {
        if self.enabled {
            self.chan1.right = v & 0b0000_0001 > 0;
            self.chan2.right = v & 0b0000_0010 > 0;
            self.chan3.right = v & 0b0000_0100 > 0;
            self.chan4.right = v & 0b0000_1000 > 0;
            self.chan1.left  = v & 0b0001_0000 > 0;
            self.chan2.left  = v & 0b0010_0000 > 0;
            self.chan3.left  = v & 0b0100_0000 > 0;
            self.chan4.left  = v & 0b1000_0000 > 0;
        }
    }

    pub fn reg_nr52_read(&self) -> u8 {
        0b0111_0000 // Unreadable bits
            | if self.chan1.on { 0b0000_0001 } else { 0 }
            | if self.chan2.on { 0b0000_0010 } else { 0 }
            | if self.chan3.on { 0b0000_0100 } else { 0 }
            | if self.chan4.on { 0b0000_1000 } else { 0 }
            | if self.enabled     { 0b1000_0000 } else { 0 }
    }

    pub fn reg_nr52_write(&mut self, v: u8) {
        self.enabled = v & 0b1000_0000 > 0;
        // Turning off sound controller turns off all voices.
        if !self.enabled {
            self.frame_seq_timer = 0;
            self.sample_cycles = 0.0;
            // When resetting the channels, we preserve the length field only.
            self.chan1 = Channel1{ length: self.chan1.length, ..Default::default() };
            self.chan2 = Channel2{ length: self.chan2.length, ..Default::default() };
            self.chan3 = Channel3{ length: self.chan3.length, ..Default::default() };
            self.chan4 = Channel4{ length: self.chan4.length, ..Default::default() };
            self.left_vol = 0;
            self.left_vin = false;
            self.right_vol = 0;
            self.right_vin = false;
        }
    }

    pub fn wave_read(&self, mut addr: u16) -> u8 {
        if self.chan3.on {
            if self.chan3.wav_read_clocks == 0 {
                return 0xFF;
            }

            addr = self.chan3.pos as u16;
        }

        let base = (addr as usize)*2;
        (self.wave_ram[base] << 4) | self.wave_ram[base+1]
    }

    pub fn wave_write(&mut self, addr: u16, v: u8) {
        let base = (addr as usize) * 2;
        self.wave_ram[base] = v >> 4;
        self.wave_ram[base + 1] = v & 0b1111;
    }
}
