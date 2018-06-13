#[derive(Debug, PartialEq)]
enum PPUState {
    Inactive,           // LCD is currently switched off.
    OAMSearch(u8),      // OAM search phase, with the number of cycles remaining.
    PixelTransfer(u8),  // 
    HBlank(u8),         // HBlank phase, with number of cycles remaining.
    VBlank(u8),
}
use self::PPUState::{*};

pub struct PPU {
    state: PPUState,
    pub scy: u8,
    pub scx: u8,
    pub ly: u8,
    pub lyc: u8,
    pub wx: u8,
    pub wy: u8,

    vram: [u8; 0x2000], // 0x8000 - 0x9FFF 
    oam:  [u8; 0x9F],   // 0xFE00 - 0xFDFF
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            state: Inactive,
            scy: 0, scx: 0,
            ly: 0, lyc: 0,
            wx: 0, wy: 0,
            vram: [0; 0x2000], oam: [0; 0x9F],
        }
    }

    pub fn oam_read(&self, addr: u16) -> u8 {
        match self.state {
            Inactive | HBlank(_) | VBlank(_) => self.oam[addr as usize],
            OAMSearch(_) | PixelTransfer(_) => 0xFF,
        }
    }

    pub fn oam_write(&mut self, addr: u16, v: u8) {
        match self.state {
            Inactive | HBlank(_) | VBlank(_) => { self.oam[addr as usize] = v },
            OAMSearch(_) | PixelTransfer(_) => { },
        }
    }

    pub fn vram_read(&self, addr: u16) -> u8 {
        match self.state {
            Inactive | OAMSearch(_) | HBlank(_) | VBlank(_) => self.vram[addr as usize],
            PixelTransfer(_) => 0xFF,
        }
    }

    pub fn vram_write(&mut self, addr: u16, v: u8) {
        match self.state {
            Inactive | OAMSearch(_) | HBlank(_) | VBlank(_) => { self.vram[addr as usize] = v },
            PixelTransfer(_) => { },
        }
    }

    // Advances PPU display by a single clock cycle. Basically, all the video magic happens in here.
    pub fn advance(&mut self) -> Option<::Interrupt> {
        let mut intr = None;

        // Each scanline is 114 clock cycles. 20 for OAM search, 43+ for pixel transfer, remaining for hblank.
        let new_state = match self.state {
            Inactive => { Inactive },
            OAMSearch(n) => {
                if n > 0 {
                    OAMSearch(n - 1)
                } else {
                    PixelTransfer(43)
                }
            },
            PixelTransfer(n) => {
                if n > 0 {
                    PixelTransfer(n - 1)
                } else {
                    HBlank(51)
                }
            },
            HBlank(n) => {
                if n > 0 {
                    HBlank(n - 1)
                } else if self.ly == 144 {
                    intr = Some(::Interrupt::VBlank);
                    VBlank(114)
                } else {
                    self.ly += 1;
                    OAMSearch(20)
                }
            },
            VBlank(n) => {
                if n > 0 {
                    VBlank(n - 1)
                } else if self.ly == 154 {
                    self.ly = 0;
                    OAMSearch(20)
                } else {
                    VBlank(114)
                }
            }
        };
        self.state = new_state;

        intr
    }

    // Compute value of the LCDC register.
    pub fn get_lcdc(&self) -> u8 {
        let mut lcdc = 0;

        if self.state != Inactive {
            lcdc |= 0x80;
        }

        lcdc
    }

    // Update PPU state based on new LCDC value.
    pub fn set_lcdc(&mut self, v: u8) {
        // Are we enabling LCD from a previously disabled state?
        if v & 0x80 > 0 && self.state == Inactive {
            self.state = OAMSearch(20);
        }
    }

    // Compute value of the STAT register.
    pub fn get_stat(&self) -> u8 {
        let stat = match self.state {
            Inactive => 0,
            HBlank(_) => 0,
            VBlank(_) => 1,
            OAMSearch(_) => 2,
            PixelTransfer(_) => 3,
        };
        stat
    }

    // Update PPU state based on new STAT value.
    pub fn set_stat(&mut self, _v: u8) {

    }
}
