#[derive(Debug, PartialEq)]
enum PPUState {
    Inactive,           // LCD is currently switched off.
    OAMSearch(u8),      // OAM search phase, with the number of cycles remaining.
    PixelTransfer(u8),  // 
    HBlank(u8),         // HBlank phase, with number of cycles remaining.
    VBlank(u8),
}
use self::PPUState::{*};

struct OAMEntry {
    y: u8,
    x: u8,
    code: u8,
    palette: u8,
    horz_flip: bool,
    vert_flip: bool,
    priority: bool,
}

pub struct PPU {
    state: PPUState,
    pub scy: u8,
    pub scx: u8,
    pub ly: u8,
    pub lyc: u8,
    pub wx: u8,
    pub wy: u8,

    vram: [u8; 0x2000], // 0x8000 - 0x9FFF 
    oam:  [u8; 0xA0],   // 0xFE00 - 0xFDFF

    scanline_objs: Vec<OAMEntry>,
}

impl PPU {
    pub fn new() -> PPU {
        PPU {
            state: Inactive,
            scy: 0, scx: 0,
            ly: 0, lyc: 0,
            wx: 0, wy: 0,
            vram: [0; 0x2000], oam: [0; 0xA0],
            scanline_objs: Vec::new(),
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
                self.oam_search(n);
                if n < 19 {
                    OAMSearch(n + 1)
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
                    OAMSearch(0)
                }
            },
            VBlank(n) => {
                if n > 0 {
                    VBlank(n - 1)
                } else if self.ly == 154 {
                    self.ly = 0;
                    OAMSearch(0)
                } else {
                    VBlank(114)
                }
            }
        };
        self.state = new_state;

        intr
    }

    // The first stage of a scanline. Takes 20 cycles.
    // In this stage, we search the OAM table for OBJs that overlap the current line (LY).
    // I'm sure it's totally different in hardware, but we simulate this process by searching 2 bytes
    // of OAM each cycle. OAM is 40 bytes and OAM Search takes 20 cycles, so this divides nicely. 
    fn oam_search(&mut self, n: u8) {
        let mut idx = 0;
        // TODO: this should be 16 in big sprite mode
        let h = 8;
        while self.scanline_objs.len() < 10 && idx < 2 {
            let obj = self.read_oam_entry(n * 2 + idx);
            if obj.x > 0 && self.ly + 16 >= obj.y && self.ly + 16 < obj.y + h {
                self.scanline_objs.push(obj);
            }
            idx += 1;
        }
    }

    fn read_oam_entry(&self, n: u8) -> OAMEntry {
        let addr = (n * 4) as usize;
        OAMEntry{
            y: self.oam[addr],
            x: self.oam[addr + 1],
            code: self.oam[addr + 2],
            palette: (self.oam[addr + 3] & 0x10) >> 4,
            horz_flip: self.oam[addr + 3] & 0x20 == 0x20,
            vert_flip: self.oam[addr + 3] & 0x40 == 0x40,
            priority: self.oam[addr + 3] & 0x80 == 0x80,
        }
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
            self.state = OAMSearch(0);
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
