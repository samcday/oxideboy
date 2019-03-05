//! DMA transfers are a faster way to copy data into the OAM entry table. Doing this manually with LD commands would
//! take at least 320 cycles. DMA takes only 160 cycles.

#[derive(Default)]
pub struct DmaController {
    pub reg: u8,
    request: bool,
    pub active: bool,
    from: u16,
    idx: usize,
}

impl DmaController {
    pub fn start(&mut self, mut v: u8) {
        self.reg = v;

        // TODO:
        // if !ctx.state.ppu.dma_ok() {
        //     return;
        // }

        self.request = true;
        self.active = false;
        self.idx = 0;

        // Source addresses higher or equal to 0xFE have 0x20 subtracted from them, because reasons.
        if v >= 0xFE {
            v -= 0x20;
        }

        self.from = u16::from(v) << 8;
    }

    /// Runs the DMA procedure when it's active. DMA transfers copy 160 bytes, one per cycle, from a configurable
    /// source address into OAM memory. DMA transfers begin one extra cycle after being requested. They cannot be
    /// stopped, but they can be restarted from a new location by writing to the DMA register again.
    /// Because of Rust borrowing rules, an active DMA copy isn't performed here, instead the address to copy from and
    /// to is returned.
    pub fn clock(&mut self) -> (bool, u16, usize) {
        // When a DMA is initiated, it doesn't begin on the next cycle, but the one after.
        if self.request {
            self.request = false;
            self.active = true;
            return (false, 0, 0);
        }

        if !self.active {
            return (false, 0, 0);
        }

        let addr = self.from + (self.idx as u16);
        let idx = self.idx;
        self.idx += 1;
        if self.idx == 160 {
            self.active = false;
        }

        (true, addr, idx)
    }
}
