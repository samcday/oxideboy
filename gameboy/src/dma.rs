use ::GameboyContext;

#[derive(Default)]
pub struct DMAState {
    pub reg: u8,
    request: bool,
    pub active: bool,
    from: u16,
    idx: usize,
}

pub fn start(ctx: &mut GameboyContext, v: u8) {
    ctx.state.dma.reg = v;

    if !ctx.state.ppu.dma_ok() {
        return;
    }

    ctx.state.dma.request = true;
    ctx.state.dma.active = false;
    ctx.state.dma.idx = 0;
    ctx.state.dma.from = ((v & 0xFF) as u16) << 8;
}

/// Runs the DMA procedure when it's active. DMA transfers copy 160 bytes, one per cycle, from a configurable
/// source address into OAM memory. DMA transfers begin one extra cycle after being requested. They cannot be
/// stopped, but they can be restarted from a new location by writing to the DMA register again.
pub fn clock(ctx: &mut GameboyContext) {
    // When a DMA is initiated, it doesn't begin on the next cycle, but the one after.
    if ctx.state.dma.request {
        ctx.state.dma.request = false;
        ctx.state.dma.active = true;
        return;
    }

    if !ctx.state.dma.active {
        return;
    }

    let addr = ctx.state.dma.from + (ctx.state.dma.idx as u16);
    let v = ctx.mem_get8(addr);
    ctx.state.ppu.oam_write(ctx.state.dma.idx, v);
    ctx.state.dma.idx += 1;
    if ctx.state.dma.idx == 160 {
        ctx.state.dma.active = false;
    }
}
