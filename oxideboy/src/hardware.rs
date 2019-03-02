//! Hardware interface for the Gameboy. Implements the 16-bit memory bus that the CPU is connected to, and handled
//! pumping clock cycles through all the relevant components.

use crate::apu::Apu;
use crate::cartridge::Cartridge;
use crate::dma::DmaController;
use crate::interrupt::InterruptController;
use crate::joypad::Joypad;
use crate::ppu::Ppu;
use crate::serial::Serial;
use crate::timer::Timer;
use crate::Model;

static DMG0_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg0.rom");
static DMG_BOOTROM: &[u8; 256] = include_bytes!("bootroms/dmg.rom");

/// Holds all the various components of the Gameboy, aside from the CPU.
pub struct GameboyHardware {
    pub model: Model,
    pub cart: Cartridge,
    pub apu: Apu,
    pub dma: DmaController,
    pub interrupts: InterruptController,
    pub joypad: Joypad,
    pub ppu: Ppu,
    pub serial: Serial,
    pub timer: Timer,
    pub ram: [u8; 0x2000], // 0xC000 - 0xDFFF (and echoed in 0xE000 - 0xFDFF)
    pub hram: [u8; 0x7F],  // 0xFF80 - 0xFFFE

    pub bootrom_enabled: bool,

    pub cycle_count: u32,

    pub new_frame: bool, // Set to true when the PPU has completed rendering of a whole frame.
}

impl GameboyHardware {
    pub fn new(model: Model, rom: Vec<u8>) -> GameboyHardware {
        let cart = Cartridge::from_rom(rom);

        GameboyHardware {
            model,
            cart,
            apu: Apu::new(),
            dma: Default::default(),
            interrupts: InterruptController::new(),
            joypad: Default::default(),
            ppu: Ppu::new(),
            serial: Serial::new(),
            timer: Timer::new(),
            ram: [0; 0x2000],
            hram: [0; 0x7F],
            bootrom_enabled: true,

            cycle_count: 0,
            new_frame: false,
        }
    }

    fn bootrom_read(&self, addr: u16) -> u8 {
        match self.model {
            Model::DMG0 => DMG0_BOOTROM[addr as usize],
            Model::DMG => DMG_BOOTROM[addr as usize],
        }
    }

    /// Handles reads from the memory bus. This method is responsible for resolving memory addresses to the correct
    /// memory segments and registers. Generally though, mem_read should be used since it ensures components are
    /// clocked correctly and reads are prevented during situations like active DMA transfers.
    pub fn mem_get(&self, addr: u16) -> u8 {
        match addr {
            0x0000...0x0100 if self.bootrom_enabled => self.bootrom_read(addr),
            0x0000...0x3FFF => self.cart.rom_lo(addr as usize),
            0x4000...0x7FFF => self.cart.rom_hi((addr - 0x4000) as usize),
            0x8000...0x9FFF => self.ppu.vram_read(addr - 0x8000),
            0xA000...0xBFFF => self.cart.ram((addr - 0xA000) as usize),
            0xC000...0xDFFF => self.ram[(addr - 0xC000) as usize],
            0xE000...0xFDFF => self.ram[(addr - 0xE000) as usize],
            0xFE00...0xFE9F => self.ppu.oam_read((addr - 0xFE00) as usize),
            0xFF00 => self.joypad.reg_p1_read(),
            0xFF01 => self.serial.reg_sb_read(),
            0xFF02 => self.serial.reg_sc_read(),
            0xFF04 => (self.timer.div >> 8) as u8,
            0xFF05 => self.timer.tima,
            0xFF06 => self.timer.tma,
            0xFF07 => self.timer.reg_tac_read(),
            0xFF0F => 0xE0 | self.interrupts.request, // Unused IF bits are always 1
            0xFF10 => self.apu.reg_nr10_read(),
            0xFF11 => self.apu.reg_nr11_read(),
            0xFF12 => self.apu.reg_nr12_read(),
            0xFF13 => self.apu.reg_nr13_read(),
            0xFF14 => self.apu.reg_nr14_read(),
            0xFF16 => self.apu.reg_nr21_read(),
            0xFF17 => self.apu.reg_nr22_read(),
            0xFF18 => self.apu.reg_nr23_read(),
            0xFF19 => self.apu.reg_nr24_read(),
            0xFF1A => self.apu.reg_nr30_read(),
            0xFF1B => self.apu.reg_nr31_read(),
            0xFF1C => self.apu.reg_nr32_read(),
            0xFF1D => self.apu.reg_nr33_read(),
            0xFF1E => self.apu.reg_nr34_read(),
            0xFF20 => self.apu.reg_nr41_read(),
            0xFF21 => self.apu.reg_nr42_read(),
            0xFF22 => self.apu.reg_nr43_read(),
            0xFF23 => self.apu.reg_nr44_read(),
            0xFF24 => self.apu.reg_nr50_read(),
            0xFF25 => self.apu.reg_nr51_read(),
            0xFF26 => self.apu.reg_nr52_read(),
            0xFF30...0xFF3F => self.apu.wave_read(addr - 0xFF30),
            0xFF40 => self.ppu.reg_lcdc_read(),
            0xFF41 => self.ppu.reg_stat_read(),
            0xFF42 => self.ppu.scy,
            0xFF43 => self.ppu.scx,
            0xFF44 => self.ppu.ly,
            0xFF45 => self.ppu.lyc,
            0xFF46 => self.dma.reg,
            0xFF47 => self.ppu.bgp.pack(),
            0xFF48 => self.ppu.obp0.pack(),
            0xFF49 => self.ppu.obp1.pack(),
            0xFF4A => self.ppu.wy as u8,
            0xFF4B => self.ppu.wx as u8,
            0xFF80...0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF => self.interrupts.enable,

            _ => 0xFF, // Reads from unhandled locations see 0xFF
        }
    }

    pub fn mem_read(&mut self, addr: u16) -> u8 {
        // While DMA transfer is in progress, reads to the OAM area will see 0xFF.
        let block_read = self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F);

        // Here's a batshit hardware quirk, the PPU is out of sync with the CPU by 2 T-cycles. To simulate this effect,
        // we perform the read *before* running the downstream hardware clocks, if we're reading a PPU register.
        if addr >= 0xFF40 && addr <= 0xFF4B {
            let v = self.mem_get(addr);
            self.clock();
            return v;
        }

        // Reading from the memory bus takes a full CPU cycle.
        self.clock();

        if block_read {
            0xFF
        } else {
            self.mem_get(addr)
        }
    }
    pub fn mem_read16(&mut self, addr: u16) -> u16 {
        let mut v = u16::from(self.mem_read(addr));
        v |= u16::from(self.mem_read(addr + 1)) << 8;
        v
    }

    pub fn mem_write(&mut self, addr: u16, v: u8) {
        // While DMA transfer is in progress, write to the OAM area will be ignored.
        let block_write = self.dma.active && (addr >= 0xFE00 && addr <= 0xFE9F);

        // Writing to the memory bus takes a full CPU cycle.
        self.clock();

        if block_write {
            return;
        }

        self.mem_set(addr, v);
    }

    pub fn mem_set(&mut self, addr: u16, v: u8) {
        match addr {
            0x0000...0x7FFF => self.cart.write(addr, v),
            0x8000...0x9FFF => self.ppu.vram_write(addr - 0x8000, v),
            0xA000...0xBFFF => self.cart.write(addr, v),
            0xC000...0xDFFF => self.ram[(addr - 0xC000) as usize] = v,
            0xE000...0xFDFF => self.ram[(addr - 0xE000) as usize] = v,
            0xFE00...0xFE9F => self.ppu.oam_write((addr - 0xFE00) as usize, v),
            0xFF00 => self.joypad.reg_p1_write(v),
            0xFF01 => self.serial.reg_sb_write(v),
            0xFF02 => self.serial.reg_sc_write(v),
            0xFF04 => self.timer.reg_div_write(),
            0xFF05 => self.timer.reg_tima_write(v),
            0xFF06 => self.timer.reg_tma_write(v),
            0xFF07 => self.timer.reg_tac_write(v & 0x7, &mut self.interrupts),
            0xFF0F => self.interrupts.reg_if_write(v & 0x1F),
            0xFF10 => self.apu.reg_nr10_write(v),
            0xFF11 => self.apu.reg_nr11_write(v),
            0xFF12 => self.apu.reg_nr12_write(v),
            0xFF13 => self.apu.reg_nr13_write(v),
            0xFF14 => self.apu.reg_nr14_write(v),
            0xFF16 => self.apu.reg_nr21_write(v),
            0xFF17 => self.apu.reg_nr22_write(v),
            0xFF18 => self.apu.reg_nr23_write(v),
            0xFF19 => self.apu.reg_nr24_write(v),
            0xFF1A => self.apu.reg_nr30_write(v),
            0xFF1B => self.apu.reg_nr31_write(v),
            0xFF1C => self.apu.reg_nr32_write(v),
            0xFF1D => self.apu.reg_nr33_write(v),
            0xFF1E => self.apu.reg_nr34_write(v),
            0xFF20 => self.apu.reg_nr41_write(v),
            0xFF21 => self.apu.reg_nr42_write(v),
            0xFF22 => self.apu.reg_nr43_write(v),
            0xFF23 => self.apu.reg_nr44_write(v),
            0xFF24 => self.apu.reg_nr50_write(v),
            0xFF25 => self.apu.reg_nr51_write(v),
            0xFF26 => self.apu.reg_nr52_write(v),
            0xFF30...0xFF3F => self.apu.wave_write(addr - 0xFF30, v),
            0xFF40 => self.ppu.reg_lcdc_write(v),
            0xFF41 => self.ppu.reg_stat_write(v),
            0xFF42 => self.ppu.scy = v,
            0xFF43 => self.ppu.scx = v,
            0xFF45 => self.ppu.lyc = v,
            0xFF46 => self.dma.start(v),
            0xFF47 => self.ppu.bgp.update(v),
            0xFF48 => self.ppu.obp0.update(v),
            0xFF49 => self.ppu.obp1.update(v),
            0xFF4A => self.ppu.wy = v,
            0xFF4B => self.ppu.wx = v,
            0xFF50 if v == 1 => self.bootrom_enabled = false,
            0xFF80...0xFFFE => self.hram[(addr - 0xFF80) as usize] = v,
            0xFFFF => self.interrupts.reg_ie_write(v),

            _ => {} // Writes to unhandled locations are simply ignored.
        }
    }
    pub fn mem_write16(&mut self, addr: u16, v: u16) {
        self.mem_write(addr + 1, ((v & 0xFF00) >> 8) as u8);
        self.mem_write(addr, (v & 0xFF) as u8);
    }

    pub fn clock(&mut self) {
        self.cycle_count += 1;

        self.timer.clock(&mut self.interrupts);
        let (dma_active, dma_src, dma_dst) = self.dma.clock();
        if dma_active {
            let v = self.mem_get(dma_src);
            self.ppu.oam_write(dma_dst, v);
        }
        self.serial.clock(&mut self.interrupts);
        if self.ppu.clock(&mut self.interrupts) {
            self.new_frame = true;
        }
        self.apu.clock();
    }
}
