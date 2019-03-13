# oxideboy

Eventually, this will be a fully featured Gameboy/Gameboy Color emulator written in Rust. A previous iteration of this project can be found in the `old` branch.

## Goals

 * High quality, thoroughly documented, and performant code, useful as a learning resource for Gameboy emulation.
 * Cycle accuracy with various original hardware models for accurate emulation.
 * Full-featured debugging interface

## Why another Gameboy emulator?

Well let's be real, I'm writing this because I want to. But here's some post-rationalization:

 * Some of the most highly accurate emulators, lke BGB & Beaten Dying Moon, are not open source and thus not very useful for learning about Gameboy emulator development.
 * Other high quality emulators, like Sameboy, mooneye-gb, etc, suffer from either a) overengineered code that is difficult to read, or b) C++ that is difficult (for me, and I'm sure many others) to reason about.

## Roadmap

- [x] Initial implementation of CPU, PPU, APU, serial, interrupts, etc.
- [x] Save states
- [ ] Frame / cycle rewinding
- [ ] Remove all panics and do graceful error handling
- [ ] Web-based debugger
  - [ ] Memory viewer/editor
  - [ ] CPU+HW register viewer/editor
  - [ ] Breakpoints
  - [x] Break
  - [x] Step forward
  - [x] Step frame
  - [ ] Step backward
  - [ ] VRAM viewer
- [ ] Cart memory bank controllers
  - [x] MBC1
  - [ ] MBC2
  - [ ] MBC3
  - [ ] MBC5
  - [ ] MBC7
- [ ] PPU cycle accuracy
- [ ] Blargg test suite
  - [x] instr_timing
  - [x] mem_timing
  - [x] cpu_instrs
  - [ ] dmg_sound
  - [ ] oam_bug
- [ ] mooneye-gb test suite
  - [x] acceptance
  - [x] acceptance/bits
  - [x] acceptance/instr
  - [x] acceptance/oam_dma
  - [x] acceptance/ppu
  - [ ] acceptance/serial
    - [x] with skipped bootrom
    - [ ] with real bootrom
  - [x] acceptance/timer
  - [ ] mbc1
- [ ] mealybug-tearoom test suite
- [ ] GBC implementation
  - [ ] Bootrom + skip
  - [ ] RAM banking
  - [ ] VRAM banking
  - [ ] KEY1 register mode select
  - [ ] CGB palettes
  - [ ] VRAM DMA
  - [ ] BG Map attributes
  - [ ] Infra port?
  - [ ] PCM registers
- [ ] SGB implementation
  - [ ] Bootrom + skip
  - [ ] SGB command packet protocol
  - [ ] SGB border
  - [ ] SGB color palettes
