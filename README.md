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
- [x] Passing all blargg cpu/memory/timing tests.
- [ ] Passing all blargg dmg_sound tests.
- [ ] Passing all blargg oam_bug tests.
- [ ] Passing all mealybug-tearoom tests.
- [x] Passing all mooneye-gb acceptance tests.
- [ ] Passing all mooneye-gb mbc1 tests.
