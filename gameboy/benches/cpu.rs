#![feature(test)]

extern crate test;
extern crate gameboy;

use test::Bencher;

#[bench]
fn bench_cpu_nop(b: &mut Bencher) {
    let rom = vec![0; 0x8000];
    let mut cpu = gameboy::CPU::new(rom);
    cpu.bootrom_enabled = false;

    b.iter(|| {
        cpu.pc = 0;
        for _ in 0..100 {
            cpu.run();
        }
    });
}
