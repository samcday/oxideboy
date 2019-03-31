use core::ops::Range;
use oxideboy::cpu::Cpu;
use oxideboy::cpu::{decode_instruction, Instruction};
use oxideboy::GameboyBus;
use std::collections::hash_map::DefaultHasher;
use std::hash::Hasher;

/// A Segment represents a range of memory that contains a number of disassembled instructions.
pub struct Segment {
    pub loc: Range<usize>,
    pub hash: u64,
    pub instructions: Vec<Instruction>,
}

impl Segment {
    pub fn id(&self) -> String {
        format!("{:X}:{:X}:{}", self.loc.start, self.loc.end, self.hash)
    }
}

pub struct Disassembler {
    pub cached_segment: Option<Segment>,
}

impl Disassembler {
    pub fn new() -> Disassembler {
        Disassembler { cached_segment: None }
    }

    /// Disassembles the current segment of code the Gameboy is executing. For now, disassembly is quite naive.
    /// We simply start disassembling from the current PC register location and keep going until we hit a terminating
    /// instruction (unconditional jump, CALL, RET/RETI, RST).
    /// To keep things efficient and reduce GC pressure, we cache the current segment we walked so that we don't need
    /// to recompute it until the program counter moves out of bounds of the current segment, or the memory contents of
    /// the current segment change.
    /// We're returning a list of segment IDs here, but currently only one id will ever be returned (the active one).
    /// Later, if/when we add support for symbol maps, we will probably start returning more here.
    pub fn disassemble(&mut self, _cpu: &Cpu, bus: &GameboyBus) -> Vec<String> {
        let start = bus.context.last_jump_addr as usize;

        // If there's already a cached segment, see if it's still valid.
        if self.cached_segment.is_some() {
            let cached_segment = self.cached_segment.as_ref().unwrap();
            let still_valid = if cached_segment.loc.start <= start && start < cached_segment.loc.end {
                let mut hasher = DefaultHasher::new();

                for addr in cached_segment.loc.clone() {
                    hasher.write_u8(bus.memory_get(addr as u16));
                }

                hasher.finish() == cached_segment.hash
            } else {
                false
            };

            if still_valid {
                return vec![self.cached_segment.as_ref().unwrap().id()];
            }

            self.cached_segment = None;
        }

        let mut instructions = Vec::new();
        let mut hasher = DefaultHasher::new();
        let mut pc = start;

        loop {
            let instruction = decode_instruction(|| {
                let val = bus.memory_get(pc as u16);
                pc = pc.wrapping_add(1);
                hasher.write_u8(val);
                val
            });

            instructions.push(instruction);

            if instruction.is_terminating() {
                // Terminal instruction, this segment is done.
                break;
            }
        }

        self.cached_segment = Some(Segment {
            loc: start..pc,
            instructions,
            hash: hasher.finish(),
        });

        vec![self.cached_segment.as_ref().unwrap().id()]
    }
}
