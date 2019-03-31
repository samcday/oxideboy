use crate::log;
use core::ops::Range;
use oxideboy::cpu::Cpu;
use oxideboy::cpu::{decode_instruction, Instruction};
use oxideboy::GameboyBus;
use std::collections::hash_map::DefaultHasher;
use std::collections::HashMap;
use std::hash::Hasher;

/// A Segment represents a range of memory that contains a number of disassembled instructions.
pub struct Segment {
    pub loc: Range<usize>,
    pub instructions: Vec<Instruction>,
    pub kind: SegmentKind,
    pub hash: u64,
}

impl Segment {
    fn id(&self) -> String {
        format!(
            "{}:{:X}:{:X}:{}",
            self.kind.id(),
            self.loc.start,
            self.loc.end,
            self.hash,
        )
    }
}

/// Segments can be in various places in the Gameboy memory map. Depending on where they are, they need to
/// be handled differently and track slightly different information.
#[derive(Clone, Copy, PartialEq)]
pub enum SegmentKind {
    /// Bootrom segment is located at 0x0000 - 0x0100, and is only reachable early on when bootrom is mapped.
    Bootrom,

    /// Rom segments are contained in the lo/hi ROM banks. They're immutable so no need to track a hash, but we
    /// do need to know which bank the segment is in.
    Rom(usize),

    /// Ram segments live in 0xC000 - 0xDFFF. Since RAM is mutable we need to track a hash to know when to invalidate
    /// a segment. We also need to know which RAM bank the segment is located in (GBC has 7 switchable banks).
    Banked(usize),

    /// Code can live anywhere really (I guess one could even put code in OAM and execute that). Most places
    /// are mutable in some way so we just need to track a hash.
    Misc,
}

impl SegmentKind {
    fn id(self) -> String {
        match self {
            SegmentKind::Bootrom => String::from("boot"),
            SegmentKind::Rom(bank) => format!("rom:{}", bank),
            SegmentKind::Banked(bank) => format!("bnk:{}", bank),
            SegmentKind::Misc => String::from("misc"),
        }
    }
}

struct Block {
    pub start: usize,
    pub end: usize,
    instructions: Vec<Instruction>,
    hash: u64,
}

impl Block {
    /// Checks if the hash for this block matches the content of memory.
    // fn check_hash(&self, memory: &[u8]) -> bool {
    //     compute_hash(&memory[self.start..self.end]) == self.hash
    // }

    /// Widens this block to include the leading instructions that came before it. Effectively this concatenates
    /// an earlier block with this one.
    // fn widen(&mut self, new_start: usize, extra_instructions: &[Instruction], memory: &[u8]) {
    //     self.start = new_start;
    //     self.hash = compute_hash(&memory[self.start..self.end]);

    //     let mut new_instructions = Vec::with_capacity(self.instructions.len() + extra_instructions.len());
    //     new_instructions.extend_from_slice(extra_instructions);
    //     new_instructions.append(&mut self.instructions);
    //     self.instructions = new_instructions;
    // }

    /// Returns true if this block contains the given memory address in its bounds.
    fn contains(&self, address: usize) -> bool {
        return self.start <= address && address < self.end;
    }

    // fn to_js(&self) -> Vec<InstructionJs> {
    //     let mut addr = self.start;
    //     self.instructions
    //         .iter()
    //         .map(|inst| {
    //             let v = InstructionJs {
    //                 addr: addr,
    //                 txt: inst.to_string(),
    //             };
    //             addr += inst.size() as usize;
    //             v
    //         })
    //         .collect::<Vec<InstructionJs>>()
    //     // let mut out = String::new();
    //     // for instruction in &self.instructions {
    //     //     out += &instruction.to_string();
    //     //     out += "\n";
    //     // }
    //     // out
    // }
}

pub struct Disassembler {
    segments: HashMap<String, Segment>,
}

impl Disassembler {
    pub fn new() -> Disassembler {
        Disassembler {
            segments: HashMap::new(),
        }
    }

    /// Disassembles a segment of code from the starting location, going no further than a given boundary.
    /// Returns the range, the instructions, and the hash of the disassembled segment, along with a list of additional
    /// reachable addresses that should be disassembled.
    fn disassemble_segment(gb: &GameboyBus, start: usize, boundary: usize, kind: SegmentKind) -> (Segment, Vec<usize>) {
        let mut hasher = DefaultHasher::new();
        let mut loc = start..start;
        let mut pc = start;
        let mut instructions = Vec::new();
        let mut addresses = Vec::new();
        let mut instruction_bytes = Vec::with_capacity(4); // Maximum length of a single instruction

        loop {
            let instr_addr = pc;
            let instruction = decode_instruction(|| {
                let val = gb.memory_get(pc as u16);
                pc += 1;
                instruction_bytes.push(val);
                val
            });

            // If decoding this instruction took us past the boundary then we're done, and we don't include the
            // instruction we just decoded.
            if pc - 1 > boundary {
                // log!("went past boundary {:X}", boundary);
                break;
            }
            loc.end = pc;
            instructions.push(instruction);
            hasher.write(&instruction_bytes);
            instruction_bytes.clear();

            // Is this a branching instruction?
            let (is_branching, branch_addr) = instruction.is_branching(instr_addr as u16);
            if is_branching {
                // log!("Found branching: {} {}", instruction, branch_addr);
                addresses.push(branch_addr as usize);
            }

            if instruction.is_terminating() {
                // Terminal instruction, this block is done.
                break;
            }
        }

        (
            Segment {
                loc,
                instructions,
                kind,
                hash: hasher.finish(),
            },
            addresses,
        )
    }

    /// Walks through the current state of the Gameboy and ensures disassembly state is in sync.
    /// Returns a list of ids for the "live" (reachable from somewhere) code segments.
    pub fn update(&mut self, cpu: &Cpu, gb: &GameboyBus) -> Vec<String> {
        let mut segment_ids = Vec::new();

        // The addresses we start walking from differ depending on whether bootrom is mapped or not.
        let mut pending_addresses: Vec<usize> = Vec::new();

        if *gb.bootrom_enabled {
            // While bootrom is enabled we disassemble 0x0000-0x0100 as a contiguous block, and then
            // follow whatever we can find starting from 0x0100.
            pending_addresses.push(0);
            pending_addresses.push(0x0100);
        } else {
            pending_addresses.append(&mut vec![
                0x0000,          // RST $00
                0x0008,          // RST $08
                0x0010,          // RST $10
                0x0018,          // RST $18
                0x0020,          // RST $20
                0x0028,          // RST $28
                0x0030,          // RST $30
                0x0038,          // RST $38
                0x0040,          // VBlank Interrupt
                0x0048,          // Stat Interrupt
                0x0050,          // Timer Interrupt
                0x0058,          // Serial Interrupt
                0x0060,          // Joypad Interrupt
                0x0100,          // ROM entrypoint
                cpu.pc as usize, // Current PC address of emulation.
            ]);
        };

        let mut seen_ranges: Vec<Range<usize>> = vec![];

        while !pending_addresses.is_empty() {
            let address = pending_addresses.pop().unwrap();

            // If we're already processed a segment for this address we move on.
            if seen_ranges.iter().any(|rng| rng.start <= address && address < rng.end) {
                continue;
            }

            let kind = match address {
                0x0000...0x009F if *gb.bootrom_enabled => SegmentKind::Bootrom,
                0x0000...0x3FFF => SegmentKind::Rom(gb.cart.lo_rom_bank as usize),
                0x4000...0x7FFF => SegmentKind::Rom(gb.cart.hi_rom_bank as usize),
                // 0x8000...0x9FFF
                _ => panic!("unimplemented"),
            };

            //             // First, see if there's already a segment that contains this address.
            //             let existing_segment = self
            //                 .segments
            //                 .values()
            //                 .find(|segment| segment.loc.start <= address && address < segment.loc.end && segment.kind == kind);

            // if existing_segment
            //                 Some(segment) => segment_ids.push(segment.id()),
            //                 _ => {}
            //             }

            // Determine the appropriate boundary to use for a given memory location.
            // For example when we're disassembling starting from 0x2123, we don't want to go past 0x3FFF, since that's
            // the end of the low ROM bank that we're in.
            let boundary = match address {
                0x0000...0x009F if *gb.bootrom_enabled => 0x009F,
                0x0000...0x0103 if gb.cart.lo_rom_bank == 0 => 0x103,
                0x0000...0x3FFF => 0x3FFF,
                0x4000...0x7FFF => 0x7FFF,
                0x8000...0x9FFF => 0x9FFF,
                0xA000...0xBFFF => 0xBFFF,
                0xC000...0xCFFF => 0xCFFF,
                0xD000...0xDFFF => 0xDFFF,
                0xE000...0xFDFF => 0, // We don't disassemble echo RAM.
                0xFE00...0xFE9F => 0xFE9F,
                0xFEA0...0xFF7F => 0xFF7F,
                0xFF80...0xFFFF => 0xFFFF,
                _ => panic!("unimplemented"),
            };

            let (segment, mut new_addresses) = Self::disassemble_segment(gb, address, boundary, kind);
            pending_addresses.append(&mut new_addresses);
            segment_ids.push(segment.id());
            seen_ranges.push(segment.loc.clone());
            self.segments.insert(segment.id(), segment);
        }

        segment_ids

        // // Check the existing blocks we last disassembled and throw away any that are no longer valid.
        // self.blocks.retain(|block| block.check_hash(memory));

        // // Start from all the known root addresses.
        // let mut addresses = vec![

        // ];
        // let mut seen_addresses = Vec::new();
        // let mut instructions = Vec::new();

        // while !addresses.is_empty() {
        //     let address = addresses.pop().unwrap();
        //     seen_addresses.push(address);

        //     // Is this address already contained by an existing block?
        //     if self.blocks.iter().any(|block| block.contains(address)) {
        //         continue;
        //     }

        //     // Okay so we need to disassemble starting from this address.
        //     // Before we start disassembly we want to know a couple of things:
        //     //  * The boundary we may not disassemble past.
        //     //  * The location where the next already-disassembled block starts. If we hit this we just widen
        //     //    that block with our start address and disassembled instructions.
        //     let boundary = 0x10000;
        //     let next_block_start = self
        //         .blocks
        //         .iter()
        //         .map(|block| block.start)
        //         .filter(|start| *start > address)
        //         .min()
        //         .unwrap_or(std::usize::MAX);
        //     let mut pc = address;

        //     instructions.clear();

        //     if pc >= next_block_start {
        //         // Okay we bumped into an existing block. We widen that block with the additional instructions
        //         // we decoded.
        //         let block = self
        //             .blocks
        //             .iter_mut()
        //             .find(|block| block.start == next_block_start)
        //             .unwrap();
        //         block.widen(address, &instructions, memory);
        //     } else {
        //         self.blocks.push(Block {
        //             start: address,
        //             end: pc,
        //             instructions: instructions.clone(),
        //             hash: compute_hash(&memory[address..pc]),
        //         });
        //     }
        // }

        // self.blocks
        //     .sort_unstable_by(|a, b| a.start.partial_cmp(&b.start).unwrap());
    }

    pub fn segment(&self, id: String) -> Option<&Segment> {
        self.segments.get(&id)
    }
}
