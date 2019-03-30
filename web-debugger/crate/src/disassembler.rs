use core::ops::Range;
use oxideboy::cpu::Cpu;
use oxideboy::cpu::{decode_instruction, Instruction};
use oxideboy::GameboyBus;
use std::collections::hash_map::DefaultHasher;
use std::hash::Hasher;

fn compute_hash(data: &[u8]) -> u64 {
    let mut hasher = DefaultHasher::new();
    hasher.write(data);
    hasher.finish()
}

/// A Segment represents a range of memory that contains a number of disassembled instructions.
struct Segment {
    loc: Range<usize>,
    instructions: Vec<Instruction>,
    kind: SegmentKind,
}

/// Segments can be in various places in the Gameboy memory map. Depending on where they are, they need to
/// be handled differently and track slightly different information.
#[derive(Clone, Copy)]
enum SegmentKind {
    /// Bootrom segment is located at 0x0000 - 0x0100, and is only reachable early on when bootrom is mapped.
    Bootrom,

    /// Rom segments are contained in the lo/hi ROM banks. They're immutable so no need to track a hash, but we
    /// do need to know which bank the segment is in.
    Rom(usize),

    /// Ram segments live in 0xC000 - 0xDFFF. Since RAM is mutable we need to track a hash to know when to invalidate
    /// a segment. We also need to know which RAM bank the segment is located in (GBC has 7 switchable banks).
    Ram(usize, u64),

    /// Code can live anywhere really (I guess one could even put code in OAM and execute that). Most places
    /// are mutable in some way so we just need to track a hash.
    Misc(u64),
}

impl SegmentKind {
    fn id(self) -> String {
        match self {
            SegmentKind::Bootrom => String::from("boot"),
            SegmentKind::Rom(bank) => format!("rom:{}", bank),
            SegmentKind::Ram(bank, hash) => format!("ram:{}:{}", bank, hash),
            SegmentKind::Misc(hash) => format!("{}", hash),
        }
    }
}

impl Segment {
    fn id(&self) -> String {
        format!("{}:{}:{}", self.loc.start, self.loc.end, self.kind.id())
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
    fn check_hash(&self, memory: &[u8]) -> bool {
        compute_hash(&memory[self.start..self.end]) == self.hash
    }

    /// Widens this block to include the leading instructions that came before it. Effectively this concatenates
    /// an earlier block with this one.
    fn widen(&mut self, new_start: usize, extra_instructions: &[Instruction], memory: &[u8]) {
        self.start = new_start;
        self.hash = compute_hash(&memory[self.start..self.end]);

        let mut new_instructions = Vec::with_capacity(self.instructions.len() + extra_instructions.len());
        new_instructions.extend_from_slice(extra_instructions);
        new_instructions.append(&mut self.instructions);
        self.instructions = new_instructions;
    }

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
    blocks: Vec<Block>,
    segments: Vec<Segment>,
    bootrom_segment: Option<Segment>,
}

impl Disassembler {
    pub fn new() -> Disassembler {
        Disassembler {
            blocks: Vec::new(),
            segments: Vec::new(),
            bootrom_segment: None,
        }
    }

    /// Disassembles a segment of code from the starting location, going no further than a given boundary.
    /// Returns the disassembled segment, and also a list of more addresses to disassemble,
    fn disassemble_segment(gb: &GameboyBus, start: usize, boundary: usize, kind: SegmentKind) -> (Segment, Vec<usize>) {
        let mut pc = start;
        let mut instructions = Vec::new();
        let mut addresses = Vec::new();

        loop {
            let instruction = decode_instruction(|| {
                let addr = pc;
                pc += 1;
                gb.memory_get(addr as u16)
            });

            // If decoding this instruction took us past the boundary then we're done, and we don't include the
            // instruction we just decoded.
            if pc > boundary {
                break;
            }
            instructions.push(instruction);

            // Is this a branching instruction?
            let (is_branching, branch_addr) = instruction.is_branching();
            if is_branching {
                addresses.push(branch_addr as usize);
            }

            if instruction.is_terminating() {
                // Terminal instruction, this block is done.
                break;
            }
        }

        (
            Segment {
                loc: start..pc,
                instructions,
                kind,
            },
            addresses,
        )
    }

    /// Determines the appropriate boundary to use for a given memory location.
    /// For example when we're disassembling starting from 0x2123, we don't want to go past 0x3FFF, since that's
    /// the end of the low ROM bank that we're in.
    fn determine_boundary(addr: usize) -> usize {
        match addr {
            0x0000...0x0067 => 0x0067, // Reset vectors + Interrupts handlers
            0x0068...0x3FFF => 0x3FFF,
            0x4000...0x7FFF => 0x7FFF,
            0x8000...0x9FFF => 0x9FFF,
            0xA000...0xBFFF => 0xBFFF,
            0xC000...0xCFFF => 0xCFFF,
            0xD000...0xDFFF => 0xDFFF,
            0xE000...0xFDFF => 0, // We don't disassemble echo RAM.
            0xFE00...0xFE9F => 0xFE9F,
            0xFEA0...0xFF7F => 0xFF7F,
            0xFF80...0xFFFF => 0xFFFF,
            _ => 0,
        }
    }

    /// Walks through the current state of the Gameboy and ensures disassembly state is in sync.
    /// Returns a list of ids for the "live" (reachable from somewhere) code segments.
    pub fn disassemble(&mut self, cpu: &Cpu, gb: &GameboyBus) -> Vec<String> {
        // TODO: go through all mutable segments and remove any that are no longer valid.

        let mut segment_ids = Vec::new();

        // The addresses we start walking from differ depending on whether bootrom is mapped or not.
        let mut pending_addresses = if *gb.bootrom_enabled {
            // While bootrom is enabled we disassemble 0x0000-0x0100 as a contiguous block, and then
            // follow whatever we can find starting from 0x0100.
            if !self.bootrom_segment.is_some() {
                let (segment, addresses) = Self::disassemble_segment(gb, 0x0000, 0x0100, SegmentKind::Bootrom);
                if addresses.len() > 0 {
                    panic!(
                        "Decoding bootrom and got more addresses to go to, seems wrong. {:?}",
                        addresses
                    );
                }
                self.bootrom_segment = Some(segment);
                segment_ids.push(self.bootrom_segment.as_ref().unwrap().id());
            }

            vec![0x0100]
        } else {
            if self.bootrom_segment.is_some() {
                // Bootrom has been unmapped. It'll never be coming back so let's free it now.
                self.bootrom_segment.take();
            }

            vec![
                0x0000, // RST $00
                0x0008, // RST $08
                0x0010, // RST $10
                0x0018, // RST $18
                0x0020, // RST $20
                0x0028, // RST $28
                0x0030, // RST $30
                0x0038, // RST $38
                0x0040, // VBlank Interrupt
                0x0048, // Stat Interrupt
                0x0050, // Timer Interrupt
                0x0058, // Serial Interrupt
                0x0060, // Joypad Interrupt
                0x0100, // ROM entrypoint
                cpu.pc, // Current PC address of emulation.
            ]
        };

        while !pending_addresses.is_empty() {
            let address = pending_addresses.pop().unwrap();

            // let (segment, new_addresses) = Self::disassemble_segment(
            //     address,
            //     Self::determine_boundary(address),
            //     Self::determine_kind(address),
            // );
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
}
