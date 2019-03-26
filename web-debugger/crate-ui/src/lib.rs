extern crate cfg_if;
extern crate wasm_bindgen;

mod utils;

use cfg_if::cfg_if;
use oxideboy::cpu::{decode_instruction, Instruction};
use serde::Serialize;
use std::collections::hash_map::DefaultHasher;
use std::hash::Hasher;
use wasm_bindgen::prelude::*;

macro_rules! log {
    ( $( $t:tt )* ) => {
        web_sys::console::log_1(&format!( $( $t )* ).into());
    }
}

cfg_if! {
    // When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
    // allocator.
    if #[cfg(feature = "wee_alloc")] {
        extern crate wee_alloc;
        #[global_allocator]
        static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;
    }
}

#[wasm_bindgen]
pub struct Disassembler {
    blocks: Vec<Block>,
}

#[wasm_bindgen]
pub struct Block {
    pub start: usize,
    pub end: usize,
    instructions: Vec<Instruction>,
    hash: u64,
}

fn compute_hash(data: &[u8]) -> u64 {
    let mut hasher = DefaultHasher::new();
    hasher.write(data);
    hasher.finish()
}

#[wasm_bindgen]
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

    fn to_js(&self) -> Vec<InstructionJs> {
        let mut addr = self.start;
        self.instructions
            .iter()
            .map(|inst| {
                let v = InstructionJs {
                    addr: addr,
                    txt: inst.to_string(),
                };
                addr += inst.size() as usize;
                v
            })
            .collect::<Vec<InstructionJs>>()
        // let mut out = String::new();
        // for instruction in &self.instructions {
        //     out += &instruction.to_string();
        //     out += "\n";
        // }
        // out
    }
}

#[wasm_bindgen]
impl Disassembler {
    pub fn new() -> Disassembler {
        Disassembler { blocks: Vec::new() }
    }

    /// Performs a full disassembly on the 65kb memory bus of the Gameboy.
    pub fn disassemble(&mut self, pc: usize, memory: &[u8]) {
        // Check the existing blocks we last disassembled and throw away any that are no longer valid.
        self.blocks.retain(|block| block.check_hash(memory));

        // Start from all the known root addresses.
        let mut addresses = vec![
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
            pc,     // Current PC address of emulation.
        ];
        let mut seen_addresses = Vec::new();
        let mut instructions = Vec::new();

        while !addresses.is_empty() {
            let address = addresses.pop().unwrap();
            seen_addresses.push(address);

            // Is this address already contained by an existing block?
            if self.blocks.iter().any(|block| block.contains(address)) {
                continue;
            }

            // Okay so we need to disassemble starting from this address.
            // Before we start disassembly we want to know a couple of things:
            //  * The boundary we may not disassemble past.
            //  * The location where the next already-disassembled block starts. If we hit this we just widen
            //    that block with our start address and disassembled instructions.
            let boundary = 0x10000;
            let next_block_start = self
                .blocks
                .iter()
                .map(|block| block.start)
                .filter(|start| *start > address)
                .min()
                .unwrap_or(std::usize::MAX);
            let mut pc = address;

            instructions.clear();

            loop {
                // Decode instructions until we hit an unconditional jump, a block boundary, or a subsequent block.
                if pc >= next_block_start || pc >= boundary {
                    break;
                }

                let instruction = decode_instruction(|| {
                    let addr = pc;
                    pc += 1;
                    memory[addr & 0xFFFF]
                });
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

            if pc >= next_block_start {
                // Okay we bumped into an existing block. We widen that block with the additional instructions
                // we decoded.
                let block = self
                    .blocks
                    .iter_mut()
                    .find(|block| block.start == next_block_start)
                    .unwrap();
                block.widen(address, &instructions, memory);
            } else {
                self.blocks.push(Block {
                    start: address,
                    end: pc,
                    instructions: instructions.clone(),
                    hash: compute_hash(&memory[address..pc]),
                });
            }
        }

        // Remove any blocks that are no longer reachable from the addresses we saw.
        self.blocks
            .retain(|block| seen_addresses.iter().any(|addr| block.contains(*addr)));

        self.blocks
            .sort_unstable_by(|a, b| a.start.partial_cmp(&b.start).unwrap());
    }

    pub fn blocks(&self) -> JsValue {
        JsValue::from_serde(
            &self
                .blocks
                .iter()
                .map(|block| BlocksJs {
                    addr: block.start,
                    hash: block.hash,
                })
                .collect::<Vec<BlocksJs>>(),
        )
        .unwrap()
    }

    pub fn block(&self, block: usize) -> JsValue {
        JsValue::from_serde(&self.blocks[block].to_js()).unwrap()
    }
}

#[derive(Serialize)]
struct BlocksJs {
    addr: usize,
    hash: u64,
}

#[derive(Serialize)]
struct InstructionJs {
    addr: usize,
    txt: String,
}
