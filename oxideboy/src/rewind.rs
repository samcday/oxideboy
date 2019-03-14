//! Functionality to rewind/fast-forward the emulator state by frame or instruction cycle.
//! This is achieved by taking regular snapshots (read: save states) of the emulator. "Rewinding" is simply restoring
//! a previous snapshot state, then running the emulator up to the desired point, while replaying any input events that
//! occurred during the replayed period. This is possible because the Gameboy architecture is so simple - it's
//! completely deterministic outside of the input events received from the joypad.

use crate::interrupt::Interrupt;
use crate::joypad::JoypadState;
use crate::simple_diff;
use crate::{Context, Gameboy, CYCLES_PER_FRAME};
use bincode;
use std::collections::{BTreeMap, VecDeque};
use std::ops::Range;

// TODO: compression

/// The maximum number of delta snapshots we'll compress before it's time to write out a new base snapshot.
/// The higher this number is, the lower the average rate of growth of the rewind log over time. But if it's too high
/// it will impact rewind speeds because we'll have so many deltas to apply to a base before we get to a point in time
/// that we're looking for.
const MAX_DELTAS_SIZE: usize = 262_144;

/// We take a snapshot every SNAPSHOT_FREQ cycles.
const SNAPSHOT_FREQ: u64 = 10 * CYCLES_PER_FRAME;

pub struct RewindManager<T: StorageAdapter> {
    adapter: T,

    last_snapshot: Vec<u8>,
    current_snapshot: Vec<u8>,
    delta_scratch: Vec<u8>,
    delta_size: usize, // Counts how many bytes of delta snapshots we've written since the last full snapshot.
    next_snapshot_at: u64, // Gameboy needs to be at this cycle (or further) befoer we take another snapshot.
    last_seen_cycle: u64, // The cycle we were at when we last took a snapshot or recorded input.
    last_joypad_state: JoypadState, // Keep track of last joypad state, to prevent key repeats writing so many events
}

pub enum SnapshotType {
    Full,
    Delta,
}

/// We can store our rewind history in a few different ways. In memory for debugging / test purposes, in an append-only
/// file, or in IndexedDB in the browser. This trait encapsulates the behaviour we need to save and retrieve snapshots
/// and input data for rewinding.
pub trait StorageAdapter {
    /// Save a newly generated snapshot for the given cycle. The snapshot may either be full or delta.
    fn record_snapshot(&mut self, cycle: u64, snapshot: &[u8], snapshot_type: SnapshotType);

    /// Record some input that the emulation received at a given cycle point.
    fn record_input(&mut self, cycle: u64, joypad: JoypadState);

    /// Find the nearest snapshot for a desired cycle, and all input events that took place between the closest snapshot
    /// and the desired cycle.
    fn find_snapshot(&mut self, cycle: u64, snapshot: &mut Vec<u8>, input: &mut VecDeque<(u64, JoypadState)>);

    /// Drop all snapshots and recorded input after the given cycle boundary.
    fn truncate(&mut self, cycle: u64);
}

impl<T: StorageAdapter> RewindManager<T> {
    pub fn new(gb: &Gameboy, adapter: T) -> Result<RewindManager<T>, String> {
        // RewindManager needs to be setup before the Gameboy has been used.
        if gb.cycle_count > 0 {
            return Err(format!(
                "Gameboy has already emulated {} cycles, RewindManager::new must be called on a pristine \
                 emulation session",
                gb.cycle_count
            ));
        }

        let snapshot_size = bincode::serialized_size(&gb).unwrap() as usize;

        let mut rewind_manager = RewindManager {
            adapter,
            last_snapshot: Vec::with_capacity(snapshot_size),
            current_snapshot: Vec::with_capacity(snapshot_size),
            delta_scratch: Vec::with_capacity(snapshot_size),
            delta_size: 0,
            next_snapshot_at: 0,
            last_seen_cycle: 0,
            last_joypad_state: Default::default(),
        };
        rewind_manager.snapshot(gb);

        Ok(rewind_manager)
    }

    /// Give the RewindManager a chance to take another snapshot of emulation state.
    pub fn snapshot(&mut self, gb: &Gameboy) {
        self.check_truncate(gb);

        // Are we ready to take another snapshot yet?
        if gb.cycle_count < self.next_snapshot_at {
            return;
        }

        self.next_snapshot_at += SNAPSHOT_FREQ;

        std::mem::swap(&mut self.last_snapshot, &mut self.current_snapshot);
        self.current_snapshot.clear();
        bincode::serialize_into(&mut self.current_snapshot, gb).unwrap();

        if self.last_snapshot.len() == 0 || self.delta_size > MAX_DELTAS_SIZE {
            self.delta_size = 0;
            self.adapter
                .record_snapshot(gb.cycle_count, &self.current_snapshot, SnapshotType::Full);
        } else {
            self.delta_scratch.clear();
            simple_diff::generate(&self.last_snapshot, &self.current_snapshot, &mut self.delta_scratch).unwrap();
            self.adapter
                .record_snapshot(gb.cycle_count, &self.delta_scratch, SnapshotType::Delta);
        }
    }

    /// Notify the RewindManager of some input that has been fed into the given Gameboy.
    pub fn notify_input(&mut self, gb: &Gameboy) {
        self.check_truncate(gb);

        if gb.joypad.state == self.last_joypad_state {
            return;
        }
        self.last_joypad_state = gb.joypad.state;
        self.adapter.record_input(gb.cycle_count, gb.joypad.state);
    }

    /// If the user rewound to a previous point in time and then began playing from that point, we need to purge all
    /// snapshots and input that come after the new present point.
    fn check_truncate(&mut self, gb: &Gameboy) {
        // If state was rewound and we're now moving forward from a previous point in time, we need to truncate all
        // states and input events that come after where we're now up to.
        if gb.cycle_count < self.last_seen_cycle {
            self.adapter.truncate(gb.cycle_count);
            self.next_snapshot_at = gb.cycle_count + SNAPSHOT_FREQ;
        }
        self.last_seen_cycle = gb.cycle_count;
    }

    /// Rewind by a single frame. This is just a convenience method for rewinding by the number of cycles a
    /// frame takes (which is 17556).
    pub fn rewind_frame(&mut self, gb: &mut Gameboy, ctx: &mut Context) {
        if gb.frame_count < 2 {
            // TODO: how do we signal this? Return a Result or an Option maybe?
            return;
        }

        // This is the point we want to rewind to.
        let desired_cycle = gb.cycle_count.saturating_sub(CYCLES_PER_FRAME);

        // In order to rewind backwards 1 frame, we need to start from two frames back and run forward, this is so we
        // have a full frame to render after the rewind.
        let start_cycle = desired_cycle.saturating_sub(CYCLES_PER_FRAME * 2);

        let mut snapshot = Vec::new();
        let mut input = VecDeque::new();
        self.adapter.find_snapshot(start_cycle, &mut snapshot, &mut input);

        // Load the new state in, then run the emulation until we get to the desired frame.
        *gb = bincode::deserialize_from(&snapshot[..]).unwrap();

        // Disable video until the next frame boundary. Otherwise, the framebuffer gets flipped by the PPU and we end
        // up with half-rendered frames tearing all over the screen.
        let prev_enable_video = ctx.enable_video;
        let prev_enable_audio = ctx.enable_audio;
        ctx.enable_video = false;
        ctx.enable_audio = false;

        let mut next_input = input.pop_front();
        let frame_count = gb.frame_count;

        while gb.cycle_count < desired_cycle {
            if next_input.is_some() && next_input.unwrap().0 == gb.cycle_count {
                gb.joypad.state = next_input.unwrap().1;
                gb.interrupts.request(Interrupt::Joypad);
                next_input = input.pop_front();
            }
            gb.run_instruction(ctx);

            if !ctx.enable_video && gb.frame_count > frame_count {
                ctx.enable_video = true;
            }
        }
        ctx.enable_video = prev_enable_video;
        // TODO: probably should be handling audio separately.
        ctx.enable_audio = prev_enable_audio;

        self.last_joypad_state = gb.joypad.state;
    }
}

#[derive(Default)]
pub struct MemoryStorageAdapter {
    full_snapshots: BTreeMap<u64, Range<usize>>,
    delta_snapshots: BTreeMap<u64, Range<usize>>,
    input_events: BTreeMap<u64, JoypadState>,

    data: Vec<u8>,
}

impl MemoryStorageAdapter {
    pub fn new() -> MemoryStorageAdapter {
        Default::default()
    }
}

impl StorageAdapter for MemoryStorageAdapter {
    fn record_snapshot(&mut self, cycle: u64, snapshot: &[u8], snapshot_type: SnapshotType) {
        match snapshot_type {
            SnapshotType::Full => {
                let pos = self.data.len();
                self.data.extend_from_slice(snapshot);
                self.full_snapshots.insert(cycle, pos..pos + snapshot.len());
            }
            SnapshotType::Delta => {
                let pos = self.data.len();
                self.data.extend_from_slice(snapshot);
                self.delta_snapshots.insert(cycle, pos..pos + snapshot.len());
            }
        }
    }

    fn record_input(&mut self, cycle: u64, joypad: JoypadState) {
        self.input_events.insert(cycle, joypad);
    }

    fn find_snapshot(&mut self, cycle: u64, mut snapshot: &mut Vec<u8>, input: &mut VecDeque<(u64, JoypadState)>) {
        // First we need to find the closest full snapshot to the desired cycle.
        let (base_cycle, base_range) = self.full_snapshots.range(0..=cycle).rev().next().unwrap();

        // Copy the base snapshot into the destination.
        snapshot.extend_from_slice(&self.data[base_range.clone()]);

        // Now apply all delta snapshots that fall between the base and the desired cycle.
        let mut last_delta_cycle = *base_cycle;
        self.delta_snapshots
            .range(*base_cycle..=cycle)
            .for_each(|(cycle, delta_range)| {
                last_delta_cycle = *cycle;
                simple_diff::apply(&mut snapshot, &self.data[delta_range.clone()]);
            });

        // Finally, grab all input events that occurred between the most recent delta snapshot and the desired cycle.
        input.extend(
            self.input_events
                .range(last_delta_cycle..=cycle)
                .map(|(input_cycle, joypad)| (*input_cycle, *joypad)),
        );
    }

    fn truncate(&mut self, cycle: u64) {
        // Now we clear out all full/delta snapshots that came after the cycle boundary.
        // We take note of the earliest position in the data blob that was encountered.
        fn purge_snapshots(cycle: u64, snapshots: &mut BTreeMap<u64, Range<usize>>) -> Option<usize> {
            let purge = snapshots.range(cycle..).map(|(cycle, _)| *cycle).collect::<Vec<u64>>();
            let data_idx = if purge.is_empty() {
                None
            } else {
                Some(purge[0] as usize)
            };
            for key in purge {
                snapshots.remove(&key);
            }
            data_idx
        }
        let full_idx = purge_snapshots(cycle, &mut self.full_snapshots);
        let delta_idx = purge_snapshots(cycle, &mut self.delta_snapshots);

        let truncate_to = std::cmp::min(
            full_idx.unwrap_or(usize::max_value()),
            delta_idx.unwrap_or(usize::max_value()),
        );
        if truncate_to < usize::max_value() {
            self.data.truncate(truncate_to);
        }

        // Finally, we clear out input events that came after the cycle boundary.
        let purge = self
            .input_events
            .range(cycle..)
            .map(|(cycle, _)| *cycle)
            .collect::<Vec<u64>>();
        for key in purge {
            self.input_events.remove(&key);
        }
    }
}
