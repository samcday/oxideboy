//! Functionality to rewind/fast-forward the emulator state by frame or instruction cycle.
//! This is achieved by taking regular snapshots (read: save states) of the emulator. "Rewinding" is simply restoring
//! a previous snapshot state, then running the emulator up to the desired point, while replaying any input events that
//! occurred during the replayed period. Because the Gameboy architecture is so simple - it's completely deterministic
//! outside of the input events received from the joypad.
use crate::joypad::Joypad;
use crate::simple_diff;
use crate::{Context, Gameboy};
use bincode;
use std::collections::BTreeMap;
use std::ops::Range;

// TODO: compression
// TODO: disable framebuffer+audio queue when catching up on frames
// TODO: properly handle when we rewind to earliest boundary

/// The maximum number of delta snapshots we'll compress before it's time to write out a new base snapshot.
/// The higher this number is, the lower the average rate of growth of the rewind log over time. But if it's too high
/// it will impact rewind speeds because we'll have so many deltas to apply to a base before we get to a point in time
/// that we're looking for.
const MAX_DELTAS_SIZE: usize = 262_144;

const SNAPSHOT_FREQ: usize = 5; // We take a snapshot every SNAPSHOT_FREQ frames.

pub struct RewindManager<T: StorageAdapter> {
    adapter: T,

    last_snapshot: Vec<u8>,
    current_snapshot: Vec<u8>,
    delta_scratch: Vec<u8>,
    delta_size: usize, // Counts how many bytes of delta snapshots we've written since the last full snapshot.
    next_snapshot_in: usize, // Countdown (in frames) to the next snapshot.
    last_snapshot_cycle: u64,
}

pub enum SnapshotType {
    Full,
    Delta,
}

/// We can store our rewind history in a few different ways. In memory for debugging / test purposes, in an append-only
/// file, or in IndexedDB in the browser. This trait encapsulates the behaviour we need to save and retrieve snapshots
/// and input data for rewinding.
pub trait StorageAdapter {
    fn record_snapshot(&mut self, cycle: u64, frame: u64, snapshot: &[u8], snapshot_type: SnapshotType);
    fn record_input(&mut self, cycle: u64, joypad: Joypad);
    fn snapshot_for_frame(&mut self, frame: u64, snapshot: &mut Vec<u8>, input: &mut Vec<Joypad>);
    fn snapshot_for_cycle(&mut self, cycle: u64, snapshot: &mut Vec<u8>, input: &mut Vec<Joypad>);
    /// Drop all snapshots and recorded input after the given cycle+frame boundary.
    fn truncate(&mut self, cycle: u64);
}

impl<T: StorageAdapter> RewindManager<T> {
    pub fn new(adapter: T) -> RewindManager<T> {
        RewindManager {
            adapter,
            last_snapshot: Vec::new(),
            current_snapshot: Vec::new(),
            delta_scratch: Vec::new(),
            delta_size: 0,
            last_snapshot_cycle: 0,
            next_snapshot_in: SNAPSHOT_FREQ,
        }
    }

    /// Notify the RewindManager that another frame has completed in the given Gameboy.
    pub fn notify_frame(&mut self, gb: &Gameboy) {
        // If state was rewound and we're now moving forward from a previous point in time, we need to truncate all
        // states and input events that come after where we're now up to.
        if gb.cycle_count < self.last_snapshot_cycle {
            self.adapter.truncate(gb.cycle_count);
        }

        // TODO: assert that notify_frame was called on the cycle that vblanked.
        self.last_snapshot_cycle = gb.cycle_count;
        self.next_snapshot_in -= 1;
        if self.next_snapshot_in > 0 {
            // Nothing to do here.
            return;
        }

        self.next_snapshot_in = SNAPSHOT_FREQ;

        std::mem::swap(&mut self.last_snapshot, &mut self.current_snapshot);
        self.current_snapshot.clear();
        bincode::serialize_into(&mut self.current_snapshot, gb).unwrap();

        if self.last_snapshot.len() == 0 || self.delta_size > MAX_DELTAS_SIZE {
            self.delta_size = 0;
            self.adapter.record_snapshot(
                gb.cycle_count,
                gb.frame_count,
                &self.current_snapshot,
                SnapshotType::Full,
            );
        } else {
            self.delta_scratch.clear();
            simple_diff::generate(&self.last_snapshot, &self.current_snapshot, &mut self.delta_scratch).unwrap();
            self.adapter
                .record_snapshot(gb.cycle_count, gb.frame_count, &self.delta_scratch, SnapshotType::Delta);
        }
    }

    /// Notify the RewindManager of some input that has been fed into the given Gameboy.
    pub fn notify_input(&mut self, gb: &Gameboy) {
        self.adapter.record_input(gb.cycle_count, gb.joypad.clone());
    }

    /// Rewind by a single frame.
    pub fn rewind_frame(&mut self, gb: &mut Gameboy, context: &mut Context) {
        if gb.frame_count < 2 {
            // TODO: how do we signal this? Return a Result or an Option maybe?
            return;
        }

        let desired_frame = gb.frame_count - 1;

        // In order to rewind backwards 1 frame, we need to get a snapshot for 2 frames back. This is so we can start
        // from two frames back and run forward, building a complete framebuffer that can be drawn.
        let mut snapshot = Vec::new();
        let mut input = Vec::new();
        self.adapter
            .snapshot_for_frame(desired_frame - 1, &mut snapshot, &mut input);

        // Load the new state in, then run the emulation until we get to the desired frame.
        gb.ppu.dirty = true;
        gb.ppu.next_dirty = true;
        *gb = bincode::deserialize_from(&snapshot[..]).unwrap();
        while gb.frame_count < desired_frame {
            gb.run_instruction(context);
        }
    }
}

#[derive(Default)]
pub struct MemoryStorageAdapter {
    frame_cycles: BTreeMap<u64, u64>,
    full_snapshots: BTreeMap<u64, Range<usize>>,
    delta_snapshots: BTreeMap<u64, Range<usize>>,
    input_events: BTreeMap<u64, Joypad>,

    data: Vec<u8>,
}

impl MemoryStorageAdapter {
    pub fn new() -> MemoryStorageAdapter {
        Default::default()
    }
}

impl StorageAdapter for MemoryStorageAdapter {
    fn record_snapshot(&mut self, cycle: u64, frame: u64, snapshot: &[u8], snapshot_type: SnapshotType) {
        self.frame_cycles.insert(frame, cycle);

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

    fn record_input(&mut self, cycle: u64, joypad: Joypad) {
        self.input_events.insert(cycle, joypad);
    }

    fn snapshot_for_cycle(&mut self, cycle: u64, mut snapshot: &mut Vec<u8>, input: &mut Vec<Joypad>) {
        // First we need to find the closest full snapshot to the desired cycle.
        // TODO: how do we handle not finding anything?
        let (base_cycle, base_range) = self.full_snapshots.range(0..=cycle).rev().next().unwrap();

        // Copy the base snapshot into the destination.
        snapshot.extend_from_slice(&self.data[base_range.clone()]);

        // Now look through all delta snapshots that fall between the base and the desired cycle.
        let mut last_delta_cycle = *base_cycle;
        self.delta_snapshots
            .range(*base_cycle..=cycle)
            .for_each(|(cycle, delta_range)| {
                last_delta_cycle = *cycle;
                simple_diff::apply(&mut snapshot, &self.data[delta_range.clone()]);
            });

        // Finally, grab all the input events that occurred between the last delta snapshot and the desired cycle.
        input.extend(
            self.input_events
                .range(last_delta_cycle..=cycle)
                .map(|(_, joypad)| joypad),
        );
    }

    fn snapshot_for_frame(&mut self, frame: u64, snapshot: &mut Vec<u8>, input: &mut Vec<Joypad>) {
        // Finding the snapshot for a frame is simply looking up the closest cycle for that frame and calling
        // snapshot_for_cycle.
        self.snapshot_for_cycle(
            *self.frame_cycles.range(0..=frame).rev().next().unwrap_or((&0, &0)).1,
            snapshot,
            input,
        );
    }

    fn truncate(&mut self, cycle: u64) {
        // Find all frames with a starting cycle that is greater than the trim boundary.
        let purge = self
            .frame_cycles
            .iter()
            .skip_while(|(_, frame_cycle)| **frame_cycle <= cycle)
            .map(|(frame, _)| *frame)
            .collect::<Vec<u64>>();
        for key in purge {
            self.frame_cycles.remove(&key);
        }

        // Now we clear out all full/delta snapshots that came after the cycle boundary.
        // We take note of the earliest position in the data blob that was encountered.
        fn purge_snapshots(cycle: u64, snapshots: &mut BTreeMap<u64, Range<usize>>) -> Option<usize> {
            let purge = snapshots
                .range(cycle + 1..)
                .map(|(cycle, _)| *cycle)
                .collect::<Vec<u64>>();
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
            .range(cycle + 1..)
            .map(|(cycle, _)| *cycle)
            .collect::<Vec<u64>>();
        for key in purge {
            self.input_events.remove(&key);
        }
    }
}
