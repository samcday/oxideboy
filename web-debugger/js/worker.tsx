import '@babel/polyfill';
import { get, set } from 'idb-keyval';

import {WebEmu, test} from '../crate/pkg';

const TICK_INTERVAL = 2; // in milliseconds

let emulator = undefined;
let lastTick = null;
let tickId = null;

function onBreakpointHit() {

}

// Runs the emulator for a single "tick". Our ticks are just regular intervals (see TICK_INTERVAL) where we run the
// emulator core for the amount of interval time that has passed.
function tick() {
  const newTick = performance.now();
  lastTick = newTick;
  emulator.run(TICK_INTERVAL * 1000);
}

// Bundles up all information about current state of the emulator (registers, memory, etc) and sends it to the UI.
function sendState(framebuffer, memory) {
  const cpuState = emulator.cpu_state();
  emulator.update_state(framebuffer, memory);
  postMessage({type: 'state', cpu: cpuState, framebuffer, memory}, [framebuffer.buffer, memory.buffer]);
}

onmessage = async (ev) => {
  const message = ev.data;
  switch (message.type) {
    case 'load': {
      emulator = WebEmu.new(message.rom, onBreakpointHit);
      lastTick = performance.now();
      tickId = setInterval(tick, TICK_INTERVAL);

      const rom_hash = emulator.rom_hash();
      await set(`${rom_hash}_rom`, message.rom);
      postMessage({type: 'loaded', rom: {
        title: emulator.rom_title(),
        hash: rom_hash,
      }});

      break;
    }
    case 'keydown': {
      if (emulator) emulator.set_joypad_state(message.key, true);
      break;
    }
    case 'keyup': {
      if (emulator) emulator.set_joypad_state(message.key, false);
      break;
    }
    case 'refresh': {
      if (emulator) sendState(message.framebuffer, message.memory);
      break;
    }
  }
};

postMessage({type: 'init'});
