import '@babel/polyfill';
import { get, set } from 'idb-keyval';

import {WebEmu} from '../crate-worker/pkg';

const TICK_INTERVAL = 2; // in milliseconds

let emulator = undefined;
let lastTick = null;
let tickId = null;

function onBreakpointHit() {

}

// Runs the emulator for a single "tick". Our ticks are just regular intervals (see TICK_INTERVAL) where we run the
// emulator core for the amount of interval time that has passed.
function tick() {
  emulator.run(TICK_INTERVAL * 1000);
}

const messageHandlers = {};  // Message handlers that are always responsive.
const loadedHandlers = {};   // Message handlers that only work when the emulator is loaded.

onmessage = async (ev) => {
  const message = ev.data;

  if (messageHandlers[message.type]) {
    messageHandlers[message.type](message);
    return;
  }

  if (emulator) {
    (loadedHandlers[message.type] || (() => {}))(message);
  }
};

messageHandlers.load = async (message) => {
  emulator = WebEmu.new(message.rom, onBreakpointHit);
  const rom_hash = emulator.rom_hash();
  await set(`${rom_hash}_rom`, message.rom);
  postMessage({type: 'loaded', rom: {
    title: emulator.rom_title(),
    hash: rom_hash,
  }});
};

loadedHandlers.start = (message) => {
  tickId = setInterval(tick, TICK_INTERVAL);
  postMessage({type: 'running'});
};

loadedHandlers.pause = (message) => {
  clearInterval(tickId);
  tickId = null;
  postMessage({type: 'paused'});
};

loadedHandlers.keydown = (message) => {
  emulator.set_joypad_state(message.key, true);
};

loadedHandlers.keyup = (message) => {
  emulator.set_joypad_state(message.key, false);
};

loadedHandlers.refresh = (message) => {
  const cpu = emulator.cpu_state();
  const {framebuffer, memory} = message;
  emulator.update_state(framebuffer, memory);
  postMessage({type: 'state', cpu, framebuffer, memory}, [framebuffer.buffer, memory.buffer]);
};

postMessage({type: 'init'});
