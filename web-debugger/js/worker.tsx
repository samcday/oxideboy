import '@babel/polyfill';
import { get, set } from 'idb-keyval';

import {WebEmu} from '../crate/pkg';

let emulator = null;
let lastTick = null;
let tickId = null;

function onFrame(framebuffer) {
  postMessage({type: 'frame', buffer: framebuffer});
}

function onBreakpointHit() {

}

function tick() {
  const newTick = performance.now();
  const elapsed = newTick - lastTick;
  lastTick = newTick;
  emulator.run(1000);
  tickId = setTimeout(tick, 1);
}

onmessage = async (ev) => {
  const message = ev.data;
  switch (message.type) {
    case 'load': {
      emulator = WebEmu.new(message.rom, onFrame, onBreakpointHit);
      lastTick = performance.now();
      tickId = setTimeout(tick, 1);

      const rom_hash = emulator.rom_hash();
      await set(`${rom_hash}_rom`, message.rom);
      postMessage({type: 'loaded', rom: {
        title: emulator.rom_title(),
        hash: rom_hash,
      }})
      break;
    }
  }
};

postMessage({type: 'init'});
