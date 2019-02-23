import "./style.scss";

import * as wasm from "web-debugger";
import 'bootstrap';
import '@fortawesome/fontawesome-free/css/all.css';

var emulator = null;
let framebuffer = null;
var lcd = null;
let ctx = null;

function toPaddedHexString(num, len) {
    const str = num.toString(16);
    return "0".repeat(len - str.length) + str;
}

window.keyDown = function(ev) {
  if(!emulator) { return; }
  emulator.set_joypad_state(ev.key, true);
}

window.keyUp = function(ev) {
  if(!emulator) { return; }
  emulator.set_joypad_state(ev.key, false);
}

window.dropHandler = function(ev) {
  document.querySelector('#lcd').style.border = "";
  ev.preventDefault();

  if (!ev.dataTransfer.items) {
    return;
  }

  if (ev.dataTransfer.items[0].kind !== 'file') {
    return;
  }

  var file = ev.dataTransfer.items[0].getAsFile();

  var reader = new FileReader();
  reader.onload = function(e) {
    let rom = new Uint8Array(e.target.result);
    framebuffer = new Uint8ClampedArray(160 * 144 * 4);
    emulator = wasm.WebEmu.new(rom);
    lcd = document.querySelector('#lcd');
    ctx = lcd.getContext('2d');
    runFrame();
  };
  reader.readAsArrayBuffer(file);
};

window.dragOverHandler = function(ev) {
  ev.preventDefault();
  document.querySelector('#lcd').style.border = "1px solid black";
};

window.dragLeaveHandler = function(ev) {
  document.querySelector('#lcd').style.border = "";
};

function updateRegisters() {
  for (const register of document.querySelectorAll('.register')) {
    register.value = toPaddedHexString(emulator.mem_read(parseInt(register.dataset.address)), 2);
  }
  for (const register of document.querySelectorAll('.cpu-register')) {
    register.value = toPaddedHexString(emulator.reg_read(register.dataset.register), 4);
  }
  document.querySelector('#ime').checked = emulator.get_ime();
  document.querySelector('#ime_defer').checked = emulator.get_ime_defer();
}

var lastFrameTimestamp = null;
let overhead_start = performance.now();
let overhead = 0;
function runFrame(timestamp) {
  requestAnimationFrame(runFrame);

  if (lastFrameTimestamp === null) {
    lastFrameTimestamp = timestamp;
    return;
  }

  const delta = Math.min(1000, timestamp - lastFrameTimestamp) // Don't try and emulate more than a second of time;
  lastFrameTimestamp = timestamp;

  const start = performance.now();
  const new_frame = emulator.run_frame(delta * 1000, framebuffer);
  if (new_frame) {
    fps.render();
    ctx.putImageData(new ImageData(framebuffer, 160, 144), 0, 0);
    ctx.drawImage( lcd, 0, 0, 2*lcd.width, 2*lcd.height );
  }
  overhead += performance.now() - start;
  if (performance.now() - overhead_start > 1000) {
    console.log("Spent", overhead, "ms emulating");
    overhead_start = performance.now();
    overhead = 0;
  }

  updateRegisters();
}

const fps = new class {
  constructor() {
    this.fps = document.getElementById("fps");
    this.frames = [];
    this.lastFrameTimeStamp = performance.now();
  }

  render() {
    // Convert the delta time since the last frame render into a measure
    // of frames per second.
    const now = performance.now();
    const delta = now - this.lastFrameTimeStamp;
    this.lastFrameTimeStamp = now;
    const fps = 1 / delta * 1000;

    // Save only the latest 100 timings.
    this.frames.push(fps);
    if (this.frames.length > 100) {
      this.frames.shift();
    }

    // Find the max, min, and mean of our 100 latest timings.
    let min = Infinity;
    let max = -Infinity;
    let sum = 0;
    for (let i = 0; i < this.frames.length; i++) {
      sum += this.frames[i];
      min = Math.min(this.frames[i], min);
      max = Math.max(this.frames[i], max);
    }
    let mean = sum / this.frames.length;

    // Render the statistics.
    this.fps.textContent = `
Frames per Second:
         latest = ${fps.toFixed(2)}
avg of last 100 = ${mean.toFixed(2)}
min of last 100 = ${min.toFixed(2)}
max of last 100 = ${max.toFixed(2)}
`.trim();
  }
};
