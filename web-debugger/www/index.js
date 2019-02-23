import * as wasm from "web-debugger";
import('./style.css');

var emulator = null;
let framebuffer = null;
var lcd = null;
let ctx = null;

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
    fpsStart = performance.now();
    totalFrames = 0;
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

var totalFrames = 0;
var fpsStart = null;

var lastFrameTimestamp = null;
let overhead_start = performance.now();
let overhead = 0;
function runFrame(timestamp) {
  requestAnimationFrame(runFrame);

  if (lastFrameTimestamp === null) {
    lastFrameTimestamp = timestamp;
    return;
  }

  const delta = timestamp - lastFrameTimestamp;
  lastFrameTimestamp = timestamp;

  const start = performance.now();
  const new_frame = emulator.run_frame(delta * 1000, framebuffer);
  if (new_frame) {
    totalFrames += 1;
    ctx.putImageData(new ImageData(framebuffer, 160, 144), 0, 0);
    ctx.drawImage( lcd, 0, 0, 2*lcd.width, 2*lcd.height );
  }
  overhead += performance.now() - start;
  if (performance.now() - overhead_start > 1000) {
    console.log("Spent", overhead, "ms emulating");
    console.log("Gameboy framerate: ", totalFrames / (performance.now() - fpsStart) * 1000);
    overhead_start = performance.now();
    overhead = 0;
  }
}

var lastTimestamp = null;
function fpsTest(timestamp) {
  requestAnimationFrame(fpsTest);

  if (lastTimestamp === null) {
    lastTimestamp = timestamp;
    return;
  }
  // console.log("Frame time:", (timestamp - lastTimestamp) * 1000, "microseconds");
  lastTimestamp = timestamp;
}
requestAnimationFrame(fpsTest);
