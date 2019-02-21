import * as wasm from "web-debugger";
import('./style.css');

var emulator = null;
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
    console.log("ok...", e.target.result.byteLength);
    let rom = new Uint8Array(e.target.result);
    console.log("ok...", rom.byteLength);
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

function runFrame() {
  const buf = emulator.run_frame();
  ctx.putImageData(new ImageData(buf, 160, 144), 0, 0);
  ctx.drawImage( lcd, 0, 0, 2*lcd.width, 2*lcd.height );
  requestAnimationFrame(runFrame);
}

let now = Date.now();
let count = 0;
function fpsTest() {
  count += 1;
  if (Date.now() - now > 1000) {
    console.log("FPS:", count);
    now = Date.now();
    count = 0;
  }
  requestAnimationFrame(fpsTest);
}
requestAnimationFrame(fpsTest);
