import "./style.scss";

import * as wasm from "web-debugger";
import 'bootstrap';
import '@fortawesome/fontawesome-free/css/all.css';

class App {
  constructor(rom) {
    this.lcd = document.querySelector('#lcd');
    this.ctx = this.lcd.getContext('2d');

    this.emulator = wasm.WebEmu.new(rom, (framebuffer) => {
      fps.render();
      try {
        this.ctx.putImageData(new ImageData(framebuffer, 160, 144), 0, 0);
        this.ctx.drawImage( this.lcd, 0, 0, 2*this.lcd.width, 2*this.lcd.height );
      } catch(err) {
        console.error(err);
      }
    });

    this.memRegisterElements = document.querySelectorAll('.register');
    this.cpuRegisterElements = document.querySelectorAll('.cpu-register');
    this.updateMemory();

    this.lastFrameTimestamp = null;
    this.overhead_start = performance.now();

    document.getElementById("start_pause").addEventListener("click", this.pause.bind(this));
  }

  updateMemory() {
    console.time("updateMemory");
    for (let i = 0; i < 65535; i++) {
      document.getElementById(`memory_cell_${i}`).innerText = toPaddedHexString(this.emulator.mem_read(i), 2);
    }
    console.timeEnd("updateMemory");
  }

  updateRegisters() {
    for (const register of this.memRegisterElements) {
      register.value = toPaddedHexString(this.emulator.mem_read(parseInt(register.dataset.address)), 2);
    }
    for (const register of this.cpuRegisterElements) {
      register.value = toPaddedHexString(this.emulator.reg_read(register.dataset.register), 4);
    }
    document.querySelector('#ime').checked = this.emulator.get_ime();
    document.querySelector('#ime_defer').checked = this.emulator.get_ime_defer();

    (document.querySelector('.memory_pc') || {}).className = '';
    document.getElementById(`memory_cell_${this.emulator.reg_read('pc')}`).className = 'memory_pc rounded-pill bg-info';
  }

  runFrame(timestamp) {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));

    if (this.lastFrameTimestamp === null) {
      this.lastFrameTimestamp = timestamp;
      return;
    }

    const delta = Math.min(1000, timestamp - this.lastFrameTimestamp) // Don't try and emulate more than a second of time;
    this.lastFrameTimestamp = timestamp;

    const start = performance.now();
    this.emulator.run(delta * 1000);
    this.overhead += performance.now() - start;
    if (performance.now() - this.overhead_start > 1000) {
      console.log("Spent", this.overhead, "ms emulating");
      this.overhead_start = performance.now();
      this.overhead = 0;
    }

    this.updateRegisters();
  }

  pause() {
    cancelAnimationFrame(this.nextFrame);
    console.log("Pausing.");
  }

  start() {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));
  }
}

let app = null;

function toPaddedHexString(num, len) {
    const str = num.toString(16);
    return "0".repeat(len - str.length) + str;
}

window.keyDown = function(ev) {
  if(!app) { return; }
  app.emulator.set_joypad_state(ev.key, true);
}

window.keyUp = function(ev) {
  if(!app) { return; }
  app.emulator.set_joypad_state(ev.key, false);
}

window.dropHandler = function(ev) {
  document.querySelector('#lcd').style.border = "";
  ev.preventDefault();

  if (!ev.dataTransfer.items || ev.dataTransfer.items[0].kind !== 'file') {
    return;
  }

  var file = ev.dataTransfer.items[0].getAsFile();
  var reader = new FileReader();
  reader.onload = function(e) {
    let rom = new Uint8Array(e.target.result);
    app = new App(rom);
    app.start();
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

(() => {
  const tbody = document.querySelector("#memory_viewer table tbody");

  // console.time("memory_viewer DOM");
  // for (var i = 0; i < 65535; ) {
  //   const row = document.createElement("tr");
  //   const addrCell = document.createElement("td");
  //   addrCell.innerText = toPaddedHexString(i, 4).toUpperCase();
  //   row.appendChild(addrCell);

  //   for (var j = 0; j < 16; j++, i++) {
  //     const cell = document.createElement("td");
  //     cell.dataset.loc = i;
  //     cell.id = `memory_cell_${i}`;
  //     cell.innerText = "FF";
  //     row.appendChild(cell);
  //   }
  //   tbody.appendChild(row);
  // }
  // console.timeEnd("memory_viewer DOM");

  console.time("memory_viewer innerHTML");
  var html = "";
  for (var i = 0; i < 65535;) {
    html += "<tr>";
    html += "<td>";
    html += toPaddedHexString(i, 4).toUpperCase();
    html += "</td>";

    for (var j = 0; j < 16; j++, i++) {
      html += `<td id="memory_cell_${i}" data-loc="${i}">`;
      html += "FF";
      html += "</td>";
    }
    html += "<td></td>";
    html += "</tr>";
  }
  tbody.innerHTML = html;
  console.timeEnd("memory_viewer innerHTML");
})();
