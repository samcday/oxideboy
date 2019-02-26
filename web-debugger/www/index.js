import "./style.scss";

import React from "react";
import ReactDOM from "react-dom";
import SplitPane from "react-split-pane";
import {FixedSizeList} from "react-window";
import * as wasm from "web-debugger";
import 'bootstrap';
import '@fortawesome/fontawesome-free/css/all.css';

function toPaddedHexString(num, len) {
    const str = num.toString(16);
    return "0".repeat(len - str.length) + str;
}

// TODO: debounce memory view resize handler.

const CPU_REGISTERS = ['AF', 'BC', 'DE', 'HL', 'SP', 'PC'];
const MEM_REGISTERS = {
    IF: 0xFF0F,
    IE: 0xFFFF,
   DIV: 0xFF04,
  TIMA: 0xFF05,
   TMA: 0xFF06,
   TAC: 0xFF07,
    P1: 0xFF00,
    SB: 0xFF01,
    SC: 0xFF02,
  LCDC: 0xFF40,
  STAT: 0xFF41,
   SCY: 0xFF42,
   SCX: 0xFF43,
    LY: 0xFF44,
   LYC: 0xFF45,
   DMA: 0xFF46,
   BGP: 0xFF47,
  OBP0: 0xFF48,
  OBP1: 0xFF49,
    WY: 0xFF4A,
    WX: 0xFF4B,
  NR10: 0xFF10,
  NR11: 0xFF11,
  NR12: 0xFF12,
  NR13: 0xFF13,
  NR14: 0xFF14,
  NR21: 0xFF16,
  NR22: 0xFF17,
  NR23: 0xFF18,
  NR24: 0xFF19,
  NR30: 0xFF1A,
  NR31: 0xFF1B,
  NR32: 0xFF1C,
  NR33: 0xFF1D,
  NR34: 0xFF1E,
  NR41: 0xFF20,
  NR42: 0xFF21,
  NR43: 0xFF22,
  NR44: 0xFF23,
  NR50: 0xFF24,
  NR51: 0xFF25,
  NR52: 0xFF26,
};

class App extends React.Component {
  constructor(props) {
    super(props);

    this.state = {memoryViewerHeight: 200, memDirty: 1, cpuDirty: 1};
  }

  componentDidMount() {
    this.lcd = this.refs.lcd;
    this.ctx = this.lcd.getContext('2d');
    this.setState({memoryViewerHeight: this.refs.split.pane1.getBoundingClientRect().height});
  }

  render() {
    return (
      <div className="d-flex min-vh-100" onDragOver={this.onDragOver} onDrop={this.onDrop.bind(this)}>
        <div className="left-sidebar min-vh-100 border-right">
          <canvas ref="lcd" width="320" height="288" className="border-bottom" />
          <div className="d-flex px-3 flex-wrap text-monospace registers">
            {
              CPU_REGISTERS.map((reg) =>
                <CpuRegister name={reg} key={reg} fn={this.read_register.bind(this)} dirty={this.state.cpuDirty} />
              )
            }
            {
              Object.keys(MEM_REGISTERS).map((reg) =>
                <Register name={reg} addr={MEM_REGISTERS[reg]} fn={this.read_memory.bind(this)} dirty={this.state.memDirty} key={reg} />
              )
            }
          </div>
        </div>
        <div className="flex-fill position-relative">
          <SplitPane ref="split" split="horizontal" defaultSize="80%" onChange={this.resizeMemoryViewer.bind(this)}>
            <MemoryViewer height={this.state.memoryViewerHeight} fn={this.read_memory.bind(this)} dirty={this.state.memDirty} />
            <div>

            </div>
          </SplitPane>
        </div>
      </div>
    );
  }

  read_register(reg) {
    if (!this.emulator) { return 0xFFFF };
    return this.emulator.reg_read(reg);
  }

  read_memory(addr) {
    if (!this.emulator) { return 0xFF };
    return this.emulator.mem_read(addr);
  }

  onDragOver(ev) {
    ev.preventDefault();
  }

  onDrop(ev) {
    ev.preventDefault();

    if (!ev.dataTransfer.items || ev.dataTransfer.items[0].kind !== 'file') {
      return;
    }

    var file = ev.dataTransfer.items[0].getAsFile();
    var reader = new FileReader();
    reader.onload = (e) => {
      const rom = new Uint8Array(e.target.result);
      this.emulator = wasm.WebEmu.new(rom, (framebuffer) => {
        // fps.render();
        try {
          this.ctx.putImageData(new ImageData(framebuffer, 160, 144), 0, 0);
          this.ctx.drawImage( this.lcd, 0, 0, 2*this.lcd.width, 2*this.lcd.height );
        } catch(err) {
          console.error(err);
        }
      });
      this.start();
    };
    reader.readAsArrayBuffer(file);
  }

  start() {
    requestAnimationFrame(this.runFrame.bind(this));
  }

  runFrame(timestamp) {
    requestAnimationFrame(this.runFrame.bind(this));

    if (!this.lastFrameTimestamp) {
      this.lastFrameTimestamp = timestamp;
      return;
    }

    const delta = Math.min(1000, timestamp - this.lastFrameTimestamp) // Don't try and emulate more than a second of time;
    this.lastFrameTimestamp = timestamp;

    // const start = performance.now();
    this.emulator.run(delta * 1000);
    // this.overhead += performance.now() - start;
    // if (performance.now() - this.overhead_start > 1000) {
    //   console.log("Spent", this.overhead, "ms emulating");
    //   this.overhead_start = performance.now();
    //   this.overhead = 0;
    // }
    this.setState(({cpuDirty, memDirty}) => {
      return {
        memDirty: memDirty + 1,
        cpuDirty: cpuDirty + 1,
      };
    })
  }

  resizeMemoryViewer(newSize) {
    this.setState({memoryViewerHeight: newSize});
  }
};

class CpuRegister extends React.Component {
  shouldComponentUpdate(nextProps) {
    return nextProps.dirty > this.props.dirty;
  }

  render() {
    return (
      <div className="register">
        <label htmlFor={`cpu_register_${this.props.name}`}>{this.props.name}:</label>
        <input readOnly id={`cpu_register_${this.props.name}`} size={4} value={toPaddedHexString(this.props.fn(this.props.name) || 65535, 4)} />
      </div>
    );
  }
}

class Register extends React.Component {
  shouldComponentUpdate(nextProps) {
    return nextProps.dirty > this.props.dirty;
  }

  render() {
    return (
      <div className="register">
        <label htmlFor={`register_${this.props.name}`}>
          <abbr title={`0x${toPaddedHexString(this.props.addr, 4).toUpperCase()}`}>{this.props.name}</abbr>:
        </label>
        <input readOnly id={`register_${this.props.name}`} size={2} value={toPaddedHexString(this.props.fn(this.props.addr), 2)} />
      </div>
    );
  }
}

class MemoryViewer extends React.Component {
  render() {
    return (
      <FixedSizeList height={this.props.height} itemCount={4096} itemSize={25} itemData={[this.props.dirty, this.props.fn]} width="100%" className="text-monospace">
        {this.row}
      </FixedSizeList>
    );
  }

  row({data, index, style}) {
    const address = index * 16;
    const [_, fn] = data;

    let values = [];
    for (let i = 0; i < 16; i++) {
      values.push(<div key={`address_${address+i}`} className="memory-cell mx-1">{toPaddedHexString(fn(address+i), 2)}</div>);
    }

    return (
      <div style={style}>
        <div className="memory-address bg-light pl-1 pr-2 mr-2">0x{toPaddedHexString(address, 4)}</div>
        {values}
      </div>
    );
  }
}

ReactDOM.render(<App/>, document.getElementById("root"));

/*
class App {
  constructor(rom) {
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


  pause() {
    cancelAnimationFrame(this.nextFrame);
    console.log("Pausing.");
  }

  start() {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));
  }
}

let app = null;

window.keyDown = function(ev) {
  if(!app) { return; }
  app.emulator.set_joypad_state(ev.key, true);
}

window.keyUp = function(ev) {
  if(!app) { return; }
  app.emulator.set_joypad_state(ev.key, false);
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

*/