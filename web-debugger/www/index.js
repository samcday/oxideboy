import "@babel/polyfill";

import "./style.scss";
import "@fortawesome/fontawesome-free/css/all.css";

import "bootstrap";
import React from "react";
import ReactDOM from "react-dom";
import SplitPane from "react-split-pane";
import {FixedSizeList} from "react-window";
import { get, set } from "idb-keyval";
import * as wasm from "web-debugger";

// TODO: debounce memory view resize handler.
// TODO: hide unhandled segments of memory (echo RAM, unused high registers).

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

    this.state = {memoryViewerHeight: 200, memDirty: 1, cpuDirty: 1, paused: true, active: false};
  }

  componentDidMount() {
    this.lcd = this.refs.lcd;
    this.ctx = this.lcd.getContext('2d');
    this.setState({memoryViewerHeight: this.refs.split.pane1.getBoundingClientRect().height});

    document.addEventListener("keydown", this.keyDown = (ev) => {
      if(!this.emulator || ev.target !== document.body) {
        return;
      }
      this.emulator.set_joypad_state(ev.key, true);
    });

    window.addEventListener("hashchange", this.hashChange = (ev) => {
      this.onHashChange();
    }, false);

    document.addEventListener("keyup", this.keyUp = (ev) => {
      if (!this.emulator || ev.target !== document.body) {
        return;
      }
      this.emulator.set_joypad_state(ev.key, false);
    });

    this.onHashChange();
  }

  componentWillUnmount() {
    document.removeEventListener("keydown", this.keyDown);
    document.removeEventListener("keyup", this.keyUp);
    document.removeEventListener("hashchange", this.hashChange);
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
            <div className="register">
              <label htmlFor={`cpu_register_ime`}>IME:</label>
              <input type="checkbox" readOnly id={`cpu_register_ime`} checked={this.emulator ? this.emulator.get_ime() : false} dirty={this.state.cpuDirty} />
            </div>
            <div className="register">
              <label htmlFor={`cpu_register_imd`}>IMD:</label>
              <input type="checkbox" readOnly id={`cpu_register_imd`} checked={this.emulator ? this.emulator.get_ime_defer() : false} dirty={this.state.cpuDirty} />
            </div>
            <div className="register">
              <label htmlFor={`cpu_register_halt`}>HALT:</label>
              <input type="checkbox" readOnly id={`cpu_register_halt`} checked={this.emulator ? this.emulator.get_halted() : false} dirty={this.state.cpuDirty} />
            </div>
            {
              Object.keys(MEM_REGISTERS).map((reg) =>
                <Register name={reg} addr={MEM_REGISTERS[reg]} fn={this.read_memory.bind(this)} dirty={this.state.memDirty} key={reg} />
              )
            }
          </div>
        </div>
        <div className="flex-fill position-relative">
          <SplitPane split="vertical" height="100%" defaultSize="80%">
            <SplitPane ref="split" split="horizontal" defaultSize="70%" onChange={this.resizeMemoryViewer.bind(this)}>
              <MemoryViewer height={this.state.memoryViewerHeight} fn={this.read_memory.bind(this)} dirty={this.state.memDirty} />
              <div className="h-100">
                <div className="btn-group" role="group">
                  { this.state.paused &&
                    <button type="button" className="btn btn-outline-secondary" id="start" onClick={this.start.bind(this)} disabled={!this.state.active}>
                      <i className="fas fa-play"></i>
                    </button>
                  }
                  { !this.state.paused &&
                    <button type="button" className="btn btn-outline-secondary" id="pause" onClick={this.pause.bind(this)} disabled={!this.state.active}>
                      <i className="fas fa-pause"></i>
                    </button>
                  }
                  <button type="button" className="btn btn-outline-secondary" id="step" onClick={this.step.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className="fas fa-forward"></i>
                  </button>
                  <button type="button" className="btn btn-outline-secondary" id="step_frame" onClick={this.stepFrame.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className="fas fa-fast-forward"></i>
                  </button>
                </div>
                <InstructionViewer instructions={this.state.instructions} />
              </div>
            </SplitPane>
            <div>
              <div className="accordion min-vh-100" id="sidebar">
                <div className="card">
                  <div className="card-header" id="breakpointsHeading">
                    <h2 className="mb-0">
                      <button className="btn btn-link" type="button" data-toggle="collapse" data-target="#breakpoints" aria-expanded="true" aria-controls="breakpoints">Breakpoints</button>
                    </h2>
                  </div>

                  <div id="breakpoints" className="collapse show" aria-labelledby="breakpointsHeading" >
                    <div className="card-body">
                      TODO.
                    </div>
                  </div>
                </div>
                <div className="card">
                  <div className="card-header" id="watchesHeading">
                    <h2 className="mb-0">
                      <button className="btn btn-link" type="button" data-toggle="collapse" data-target="#watches" aria-expanded="true" aria-controls="watches">Watches</button>
                    </h2>
                  </div>

                  <div id="watches" className="collapse show" aria-labelledby="breakpointsHeading" >
                    <div className="card-body">
                      TODO.
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </SplitPane>
        </div>
      </div>
    );
  }

  async onHashChange() {
    if (window.location.hash) {
      const hash = window.location.hash.substring(1);
      const rom = await get(`${hash}_rom`);
      if (rom) {
        this.loadRom(rom);
      }
    }
  }

  loadRom(rom) {
    this.emulator = wasm.WebEmu.new(rom, this.newFrame.bind(this), this.breakpointHit.bind(this));
    this.setState({active: true});
    document.title = `oxideboy-debugger: ${this.emulator.rom_title()}`;
    this.start();
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
    reader.onload = async (e) => {
      const rom = new Uint8Array(e.target.result);
      this.loadRom(rom);

      const hash = this.emulator.rom_hash();
      await set(`${hash}_rom`, rom);
      window.location = `#${hash}`;
    };
    reader.readAsArrayBuffer(file);
  }

  start() {
    if (!this.state.paused) {
      return;
    }
    this.setState({paused: false});

    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));
    this.viewUpdates = setInterval(this.updateDebuggerView.bind(this), 100);
  }

  pause() {
    if (this.state.paused) {
      return;
    }

    cancelAnimationFrame(this.nextFrame);
    clearTimeout(this.viewUpdates);
    this.lastFrameTimestamp = null;

    setTimeout(() => {
      this.setState(({cpuDirty, memDirty}) => {
        return {
          paused: true,
          memDirty: memDirty + 1,
          cpuDirty: cpuDirty + 1,
        };
      });
      this.updateInstruction();
      // TODO: Scroll to PC.
    });
  }

  step() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step();
    this.setState(({cpuDirty, memDirty}) => {
      return {
        memDirty: memDirty + 1,
        cpuDirty: cpuDirty + 1,
      };
    });
    this.updateInstruction();
  }

  stepFrame() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step_frame();
    this.setState(({cpuDirty, memDirty}) => {
      return {
        memDirty: memDirty + 1,
        cpuDirty: cpuDirty + 1,
      };
    });
    this.updateInstruction();
  }

  newFrame(framebuffer) {
    try {
      this.ctx.putImageData(new ImageData(framebuffer, 160, 144), 0, 0);
      this.ctx.drawImage( this.lcd, 0, 0, 2*this.lcd.width, 2*this.lcd.height );
    } catch(err) {
      console.error(err);
    }
  }

  breakpointHit() {
    if (this.state.paused) {
      return;
    }

    this.pause();
  }

  updateInstruction() {
    this.setState({instructions: this.emulator.current_instructions(this.emulator.reg_read('PC'), 11)});
  }

  runFrame(timestamp) {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));

    let delta = 16; // Default delta amount - enough to render a frame.
    if (this.lastFrameTimestamp) {
      delta = Math.min(1000, timestamp - this.lastFrameTimestamp) // Don't try and emulate more than a second of time.
    }
    this.lastFrameTimestamp = timestamp;

    this.emulator.run(delta * 1000);
  }

  updateDebuggerView() {
    this.updateInstruction();
    this.setState(({cpuDirty, memDirty}) => {
      return {
        memDirty: memDirty + 1,
        cpuDirty: cpuDirty + 1,
      };
    });
  }

  resizeMemoryViewer(newSize) {
    clearTimeout(this.resizeTimeout);
    this.resizeTimeout = setTimeout(() => { this.setState({memoryViewerHeight: newSize}) }, 10);
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
        <input type="text" readOnly id={`cpu_register_${this.props.name}`} size={4} value={toPaddedHexString(this.props.fn(this.props.name) || 65535, 4)} />
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
        <input type="text" readOnly id={`register_${this.props.name}`} size={2} value={toPaddedHexString(this.props.fn(this.props.addr), 2)} />
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

class InstructionViewer extends React.Component {
  render() {
    return (
      <div className="text-monospace">
        { (this.props.instructions || []).map((inst) => (
            <div>0x{toPaddedHexString(inst.loc, 4)}: {inst.txt}</div>
        ))}
      </div>
    );
  }
}

ReactDOM.render(<App/>, document.getElementById("root"));

function toPaddedHexString(num, len) {
    const str = num.toString(16);
    return "0".repeat(len - str.length) + str;
}
