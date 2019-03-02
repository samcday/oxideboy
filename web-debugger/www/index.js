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
// TODO: window resize handler for memory view.
// TODO: persist scrollpane preferences in indexeddb
// TODO: hide unhandled segments of memory (echo RAM, unused high registers).
// TODO: visual indicator when we pause execution.

const CPU_REGISTERS = ['af', 'bc', 'de', 'hl', 'sp', 'pc'];
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

    this.state = {
      memoryViewerHeight: 200,
      memDirty: 1,
      cpuState: {
        af: 0xFFFF,
        bc: 0xFFFF,
        de: 0xFFFF,
        hl: 0xFFFF,
        sp: 0xFFFF,
        pc: 0xFFFF,
        ime: false,
        ime_defer: false,
        halted: false,
      },
      paused: false,
      active: false
    };
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
                <CpuRegister
                  name={reg}
                  key={reg}
                  value={this.state.cpuState[reg]}
                  enabled={this.state.paused}
                  onUpdate={this.updateCpuRegister.bind(this, reg)} />
              )
            }
            <div className="register">
              <label htmlFor={`cpu_register_ime`}>IME:</label>
              <input type="checkbox" readOnly={!this.state.paused} id={`cpu_register_ime`} checked={this.state.cpuState.ime} />
            </div>
            <div className="register">
              <label htmlFor={`cpu_register_imd`}>IMD:</label>
              <input type="checkbox" readOnly={!this.state.paused} id={`cpu_register_imd`} checked={this.state.cpuState.ime_defer} />
            </div>
            <div className="register">
              <label htmlFor={`cpu_register_halt`}>HALT:</label>
              <input type="checkbox" readOnly={!this.state.paused} id={`cpu_register_halt`} checked={this.state.cpuState.halted} />
            </div>
            {
              Object.keys(MEM_REGISTERS).map((reg) =>
                <Register
                  name={reg}
                  addr={MEM_REGISTERS[reg]}
                  fn={this.read_memory.bind(this)}
                  enabled={this.state.paused}
                  dirty={this.state.memDirty}
                  key={reg}
                  onUpdate={this.writeMem.bind(this, MEM_REGISTERS[reg])} />
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
                  <button type="button" className="btn btn-outline-secondary" id="restart" onClick={this.restart.bind(this)} disabled={!this.state.active}>
                    <i className="fas fa-redo"></i>
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
                      <Breakpoints list={this.state.breakpoints} onChange={this.updateBreakpoints.bind(this)} disabled={!this.state.active} />
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

  async loadRom(rom) {
    this.emulator = wasm.WebEmu.new(rom, this.newFrame.bind(this), this.breakpointHit.bind(this));

    const breakpoints = await get(`${this.emulator.rom_hash()}-breakpoints`);
    this.emulator.set_breakpoints(breakpoints || []);
    this.setState({active: true, paused: false, breakpoints});
    document.title = `oxideboy-debugger: ${this.emulator.rom_title()}`;
    this.start();
  }

  updateCpuRegister(reg, newVal) {
    let cpuState = this.emulator.cpu_state();
    cpuState[reg] = parseInt(newVal, 16);
    this.emulator.set_cpu_state(cpuState);
    this.update();
  }

  writeMem(addr, newVal) {
    this.emulator.mem_write(addr, parseInt(newVal, 16));
    this.update();
  }

  read_memory(addr) {
    if (!this.emulator) { return 0xFF };
    return this.emulator.mem_read(addr);
  }

  async updateBreakpoints(breakpoints) {
    await set(`${this.emulator.rom_hash()}-breakpoints`, breakpoints);
    this.emulator.set_breakpoints(breakpoints);
    this.setState({breakpoints});
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
    this.setState({paused: false});

    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));
    // this.viewUpdates = setInterval(this.update.bind(this), 100);
  }

  pause() {
    cancelAnimationFrame(this.nextFrame);
    clearTimeout(this.viewUpdates);
    this.lastFrameTimestamp = null;

    setTimeout(() => {
      this.setState({paused: true});
      this.update();
    });
  }

  update() {
    const cpu = this.emulator.cpu_state();
    this.setState(({memDirty}) => ({
      cpuState: cpu,
      instructions: this.emulator.current_instructions(cpu.pc, 11),
      memDirty: memDirty + 1,
    }));
  }

  step() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step();
    this.update();
  }

  stepFrame() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step_frame();
    this.update();
  }

  async restart() {
    this.pause();
    const rom = await get(`${this.emulator.rom_hash()}_rom`);
    this.loadRom(rom);
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
    this.pause();
  }

  runFrame(timestamp) {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));

    let delta = 16; // Default delta amount - enough to render a frame.
    if (this.lastFrameTimestamp) {
      delta = Math.min(1000, timestamp - this.lastFrameTimestamp) // Don't try and emulate more than a second of time.
    }
    this.lastFrameTimestamp = timestamp;

    this.emulator.run(delta * 1000);
    this.update();
  }

  resizeMemoryViewer(newSize) {
    clearTimeout(this.resizeTimeout);
    this.resizeTimeout = setTimeout(() => { this.setState({memoryViewerHeight: newSize}) }, 10);
  }
};

class CpuRegister extends React.Component {
  constructor(props) {
    super(props);
    this.state = {editing: false};
    ['onChange', 'onFocus', 'onBlur', 'onKeyPress'].forEach((name) => this[name] = this[name].bind(this));
  }

  render() {
    return (
      <div className="register">
        <label htmlFor={`cpu_register_${this.props.name}`}>{this.props.name.toUpperCase()}:</label>
        <input  type="text"
                readOnly={!this.props.enabled}
                id={`cpu_register_${this.props.name}`}
                size={4}
                value={this.state.editing ? this.state.value : toPaddedHexString(this.props.value, 4)}
                onChange={this.onChange}
                onBlur={this.onBlur}
                onKeyPress={this.onKeyPress}
                onFocus={this.onFocus} />
      </div>
    );
  }

  onFocus(ev) {
    this.setState({editing: true, value: this.props.value.toString(16)});
  }

  onBlur(ev) {
    this.setState({editing: false});
  }

  onKeyPress(ev) {
    if (ev.key !== 'Enter') {
      return;
    }
    this.setState({editing: false});
    this.props.onUpdate(this.state.value);
    ev.target.blur();
  }

  onChange(ev) {
    this.setState({value: ev.target.value.replace(/[^0-9A-Fa-f]+/g, "").replace(/^0+([1-9a-fA-F].*)/, "$1").substring(0, 4) });
  }
}

class Register extends React.Component {
  constructor(props) {
    super(props);
    this.state = {editing: false};
    ['onChange', 'onFocus', 'onBlur', 'onKeyPress'].forEach((name) => this[name] = this[name].bind(this));
  }

  render() {
    return (
      <div className="register">
        <label htmlFor={`register_${this.props.name}`}>
          <abbr title={`0x${toPaddedHexString(this.props.addr, 4).toUpperCase()}`}>{this.props.name}</abbr>:
        </label>

        <input  type="text"
                readOnly={!this.props.enabled}
                id={`cpu_register_${this.props.name}`}
                size={2}
                value={this.state.editing ? this.state.value : toPaddedHexString(this.props.fn(this.props.addr), 2)}
                onChange={this.onChange}
                onBlur={this.onBlur}
                onKeyPress={this.onKeyPress}
                onFocus={this.onFocus} />
      </div>
    );
  }

  onFocus(ev) {
    this.setState({editing: true, value: this.props.fn(this.props.addr).toString(16)});
  }

  onBlur(ev) {
    this.setState({editing: false});
  }

  onKeyPress(ev) {
    if (ev.key !== 'Enter') {
      return;
    }
    this.setState({editing: false});
    this.props.onUpdate(this.state.value);
    ev.target.blur();
  }

  onChange(ev) {
    this.setState({value: ev.target.value.replace(/[^0-9A-Fa-f]+/g, "").replace(/^0+([1-9a-fA-F].*)/, "$1").substring(0, 2) });
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
            <div key={`inst_${inst.loc}`}>0x{toPaddedHexString(inst.loc, 4)}: {inst.txt}</div>
        ))}
      </div>
    );
  }
}

class Breakpoints extends React.Component {
  constructor(props) {
    super(props);
    this.state = {text: ''};
    this.removeBreakpoint = this.removeBreakpoint.bind(this);
    this.handleChange = this.handleChange.bind(this);
    this.handleSubmit = this.handleSubmit.bind(this);
  }

  render() {
    return (
      <React.Fragment>
      <ul>
        { (this.props.list || []).map((breakpoint, idx) =>
          <li key={idx}>0x{toPaddedHexString(breakpoint, 4)} <a onClick={this.removeBreakpoint.bind(this, idx)}>x</a></li>
        )}
      </ul>

      <form onSubmit={this.handleSubmit}>
        <div className="input-group mb-3">
          <div className="input-group-prepend">
            <span className="input-group-text">0x</span>
          </div>
          <input type="text" className="form-control" value={this.state.text} onChange={this.handleChange} disabled={this.props.disabled} size={4} />
        </div>
      </form>
      </React.Fragment>
    );
  }

  removeBreakpoint(idx, ev) {
    ev.preventDefault();
    this.props.list.splice(idx, 1);
    this.props.onChange(this.props.list);

  }

  handleChange(ev) {
    this.setState({text: ev.target.value.replace(/[^0-9A-Fa-f]+/g, "").replace(/^0+([1-9a-fA-F].*)/, "$1").substring(0, 4) });
  }

  handleSubmit(ev) {
    if (!this.state.text) {
      return;
    }

    ev.preventDefault();
    this.props.onChange((this.props.list || []).concat(parseInt(this.state.text, 16)));
    this.setState({text: ''});
  }
}

ReactDOM.render(<App/>, document.getElementById("root"));

function toPaddedHexString(num, len) {
    const str = num.toString(16);
    return "0".repeat(len - str.length) + str;
}
