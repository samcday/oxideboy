import "../style.scss";
import "@fortawesome/fontawesome-free/css/all.css";
import "golden-layout/src/css/goldenlayout-base.css";
import "golden-layout/src/css/goldenlayout-light-theme.css";

import "@babel/polyfill";
import "jquery";
import "bootstrap";
import GoldenLayout from "golden-layout";

import { toPaddedHexString } from "./util";
import React from "react";
import ReactDOM from "react-dom";
import {FixedSizeList} from "react-window";
import { get, set } from "idb-keyval";
// import { WebEmu } from "../crate/pkg";

import Screen from "./components/Screen";
import Registers from "./components/Registers";

import Worker from "@samcday/worker-loader?name=hash.worker.js!./worker-bootstrap";

// GoldenLayout is a bit of a relic - expects React+ReactDOM to be available on global namespace.
window.React = React;
window.ReactDOM = ReactDOM;

// TODO: hide unhandled segments of memory (echo RAM, unused high registers).
// TODO: visual indicator when we pause execution.

class AppOld extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      memDirty: 1,
      paused: false,
      active: false,
    };
  }

  componentDidMount() {
    document.addEventListener("keydown", this.keyDown = (ev) => {
      if(!this.emulator || ev.target !== document.body) {
        return;
      }
      this.emulator.set_joypad_state(ev.key, true);
    });

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
      <div className="d-flex min-vh-100">
        <div className="left-sidebar min-vh-100 border-right">
        </div>
        <div className="flex-fill position-relative">
              <MemoryViewer height={200} fn={this.read_memory.bind(this)} dirty={this.state.memDirty} />
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
                  <button type="button" className="btn btn-outline-secondary" id="step_frame_backward" onClick={this.stepFrameBack.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className="fas fa-fast-backward"></i>
                  </button>
                  <button type="button" className="btn btn-outline-secondary" id="step_backward" onClick={this.stepBack.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className="fas fa-backward"></i>
                  </button>
                  <button type="button" className="btn btn-outline-secondary" id="step_forward" onClick={this.stepForward.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className="fas fa-forward"></i>
                  </button>
                  <button type="button" className="btn btn-outline-secondary" id="step_frame_forward" onClick={this.stepFrameForward.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className="fas fa-fast-forward"></i>
                  </button>
                  <button type="button" className="btn btn-outline-secondary" id="restart" onClick={this.restart.bind(this)} disabled={!this.state.active}>
                    <i className="fas fa-redo"></i>
                  </button>
                </div>
                <InstructionViewer instructions={this.state.instructions} />
              </div>
            <div>
              <div className="accordion min-vh-100" id="sidebar">


                <div className="card">
                  <div className="card-header" id="cpuHeading">
                    <h2 className="mb-0">
                      <button className="btn btn-link" type="button" data-toggle="collapse" data-target="#cpu" aria-expanded="true" aria-controls="cpu">CPU</button>
                    </h2>
                  </div>

                  <div id="cpu" className="collapse show" aria-labelledby="cpuHeading" >
                    <div className="card-body text-monospace registers form-inline">
                    </div>
                  </div>
                </div>

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
        </div>
      </div>
    );
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

  start() {
    this.setState({paused: false});
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

  stepFrameBack() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step_frame_backward();
    this.update();
  }

  stepBack() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step_backward();
    this.update();
  }

  stepForward() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step_forward();
    this.update();
  }

  stepFrameForward() {
    if (!this.state.paused) {
      return;
    }

    this.emulator.step_frame_forward();
    this.update();
  }

  async restart() {
    this.pause();
    const rom = await get(`${this.emulator.rom_hash()}_rom`);
    this.loadRom(rom);
  }

  newFrame(framebuffer) {
    try {
      
    } catch(err) {
      console.error(err);
    }
  }

  breakpointHit() {
    this.pause();
  }
};

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

  handleSubmit(ev: Event): void {
    if (!this.state.text) {
      return;
    }

    ev.preventDefault();
    this.props.onChange((this.props.list || []).concat(parseInt(this.state.text, 16)));
    this.setState({text: ''});
  }
}

class Dummy extends React.Component {
  render() {
    return (
      <span>ok cool</span>
    );
  }
}

class App {
  container: GoldenLayout;
  worker: Worker;

  constructor() {
    this.worker = new Worker;
    this.worker.onmessage = this.onWorkerMessage;

    this.container = new GoldenLayout({
      settings: {
        showPopoutIcon: false,
        showCloseIcon: false,
      },
      dimensions: {
        minItemHeight: 144,
        minItemWidth: 160,
      },
      content: [
        {
          type: 'row',
          content: [
            {
              type: 'column',
              width: 30,
              content: [
                {
                  type: 'react-component',
                  component: 'Screen',
                  id: 'screen',
                  title: 'LCD',
                },
                {
                  type: 'react-component',
                  component: 'Registers',
                  title: 'Registers',
                },
              ]
            },
            {
              type: 'column',
              width: 70,
              content: [
                {
                  type:'react-component',
                  component: 'Dummy',
                },
                {
                  type:'react-component',
                  component: 'Dummy',
                },
              ]
            }
          ]
        }
      ]
    });

    this.container.registerComponent('Screen', Screen);
    this.container.registerComponent('Registers', Registers);
    this.container.registerComponent('Dummy', Dummy);
  }

  init() {
    this.container.init();

    window.addEventListener("hashchange", this.onHashChange);

    document.body.addEventListener('dragover', this.onDragOver);
    document.body.addEventListener('drop', this.onDrop);
  }


  onWorkerMessage = async (ev) => {
    const message = ev.data;
    
    switch (message.type) {
      case "init": {
        this.onWorkerInit();
        break;
      }
      case "loaded": {
        document.title = `oxideboy-debugger: ${message.rom.title}`;
        window.location = `#${message.rom.hash}`;
        break;
      }
      case "frame": {
        this.container.eventHub.emit('oxideboy:frame', message.buffer);
        break;
      }
    }
  }

  onWorkerInit() {
    this.onHashChange();
  }

  onHashChange = async (ev: HashChangeEvent) => {
    if (window.location.hash) {
      const hash = window.location.hash.substring(1);
      const rom = await get(`${hash}_rom`);
      if (rom) {
        this.loadRom(rom as Uint8Array);
      }
    }
  }

  onDragOver = (ev: Event) => {
    ev.preventDefault();
  }

  onDrop = (ev: DragEvent) => {
    ev.preventDefault();

    if (!ev.dataTransfer || !ev.dataTransfer.items || ev.dataTransfer.items[0].kind !== 'file') {
      return;
    }

    let file = ev.dataTransfer.items[0].getAsFile();
    if (!file) {
      return;
    }

    let reader = new FileReader();
    reader.onload = async () => {
      const rom = new Uint8Array(reader.result as ArrayBuffer);
      this.loadRom(rom);
    };
    reader.readAsArrayBuffer(file);
  }

  onFrame = (framebuffer: Uint16Array) => {
    this.container.eventHub.emit('oxideboy:frame', framebuffer);
  }

  onBreakpointHit = () => {
    console.log("todo");
  }

  loadRom(rom) {
    this.worker.postMessage({type: 'load', rom: rom});
  }

  start() {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));
    this.container.eventHub.emit('oxideboy:start');
  }

  runFrame(timestamp) {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));

    let delta = 16; // Default delta amount - enough to render a frame.
    if (this.lastFrameTimestamp) {
      delta = Math.min(1000, timestamp - this.lastFrameTimestamp) // Don't try and emulate more than a second of time.
    }
    this.lastFrameTimestamp = timestamp;

    this.refreshState();
  }

  refreshState() {
    // const cpu = this.emulator.cpu_state();
    // this.container.eventHub.emit('oxideboy:cpu-state', cpu);
  }
}

new App().init();

/*
  onDrop(ev) {

  }

  async loadRom(rom) {
    const breakpoints = await get(`${this.emulator.rom_hash()}-breakpoints`);
    this.emulator.set_breakpoints(breakpoints || []);
    this.setState({active: true, paused: false, breakpoints});
    this.start();
  }

*/
