import '../style.scss';
import '@fortawesome/fontawesome-free/css/all.css';
import 'golden-layout/src/css/goldenlayout-base.css';
import 'golden-layout/src/css/goldenlayout-dark-theme.css';

import '@babel/polyfill';
import 'jquery';
import 'bootstrap';
import GoldenLayout from 'golden-layout';

import { toPaddedHexString } from './util';
import React from 'react';
import ReactDOM from 'react-dom';
import { get, set } from 'idb-keyval';

import Screen from './components/Screen';
import Registers from './components/Registers';
import Memory from './components/Memory';
import Disassembler from './components/Disassembler';

import Worker from '@samcday/worker-loader?name=hash.worker.js!./worker-bootstrap';

// GoldenLayout is a bit of a relic - expects React+ReactDOM to be available on global namespace.
window.React = React;
window.ReactDOM = ReactDOM;

// TODO: hide unhandled segments of memory (echo RAM, unused high registers).
// TODO: visual indicator when we pause execution.

class AppOld extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      paused: false,
      active: false,
    };
  }

  render() {
    return (
      <div className='d-flex min-vh-100'>
        <div className='left-sidebar min-vh-100 border-right'>
        </div>
        <div className='flex-fill position-relative'>
              <div className='h-100'>
                <div className='btn-group' role='group'>
                  { this.state.paused &&
                    <button type='button' className='btn btn-outline-secondary' id='start' onClick={this.start.bind(this)} disabled={!this.state.active}>
                      <i className='fas fa-play'></i>
                    </button>
                  }
                  { !this.state.paused &&
                    <button type='button' className='btn btn-outline-secondary' id='pause' onClick={this.pause.bind(this)} disabled={!this.state.active}>
                      <i className='fas fa-pause'></i>
                    </button>
                  }
                  <button type='button' className='btn btn-outline-secondary' id='step_frame_backward' onClick={this.stepFrameBack.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className='fas fa-fast-backward'></i>
                  </button>
                  <button type='button' className='btn btn-outline-secondary' id='step_backward' onClick={this.stepBack.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className='fas fa-backward'></i>
                  </button>
                  <button type='button' className='btn btn-outline-secondary' id='step_forward' onClick={this.stepForward.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className='fas fa-forward'></i>
                  </button>
                  <button type='button' className='btn btn-outline-secondary' id='step_frame_forward' onClick={this.stepFrameForward.bind(this)} disabled={!this.state.active || !this.state.paused}>
                    <i className='fas fa-fast-forward'></i>
                  </button>
                  <button type='button' className='btn btn-outline-secondary' id='restart' onClick={this.restart.bind(this)} disabled={!this.state.active}>
                    <i className='fas fa-redo'></i>
                  </button>
                </div>
                <InstructionViewer instructions={this.state.instructions} />
              </div>
            <div>
              <div className='accordion min-vh-100' id='sidebar'>


                <div className='card'>
                  <div className='card-header' id='cpuHeading'>
                    <h2 className='mb-0'>
                      <button className='btn btn-link' type='button' data-toggle='collapse' data-target='#cpu' aria-expanded='true' aria-controls='cpu'>CPU</button>
                    </h2>
                  </div>

                  <div id='cpu' className='collapse show' aria-labelledby='cpuHeading' >
                    <div className='card-body text-monospace registers form-inline'>
                    </div>
                  </div>
                </div>

                <div className='card'>
                  <div className='card-header' id='breakpointsHeading'>
                    <h2 className='mb-0'>
                      <button className='btn btn-link' type='button' data-toggle='collapse' data-target='#breakpoints' aria-expanded='true' aria-controls='breakpoints'>Breakpoints</button>
                    </h2>
                  </div>

                  <div id='breakpoints' className='collapse show' aria-labelledby='breakpointsHeading' >
                    <div className='card-body'>
                      <Breakpoints list={this.state.breakpoints} onChange={this.updateBreakpoints.bind(this)} disabled={!this.state.active} />
                    </div>
                  </div>
                </div>
                <div className='card'>
                  <div className='card-header' id='watchesHeading'>
                    <h2 className='mb-0'>
                      <button className='btn btn-link' type='button' data-toggle='collapse' data-target='#watches' aria-expanded='true' aria-controls='watches'>Watches</button>
                    </h2>
                  </div>

                  <div id='watches' className='collapse show' aria-labelledby='breakpointsHeading' >
                    <div className='card-body'>
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
        <div className='input-group mb-3'>
          <div className='input-group-prepend'>
            <span className='input-group-text'>0x</span>
          </div>
          <input type='text' className='form-control' value={this.state.text} onChange={this.handleChange} disabled={this.props.disabled} size={4} />
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
    this.setState({text: ev.target.value.replace(/[^0-9A-Fa-f]+/g, '').replace(/^0+([1-9a-fA-F].*)/, '$1').substring(0, 4) });
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

class App {
  container: GoldenLayout;
  worker: Worker;
  refreshRaf: number;
  framebuffer?: Uint16Array = undefined;
  memory?: Uint8Array = undefined;

  constructor() {
    this.worker = new Worker;
    this.worker.onmessage = this.onWorkerMessage;

    this.refreshRaf = requestAnimationFrame(this.requestRefresh);

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
                  component: 'Memory',
                  title: 'Memory',
                },
                {
                  type:'react-component',
                  component: 'Disassembler',
                  title: 'Disassembly',
                },
              ]
            }
          ]
        }
      ]
    });

    this.container.registerComponent('Screen', Screen);
    this.container.registerComponent('Registers', Registers);
    this.container.registerComponent('Memory', Memory);
    this.container.registerComponent('Disassembler', Disassembler);
  }

  init() {
    this.container.init();

    document.addEventListener('keydown', this.keyDown = (ev) => {
      if(ev.target !== document.body) {
        return;
      }
      this.worker.postMessage({type: 'keydown', key: ev.key});
    });

    document.addEventListener('keyup', this.keyUp = (ev) => {
      if (ev.target !== document.body) {
        return;
      }
      this.worker.postMessage({type: 'keyup', key: ev.key});
    });

    window.addEventListener('hashchange', this.onHashChange);

    document.body.addEventListener('dragover', this.onDragOver);
    document.body.addEventListener('drop', this.onDrop);
  }

  requestRefresh = () => {
    requestAnimationFrame(this.requestRefresh);
    if (!this.framebuffer) {
      return;
    }

    this.container.eventHub.emit('oxideboy:frame', this.framebuffer);
    this.worker.postMessage({
      type: 'refresh',
      framebuffer: this.framebuffer,
      memory: this.memory,
    }, [this.framebuffer.buffer, this.memory.buffer]);
    this.framebuffer = null;
    this.memory = null;
  }

  onWorkerMessage = async (ev) => {
    const message = ev.data;
    
    switch (message.type) {
      case 'init': {
        this.onWorkerInit();
        break;
      }
      case 'loaded': {
        this.onRomLoaded(message);
        break;
      }
      case 'state': {
        this.framebuffer = message.framebuffer;
        this.memory = message.memory;
        this.container.eventHub.emit('oxideboy:cpu-state', message.cpu);
        this.container.eventHub.emit('oxideboy:memory', message.memory);
        this.container.eventHub.emit('oxideboy:instructions', message.instructions);
        break;
      }
    }
  }

  onWorkerInit() {
    this.onHashChange();
  }

  onRomLoaded(info) {
    document.title = `oxideboy-debugger: ${info.rom.title}`;
    window.location = `#${info.rom.hash}`;

    // This is the buffer we'll start passing back and forth with the Worker. The buffer is Transferable, so there's
    // zero copying going on. Instead, each time we're ready for a new frame (dictated by rAF), we simply send the
    // framebuffer to the worker to have the latest frame copied into it (from the core framebuffer in Rust/WASM land).
    // Once the copy is done the buffer is handed back. And on it goes.
    this.framebuffer = new Uint16Array(160*144);
    this.memory = new Uint8Array(0x10000);
    this.worker.postMessage({type: 'start'});
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
    console.log('todo');
  }

  loadRom(rom) {
    this.worker.postMessage({type: 'load', rom: rom});
  }

  start() {
    this.nextFrame = requestAnimationFrame(this.runFrame.bind(this));
    this.container.eventHub.emit('oxideboy:start');
  }

  runFrame(timestamp) {
    let delta = 16; // Default delta amount - enough to render a frame.
    if (this.lastFrameTimestamp) {
      delta = Math.min(1000, timestamp - this.lastFrameTimestamp) // Don't try and emulate more than a second of time.
    }
    this.lastFrameTimestamp = timestamp;
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
