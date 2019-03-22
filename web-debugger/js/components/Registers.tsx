import React from "react";

import RegisterCPU from "./RegisterCPU";
import RegisterMem from "./RegisterMem";

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

export interface Props {
  glContainer: GoldenLayout.Container;
  glEventHub: GoldenLayout.EventEmitter;
}

export interface State {
  cpuState: any;
}

export default class Registers extends React.Component<Props, State> {
  constructor(props: Props) {
    super(props);

    this.state = {
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
    };
  }

  componentDidMount() {
    this.props.glEventHub.on('oxideboy:cpu-state', this.updateCpuState);
  }

  updateCpuRegister(reg, newVal) {
    let cpuState = this.emulator.cpu_state();
    cpuState[reg] = parseInt(newVal, 16);
    this.emulator.set_cpu_state(cpuState);
    this.update();
  }

  toggleCpuFlag(flag) {
    let cpuState = this.emulator.cpu_state();
    cpuState[flag] = !cpuState[flag];
    this.emulator.set_cpu_state(cpuState);
    this.update();
  }

  readMemory() {
    return 255;
  }

  writeMemory() {
    
  }

  updateCpuState = (state) => {
    this.setState({cpuState: state});
  }

  render() {
    return (
      <div className="d-flex px-3 flex-wrap text-monospace registers">
        {
          CPU_REGISTERS.map((reg) =>
            <RegisterCPU
              name={reg}
              key={reg}
              value={this.state.cpuState[reg]}
              enabled={this.state.paused}
              onUpdate={this.updateCpuRegister.bind(this, reg)} />
          )
        }
        <div className="register">
          <label htmlFor={`cpu_register_ime`}>IME:</label>
          <input type="checkbox"
                 id="cpu_register_ime"
                 readOnly={!this.state.paused}
                 onChange={this.toggleCpuFlag.bind(this, 'ime')}
                 checked={this.state.cpuState.ime} />
        </div>
        <div className="register">
          <label htmlFor={`cpu_register_imd`}>IMD:</label>
          <input type="checkbox"
                 id="cpu_register_imd"
                 readOnly={!this.state.paused}
                 onChange={this.toggleCpuFlag.bind(this, 'ime_defer')}
                 checked={this.state.cpuState.ime_defer} />
        </div>
        <div className="register">
          <label htmlFor={`cpu_register_halt`}>HALT:</label>
          <input type="checkbox"
                 id="cpu_register_halt"
                 readOnly={!this.state.paused}
                 onChange={this.toggleCpuFlag.bind(this, 'halted')}
                 checked={this.state.cpuState.halted} />
        </div>
        {
          Object.keys(MEM_REGISTERS).map((reg) =>
            <RegisterMem
              name={reg}
              addr={MEM_REGISTERS[reg]}
              fn={this.readMemory.bind(this)}
              enabled={this.state.paused}
              dirty={this.state.memDirty}
              key={reg}
              onUpdate={this.writeMemory.bind(this, MEM_REGISTERS[reg])} />
          )
        }
      </div>
    );
  }
}
