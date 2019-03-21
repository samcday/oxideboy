import React from "react";

export default class Registers extends React.Component {
  render() {
    return (
      <div>hi</div>
      // <div className="d-flex px-3 flex-wrap text-monospace registers">
      //   <div className="register">
      //     <label htmlFor={`cpu_register_ime`}>IME:</label>
      //     <input type="checkbox"
      //            id="cpu_register_ime"
      //            readOnly={!this.state.paused}
      //            onChange={this.toggleCpuFlag.bind(this, 'ime')}
      //            checked={this.state.cpuState.ime} />
      //   </div>
      //   <div className="register">
      //     <label htmlFor={`cpu_register_imd`}>IMD:</label>
      //     <input type="checkbox"
      //            id="cpu_register_imd"
      //            readOnly={!this.state.paused}
      //            onChange={this.toggleCpuFlag.bind(this, 'ime_defer')}
      //            checked={this.state.cpuState.ime_defer} />
      //   </div>
      //   <div className="register">
      //     <label htmlFor={`cpu_register_halt`}>HALT:</label>
      //     <input type="checkbox"
      //            id="cpu_register_halt"
      //            readOnly={!this.state.paused}
      //            onChange={this.toggleCpuFlag.bind(this, 'halted')}
      //            checked={this.state.cpuState.halted} />
      //   </div>
      //   {
      //     Object.keys(MEM_REGISTERS).map((reg) =>
      //       <RegisterMem
      //         name={reg}
      //         addr={MEM_REGISTERS[reg]}
      //         fn={this.read_memory.bind(this)}
      //         enabled={this.state.paused}
      //         dirty={this.state.memDirty}
      //         key={reg}
      //         onUpdate={this.writeMem.bind(this, MEM_REGISTERS[reg])} />
      //     )
      //   }
      // </div>
    );
  }
}
