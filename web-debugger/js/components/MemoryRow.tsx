import React from "react";
import { toPaddedHexString } from "../util";

export interface Props {
  index: number;
  style: any;
  data: {memory: Uint8Array};
}

export default class MemoryRow extends React.Component<Props, {}> {
  renderedHash: number;

  constructor(props: Props) {
    super(props);
    this.renderedHash = 0;
  }

  shouldComponentUpdate() {
    // We don't need to rerender if the memory contents for this row have not changed.
    return this.calculateMemoryHash() != this.renderedHash;
  }

  calculateMemoryHash(): number {
    let hash = 0;
    const baseAddr = this.props.index * 16;
    for (let i = 0; i < 16; i++) {
      hash = ((hash << 5) - hash) + this.props.data.memory[baseAddr+i];
    }
    return hash;
  }

  render() {
    const address = this.props.index * 16;
    this.renderedHash = this.calculateMemoryHash();

    let values = [];
    for (let i = 0; i < 16; i++) {
      values.push(<div key={`address_${address+i}`} className='memory-cell mx-1'>{toPaddedHexString(this.props.data.memory[address + i], 2)}</div>);
    }

    return (
      <div style={this.props.style}>
        <div className='memory-address bg-light pl-1 pr-2 mr-2'>0x{toPaddedHexString(address, 4)}</div>
        {values}
      </div>
    );
  }
}
