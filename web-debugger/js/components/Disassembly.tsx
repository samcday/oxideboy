import GoldenLayout from "golden-layout";
import React from "react";
import { toPaddedHexString } from "../util";

import {Disassembler} from '../../crate-ui/pkg';

export interface Props {
  glContainer: GoldenLayout.Container;
  glEventHub: GoldenLayout.EventEmitter;
}

export interface State {
  instructions: [{loc: number, txt: string}];
}

export default class Disassembly extends React.Component<Props, State> {
  disassembler: Disassembler;

  constructor(props: Props) {
    super(props);
    this.disassembler = Disassembler.new();
    this.state = {instructions: []};
  }

  componentDidMount() {
    this.props.glEventHub.on('oxideboy:state', this.onState);
    this.props.glContainer.parent.element[0].querySelector('.lm_content').style['overflow-y'] = 'scroll';
  }

  componentWillUnmount() {
    this.props.glEventHub.off('oxideboy:state', this.onState);
  }

  onState = (state) => {
    if (!this.foo) {
      // this.foo = true;
      // console.time('foo');
      this.disassembler.disassemble(state.cpu.pc, state.memory);
      // console.timeEnd('foo');
      this.forceUpdate();
    }
    // this.setState({instructions});
  }

  render() {
    return (
        //<div key={`inst_${inst.loc}`}>0x{toPaddedHexString(inst.loc, 4)}: {inst.txt}</div>
      <div className='text-monospace'>
        { this.disassembler.blocks().map((block, idx) => (
          <DisassemblyBlock block={idx} disassembler={this.disassembler} addr={block.addr} hash={block.hash} key={block.hash} />
        ))}
      </div>
    );
  }
}

export class DisassemblyBlock extends React.PureComponent<{}, {}> {
  render() {
    const instructions = this.props.disassembler.block(this.props.block);
    return (
      <div>
        { instructions.map(inst => (
          <div key={inst.addr}>
            <div className='memory-address bg-light pl-1 pr-2 mr-2'>0x{toPaddedHexString(inst.addr, 4)}</div>
            {inst.txt}
          </div>
        ))}
      </div>
    );
  }
}
