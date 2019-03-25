import GoldenLayout from "golden-layout";
import React from "react";
import { toPaddedHexString } from "../util";

export interface Props {
  glContainer: GoldenLayout.Container;
  glEventHub: GoldenLayout.EventEmitter;
}

export interface State {
  instructions: [{loc: number, txt: string}];
}

export default class Disassembler extends React.Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {instructions: []};
  }

  componentDidMount() {
    this.props.glEventHub.on('oxideboy:instructions', this.onUpdate);
  }

  componentWillUnmount() {
    this.props.glEventHub.off('oxideboy:instructions', this.onUpdate);
  }

  onUpdate = (instructions) => {
    this.setState({instructions});
  }

  render() {
    return (
      <div className='text-monospace'>
        { this.state.instructions.map((inst) => (
            <div key={`inst_${inst.loc}`}>0x{toPaddedHexString(inst.loc, 4)}: {inst.txt}</div>
        ))}
      </div>
    );
  }
}
