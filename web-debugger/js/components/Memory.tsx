import GoldenLayout from "golden-layout";
import React from "react";
import { FixedSizeList } from 'react-window';
import MemoryRow from './MemoryRow';

export interface Props {
  glContainer: GoldenLayout.Container;
  glEventHub: GoldenLayout.EventEmitter;
}

export interface State {
  height: number;
}

export default class Memory extends React.Component<Props, State> {
  memory: Uint8Array;

  constructor(props: Props) {
    super(props);

    this.memory = new Uint8Array(0xA0000);

    this.state = {
      height: this.props.glContainer.height,
    };
  }

  componentDidMount() {
    this.props.glContainer.on('resize', this.onResize);
    this.props.glEventHub.on('oxideboy:memory', this.onMemoryUpdate);
  }

  componentWillUnmount() {
    this.props.glContainer.off('resize', this.onResize);
    this.props.glEventHub.off('oxideboy:memory', this.onMemoryUpdate);
  }

  onResize = () => {
    this.setState({height: this.props.glContainer.height});
  }

  onMemoryUpdate = (mem: Uint8Array) => {
    this.memory.set(mem, 0);
    this.forceUpdate();
  }

  render() {
    return (
      <FixedSizeList height={this.state.height} itemCount={4096} itemSize={25} itemData={{memory: this.memory}} width='100%' className='text-monospace'>
        {MemoryRow}
      </FixedSizeList>
    );
  }
}
