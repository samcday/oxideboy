import GoldenLayout from "golden-layout";
import React from "react";
import { toPaddedHexString } from "../util";

export interface Props {
  glContainer: GoldenLayout.Container;
  glEventHub: GoldenLayout.EventEmitter;
}

export interface State {
  activeSegments: string[];
}

interface Segment {
  id: string,
  instructions: {addr: number, txt: string}[];
}

export default class Disassembly extends React.Component<Props, State> {
  segments: Map<String, Segment>;
  activeSegments: string[];

  constructor(props: Props) {
    super(props);

    this.state = {
      activeSegments: [],
    };

    this.activeSegments = [];
    this.segments = new Map();
  }

  componentDidMount() {
    this.props.glEventHub.on('oxideboy:state', this.onState);
    this.props.glEventHub.on('oxideboy:segments', this.onSegments);
    this.props.glContainer.parent.element[0].querySelector('.lm_content').style['overflow-y'] = 'scroll';
  }

  componentWillUnmount() {
    this.props.glEventHub.off('oxideboy:state', this.onState);
  }

  onState = (state) => {
    if (!this.foo) {
      this.foo = true;

      this.activeSegments = state.disassembly;

      // Check if there's any segments we're missing.
      const missingSegments = this.activeSegments.filter(segment_id => !this.segments.has(segment_id));

      if (missingSegments.length) {
        this.props.glEventHub.emit('oxideboy:request-segments', missingSegments);
        return;
      }

      // We already have all live segments, so we can render now.
      this.setState({activeSegments: this.activeSegments});

      // console.time('foo');
      // this.disassembler.disassemble(state.cpu.pc, state.memory);
      // console.timeEnd('foo');
      // this.forceUpdate();
    }
    // this.setState({instructions});
  }

  onSegments = (segments: Segment[]) => {
    for (const segment of segments) {
      this.segments.set(segment.id, segment);
    }
    this.setState({activeSegments: this.activeSegments});
  }

  render() {
    const segments = new Array(...this.segments.values());
    segments.sort((a, b) => a.addr - b.addr);

    return (
      <div className='text-monospace'>
        { segments.map(segment => (
          <DisassemblySegment
            active={this.state.activeSegments.includes(segment.id)}
            segment={segment}
            key={segment.id} />
        ))}
      </div>
    );

    // return (
    // );
  }
}

export class DisassemblySegment extends React.PureComponent<{}, {}> {
  render() {
    console.log('render.', this.props.segment);
    return (
      <div style={{display: this.props.active ? 'block' : 'none'}}>
        { this.props.segment.instructions.map(inst => (
          <div key={inst.addr}>
            <div className='memory-address bg-light pl-1 pr-2 mr-2'>0x{toPaddedHexString(inst.addr, 4)}</div>
            {inst.txt}
          </div>
        ))}
      </div>
    );
  }
}
