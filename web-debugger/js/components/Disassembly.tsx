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
    this.activeSegments = state.codeSegments;

    // Check if there's any segments we're missing.
    const missingSegments = this.activeSegments.filter(segment_id => !this.segments.has(segment_id));

    if (missingSegments.length) {
      this.props.glEventHub.emit('oxideboy:request-segments', missingSegments);
      return;
    }

    // We already have all live segments, so we can render now.
    // this.setState({activeSegments: this.activeSegments});
  }

  onSegments = (segments: Segment[]) => {
    for (const segment of segments) {
      this.segments.set(segment.id, segment);
    }
    // this.setState({activeSegments: this.activeSegments});
  }

  render() {
    const segments = new Array(...this.segments.values());
    segments.sort((a, b) => a.addr - b.addr);

    return (
      <div className='text-monospace'>
        { segments.map(segment => (
          <div key={segment.id} style={{display: this.state.activeSegments.includes(segment.id) ? 'block' : 'none'}}>
            <DisassemblySegment segment={segment} />
          </div>
        ))}
      </div>
    );
  }
}

export class DisassemblySegment extends React.PureComponent<{}, {}> {
  render() {
    return (
      <div>
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
