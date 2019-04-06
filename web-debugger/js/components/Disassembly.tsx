import GoldenLayout from "golden-layout";
import React from "react";
import { toPaddedHexString } from "../util";

export interface Props {
  glContainer: GoldenLayout.Container;
  glEventHub: GoldenLayout.EventEmitter;
}

export interface State {
  active: boolean;
  paused: boolean;
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
    this.props.glEventHub.on('oxideboy:rom-loaded', this.onRomLoaded);
    this.props.glEventHub.on('oxideboy:running', this.onRunning);
    this.props.glEventHub.on('oxideboy:paused', this.onPaused);
    this.props.glContainer.parent.element[0].querySelector('.lm_content').style['overflow-y'] = 'scroll';
  }

  componentWillUnmount() {
    this.props.glEventHub.off('oxideboy:state', this.onState);
    this.props.glEventHub.off('oxideboy:rom-loaded', this.onRomLoaded);
    this.props.glEventHub.off('oxideboy:running', this.onRunning);
    this.props.glEventHub.off('oxideboy:paused', this.onPaused);
  }

  onRomLoaded = () => {
    this.setState({active: true});
  }

  onRunning = () => {
    this.setState({paused: false});
  }

  onPaused = () => {
    this.setState({paused: true});
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
    this.setState({activeSegments: this.activeSegments});
  }

  onSegments = (segments: Segment[]) => {
    for (const segment of segments) {
      this.segments.set(segment.id, segment);
    }
    this.setState({activeSegments: this.activeSegments});
  }

  request(action) {
    this.props.glEventHub.emit('oxideboy:request', action);
  }

  render() {
    const segments = new Array(...this.segments.values());
    segments.sort((a, b) => a.addr - b.addr);

    return (
      <div>
        <div className='btn-group' role='group'>
          { this.state.paused &&
            <button type='button' className='btn btn-outline-secondary' id='start' onClick={this.request.bind(this, 'start')} disabled={!this.state.active}>
              <i className='fas fa-play'></i>
            </button>
          }
          { !this.state.paused &&
            <button type='button' className='btn btn-outline-secondary' id='pause' onClick={this.request.bind(this, 'pause')} disabled={!this.state.active}>
              <i className='fas fa-pause'></i>
            </button>
          }
          <button type='button' className='btn btn-outline-secondary' id='step_frame_backward' onClick={this.request.bind(this, 'frame-back')} disabled={!this.state.active || !this.state.paused}>
            <i className='fas fa-fast-backward'></i>
          </button>
          <button type='button' className='btn btn-outline-secondary' id='step_backward' onClick={this.request.bind(this, 'back')} disabled={!this.state.active || !this.state.paused}>
            <i className='fas fa-backward'></i>
          </button>
          <button type='button' className='btn btn-outline-secondary' id='step_forward' onClick={this.request.bind(this, 'forward')} disabled={!this.state.active || !this.state.paused}>
            <i className='fas fa-forward'></i>
          </button>
          <button type='button' className='btn btn-outline-secondary' id='step_frame_forward' onClick={this.request.bind(this, 'forward-frame')} disabled={!this.state.active || !this.state.paused}>
            <i className='fas fa-fast-forward'></i>
          </button>
          <button type='button' className='btn btn-outline-secondary' id='restart' onClick={this.request.bind(this, 'restart')} disabled={!this.state.active}>
            <i className='fas fa-redo'></i>
          </button>
        </div>

        { this.state.paused && <div className='text-monospace'>
          { segments.map(segment => (
            <div key={segment.id} style={{display: this.state.activeSegments.includes(segment.id) ? 'block' : 'none'}}>
              <DisassemblySegment segment={segment} />
            </div>
          ))}
        </div> }
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
