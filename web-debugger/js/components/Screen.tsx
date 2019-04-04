import GoldenLayout from "golden-layout";
import React from "react";
import regl from "regl";

export interface ScreenProps {
  glContainer: GoldenLayout.Container;
  glEventHub: GoldenLayout.EventEmitter;
}

interface ScreenState {
  canvasWidth: number;
  canvasHeight: number;
}

export default class Screen extends React.Component<ScreenProps, ScreenState> {
  canvasRef: React.RefObject<HTMLCanvasElement>;
  regl?: regl.Regl = undefined;
  screen?: regl.Texture2D = undefined;
  drawScreen?: regl.DrawCommand<regl.DefaultContext, {}> = undefined;

  constructor(props: ScreenProps) {
    super(props);

    this.canvasRef = React.createRef();
    this.state = this.calculateCanvasDimensions();
  }

  componentDidMount() {
    this.props.glContainer.on('resize', this.onResize);
    this.props.glEventHub.on('oxideboy:frame', this.onFrameUpdate);

    this.regl = regl(this.canvasRef.current!);
    this.screen = this.regl.texture();
    this.drawScreen = this.regl({
      frag: `
      precision mediump float;
      uniform sampler2D texture;
      varying vec2 uv;
      void main () {
        gl_FragColor = texture2D(texture, uv);
      }`,

      vert: `
      precision mediump float;
      attribute vec2 position;
      varying vec2 uv;
      void main () {
        uv = position;
        gl_Position = vec4(2.0 * position.x - 1.0, 1.0 - 2.0 * position.y, 0, 1);
      }`,

      attributes: {
        position: [
          -2, 0,
          0, -2,
          2, 2]
      },
      uniforms: {
        texture: this.screen,
      },

      count: 3,
    });
  }

  componentWillUnmount() {
    this.props.glContainer.off('resize', this.onResize);
    this.props.glEventHub.off('oxideboy:frame', this.onFrameUpdate);
  }

  onResize = () => {
    // Some garbage code I'll keep around for now. What I was trying to do here is figure out if it's possible to clamp
    // the container to multiple of screen dimensions (160x144). Golden-Layout makes it insanely painful.
    // requestAnimationFrame(() => {
    //   // When resizing, we want to keep the Screen container bounded to a multiple of 160x144 so that the pixels don't
    //   // look screwy. We can only do this if we're contained inside a row and column.

    //   // We only do this if we're not maximized, and if our current dimensions aren't already looking good.
    //   let parent = this.props.glContainer.parent;
    //   while (!parent.isRoot) {
    //     if (parent.isMaximised) {
    //       return;
    //     }
    //     parent = parent.parent;
    //   }

    //   if (this.props.glContainer.width % 160 == 0 &&
    //       this.props.glContainer.height % 144 == 0) {
    //     return;
    //   }

    //   this.props.glContainer.setSize(160, 144);
    //   console.log('hmm.', this.props.glContainer);

    //   // We can only bound our container if we're inside a row and a column.
    //   let column = this.props.glContainer.parent;
    //   while (!column.isColumn) {
    //     column = column.parent;
    //     if (column.isRoot) {
    //       return;
    //     }
    //   }
    //   let row = this.props.glContainer.parent;
    //   while (!row.isRow) {
    //     row = row.parent;
    //     if (row.isRoot) {
    //       return;
    //     }
    //   }
    //   console.log('found stuff?', column, row);
    // });
    this.setState(this.calculateCanvasDimensions());
  }

  onFrameUpdate = (framebuffer: Uint16Array) => {
    this.screen!({
      format: "rgb565",
      data: framebuffer,
      width: 160,
      height: 144,
    });
    this.regl.poll();
    this.drawScreen!();
  }

  calculateCanvasDimensions() {
    // TODO: add a button to tab to allow locking the dimensions to fixed multiples 160x144.
    const lockDimensions = false;

    let newWidth = this.props.glContainer.width;
    if (lockDimensions) {
      newWidth -= newWidth % 160;
    }
    let newHeight = newWidth * (144/160);

    if (newHeight > this.props.glContainer.height) {
      newHeight = this.props.glContainer.height;
      if (lockDimensions) {
        newHeight -= newHeight % 144;
      }
      newWidth = newHeight * (160/144);
    }
    const currentCanvas = this.canvasRef.current;
    if (currentCanvas) {
      currentCanvas.width = newWidth;
      currentCanvas.height = newHeight;
    }
    return {
      canvasWidth: newWidth,
      canvasHeight: newHeight,
    };
  }

  render() {
    return (
      <div className='d-flex justify-content-center align-items-center h-100'>
        <canvas ref={this.canvasRef}
                width={this.state.canvasWidth}
                height={this.state.canvasHeight} />
      </div>
    );
  }
}
