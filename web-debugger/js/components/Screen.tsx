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
    let newWidth = this.props.glContainer.width;
    newWidth -= newWidth % 160;
    let newHeight = newWidth * (144/160);

    if (newHeight > this.props.glContainer.height) {
      newHeight = this.props.glContainer.height;
      newHeight -= newHeight % 144;
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
