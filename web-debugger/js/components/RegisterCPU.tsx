import React from "react";
import { toPaddedHexString } from "../util";

export interface Props {
  name: string;
  enabled: boolean;
  value: number;
  onUpdate(this: void, value: number): void;
}

interface State {
  editing: boolean;
  value?: string;
}

export default class RegisterCPU extends React.Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {editing: false};
    ['onChange', 'onFocus', 'onBlur', 'onKeyPress'].forEach((name) => this[name] = this[name].bind(this));
  }

  render() {
    return (
      <div className="register">
        <label htmlFor={`cpu_register_${this.props.name}`}>
          {this.props.name.toUpperCase()}
        </label>
        <input  type="text"
                readOnly={!this.props.enabled}
                id={`cpu_register_${this.props.name}`}
                size={4}
                value={this.state.editing ? this.state.value : toPaddedHexString(this.props.value, 4)}
                onChange={this.onChange}
                onBlur={this.onBlur}
                onKeyPress={this.onKeyPress}
                onFocus={this.onFocus} />
      </div>
    );
  }

  onFocus(ev: React.FocusEvent<HTMLInputElement>) {
    this.setState({editing: true, value: this.props.value.toString(16)});
  }

  onBlur(ev: React.FocusEvent<HTMLInputElement>) {
    this.setState({editing: false});
  }

  onKeyPress(ev: React.KeyboardEvent<HTMLInputElement>) {
    if (ev.key !== 'Enter') {
      return;
    }
    this.setState({editing: false});
    this.props.onUpdate(parseInt(this.state.value || "0", 16));
    ev.target.blur();
  }

  onChange(ev: React.ChangeEvent<HTMLInputElement>) {
    this.setState({value: ev.target.value.replace(/[^0-9A-Fa-f]+/g, "").replace(/^0+([1-9a-fA-F].*)/, "$1").substring(0, 4) });
  }
}
