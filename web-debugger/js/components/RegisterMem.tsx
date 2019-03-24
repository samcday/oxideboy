import React from "react";
import { toPaddedHexString } from "../util";

export interface Props {
  name: string;
  addr: number;
  enabled: boolean;
  value: number;
  onUpdate(this: void, value: number): void;
}

interface State {
  editing: boolean;
  value?: string;
}

export default class RegisterMem extends React.Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = {editing: false};
    ['onChange', 'onFocus', 'onBlur', 'onKeyPress'].forEach((name) => this[name] = this[name].bind(this));
  }

  shouldComponentUpdate(nextProps) {
    if (this.state.editing) {
      return true;
    }

    return this.props.value != nextProps.value;
  }

  render() {
    return (
      <div className="register">
        <label htmlFor={`register_${this.props.name}`}>
          <abbr title={`0x${toPaddedHexString(this.props.addr, 4).toUpperCase()}`}>{this.props.name}</abbr>:
        </label>

        <input  type="text"
                readOnly={!this.props.enabled}
                id={`cpu_register_${this.props.name}`}
                size={2}
                value={this.state.editing ? this.state.value : toPaddedHexString(this.props.value, 2)}
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
    this.setState({value: ev.target.value.replace(/[^0-9A-Fa-f]+/g, "").replace(/^0+([1-9a-fA-F].*)/, "$1").substring(0, 2) });
  }
}