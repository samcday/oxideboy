extern crate lr35902;

pub mod cartridges;

#[cfg(test)]
mod tests {
    #[test]
    fn cpu_instructions() {
        let rom = include_bytes!("../test/cpu_instrs.gb");
        let mut cart = ::cartridges::MBC1Cart::new(rom);
        let mut cpu = ::lr35902::CPU::new(&mut cart);

        let mut output = String::new();
        while cpu.pc() != 0x681 {
            cpu.run();
            let ser = cpu.serial_get();
            if ser.is_some() {
                output.push(ser.unwrap() as char);
            }
        }

        assert_eq!(output, "cpu_instrs\n\n01:ok  02:ok  03:ok  04:ok  05:ok  06:ok  07:ok  08:ok  09:ok  10:ok  11:ok  ");
        // println!("orly? {}", output);
    }
}
