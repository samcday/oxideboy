extern crate lr35902;

pub mod cartridges;

pub struct Gameboy {
    cpu: lr35902::CPU
}

impl Gameboy {
    /// Creates a new Gameboy instance with the given ROM data.
    pub fn new(rom: &[u8]) -> Result<Gameboy, String> {
        let cart = cartridges::create(rom)?;
        Ok(Gameboy{cpu: lr35902::CPU::new(cart)})
    }

    /// Runs the Gameboy emulation core until a whole video frame has been generated.
    /// The native Gameboy renders frames at a rate of 59.7Hz, it is up to the caller to rate limit
    /// itself to provide a realistic frame rate.
    pub fn run_frame(&mut self) -> &[u32; lr35902::SCREEN_SIZE] {
        self.cpu.run();
        while !self.cpu.is_vblank() {
            self.cpu.run();
        }
        self.cpu.framebuffer()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn cpu_instructions() {
        let rom = include_bytes!("../test/cpu_instrs.gb");
        let cart = ::cartridges::MBC1Cart::new(rom);
        let mut cpu = ::lr35902::CPU::new(Box::new(cart));

        let mut output = String::new();
        while cpu.pc() != 0x681 {
            cpu.run();
            let ser = cpu.serial_get();
            if ser.is_some() {
                output.push(ser.unwrap() as char);
            }
        }

        assert_eq!(output, "cpu_instrs\n\n01:ok  02:ok  03:ok  04:ok  05:ok  06:ok  07:ok  08:ok  09:ok  10:ok  11:ok  ");
    }

    #[test]
    fn instruction_timing() {
        let rom = include_bytes!("../test/instr_timing.gb");
        let cart = ::cartridges::MBC1Cart::new(rom);
        let mut cpu = ::lr35902::CPU::new(Box::new(cart));

        let mut output = String::new();
        while cpu.pc() != 0xC8A6 {
            cpu.run();
            let ser = cpu.serial_get();
            if ser.is_some() {
                output.push(ser.unwrap() as char);
            }
        }

        assert_eq!(output, "instr_timing\n\n\nPassed\n");
    }

    #[test]
    fn mem_timing() {
        let rom = include_bytes!("../test/mem_timing.gb");
        let cart = ::cartridges::MBC1Cart::new(rom);
        let mut cpu = ::lr35902::CPU::new(Box::new(cart));

        let mut output = String::new();
        while cpu.pc() != 0x06A1 {
            cpu.run();
            let ser = cpu.serial_get();
            if ser.is_some() {
                output.push(ser.unwrap() as char);
            }
        }

        assert_eq!(output, "mem_timing\n\n01:ok  02:ok  03:ok  \n\nPassed all tests");
    }
}
