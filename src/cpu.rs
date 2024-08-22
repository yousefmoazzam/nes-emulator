struct CPU {
    status: u8,
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            status: 0b0010_0000,
        }
    }

    pub fn interpret(&mut self, program: Vec<u8>) {
        loop {
            let opcode = program[0];
            match opcode {
                0x00 => {
                    // Set bit 4 to indicate break flag
                    self.status |= 0b0001_0000;
                    break;
                }
                _ => todo!(),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brk_flag_set() {
        let mut cpu = CPU::new();
        let program = vec![0x00];
        cpu.interpret(program);
        let expected_status = 0b0011_0000;
        assert_eq!(expected_status, cpu.status);
    }
}
