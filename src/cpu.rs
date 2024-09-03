struct CPU {
    status: u8,
    register_a: u8,
    register_x: u8,
    program_counter: u8,
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            status: 0b0010_0000, // bit 5 is always set to 1
            register_a: 0x00,
            register_x: 0x00,
            program_counter: 0x00,
        }
    }

    pub fn interpret(&mut self, program: Vec<u8>) {
        loop {
            let opcode = program[self.program_counter as usize];
            self.program_counter += 1;

            match opcode {
                0x00 => {
                    // Set bit 4 to indicate break flag
                    self.status |= 0b0001_0000;
                    break;
                }
                0xA9 => {
                    self.lda_immediate(program[self.program_counter as usize]);
                    self.program_counter += 1;
                }
                0xAA => self.tax(),
                _ => todo!(),
            }
        }
    }

    /// Immediate addressing mode for `LDA` instruction
    fn lda_immediate(&mut self, value: u8) {
        self.register_a = value;

        // Set zero flag appropriately
        if value == 0 {
            self.status |= 0b0000_0010;
        } else {
            self.status &= 0b1111_1101;
        }

        // set negative flag appropriately
        if value & 0b1000_0000 != 0 {
            self.status |= 0b1000_0000;
        } else {
            self.status &= 0b0111_1111;
        }
    }

    /// `TAX` instruction
    fn tax(&mut self) {
        if self.register_a == 0 {
            self.status |= 0b0000_0010;
        } else {
            self.status &= 0b1111_1101;
        }

        self.register_x = self.register_a;
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

    #[test]
    fn lda_immediate_addressing() {
        let mut cpu = CPU::new();
        let lda_immediate_addressing_opcode = 0xA9;
        let value_to_load = 0x05 as u8;
        let program = vec![lda_immediate_addressing_opcode, value_to_load, 0x00];
        let expected_status = 0b0011_0000; // bit 5 + break flag
        cpu.interpret(program);
        assert_eq!(cpu.register_a, value_to_load);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn lda_immediate_addressing_set_zero_flag() {
        let mut cpu = CPU::new();
        let lda_immediate_addressing_opcode = 0xA9;
        let value_to_load = 0x00 as u8;
        let program = vec![lda_immediate_addressing_opcode, value_to_load, 0x00];
        let expected_status = 0b0011_0010; // bit 5 + break flag + zero flag
        cpu.interpret(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn lda_immediate_addressing_set_negative_flag() {
        let mut cpu = CPU::new();
        let lda_immediate_addressing_opcode = 0xA9;
        let value_to_load = 0b1000_0000; // -128 in two's complement representation
        let program = vec![lda_immediate_addressing_opcode, value_to_load, 0x00];
        let expected_status = 0b1011_0000; // bit 5 + negative flag + break flag
        cpu.interpret(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn tax_sets_correct_value() {
        let mut cpu = CPU::new();
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x05;
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        cpu.interpret(program);
        assert_eq!(cpu.register_x, value_to_load);
    }

    #[test]
    fn tax_set_zero_flag() {
        let mut cpu = CPU::new();
        let tax_opcode = 0xAA;
        let program = vec![tax_opcode, 0x00];
        let expected_status = 0b0011_0010; // bit 5 + break flag + zero flag
        cpu.interpret(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn tax_clear_zero_flag() {
        let mut cpu = CPU::new();
        let tax_opcode = 0xAA;
        let program = vec![tax_opcode, 0x00];
        let expected_status = 0b0011_0000; // bit 5 + break flag
        cpu.register_a = 1; // set register A to a non-zero value
        cpu.status = 0b0010_0010; // bit 5 + zero flag
                                  // Assume that a previous operation has set the zero flag, in
                                  // order to be able to detect if the zero flag is cleared
        cpu.interpret(program);
        assert_eq!(cpu.status, expected_status);
    }
}
