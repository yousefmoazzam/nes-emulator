struct CPU {
    status: u8,
    register_a: u8,
    register_x: u8,
    program_counter: u16,
    memory: [u8; 0xFFFF],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            status: 0b0010_0000, // bit 5 is always set to 1
            register_a: 0x00,
            register_x: 0x00,
            program_counter: 0x00,
            memory: [0x00; 0xFFFF],
        }
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.run();
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    fn load(&mut self, program: Vec<u8>) {
        let program_rom_start: u16 = 0x8000;
        self.memory[program_rom_start as usize..(program_rom_start as usize + program.len())]
            .copy_from_slice(&program[..]);
        self.program_counter = program_rom_start;
    }

    fn run(&mut self) {
        loop {
            let opcode = self.mem_read(self.program_counter);
            self.program_counter += 1;

            match opcode {
                0x00 => {
                    // Set bit 4 to indicate break flag
                    self.status |= 0b0001_0000;
                    break;
                }
                0xA9 => {
                    self.lda_immediate(self.mem_read(self.program_counter));
                    self.program_counter += 1;
                }
                0xAA => self.tax(),
                0xE8 => self.inx(),
                _ => todo!(),
            }
        }
    }

    fn update_negative_and_zero_flags(&mut self, value: u8) {
        // Set zero flag appropriately
        if value == 0 {
            self.status |= 0b0000_0010;
        } else {
            self.status &= 0b1111_1101;
        }

        // Set negative flag appropriately
        if value & 0b1000_0000 != 0 {
            self.status |= 0b1000_0000;
        } else {
            self.status &= 0b0111_1111;
        }
    }

    /// Immediate addressing mode for `LDA` instruction
    fn lda_immediate(&mut self, value: u8) {
        self.register_a = value;
        self.update_negative_and_zero_flags(value);
    }

    /// `TAX` instruction
    fn tax(&mut self) {
        self.update_negative_and_zero_flags(self.register_a);
        self.register_x = self.register_a;
    }

    /// `INX` instruction
    fn inx(&mut self) {
        // Cast value represented by bits stored in `self.register_x` to `i8`, to be able to
        // increment the signed integer representation using rust's `+=` operator, rather than
        // doing the bit fiddling ourselves
        let mut signed_int = self.register_x as i8;
        signed_int += 1;
        self.register_x = signed_int as u8;
        self.update_negative_and_zero_flags(self.register_x);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brk_flag_set() {
        let mut cpu = CPU::new();
        let program = vec![0x00];
        cpu.load_and_run(program);
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
        cpu.load_and_run(program);
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
        cpu.load_and_run(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn lda_immediate_addressing_set_negative_flag() {
        let mut cpu = CPU::new();
        let lda_immediate_addressing_opcode = 0xA9;
        let value_to_load = 0b1000_0000; // -128 in two's complement representation
        let program = vec![lda_immediate_addressing_opcode, value_to_load, 0x00];
        let expected_status = 0b1011_0000; // bit 5 + negative flag + break flag
        cpu.load_and_run(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn tax_sets_correct_value() {
        let mut cpu = CPU::new();
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x05;
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_x, value_to_load);
    }

    #[test]
    fn tax_set_zero_flag() {
        let mut cpu = CPU::new();
        let tax_opcode = 0xAA;
        let program = vec![tax_opcode, 0x00];
        let expected_status = 0b0011_0010; // bit 5 + break flag + zero flag
        cpu.load_and_run(program);
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
        cpu.load_and_run(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn tax_set_negative_flag() {
        let mut cpu = CPU::new();
        let tax_opcode = 0xAA;
        let program = vec![tax_opcode, 0x00];
        let expected_status = 0b1011_0000; // bit 5 + negative flag + break flag
        cpu.register_a = 0b1000_0000; // -128 in two's-complement representation
        cpu.load_and_run(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn tax_clear_negative_flag() {
        let mut cpu = CPU::new();
        let tax_opcode = 0xAA;
        let program = vec![tax_opcode, 0x00];
        let expected_status = 0b0011_0000; // bit 5 + break flag
        cpu.register_a = 1;
        cpu.status = 0b1010_0000; // Assume that a previous operation has set the negative
                                  // flag, in order to be able to detect if the negative
                                  // flag is cleared
        cpu.load_and_run(program);
        assert_eq!(cpu.status, expected_status);
    }

    #[test]
    fn inx_increments_value_correctly() {
        let mut cpu = CPU::new();
        let inx_opcode = 0xE8;
        let program = vec![inx_opcode, 0x00];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_x, 0x01);
    }

    #[test]
    fn inx_sets_zero_flag() {
        let mut cpu = CPU::new();
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let inx_opcode = 0xE8;
        let negative_one = 0b1111_1111; // -1 in two's-complement representation

        // Program does the following:
        // - load the value representing -1 into register A
        // - transfer -1 store in register A to register X
        // - increment the -1 stored in register X to 0 (zero flag should then be set)
        // - break
        let program = vec![lda_opcode, negative_one, tax_opcode, inx_opcode, 0x00];
        cpu.load_and_run(program);
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn inx_clears_zero_flag() {
        let mut cpu = CPU::new();
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let inx_opcode = 0xE8;
        let negative_one = 0b1111_1111; // -1 in two's-complement representation

        // Program does the following:
        // - load the value representing -1 into register A
        // - transfer -1 store in register A to register X
        // - increment the -1 stored in register X to 0 (zero flag should then be set)
        // - increment the 0 stored in register X to 1 (zero flag should then be clear)
        // - break
        let program = vec![
            lda_opcode,
            negative_one,
            tax_opcode,
            inx_opcode,
            inx_opcode,
            0x00,
        ];
        cpu.load_and_run(program);
        let is_zero_flag_set = cpu.status & 0b0000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, false);
    }

    #[test]
    fn inx_sets_negative_flag() {
        let mut cpu = CPU::new();
        let inx_opcode = 0xE8;
        let negative_value = 0b1000_0000; // -128 in two's complement representation

        // It's not possible to have a negative value in register X without the negative flag
        // already being set, which in turn makes it difficult to test that the negative flag gets
        // set.
        //
        // Therefore, instead of using instructions to get a negative value into register X and
        // running the `INX` instruction and checking the negative flag, the register X has been
        // set manually to a negative value. This means that the negative flag will not be set when
        // the `INX` instruction runs, and thus it can be checked if the `inx()` method sets the
        // negative flag as expected or not
        cpu.register_x = negative_value;
        let program = vec![inx_opcode, 0x00];
        cpu.load_and_run(program);
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn inx_clears_negative_flag() {
        let mut cpu = CPU::new();
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let inx_opcode = 0xE8;
        let negative_one = 0b1111_1111; // -1 in two's-complement representation

        // Program does the following:
        // - load the value representing -1 into register A
        // - transfer -1 store in register A to register X (negative flag will be set)
        // - increment the -1 stored in register X to 0 (negative flag should then be cleared)
        // - break
        let program = vec![lda_opcode, negative_one, tax_opcode, inx_opcode, 0x00];
        cpu.load_and_run(program);
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, false);
    }
}
