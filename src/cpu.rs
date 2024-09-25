static PROGRAM_ROM_START_ADDR: u16 = 0xFFFC;

enum AddressingMode {
    Immediate,
    Absolute,
    ZeroPage,
    ZeroPageX,
}

pub struct CPU<'a> {
    status: u8,
    register_a: u8,
    register_x: u8,
    register_y: u8,
    program_counter: u16,
    memory: &'a mut [u8],
}

impl<'a> CPU<'a> {
    pub fn new(ram: &'a mut [u8]) -> Self {
        CPU {
            status: 0b0010_0000, // bit 5 is always set to 1
            register_a: 0x00,
            register_x: 0x00,
            register_y: 0x00,
            program_counter: 0x00,
            memory: ram,
        }
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    /// Read `u16` value stored in little-endian format, from two contiguous memory addresses each
    /// storing a single `u8` value
    fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos);
        let hi = self.mem_read(pos + 1);
        u16::from_le_bytes([lo, hi])
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    /// Write `u16` value to two contiguous memory addresses, in little-endian format
    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let bytes = u16::to_le_bytes(data);
        self.mem_write(pos, bytes[0]);
        self.mem_write(pos + 1, bytes[1]);
    }

    fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.status = 0b0010_0000;
        self.program_counter = self.mem_read_u16(PROGRAM_ROM_START_ADDR);
    }

    fn load(&mut self, program: Vec<u8>) {
        let program_rom_start: u16 = 0x8000;
        self.memory[program_rom_start as usize..(program_rom_start as usize + program.len())]
            .copy_from_slice(&program[..]);
        self.mem_write_u16(PROGRAM_ROM_START_ADDR, program_rom_start);
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
                    self.lda(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xAA => self.tax(),
                0xE8 => self.inx(),
                0x8D => {
                    self.sta(&AddressingMode::Absolute);
                    self.program_counter += 2;
                }
                0xA6 => {
                    self.ldx(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0xA2 => {
                    self.ldx(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xB4 => {
                    self.ldy(&AddressingMode::ZeroPageX);
                    self.program_counter += 1;
                }
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

    fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,
            AddressingMode::Absolute => {
                let lo = self.mem_read(self.program_counter);
                let hi = self.mem_read(self.program_counter + 1);
                u16::from_le_bytes([lo, hi])
            }
            AddressingMode::ZeroPage => self.mem_read(self.program_counter) as u16,
            AddressingMode::ZeroPageX => {
                let pos = self.mem_read(self.program_counter) as u16;
                pos + (self.register_x as u16)
            }
        }
    }

    /// `LDA` instruction
    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_a = value;
        self.update_negative_and_zero_flags(value);
    }

    /// `LDX` instruction
    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_x = value;
        self.update_negative_and_zero_flags(value);
    }

    /// `LDY` instruction
    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_y = value;
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

    /// `STA` instruction
    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brk_flag_set() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let program = vec![0x00];
        cpu.load_and_run(program);
        let is_brk_flag_set = cpu.status & 0b001_0000 == 0b0001_0000;
        assert_eq!(is_brk_flag_set, true);
    }

    #[test]
    fn lda_immediate_addressing() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let lda_immediate_addressing_opcode = 0xA9;
        let value_to_load = 0x05 as u8;
        let program = vec![lda_immediate_addressing_opcode, value_to_load, 0x00];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_a, value_to_load);
    }

    #[test]
    fn tax_sets_correct_value() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x05;
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_x, value_to_load);
    }

    #[test]
    fn tax_set_zero_flag() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let tax_opcode = 0xAA;

        // Program does the following:
        // - load the value representing 0 that is in register X into register A (zero flag should
        // be set)
        // - break
        let program = vec![tax_opcode, 0x00];
        cpu.load_and_run(program);
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn tax_clear_zero_flag() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x04;

        // Program does the following:
        // - load the value representing 4 into register A (zero flag should be cleared)
        // - transfer 4 stored in register A to register X (zero flag should stay cleared)
        // - break
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        cpu.load_and_run(program);
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, false);
    }

    #[test]
    fn tax_set_negative_flag() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let negative_value = 0b1000_0000; // -128 in two's complement representation

        // Program does the following:
        // - load the value representing -1 into register A (negative flag should be set)
        // - transfer -1 stored in register A to register X (negative flag should stay set)
        // - break
        let program = vec![lda_opcode, negative_value, tax_opcode, 0x00];
        cpu.load_and_run(program);
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn tax_clear_negative_flag() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x04;

        // Program does the following:
        // - load the value representing 4 into register A (negative flag should be cleared)
        // - transfer 4 stored in register A to register X (negative flag should stay cleared)
        // - break
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        cpu.load_and_run(program);
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, false);
    }

    #[test]
    fn inx_increments_value_correctly() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let inx_opcode = 0xE8;
        let program = vec![inx_opcode, 0x00];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_x, 0x01);
    }

    #[test]
    fn sta_absolute_addressing_stores_correct_value() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let mut cpu = CPU::new(&mut ram);
        let lda_opcode = 0xA9;
        let sta_abs_addr_mode_opcode = 0x8D;
        let addr_lo = 0x00;
        let addr_hi = 0x10;
        let value = 0x23;

        // Program does the following:
        // - load value into register A / accumulator
        // - store value of register A in 16-bit address given by `addr_lo` and `addr_hi`
        // - break
        let program = vec![
            lda_opcode,
            value,
            sta_abs_addr_mode_opcode,
            addr_lo,
            addr_hi,
            0x00,
        ];
        cpu.load_and_run(program);
        assert_eq!(cpu.mem_read(u16::from_le_bytes([addr_lo, addr_hi])), value);
    }

    #[test]
    fn ldx_zero_page_addressing_loads_correct_value() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let zero_page_addr = 0x10;
        let value = 0x23;

        // Write `value` into `zero_page_addr` in the `ram` array
        ram[zero_page_addr as usize] = value;

        let mut cpu = CPU::new(&mut ram);
        let ldx_zero_page_addr_mode_opcode = 0xA6;

        // Program does the following:
        // - load contents of `zero_page_addr` into register X
        // - break
        let program = vec![ldx_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_x, value);
    }

    #[test]
    fn ldy_zero_page_x_addressing_loads_correct_value() {
        let mut ram: [u8; 0xFFFF] = [0x00; 0xFFFF];
        let zero_page_addr = 0x10;
        let offset = 0x05;
        let value = 0x23;

        // Write `value` into `zero_page_addr + offset` in the `ram` array
        ram[(zero_page_addr + offset) as usize] = value;

        let mut cpu = CPU::new(&mut ram);
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let ldy_zero_page_x_addr_mode_opcode = 0xB4;

        // Program does the following:
        // - store offset in register X
        // - load contents of `zero_page_addr + offset` into register Y
        // - break
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            offset,
            ldy_zero_page_x_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_y, value);
    }
}
