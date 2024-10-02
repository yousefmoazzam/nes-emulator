static PROGRAM_ROM_START_ADDR: u16 = 0xFFFC;

enum AddressingMode {
    Immediate,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Indirect,
    IndirectX,
    IndirectY,
}

pub struct CPU<'a> {
    status: u8,
    register_a: u8,
    register_x: u8,
    register_y: u8,
    stack_register: u8,
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
            stack_register: 0x00,
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
        self.register_y = 0;
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
                0xC6 => {
                    self.dec(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0x8D => {
                    self.sta(&AddressingMode::Absolute);
                    self.program_counter += 2;
                }
                0x96 => {
                    self.stx(&AddressingMode::ZeroPageY);
                    self.program_counter += 1;
                }
                0x8C => {
                    self.sty(&AddressingMode::Absolute);
                    self.program_counter += 2;
                }
                0x9A => self.txs(),
                0x48 => self.pha(),
                0xA6 => {
                    self.ldx(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0xA2 => {
                    self.ldx(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xA0 => {
                    self.ldy(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xB4 => {
                    self.ldy(&AddressingMode::ZeroPageX);
                    self.program_counter += 1;
                }
                0x3D => {
                    self.and(&AddressingMode::AbsoluteX);
                    self.program_counter += 2;
                }
                0x51 => {
                    self.eor(&AddressingMode::IndirectY);
                    self.program_counter += 1;
                }
                0x19 => {
                    self.ora(&AddressingMode::AbsoluteY);
                    self.program_counter += 2;
                }
                0xC1 => {
                    self.cmp(&AddressingMode::IndirectX);
                    self.program_counter += 1;
                }
                0x6C => {
                    self.jmp(&AddressingMode::Indirect);
                    // Do not increment for the two bytes given to the instruction. Also do not
                    // increment to move past the instruction, so the increment done in the loop
                    // outside the `match` must be undone. Hence, decrement by 1.
                    self.program_counter -= 1;
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
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),
            AddressingMode::AbsoluteX => {
                self.mem_read_u16(self.program_counter) + self.register_x as u16
            }
            AddressingMode::AbsoluteY => {
                self.mem_read_u16(self.program_counter) + self.register_y as u16
            }
            AddressingMode::ZeroPage => self.mem_read(self.program_counter) as u16,
            AddressingMode::ZeroPageX => {
                let pos = self.mem_read(self.program_counter) as u16;
                pos + (self.register_x as u16)
            }
            AddressingMode::ZeroPageY => {
                let pos = self.mem_read(self.program_counter) as u16;
                pos + (self.register_y as u16)
            }
            AddressingMode::Indirect => self.mem_read_u16(self.program_counter),
            AddressingMode::IndirectX => {
                let pos = self.mem_read(self.program_counter);
                let pos_offsetted = pos.wrapping_add(self.register_x);
                self.mem_read_u16(pos_offsetted as u16)
            }
            AddressingMode::IndirectY => {
                let pos = self.mem_read(self.program_counter);
                let lo = self.mem_read(pos as u16);
                let hi = self.mem_read((pos + 1) as u16);
                u16::from_le_bytes([lo + self.register_y, hi])
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

    /// `DEC` instruction
    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        // Cast value represented by bits stored in `addr` to `i8`, to be able to decrement the
        // signed integer representation using rust's `-=` operator, rather than doing the bit
        // fiddling ourselves
        let mut signed_int = self.mem_read(addr);
        signed_int -= 1;
        self.mem_write(addr, signed_int as u8);
        self.update_negative_and_zero_flags(signed_int);
    }

    /// `STA` instruction
    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_a);
    }

    /// `STX` instruction
    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_x);
    }

    /// `STY` instruction
    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.mem_write(addr, self.register_y);
    }

    /// `TXS` instruction
    fn txs(&mut self) {
        self.stack_register = self.register_x;
    }

    /// `PHA` instruction
    fn pha(&mut self) {
        self.stack_register = self.register_a;
    }

    /// `AND` instruction
    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.register_a &= value;
        self.update_negative_and_zero_flags(self.register_a);
    }

    /// `EOR` instruction
    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let res = self.register_a ^ value;
        self.update_negative_and_zero_flags(res);
    }

    /// `ORA` instruction
    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let res = self.register_a | value;
        self.update_negative_and_zero_flags(res);
    }

    /// `CMP` instruction
    fn cmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let res = self.register_a as i8 - value as i8;

        // Update carry flag
        if res >= 0 {
            self.status |= 0b0000_0001;
        }

        self.update_negative_and_zero_flags(res as u8);
    }

    /// `JMP` instruction
    fn jmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.program_counter = addr;
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
        let stored_16bit_addr = u16::from_le_bytes([addr_lo, addr_hi]);
        assert_eq!(ram[stored_16bit_addr as usize], value);
    }

    #[test]
    fn stx_zero_page_y_addressing_stores_correct_value() {
        let mut ram = [0x00; 0xFFFF];
        let zero_page_addr = 0x10;
        let offset = 0x05;
        let value = 0x23;

        // Write `value` into `zero_page_addr + offset` in the `ram` array
        ram[(zero_page_addr + offset) as usize] = value;

        let mut cpu = CPU::new(&mut ram);
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let stx_zero_page_y_addr_mode_opcode = 0x96;

        // Program does the following:
        // - store offset in register Y
        // - load value into register X
        // - store contents of register X in `zero_page_addr` offset by the value in register Y
        // - break
        let program = vec![
            ldy_immediate_addr_mode_opcode,
            offset,
            ldx_immediate_addr_mode_opcode,
            value,
            stx_zero_page_y_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];

        cpu.load_and_run(program);
        let stored_value = ram[(zero_page_addr + offset) as usize];
        assert_eq!(stored_value, value);
    }

    #[test]
    fn sty_absolute_addressing_stores_correct_value() {
        let mut ram = [0x00; 0xFFFF];
        let lo = 0x00;
        let hi = 0x10;
        let value = 0x23;

        let mut cpu = CPU::new(&mut ram);
        let ldy_immediate_addr_opcode = 0xA0;
        let sty_absolute_addr_mode_opcode = 0x8C;

        // Program does the following:
        // - load value into register Y
        // - store contents of register Y in 16-bit address denoted by `lo` and `hi`
        // - break
        let program = vec![
            ldy_immediate_addr_opcode,
            value,
            sty_absolute_addr_mode_opcode,
            lo,
            hi,
            0x00,
        ];
        cpu.load_and_run(program);
        let stored_value = ram[u16::from_le_bytes([lo, hi]) as usize];
        assert_eq!(stored_value, value);
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

    #[test]
    fn and_absolute_x_addressing_sets_correct_value() {
        let mut ram = [0x00; 0xFFFF];
        let lo = 0x00;
        let hi = 0x10;
        let offset = 0x05;
        let register_a_value = 0b01100011;
        let memory_value = 0b10101110;

        // Write value to 16-bit addr + offset
        let addr_16bit = u16::from_le_bytes([lo, hi]) + offset as u16;
        ram[addr_16bit as usize] = memory_value;

        let mut cpu = CPU::new(&mut ram);
        let lda_immediate_addr_mode_opcode = 0xA9;
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let and_abs_x_addr_mode_opcode = 0x3D;

        // Program does the following:
        // - load value into register A
        // - load offset into register X
        // - perform bitwise AND between bits in register A and value in memory, and store result
        // in register A
        // - break
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            ldx_immediate_addr_mode_opcode,
            offset,
            and_abs_x_addr_mode_opcode,
            lo,
            hi,
            0x00,
        ];
        cpu.load_and_run(program);
        assert_eq!(cpu.register_a, register_a_value & memory_value);
    }

    #[test]
    fn cmp_indirect_x_addressing_sets_carry_flag() {
        let mut ram = [0x00; 0xFFFF];
        let zero_page_addr = 0x10;
        let zero_page_addr_offset = 0x05;
        let lo = 0x15;
        let hi = 0x20;
        let register_a_value = 0b00001010; // 10 in two's complement representation
        let memory_value = 0b00001000; // 8 in two's complement representation

        // Write target address value's least and most significant bytes starting at
        // `zero_page_addr + lo_offset`
        ram[(zero_page_addr + zero_page_addr_offset) as usize] = lo;
        ram[(zero_page_addr + zero_page_addr_offset + 1) as usize] = hi;

        // Write value to target address
        ram[u16::from_le_bytes([lo, hi]) as usize] = memory_value;

        let mut cpu = CPU::new(&mut ram);
        let lda_immediate_addr_mode_opcode = 0xA9;
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let cmp_indirect_x_addr_mode_opcode = 0xC1;

        // Program does the following:
        // - load value into register A
        // - load offset into register X
        // - compare value in register A to value in memory
        // - break
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            ldx_immediate_addr_mode_opcode,
            zero_page_addr_offset,
            cmp_indirect_x_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        cpu.load_and_run(program);
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn eor_indirect_y_addressing_sets_zero_flag() {
        let mut ram = [0x00; 0xFFFF];
        let zero_page_addr = 0x10;
        let lo = 0x15;
        let hi = 0x20;
        let lo_offset = 0x05;
        let memory_value = 0b0101_0101;
        let register_a_value = 0b0101_0101;

        // Write value of least significant byte of target address (without offset) to zero page
        // addr
        ram[zero_page_addr as usize] = lo;

        // Write value of most significant byte of target address to address right after zero page
        // addr
        ram[(zero_page_addr + 1) as usize] = hi;

        // Write value to memory in two consecutive addresses (least significant byte given by `lo
        // + offset`, most significant byte given by `hi`)
        ram[u16::from_le_bytes([lo + lo_offset, hi]) as usize] = memory_value;

        let mut cpu = CPU::new(&mut ram);
        let lda_immediate_addr_mode_opcode = 0xA9;
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let eor_indirect_y_addr_mode_opcode = 0x51;

        // Program does the following:
        // - load value into register A
        // - load least significant byte offset into register Y
        // - perform XOR between value in register A and memory value
        // - break
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            ldy_immediate_addr_mode_opcode,
            lo_offset,
            eor_indirect_y_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        cpu.load_and_run(program);
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn ora_absolute_y_addressing_mode_sets_negative_flag() {
        let mut ram = [0x00; 0xFFFF];
        let lo = 0x1F;
        let hi = 0x25;
        let offset = 0x10;
        let register_a_value = 0b1001_1001;
        let memory_value = 0b0110_0000;

        // Write value to memory address given by `offset` applied to the 16-bit address described
        // by `lo` and `hi`
        ram[(u16::from_le_bytes([lo, hi]) + offset as u16) as usize] = memory_value;

        let mut cpu = CPU::new(&mut ram);
        let lda_immediate_addressing_opcode = 0xA9;
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let ora_absolute_y_addr_mode_opcode = 0x19;

        // Program does the following:
        // - load value into register A
        // - load offset into register Y
        // - perform OR between value in register A and memory value
        // - break
        let program = vec![
            lda_immediate_addressing_opcode,
            register_a_value,
            ldy_immediate_addr_mode_opcode,
            offset,
            ora_absolute_y_addr_mode_opcode,
            lo,
            hi,
            0x00,
        ];
        cpu.load_and_run(program);
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn jmp_indirect_addressing_updates_program_counter_correctly() {
        let mut ram = [0x00; 0xFFFF];
        let lo = 0x1F;
        let hi = 0x25;
        let mut cpu = CPU::new(&mut ram);
        let jmp_indirect_addr_mode_opcode = 0x6C;

        // Program does the following:
        // - jump to 16-bit address given by `lo` and `hi`
        // - break
        let program = vec![jmp_indirect_addr_mode_opcode, lo, hi, 0x00];
        cpu.load_and_run(program);
        assert_eq!(cpu.program_counter, u16::from_le_bytes([lo, hi]));
    }

    #[test]
    fn dec_zero_page_addressing_mode_modifies_value_correctly() {
        let mut ram = [0x00; 0xFFFF];
        let zero_page_addr = 0x25;
        let value = 0x05;
        ram[zero_page_addr as usize] = value;

        let mut cpu = CPU::new(&mut ram);
        let dec_zero_page_addr_mode_opcode = 0xC6;

        // Program does the following:
        // - decrement value in memory address
        // - break
        let program = vec![dec_zero_page_addr_mode_opcode, zero_page_addr];
        cpu.load_and_run(program);
        assert_eq!((value as i8) - 1, ram[zero_page_addr as usize] as i8);
    }

    #[test]
    fn txs_transfers_register_x_value_to_stack_register() {
        let mut ram = [0x00; 0xFFFF];
        let register_x_value = 0x04;
        let mut cpu = CPU::new(&mut ram);
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let txs_opcode = 0x9A;

        // Program does the following:
        // - load value in register X
        // - transfer value from register X to stack register
        // - break
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            txs_opcode,
            0x00,
        ];
        cpu.load_and_run(program);
        assert_eq!(cpu.stack_register, register_x_value);
    }

    #[test]
    fn pha_transfers_register_a_value_to_stack_register() {
        let mut ram = [0x00; 0xFFFF];
        let register_a_value = 0x06;
        let mut cpu = CPU::new(&mut ram);
        let lda_immediate_addressing_opcode = 0xA9;
        let pha_opcode = 0x048;

        // Program does the following:
        // - load value into register A
        // - put copy of value in register A in stack register
        // - break
        let program = vec![
            lda_immediate_addressing_opcode,
            register_a_value,
            pha_opcode,
        ];
        cpu.load_and_run(program);
        assert_eq!(cpu.stack_register, register_a_value);
    }
}
