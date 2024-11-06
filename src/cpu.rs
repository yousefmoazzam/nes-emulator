use crate::bus::Bus;

static PROGRAM_ROM_START_ADDR: u16 = 0xFFFC;
static STACK_REGISTER_HI: u8 = 0x01;
static STACK_REGISTER_LO_START: u8 = 0xFF;

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
    bus: Bus<'a>,
}

impl<'a> CPU<'a> {
    pub fn new(bus: Bus<'a>) -> Self {
        CPU {
            status: 0b0010_0000, // bit 5 is always set to 1
            register_a: 0x00,
            register_x: 0x00,
            register_y: 0x00,
            stack_register: STACK_REGISTER_LO_START,
            program_counter: 0x00,
            bus,
        }
    }

    pub fn load_and_run(&mut self) {
        self.reset();
        self.run();
    }

    fn mem_read(&self, addr: u16) -> u8 {
        self.bus.mem_read(addr)
    }

    fn mem_read_u16(&self, pos: u16) -> u16 {
        self.bus.mem_read_u16(pos)
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.bus.mem_write(addr, data);
    }

    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        self.bus.mem_write_u16(pos, data);
    }

    fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.status = 0b0010_0000;
        self.program_counter = self.mem_read_u16(PROGRAM_ROM_START_ADDR);
    }

    fn run(&mut self) {
        loop {
            let opcode = self.mem_read(self.program_counter);
            self.program_counter += 1;

            match opcode {
                0x00 => {
                    self.brk();
                    break;
                }
                0x40 => self.rti(),
                0xA9 => {
                    self.lda(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xAA => self.tax(),
                0xA8 => self.tay(),
                0xE6 => {
                    self.inc(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0xE8 => self.inx(),
                0xC8 => self.iny(),
                0xC6 => {
                    self.dec(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0xCA => self.dex(),
                0x88 => self.dey(),
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
                0xBA => self.tsx(),
                0x8A => self.txa(),
                0x98 => self.tya(),
                0x48 => self.pha(),
                0x08 => self.php(),
                0x68 => self.pla(),
                0x28 => self.plp(),
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
                0xE0 => {
                    self.cpx(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xC0 => {
                    self.cpy(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0x6C => {
                    self.jmp(&AddressingMode::Indirect);
                    // Do not increment for the two bytes given to the instruction. Also do not
                    // increment to move past the instruction, so the increment done in the loop
                    // outside the `match` must be undone. Hence, decrement by 1.
                    self.program_counter -= 1;
                }
                0x20 => {
                    self.jsr(&AddressingMode::Absolute);
                    // Do not increment the program counter any more than to get to the first
                    // operand.
                }
                0x60 => self.rts(),
                0x24 => {
                    self.bit(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0x2C => {
                    self.bit(&AddressingMode::Absolute);
                    self.program_counter += 2;
                }
                0x90 => self.bcc(),
                0xB0 => self.bcs(),
                0xF0 => self.beq(),
                0x30 => self.bmi(),
                0x10 => self.bpl(),
                0xD0 => self.bne(),
                0x50 => self.bvc(),
                0x70 => self.bvs(),
                0x38 => self.sec(),
                0x18 => self.clc(),
                0xB8 => self.clv(),
                0x06 => {
                    self.asl(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0x26 => {
                    self.rol(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0x46 => {
                    self.lsr(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0x66 => {
                    self.ror(&AddressingMode::ZeroPage);
                    self.program_counter += 1;
                }
                0x69 => {
                    self.adc(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xE9 => {
                    self.sbc(&AddressingMode::Immediate);
                    self.program_counter += 1;
                }
                0xF8 => self.sed(),
                0xD8 => self.cld(),
                0x78 => self.sei(),
                0x58 => self.cli(),
                0xEA => continue,
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

    /// `TAY` instruction
    fn tay(&mut self) {
        self.register_y = self.register_a;
        self.update_negative_and_zero_flags(self.register_y);
    }

    /// `INC` instruction
    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr) as i8;
        let incremented_value = value + 1;
        self.mem_write(addr, incremented_value as u8);
        self.update_negative_and_zero_flags(incremented_value as u8);
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

    /// `INY` instruction
    fn iny(&mut self) {
        let mut signed_int = self.register_y as i8;
        signed_int += 1;
        self.register_y = signed_int as u8;
        self.update_negative_and_zero_flags(self.register_y);
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

    /// `DEX` instruction
    fn dex(&mut self) {
        let mut signed_int = self.register_x as i8;
        signed_int -= 1;
        self.update_negative_and_zero_flags(signed_int as u8);
        self.register_x = signed_int as u8;
    }

    /// `DEY` instruction
    fn dey(&mut self) {
        let mut signed_int = self.register_y as i8;
        signed_int -= 1;
        self.update_negative_and_zero_flags(signed_int as u8);
        self.register_y = signed_int as u8;
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

    /// `TSX` instruction
    fn tsx(&mut self) {
        self.register_x = self.stack_register;
    }

    /// `TXA` instruction
    fn txa(&mut self) {
        self.register_a = self.register_x;
    }

    /// `TYA` instruction
    fn tya(&mut self) {
        self.register_a = self.register_y;
    }

    fn push_onto_stack(&mut self, value: u8) {
        let addr = u16::from_le_bytes([self.stack_register, STACK_REGISTER_HI]);
        self.mem_write(addr, value);
        self.stack_register -= 1;
    }

    fn pull_off_of_stack(&mut self) -> u8 {
        self.stack_register += 1;
        let addr = u16::from_le_bytes([self.stack_register, STACK_REGISTER_HI]);
        return self.mem_read(addr);
    }

    /// `PHA` instruction
    fn pha(&mut self) {
        self.push_onto_stack(self.register_a);
    }

    /// `PHP` instruction
    fn php(&mut self) {
        self.push_onto_stack(self.status);
    }

    /// `PLA` instruction
    fn pla(&mut self) {
        self.register_a = self.pull_off_of_stack();
    }

    /// `PLP` instruction
    fn plp(&mut self) {
        self.status = self.pull_off_of_stack();
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

        // Update carry flag
        if self.register_a >= value {
            self.status |= 0b0000_0001;
        }

        let res_i8 = i8::wrapping_sub(self.register_x as i8, value as i8);
        self.update_negative_and_zero_flags(res_i8 as u8);
    }

    /// `CPX` instruction
    fn cpx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        if self.register_x >= value {
            self.status |= 0b0000_0001;
        }

        let res_i8 = i8::wrapping_sub(self.register_x as i8, value as i8);
        self.update_negative_and_zero_flags(res_i8 as u8);
    }

    /// `CPY` instruction
    fn cpy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        if self.register_y >= value {
            self.status |= 0b0000_0001;
        }

        let res_i8 = i8::wrapping_sub(self.register_x as i8, value as i8);
        self.update_negative_and_zero_flags(res_i8 as u8);
    }

    /// `JMP` instruction
    fn jmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        self.program_counter = addr;
    }

    /// `JSR` instruction
    fn jsr(&mut self, mode: &AddressingMode) {
        let target_addr = self.get_operand_address(mode);
        let first_operand_addr = self.program_counter;
        let return_addr = first_operand_addr + 1;
        let return_addr_bytes = u16::to_le_bytes(return_addr);
        self.push_onto_stack(return_addr_bytes[1]);
        self.push_onto_stack(return_addr_bytes[0]);
        self.program_counter = target_addr;
    }

    /// `RTS` instruction
    fn rts(&mut self) {
        let lo = self.pull_off_of_stack();
        let hi = self.pull_off_of_stack();
        let return_addr = u16::from_le_bytes([lo, hi]);
        // Increment the return address by one, due to the `JSR` instruction pushing onto the stack
        // the address of its second operands, rather than the address of the next opcode after its
        // second operand
        self.program_counter = return_addr + 1;
    }

    /// `BIT` instruction
    fn bit(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let res = self.register_a & value;

        if value & 0b0100_0000 == 0b0100_0000 {
            self.status |= 0b0100_0000;
        } else {
            self.status &= 0b1011_1111;
        }

        if value & 0b1000_0000 != 0 {
            self.status |= 0b1000_0000;
        } else {
            self.status &= 0b0111_1111;
        }

        if res == 0 {
            self.status |= 0b0000_0010;
        } else {
            self.status &= 0b1111_1101;
        }
    }

    /// `BCC` instruction
    fn bcc(&mut self) {
        if self.status & 0b0000_0001 == 0 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `BCS` instruction
    fn bcs(&mut self) {
        if self.status & 0b0000_0001 == 0b0000_0001 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `BEQ` instruction
    fn beq(&mut self) {
        if self.status & 0b0000_0010 == 0b0000_0010 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `BMI` instruction
    fn bmi(&mut self) {
        if self.status & 0b1000_0000 == 0b1000_0000 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `BPL` instruction
    fn bpl(&mut self) {
        if self.status & 0b1000_0000 == 0 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `BNE` instruction
    fn bne(&mut self) {
        if self.status & 0b0000_0010 == 0 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `BVC` instruction
    fn bvc(&mut self) {
        if self.status & 0b0100_0000 == 0 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `BVS` instruction
    fn bvs(&mut self) {
        if self.status & 0b0100_0000 == 0b0100_0000 {
            let offset = self.mem_read(self.program_counter) as i8;
            let offsetted_program_counter = self.program_counter as i32 + offset as i32;
            self.program_counter = offsetted_program_counter as u16;
        }
    }

    /// `SEC` instruction
    fn sec(&mut self) {
        self.status |= 0b0000_0001;
    }

    /// `CLC` instruction
    fn clc(&mut self) {
        self.status &= 0b1111_1110;
    }

    /// `CLV` instruction
    fn clv(&mut self) {
        self.status &= 0b1011_1111;
    }

    /// `ASL` instruction
    fn asl(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let is_bit_seven_orig_set = value & 0b1000_0000 == 0b1000_0000;
        if is_bit_seven_orig_set {
            self.status |= 0b0000_0001;
        } else {
            self.status &= 0b1111_1110;
        }

        let shifted_value = value << 1;
        self.mem_write(addr, shifted_value);

        if shifted_value == 0 {
            self.status |= 0b0000_0010;
        }

        let is_bit_seven_res_set = shifted_value & 0b1000_0000 == 0b1000_0000;
        if is_bit_seven_res_set {
            self.status |= 0b1000_0000;
        }
    }

    /// `ROL` instruction
    fn rol(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let mut shifted_value = value << 1;
        if self.status & 0b0000_0001 == 0b0000_0001 {
            shifted_value |= 0b0000_0001;
        }
        if shifted_value == 0 {
            self.status |= 0b0000_0010;
        }
        self.mem_write(addr, shifted_value);

        let is_bit_seven_orig_set = value & 0b1000_0000 == 0b1000_0000;
        if is_bit_seven_orig_set {
            self.status |= 0b0000_0001;
        } else {
            self.status &= 0b1111_1110;
        }

        let is_bit_seven_res_set = shifted_value & 0b1000_0000 == 0b1000_0000;
        if is_bit_seven_res_set {
            self.status |= 0b1000_0000;
        }
    }

    /// `LSR` instruction
    fn lsr(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);

        let is_bit_zero_set = value & 0b0000_0001 == 0b0000_0001;
        if is_bit_zero_set {
            self.status |= 0b0000_0001;
        } else {
            self.status &= 0b1111_1110;
        }

        let shifted_value = value >> 1;
        self.mem_write(addr, shifted_value);

        if shifted_value == 0 {
            self.status |= 0b0000_0010;
        }

        self.status &= 0b0111_1111;
    }

    /// `ROR` instruction
    fn ror(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        let mut shifted_value = value >> 1;
        if self.status & 0b0000_0001 == 0b0000_0001 {
            shifted_value |= 0b1000_0000;
        }
        self.mem_write(addr, shifted_value);

        if shifted_value == 0 {
            self.status |= 0b0000_0010;
        }

        let is_bit_zero_orig_set = value & 0b0000_0001 == 0b0000_0001;
        if is_bit_zero_orig_set {
            self.status |= 0b0000_0001;
        } else {
            self.status &= 0b1111_1110;
        }

        self.status &= 0b0111_1111;
    }

    /// `ADC` instruction
    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr);
        self.adc_helper(value);
    }

    fn adc_helper(&mut self, value: u8) {
        let is_carry_flag_set = self.status & 0b0000_0001 == 0b0000_0001;
        let carry_bit = if is_carry_flag_set { 1 } else { 0 };
        let register_a_i8 = self.register_a as i8;
        let value_i8 = value as i8;
        let res_i8 = i8::wrapping_add(i8::wrapping_add(register_a_i8, value_i8), carry_bit as i8);

        // As `i8`, check if both main values being added were positive. If they were, check if the
        // result is positive or negative. If negative, then overflow has occurred at the top
        // boundary of the `i8` range, so set the overflow flag.
        if register_a_i8 > 0 && value_i8 > 0 && res_i8 < 0 {
            self.status |= 0b0100_0000;
        }

        // As `i8`, check if both main values being added were negative. If they were, check if the
        // result is positive or negative. If positive, then overflow has occurred at the bottom
        // boundary of the `i8` range, so set the overflow flag.
        if register_a_i8 < 0 && value_i8 < 0 && res_i8 > 0 {
            self.status |= 0b0100_0000;
        }

        // As `u8`, check if the sum of the values overflow the range of `u8`. If so, set carry
        // flag.
        let res_u8 = u8::wrapping_add(u8::wrapping_add(self.register_a, value), carry_bit);
        if res_u8 < self.register_a && res_u8 < value {
            self.status |= 0b0000_0001;
        } else {
            // If the sum of the values don't overflow the range of `u8`, but the carry flag was set to
            // begin with, then it should be cleared
            if self.status & 0b0000_0001 == 0b0000_00001 {
                self.status &= 0b1111_1110;
            }
        }

        self.register_a = res_u8;
        self.update_negative_and_zero_flags(self.register_a);
    }

    /// `SBC` instruction
    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let value = self.mem_read(addr) as i8;
        // Invert bits, but don't add 1 to get the complement; instead, rely on:
        // - the carry flag being set prior to SBC
        // - and the addition of 1 when the carry flag is set in ADC (which is reused here)
        //
        // to do the +1 which "completes" the generation of the complement.
        let inverted = !value;
        self.adc_helper(inverted as u8);
    }

    /// `BRK` instruction
    fn brk(&mut self) {
        self.status |= 0b0001_0000;
    }

    /// `RTI` instruction
    fn rti(&mut self) {
        self.status = self.pull_off_of_stack();
        let lo = self.pull_off_of_stack();
        let hi = self.pull_off_of_stack();
        let addr = u16::from_le_bytes([lo, hi]);
        self.program_counter = addr;
    }

    /// `SED` instruction
    fn sed(&mut self) {
        self.status |= 0b0000_1000;
    }

    /// `CLD` instruction
    fn cld(&mut self) {
        self.status &= 0b1111_0111;
    }

    /// `SEI` instruction
    fn sei(&mut self) {
        self.status |= 0b0000_0100;
    }

    /// `CLI` instruction
    fn cli(&mut self) {
        self.status &= 0b1111_1011;
    }
}

#[cfg(test)]
mod tests {
    use crate::rom::Rom;

    use super::*;

    // TODO: Duplicate of private binding in `rom.rs`, think about if that should be made public
    // for being reusable in this test module or not (or do something better)
    static ROM_HEADER_MAGIC_STRING: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
    static PRG_ROM_PAGE_SIZE: usize = 0x8000;

    fn create_rom(program_data: &[u8]) -> Rom {
        let mut data = ROM_HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_0000;
        let control_byte_two = 0b1111_0000;
        let mut bytes_after_magic_string = vec![
            no_of_16kib_rom_banks,
            no_of_8kib_vrom_banks,
            control_byte_one,
            control_byte_two,
        ];
        data.append(&mut bytes_after_magic_string);
        // TODO: In addition to the reserved six bytes at the end of the header being zero, also
        // makes the PRG RAM byte and the unknown byte after that, be zero. Find out how to set
        // these two bytes properly at some point.
        let reserved_empty_bytes = [0x00; 8];
        data.append(&mut reserved_empty_bytes.to_vec());
        let mut program_rom_data = vec![0; PRG_ROM_PAGE_SIZE - 6];
        program_rom_data[0..program_data.len()].copy_from_slice(&program_data[..]);
        let initial_addr_lo = 0x00;
        let initial_addr_hi = 0x80;
        let interrupt_vectors = vec![0x0, 0x0, initial_addr_lo, initial_addr_hi, 0x0, 0x0];
        program_rom_data.append(&mut interrupt_vectors.to_vec());
        data.append(&mut program_rom_data.to_vec());
        let chr_rom_data = [0x02; 0x1FFF];
        data.append(&mut chr_rom_data.to_vec());
        Rom::new(&data[..])
    }

    #[test]
    fn brk_flag_set() {
        let mut ram = [0x00; 2048];
        let program = vec![0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        let is_brk_flag_set = cpu.status & 0b001_0000 == 0b0001_0000;
        assert_eq!(is_brk_flag_set, true);
    }

    #[test]
    fn lda_immediate_addressing() {
        let mut ram = [0x00; 2048];
        let lda_immediate_addressing_opcode = 0xA9;
        let value_to_load = 0x05 as u8;
        let program = vec![lda_immediate_addressing_opcode, value_to_load, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        assert_eq!(cpu.register_a, value_to_load);
    }

    #[test]
    fn tax_sets_correct_value() {
        let mut ram = [0x00; 2048];
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x05;
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        assert_eq!(cpu.register_x, value_to_load);
    }

    #[test]
    fn tax_set_zero_flag() {
        let mut ram = [0x00; 2048];

        // Program does the following:
        // - load the value representing 0 that is in register X into register A (zero flag should
        // be set)
        // - break
        let tax_opcode = 0xAA;
        let program = vec![tax_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn tax_clear_zero_flag() {
        let mut ram = [0x00; 2048];

        // Program does the following:
        // - load the value representing 4 into register A (zero flag should be cleared)
        // - transfer 4 stored in register A to register X (zero flag should stay cleared)
        // - break
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x04;
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, false);
    }

    #[test]
    fn tax_set_negative_flag() {
        let mut ram = [0x00; 2048];

        // Program does the following:
        // - load the value representing -1 into register A (negative flag should be set)
        // - transfer -1 stored in register A to register X (negative flag should stay set)
        // - break
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let negative_value = 0b1000_0000; // -128 in two's complement representation
        let program = vec![lda_opcode, negative_value, tax_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn tax_clear_negative_flag() {
        let mut ram = [0x00; 2048];

        // Program does the following:
        // - load the value representing 4 into register A (negative flag should be cleared)
        // - transfer 4 stored in register A to register X (negative flag should stay cleared)
        // - break
        let lda_opcode = 0xA9;
        let tax_opcode = 0xAA;
        let value_to_load = 0x04;
        let program = vec![lda_opcode, value_to_load, tax_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, false);
    }

    #[test]
    fn inc_zero_page_addressing_mode_increments_value() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0x23;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute INC instruction on memory address containing value
        // - break
        let inc_zero_page_addr_mode_opcode = 0xE6;
        let program = vec![inc_zero_page_addr_mode_opcode, zero_page_addr];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(memory_value + 1, ram[zero_page_addr as usize]);
    }

    #[test]
    fn inx_increments_value_correctly() {
        let mut ram = [0x00; 2048];
        let inx_opcode = 0xE8;
        let program = vec![inx_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        assert_eq!(cpu.register_x, 0x01);
    }

    #[test]
    fn iny_increments_register_y_value() {
        let mut ram = [0x00; 2048];
        let register_y_value = 0x13;

        // Program does the following:
        // - load value into register Y
        // - execute INY instruction
        // - break
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let iny_zero_page_addr_mode_opcode = 0xC8;
        let program = vec![
            ldy_immediate_addr_mode_opcode,
            register_y_value,
            iny_zero_page_addr_mode_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(register_y_value + 1, cpu.register_y);
    }

    #[test]
    fn sta_absolute_addressing_stores_correct_value() {
        let mut ram = [0x00; 2048];

        // Program does the following:
        // - load value into register A / accumulator
        // - store value of register A in 16-bit address given by `addr_lo` and `addr_hi`
        // - break
        let lda_opcode = 0xA9;
        let sta_abs_addr_mode_opcode = 0x8D;
        let addr_lo = 0x00;
        let addr_hi = 0x01;
        let value = 0x23;
        let program = vec![
            lda_opcode,
            value,
            sta_abs_addr_mode_opcode,
            addr_lo,
            addr_hi,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let stored_16bit_addr = u16::from_le_bytes([addr_lo, addr_hi]);
        assert_eq!(ram[stored_16bit_addr as usize], value);
    }

    #[test]
    fn stx_zero_page_y_addressing_stores_correct_value() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x10;
        let offset = 0x05;
        let value = 0x23;

        // Write `value` into `zero_page_addr + offset` in the `ram` array
        ram[(zero_page_addr + offset) as usize] = value;

        // Program does the following:
        // - store offset in register Y
        // - load value into register X
        // - store contents of register X in `zero_page_addr` offset by the value in register Y
        // - break
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let stx_zero_page_y_addr_mode_opcode = 0x96;
        let program = vec![
            ldy_immediate_addr_mode_opcode,
            offset,
            ldx_immediate_addr_mode_opcode,
            value,
            stx_zero_page_y_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let stored_value = ram[(zero_page_addr + offset) as usize];
        assert_eq!(stored_value, value);
    }

    #[test]
    fn sty_absolute_addressing_stores_correct_value() {
        let mut ram = [0x00; 2048];
        let lo = 0x00;
        let hi = 0x01;
        let value = 0x23;

        // Program does the following:
        // - load value into register Y
        // - store contents of register Y in 16-bit address denoted by `lo` and `hi`
        // - break
        let ldy_immediate_addr_opcode = 0xA0;
        let sty_absolute_addr_mode_opcode = 0x8C;
        let program = vec![
            ldy_immediate_addr_opcode,
            value,
            sty_absolute_addr_mode_opcode,
            lo,
            hi,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let stored_value = ram[u16::from_le_bytes([lo, hi]) as usize];
        assert_eq!(stored_value, value);
    }

    #[test]
    fn ldx_zero_page_addressing_loads_correct_value() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x10;
        let value = 0x23;

        // Write `value` into `zero_page_addr` in the `ram` array
        ram[zero_page_addr as usize] = value;

        // Program does the following:
        // - load contents of `zero_page_addr` into register X
        // - break
        let ldx_zero_page_addr_mode_opcode = 0xA6;
        let program = vec![ldx_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(cpu.register_x, value);
    }

    #[test]
    fn ldy_zero_page_x_addressing_loads_correct_value() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x10;
        let offset = 0x05;
        let value = 0x23;

        // Write `value` into `zero_page_addr + offset` in the `ram` array
        ram[(zero_page_addr + offset) as usize] = value;

        // Program does the following:
        // - store offset in register X
        // - load contents of `zero_page_addr + offset` into register Y
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let ldy_zero_page_x_addr_mode_opcode = 0xB4;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            offset,
            ldy_zero_page_x_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(cpu.register_y, value);
    }

    #[test]
    fn and_absolute_x_addressing_sets_correct_value() {
        let mut ram = [0x00; 2048];
        let lo = 0x00;
        let hi = 0x01;
        let offset = 0x05;
        let register_a_value = 0b01100011;
        let memory_value = 0b10101110;

        // Write value to 16-bit addr + offset
        let addr_16bit = u16::from_le_bytes([lo, hi]) + offset as u16;
        ram[addr_16bit as usize] = memory_value;

        // Program does the following:
        // - load value into register A
        // - load offset into register X
        // - perform bitwise AND between bits in register A and value in memory, and store result
        // in register A
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let and_abs_x_addr_mode_opcode = 0x3D;
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
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(cpu.register_a, register_a_value & memory_value);
    }

    #[test]
    fn cmp_indirect_x_addressing_sets_carry_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x10;
        let zero_page_addr_offset = 0x05;
        let lo = 0x15;
        let hi = 0x01;
        let register_a_value = 0b00001010; // 10 in two's complement representation
        let memory_value = 0b00001000; // 8 in two's complement representation

        // Write target address value's least and most significant bytes starting at
        // `zero_page_addr + lo_offset`
        ram[(zero_page_addr + zero_page_addr_offset) as usize] = lo;
        ram[(zero_page_addr + zero_page_addr_offset + 1) as usize] = hi;

        // Write value to target address
        ram[u16::from_le_bytes([lo, hi]) as usize] = memory_value;

        // Program does the following:
        // - load value into register A
        // - load offset into register X
        // - compare value in register A to value in memory
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let cmp_indirect_x_addr_mode_opcode = 0xC1;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            ldx_immediate_addr_mode_opcode,
            zero_page_addr_offset,
            cmp_indirect_x_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn cpx_immediate_addressing_sets_carry_flag_if_register_x_geq_to_memory_value() {
        let mut ram = [0x00; 2048];
        let register_x_value = 20u8;
        let memory_value = 15u8;

        // Program does the following:
        // - load value into register X
        // - execute CPX instruction
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let cpx_immediate_addr_mode_opcode = 0xE0;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            cpx_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn cpx_immediate_addressing_sets_negative_flag() {
        let mut ram = [0x00; 2048];
        let register_x_value = 0x7F;
        let memory_value = 0x80;

        // Program does the following:
        // - load value into register X
        // - execute CPX instruction
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let cpx_immediate_addr_mode_opcode = 0xE0;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            cpx_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn cpy_immediate_addressing_sets_carry_flag_if_register_x_geq_to_memory_value() {
        let mut ram = [0x00; 2048];
        let register_y_value = 20u8;
        let memory_value = 15u8;

        // Program does the following:
        // - load value into register Y
        // - execute CPY instruction
        // - break
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let cpy_immediate_addr_mode_opcode = 0xC0;
        let program = vec![
            ldy_immediate_addr_mode_opcode,
            register_y_value,
            cpy_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn cpy_immediate_addressing_sets_negative_flag() {
        let mut ram = [0x00; 2048];
        let register_y_value = 0x7F;
        let memory_value = 0x80;

        // Program does the following:
        // - load value into register Y
        // - execute CPY instruction
        // - break
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let cpy_immediate_addr_mode_opcode = 0xC0;
        let program = vec![
            ldy_immediate_addr_mode_opcode,
            register_y_value,
            cpy_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn eor_indirect_y_addressing_sets_zero_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x10;
        let lo = 0x15;
        let hi = 0x01;
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

        // Program does the following:
        // - load value into register A
        // - load least significant byte offset into register Y
        // - perform XOR between value in register A and memory value
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let eor_indirect_y_addr_mode_opcode = 0x51;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            ldy_immediate_addr_mode_opcode,
            lo_offset,
            eor_indirect_y_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn ora_absolute_y_addressing_mode_sets_negative_flag() {
        let mut ram = [0x00; 2048];
        let lo = 0x1F;
        let hi = 0x01;
        let offset = 0x10;
        let register_a_value = 0b1001_1001;
        let memory_value = 0b0110_0000;

        // Write value to memory address given by `offset` applied to the 16-bit address described
        // by `lo` and `hi`
        ram[(u16::from_le_bytes([lo, hi]) + offset as u16) as usize] = memory_value;

        // Program does the following:
        // - load value into register A
        // - load offset into register Y
        // - perform OR between value in register A and memory value
        // - break
        let lda_immediate_addressing_opcode = 0xA9;
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let ora_absolute_y_addr_mode_opcode = 0x19;
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
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn jmp_indirect_addressing_updates_program_counter_correctly() {
        let mut ram = [0x00; 2048];
        let lo = 0x1F;
        let hi = 0x85;

        // Program does the following:
        // - jump to 16-bit address given by `lo` and `hi`
        // - break
        let jmp_indirect_addr_mode_opcode = 0x6C;
        let program = vec![jmp_indirect_addr_mode_opcode, lo, hi, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(cpu.program_counter, u16::from_le_bytes([lo, hi]));
    }

    #[test]
    fn jsr_sets_stack_register_and_program_counter_correctly() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let lo = 0x1F;
        let hi = 0x85;

        // Program does the following:
        // - execute JSR instruction
        let jsr_absolute_addr_mode_opcode = 0x20;
        let program = vec![jsr_absolute_addr_mode_opcode, lo, hi];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions = program.len();
        cpu.load_and_run();

        // Check the final program counter value
        let target_address = u16::from_le_bytes([lo, hi]);
        let increment_after_jump_from_opcode_read = 1;
        let expected_program_counter = target_address + increment_after_jump_from_opcode_read;
        assert_eq!(expected_program_counter, cpu.program_counter);

        // Check the return address stored in the stack
        let lo_addr = u16::from_le_bytes([STACK_REGISTER_LO_START - 1, STACK_REGISTER_HI]);
        let hi_addr = u16::from_le_bytes([STACK_REGISTER_LO_START, STACK_REGISTER_HI]);
        let lo = ram[lo_addr as usize];
        let hi = ram[hi_addr as usize];
        let return_address = u16::from_le_bytes([lo, hi]);
        assert_eq!(
            program_counter_start + (no_of_instructions as u16) - 1,
            return_address
        );
    }

    #[test]
    fn rts_enables_subroutine_to_execute_and_pops_stack_when_done() {
        let mut ram = [0x00; 2048];
        let subroutine_start_addr_lo = 0x1F;
        let subroutine_start_addr_hi = 0x01;
        let subroutine_start_addr =
            u16::from_le_bytes([subroutine_start_addr_lo, subroutine_start_addr_hi]);
        let lda_immediate_addr_mode_opcode = 0xA9;
        let register_a_value = 0x6A;
        let rts_opcode = 0x60;
        ram[subroutine_start_addr as usize] = lda_immediate_addr_mode_opcode;
        ram[(subroutine_start_addr + 1) as usize] = register_a_value;
        ram[(subroutine_start_addr + 2) as usize] = rts_opcode;

        // Program does the follow:
        // - jump to address given by `subroutine_start_addr_lo` and `subroutine_start_addr_hi`
        // - load value into register A
        // - exeucte RTS instruction
        // - break
        let jsr_absolute_addr_mode_opcode = 0x20;
        let program = vec![
            jsr_absolute_addr_mode_opcode,
            subroutine_start_addr_lo,
            subroutine_start_addr_hi,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(register_a_value, cpu.register_a);
    }

    #[test]
    fn dec_zero_page_addressing_mode_modifies_value_correctly() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x25;
        let value = 0x05;
        ram[zero_page_addr as usize] = value;

        // Program does the following:
        // - decrement value in memory address
        // - break
        let dec_zero_page_addr_mode_opcode = 0xC6;
        let program = vec![dec_zero_page_addr_mode_opcode, zero_page_addr];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!((value as i8) - 1, ram[zero_page_addr as usize] as i8);
    }

    #[test]
    fn dex_decrements_register_x_value() {
        let mut ram = [0x00; 2048];
        let register_x_value = 0x10;

        // Program does the following:
        // - load value into register X
        // - execute DEX instruction
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let dex_opcode = 0xCA;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            dex_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(register_x_value - 1, cpu.register_x);
    }

    #[test]
    fn dey_decrements_register_y_value() {
        let mut ram = [0x00; 2048];
        let register_y_value = 0x10;

        // Program does the following:
        // - load value into register Y
        // - executes DEY instruction
        // - break
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let dey_opcode = 0x88;
        let program = vec![
            ldy_immediate_addr_mode_opcode,
            register_y_value,
            dey_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(register_y_value - 1, cpu.register_y);
    }

    #[test]
    fn txs_transfers_register_x_value_to_stack_register() {
        let mut ram = [0x00; 2048];
        let register_x_value = 0x04;

        // Program does the following:
        // - load value in register X
        // - transfer value from register X to stack register
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let txs_opcode = 0x9A;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            txs_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(cpu.stack_register, register_x_value);
    }

    #[test]
    fn pha_pushes_register_a_value_onto_stack() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0x06;

        // Program does the following:
        // - load value into register A
        // - push value in register A onto stack
        // - break
        let lda_immediate_addressing_opcode = 0xA9;
        let pha_opcode = 0x048;
        let program = vec![
            lda_immediate_addressing_opcode,
            register_a_value,
            pha_opcode,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let stack_address_containing_value =
            u16::from_le_bytes([cpu.stack_register + 1, STACK_REGISTER_HI]);
        assert_eq!(
            ram[stack_address_containing_value as usize],
            register_a_value
        );
    }

    #[test]
    fn tsx_transfers_stack_register_value_to_register_x() {
        let mut ram = [0x00; 2048];

        // Program does the following:
        // - copy vlaue in stack register to register X
        // - break
        let tsx_opcode = 0xBA;
        let program = vec![tsx_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(cpu.register_x, STACK_REGISTER_LO_START);
    }

    #[test]
    fn bit_zero_page_addressing_mode_sets_overflow_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x25;
        let memory_value = 0b0100_1000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - perform BIT instruction with value in address `zero_page_addr`
        // - break
        let bit_zero_page_addr_mode_opcode = 0x24;
        let program = vec![bit_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(is_overflow_flag_set, true);
    }

    #[test]
    fn bit_zero_page_addressing_mode_clears_overflow_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr_set = 0x25;
        let zero_page_addr_clear = 0x30;
        let memory_value_set = 0b0100_1000;
        let memory_value_clear = 0b0000_1100;
        ram[zero_page_addr_set as usize] = memory_value_set;
        ram[zero_page_addr_clear as usize] = memory_value_clear;

        // Program does the following:
        // - perform BIT instruction with value in address `zero_page_addr_set` (should set the
        // overflow flag)
        // - perform BIT instruction with value in address `zero_page_addr_clear` (should clear the
        // overflow flag)
        // - break
        let bit_zero_page_addr_mode_opcode = 0x24;
        let program = vec![
            bit_zero_page_addr_mode_opcode,
            zero_page_addr_set,
            bit_zero_page_addr_mode_opcode,
            zero_page_addr_clear,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(is_overflow_flag_set, false);
    }

    #[test]
    fn bit_absolute_addressing_mode_sets_negative_flag() {
        let mut ram = [0x00; 2048];
        let lo = 0x10;
        let hi = 0x01;
        let memory_value = 0b1000_0000;
        ram[u16::from_le_bytes([lo, hi]) as usize] = memory_value;

        // Program does the following:
        // - perform BIT instruction with value in 16-bit address given by `lo` and `hi`
        // - break
        let bit_absolute_addr_mode_opcode = 0x2C;
        let program = vec![bit_absolute_addr_mode_opcode, lo, hi, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn bit_absolute_addressing_mode_clears_negative_flag() {
        let mut ram = [0x00; 2048];
        let lo_set = 0x10;
        let hi_set = 0x01;
        let lo_clear = 0x15;
        let hi_clear = 0x02;
        let memory_value_set = 0b1000_0000;
        let memory_value_clear = 0b0100_0000;
        ram[u16::from_le_bytes([lo_set, hi_set]) as usize] = memory_value_set;
        ram[u16::from_le_bytes([lo_clear, hi_clear]) as usize] = memory_value_clear;

        // Program does the following:
        // - perform BIT instruction with value in 16-bit address given by `lo_set` and `hi_set`
        // (should set negative flag)
        // - perform BIT instruction with value in 16-bit address given by `lo_clear` and
        // `hi_clear` (should clear negative flag)
        // - break
        let bit_absolute_addr_mode_opcode = 0x2C;
        let program = vec![
            bit_absolute_addr_mode_opcode,
            lo_set,
            hi_set,
            bit_absolute_addr_mode_opcode,
            lo_clear,
            hi_clear,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, false);
    }

    #[test]
    fn bit_zero_page_addressing_mode_sets_zero_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0010_0100;
        let register_a_value = 0b1101_1011;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - load value into register A
        // - perform BIT instruction with value in address `zero_page_addr`
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let bit_zero_page_addr_mode_opcode = 0x24;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            bit_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn bit_zero_page_addressing_mode_clears_zero_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr_set = 0x15;
        let zero_page_addr_clear = 0x20;
        let memory_value_set = 0b0010_0100;
        let memory_value_clear = 0b1000_0100;
        let register_a_value = 0b1101_1011;
        ram[zero_page_addr_set as usize] = memory_value_set;
        ram[zero_page_addr_clear as usize] = memory_value_clear;

        // Program does the following:
        // - load value into register A
        // - perform BIT instruction with value in address `zero_page_addr_set` (should set zero
        // flag)
        // - perform BIT instruction with value in address `zero_page_addr_clear` (should clear
        // zero flag)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let bit_zero_page_addr_mode_opcode = 0x24;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            bit_zero_page_addr_mode_opcode,
            zero_page_addr_set,
            bit_zero_page_addr_mode_opcode,
            zero_page_addr_clear,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, false);
    }

    #[test]
    fn beq_correctly_offsets_program_counter() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -6i8;

        // Program does the following:
        // - load zero value into register A (so then the zero flag is set)
        // - perform BEQ instruction (which does non-trivial action if the zero flag is set)
        let lda_immediate_addr_mode_opcode = 0xA9;
        let beq_opcode = 0xF0;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            0x00,
            beq_opcode,
            offset as u8,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions_before_offset_is_read = 3;
        let increment_after_reading_beq_instruction = 1;
        let expected_program_counter_value = program_counter_start as i32
            + no_of_instructions_before_offset_is_read
            + offset as i32
            + increment_after_reading_beq_instruction;
        cpu.load_and_run();
        assert_eq!(expected_program_counter_value as u16, cpu.program_counter);
    }

    #[test]
    fn sec_sets_carry_flag() {
        let mut ram = [0x00; 2048];
        let sec_opcode = 0x38;
        let program = vec![sec_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn clc_clears_carry_flag() {
        let mut ram = [0x00; 2048];

        // Prorgam does the following:
        // - set the carry flag (to be able to verify that the carry flag has been cleared)
        // - clear the carry flag
        // - break
        let sec_opcode = 0x38;
        let clc_opcode = 0x18;
        let program = vec![sec_opcode, clc_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, false);
    }

    #[test]
    fn bcs_correctly_offsets_program_counter() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -6i8;

        // Program does the following:
        // - set carry flag
        // - execute BCS instruction (which does non-trivial action if carry flag is set)
        let sec_opcode = 0x38;
        let bcs_opcode = 0xB0;
        let program = vec![sec_opcode, bcs_opcode, offset as u8];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions_before_offset_is_read = 2;
        let increment_after_reading_bcs_instruction = 1;
        let expected_program_counter_value = program_counter_start as i32
            + no_of_instructions_before_offset_is_read
            + offset as i32
            + increment_after_reading_bcs_instruction;
        cpu.load_and_run();
        assert_eq!(expected_program_counter_value as u16, cpu.program_counter);
    }

    #[test]
    fn bcc_correctly_offsets_program_counter() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -6i8;

        // Program does the following:
        // - execute BCC instruction (which does non-trivial action if carry flag is clear)
        let bcc_opcode = 0x90;
        let program = vec![bcc_opcode, offset as u8];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions_before_offset_is_read = 1;
        let increment_after_reading_bcc_instruction = 1;
        let expected_program_counter_value = program_counter_start as i32
            + no_of_instructions_before_offset_is_read
            + offset as i32
            + increment_after_reading_bcc_instruction;
        cpu.load_and_run();
        assert_eq!(expected_program_counter_value as u16, cpu.program_counter);
    }

    #[test]
    fn bmi_correctly_offsets_program_counter() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -6i8;
        let register_a_value = -3i8;

        // Program does the following:
        // - load negative value into register A (to set the negative flag)
        // - execute BMI instruction (which does non-trivial action if negative flag is set)
        let lda_immediate_addr_mode_opcode = 0xA9;
        let bmi_opcode = 0x30;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value as u8,
            bmi_opcode,
            offset as u8,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions_before_offset_is_read = 3;
        let increment_after_reading_bmi_instruction = 1;
        let expected_program_counter_value = program_counter_start as i32
            + no_of_instructions_before_offset_is_read
            + offset as i32
            + increment_after_reading_bmi_instruction;
        cpu.load_and_run();
        assert_eq!(expected_program_counter_value as u16, cpu.program_counter);
    }

    #[test]
    fn bpl_correctly_offsets_program_counter() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -6i8;

        // Program does the following:
        // - execute BPL instruction (which does non-trivial action if negative flag is clear)
        let bpl_opcode = 0x10;
        let program = vec![bpl_opcode, offset as u8];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions_before_offset_value = 1;
        let increment_reading_next_opcode_after_offsetting = 1;
        let expected_program_counter_value = program_counter_start as i32
            + no_of_instructions_before_offset_value
            + offset as i32
            + increment_reading_next_opcode_after_offsetting;
        cpu.load_and_run();
        assert_eq!(expected_program_counter_value as u16, cpu.program_counter);
    }

    #[test]
    fn bne_correctly_offsets_program_counter() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -6i8;

        // Program does the following:
        // - execute BNE instruction (which does non-trivial action if zero flag is clear)
        let bne_opcode = 0xD0;
        let program = vec![bne_opcode, offset as u8];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions_before_offset_value = 1;
        let increment_reading_next_opcode_after_offsetting = 1;
        let expected_program_counter_value = program_counter_start as i32
            + no_of_instructions_before_offset_value
            + offset as i32
            + increment_reading_next_opcode_after_offsetting;
        cpu.load_and_run();
        assert_eq!(expected_program_counter_value as u16, cpu.program_counter);
    }

    #[test]
    fn rol_zero_page_addressing_mode_includes_carry_flag_in_modification() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0001;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - set carry flag
        // - execute ROL instruction on value in zero page addr (which should take into account the
        // carry flag being set)
        // - break
        let sec_opcode = 0x38;
        let rol_zero_page_addr_mode_opcode = 0x26;
        let program = vec![
            sec_opcode,
            rol_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!((memory_value << 1) + 1, ram[zero_page_addr as usize]);
    }

    #[test]
    fn rol_zero_page_addressing_mode_sets_zero_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b1000_0000; // arithmetic left shift produces zero value
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ROL instruction on value in zero page addr
        // - break
        let rol_zero_page_addr_mode_opcode = 0x26;
        let program = vec![rol_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn rol_zero_page_addressing_mode_sets_carry_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b1000_0000; // bit 7 is set on original value
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ROL instruction on value in zero page addr
        // - break
        let rol_zero_page_addr_mode_opcode = 0x26;
        let program = vec![rol_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b000_0001 == 0b0000_001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn rol_zero_page_addressing_mode_clears_carry_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0100_0000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - set carry flag
        // - execute ROL instruction on value in zero page addr (should clear carry flag)
        // - break
        let sec_opcode = 0x38;
        let rol_zero_page_addr_mode_opcode = 0x26;
        let program = vec![
            sec_opcode,
            rol_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, false);
    }

    #[test]
    fn rol_zero_page_addressing_mode_sets_negative_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0100_0000; // arithmetic left shift would set bit 7
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ROL instruction on value in zero page addr (should set negative flag)
        // - break
        let rol_zero_page_addr_mode_opcode = 0x26;
        let program = vec![rol_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn asl_zero_page_addressing_mode_modifies_value_correctly() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0010;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ASL instruction on value in zero page addr
        // - break
        let asl_zero_page_addr_mode_opcode = 0x06;
        let program = vec![asl_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(memory_value << 1, ram[zero_page_addr as usize]);
    }

    #[test]
    fn asl_zero_page_addressing_mode_sets_zero_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b1000_0000; // arithmetic left shift produces zero value
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ASL instruction on value in zero page addr
        // - break
        let asl_zero_page_addr_mode_opcode = 0x06;
        let program = vec![asl_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    // NOTE: Practically identical to zero flag test, just checking a different bit in the status
    // register, so maybe some kind of merging of the tests is possible to remove duplication;
    // parametrisation maybe?
    fn asl_zero_page_addressing_mode_sets_carry_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b1000_0000; // arithmetic left shift produces zero value
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ASL instruction on value in zero page addr
        // - break
        let asl_zero_page_addr_mode_opcode = 0x06;
        let program = vec![asl_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b000_0001 == 0b0000_001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn asl_zero_page_addressing_mode_clears_carry_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0100_0000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - set carry flag
        // - execute ASL instruction on value in zero page addr (should clear carry flag)
        // - break
        let sec_opcode = 0x38;
        let asl_zero_page_addr_mode_opcode = 0x06;
        let program = vec![
            sec_opcode,
            asl_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, false);
    }

    #[test]
    fn asl_zero_page_addressing_mode_sets_negative_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0100_0000; // arithmetic left shift would set bit 7
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ASL instruction on value in zero page addr (should set negative flag)
        // - break
        let asl_zero_page_addr_mode_opcode = 0x06;
        let program = vec![asl_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, true);
    }

    #[test]
    fn lsr_zero_page_addressing_mode_modifes_value_correctly() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_1000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute LSR instruction on value in zero page addr
        // - break
        let lsr_zero_page_addr_mode_opcode = 0x46;
        let program = vec![lsr_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(memory_value >> 1, ram[zero_page_addr as usize]);
    }

    #[test]
    fn lsr_zero_page_addressing_mode_sets_zero_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0001; // logical right shift produces zero value
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute LSR instruction on value in zero page addr
        // - break
        let lsr_zero_page_addr_mode_opcode = 0x46;
        let program = vec![lsr_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn lsr_zero_page_addressing_mode_sets_carry_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0001;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute LSR instruction on value in zero page addr
        // - break
        let lsr_zero_page_addr_mode_opcode = 0x46;
        let program = vec![lsr_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn lsr_zero_page_addressing_mode_clears_carry_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0010;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - set carry flag
        // - execute LSR instruction on value in zero page addr (should clear carry flag)
        // - break
        let sec_opcode = 0x38;
        let lsr_zero_page_addr_mode_opcode = 0x46;
        let program = vec![
            sec_opcode,
            lsr_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, false);
    }

    #[test]
    fn lsr_zero_page_addressing_mode_clears_negative_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let register_a_value = 0b1000_0000;
        let memory_value = 0b0100_0000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - load value into register A (should set negative flag)
        // - execute LSR instruction on value in zero page addr (should clear negative flag)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let lsr_zero_page_addr_mode_opcode = 0x46;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            lsr_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, false);
    }

    #[test]
    fn ror_zero_page_addressing_mode_leaves_bit_seven_if_carry_flag_clear() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_1000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ROL instruction on value in zero page addr
        // - break
        let ror_zero_page_addr_mode_opcode = 0x66;
        let program = vec![ror_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(memory_value >> 1, ram[zero_page_addr as usize]);
    }

    #[test]
    fn ror_zero_page_addressing_mode_sets_bit_seven_if_carry_flag_set() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_1000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - set carry flag
        // - execute ROL instruction on value in zero page addr (which should take into account the
        // carry flag being set)
        // - break
        let sec_opcode = 0x38;
        let ror_zero_page_addr_mode_opcode = 0x66;
        let program = vec![
            sec_opcode,
            ror_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(0b1000_0100, ram[zero_page_addr as usize]);
    }

    #[test]
    fn ror_zero_page_addressing_mode_sets_zero_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0001; // right shift produces zero value
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - execute ROR instruction
        // - break
        let ror_zero_page_addr_mode_opcode = 0x66;
        let program = vec![ror_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(is_zero_flag_set, true);
    }

    #[test]
    fn ror_zero_page_addressing_mode_sets_carry_flag_if_orig_bit_zero_set() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0001; // bit 0 is set on original value

        // Program does the following:
        // - execute ROR instruction on value in zero page addr
        // - break
        let ror_zero_page_addr_mode_opcode = 0x66;
        let program = vec![ror_zero_page_addr_mode_opcode, zero_page_addr, 0x00];
        ram[zero_page_addr as usize] = memory_value;
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn ror_zero_page_addressing_mode_clears_carry_flag_if_orig_bit_zero_clear() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let memory_value = 0b0000_0010;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - set carry flag
        // - execute ROR instruction on value in zero page addr (should clear carry flag)
        // - break
        let sec_opcode = 0x38;
        let ror_zero_page_addr_mode_opcode = 0x66;
        let program = vec![
            sec_opcode,
            ror_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, false);
    }

    #[test]
    fn ror_zero_page_addressing_mode_clears_negative_flag() {
        let mut ram = [0x00; 2048];
        let zero_page_addr = 0x15;
        let register_a_value = 0b1000_0000;
        let memory_value = 0b0100_0000;
        ram[zero_page_addr as usize] = memory_value;

        // Program does the following:
        // - load value into register A (should set negative flag)
        // - execute ROR instruction on value in zero page addr (should clear negative flag)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let ror_zero_page_addr_mode_opcode = 0x66;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            ror_zero_page_addr_mode_opcode,
            zero_page_addr,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(is_negative_flag_set, false);
    }

    #[test]
    fn php_pushes_status_flags_onto_stack() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b1000_0000;

        // Program does the following:
        // - load value into register A (should set negative flag)
        // - set carry flag
        // - execute PHP instruction
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let sec_opcode = 0x38;
        let php_opcode = 0x08;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            sec_opcode,
            php_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let expected_status = 0b1010_0001; // negative flag + bit 5 + carry flag
        let stack_address_containing_value =
            u16::from_le_bytes([cpu.stack_register + 1, STACK_REGISTER_HI]);
        assert_eq!(
            ram[stack_address_containing_value as usize],
            expected_status
        );
    }

    #[test]
    fn pla_sets_register_a_correctly() {
        let mut ram = [0x00; 2048];
        let register_x_value = 0b1000_0000;

        // Program does the following:
        // - load value into register X (should set negative flag)
        // - set carry flag
        // - put status flag values into stack register
        // - execute PLA instruction
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let sec_opcode = 0x38;
        let php_opcode = 0x08;
        let pla_opcode = 0x68;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            sec_opcode,
            php_opcode,
            pla_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let expected_status = 0b1010_0001; // negative flag + bit 5 + carry flag
        assert_eq!(expected_status, cpu.register_a);
    }

    #[test]
    fn plp_sets_status_flags_correctly() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b1100_0101;

        // Program does the following:
        // - load value into register A
        // - put value in register A into stack register
        // - execute PLP instruction
        let lda_immediate_addr_mode_opcode = 0xA9;
        let pha_opcode = 0x48;
        let plp_opcode = 0x28;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            pha_opcode,
            plp_opcode,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let expected_status = register_a_value | 0b0001_0000; // expecting bit 5 to be set too
        assert_eq!(expected_status, cpu.status);
    }

    #[test]
    fn adc_immediate_addressing_mode_correct_value_when_carry_flag_clear() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0x20;
        let memory_value = 0x16;

        // Program does the following:
        // - load value into register A
        // - execute ADC instruction with value given
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let adc_immediate_addr_mode_opcode = 0x69;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            adc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(register_a_value + memory_value, cpu.register_a);
    }

    #[test]
    fn adc_immediate_addressing_mode_correct_value_when_carry_flag_set() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0x20;
        let memory_value = 0x16;

        // Program does the following:
        // - load value into register A
        // - set carry flag
        // - execute ADC instruction with value given
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let sec_opcode = 0x38;
        let adc_immediate_addr_mode_opcode = 0x69;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            sec_opcode,
            adc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(register_a_value + memory_value + 1, cpu.register_a);
    }

    #[test]
    fn adc_immediate_addressing_mode_sets_overflow_flag_two_positives_output_negative() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b0011_1111; // 63 in two's complement representation
        let memory_value = 0b0100_0001; // 65 in two's complement representation

        // Program does the following:
        // - load value into register A
        // - execute ADC instruction (adding two positive numbers which overflow an `i8`, so should
        // produce an 8-bit value whose sign bit is set, and thus has the wrong sign for adding two
        // positive numbers)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let adc_immediate_addr_mode_opcode = 0x69;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            adc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        // Overflow `i8` at the top boundary and wrap around to a negative value
        let expected_result = 0b1000_0000;
        assert_eq!(expected_result, cpu.register_a);
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(is_overflow_flag_set, true);
    }

    #[test]
    fn adc_immediate_addressing_mode_sets_overflow_flag_two_negatives_output_positive() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b1100_0000; // -64 in two's complement representation
        let memory_value = 0b1011_1111; // -65 in two's complement representation

        // Program does the following:
        // - load value into register A
        // - execute ADC instruction (adding two negative numbers which overflow an `i8`, so should
        // produce an 8-bit value whose sign bit is clear, and thus has the wrong sign for adding two
        // negative numbers)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let adc_immediate_addr_mode_opcode = 0x69;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            adc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        // Overflow `i8` at the bottom boundary and wrap around to a positive value
        let expected_result = 0b0111_1111;
        assert_eq!(expected_result, cpu.register_a);
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(is_overflow_flag_set, true);
    }

    #[test]
    fn adc_immediate_addressing_mode_sets_carry_flag_if_unsigned_overflow() {
        let mut ram = [0x00; 2048];
        let register_a_value = 127u8;
        let memory_value = 129u8;

        // Program does the following:
        // - load value into register A
        // - execute ADC instruction (adding two values whose unsigned int representation would
        // overflow the range of `u8`)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let adc_immediate_addr_mode_opcode = 0x69;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            adc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        // Overflow `u8` at the top boundary and lose info in extra bit (should be 256, but appears
        // to be 0 due to info in 8th bit - counting from 0 - being missing from a `u8`)
        let expected_result = 0b0000_0000;
        assert_eq!(expected_result, cpu.register_a);
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, true);
    }

    #[test]
    fn sbc_immediate_addressing_mode_correct_value_when_carry_flag_set() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b0010_0000; // 32
        let memory_value = 0b0001_0110; // 22

        // Program does the following:
        // - load value into register A
        // - set carry flag
        // - execute SBC instruction with value given
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let sec_opcode = 0x38;
        let sbc_immediate_addr_mode_opcode = 0xE9;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            sec_opcode,
            sbc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(register_a_value - memory_value, cpu.register_a);
    }

    #[test]
    fn sbc_immediate_addr_mode_sets_overflow_flag_negative_subtract_positive_outputs_positive() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b1100_0000; // -64 in two's complement representation
        let memory_value = 0b0100_0001; // 65 in two's complement representation

        // Program does the following:
        // - load value into register A
        // - execute SBC instruction (subtracting positive number from negative number should
        // produce a larger negative number, but will underflow and wrap around, producing an
        // incorrect positive number)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let sec_opcode = 0x38;
        let sbc_immediate_addr_mode_opcode = 0xE9;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            sec_opcode,
            sbc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        // Underflow `i8` and wrap around to a positive value
        let expected_result = 0b0111_1111;
        assert_eq!(expected_result as i8, cpu.register_a as i8);
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(is_overflow_flag_set, true);
    }

    #[test]
    fn sbc_immediate_addr_mode_sets_overflow_flag_positive_subtract_negative_outputs_negative() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b0011_1111; // 63 in two's complement representation
        let memory_value = 0b1011_1111; // -65 in two's complement representation

        // Program does the following:
        // - load value into register A
        // - execute SBC instruction (subtracting negative number from positive number should
        // produce a larger positive number, but will oveflow and wrap around, producing an
        // incorrect negative number)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let sec_opcode = 0x38;
        let sbc_immediate_addr_mode_opcode = 0xE9;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            sec_opcode,
            sbc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        // Overflow `i8` and wrap around to a negative value
        let expected_result = 0b1000_0000;
        assert_eq!(expected_result as i8, cpu.register_a as i8);
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(is_overflow_flag_set, true);
    }

    #[test]
    fn sbc_immediate_addressing_mode_clears_carry_flag_if_unsigned_underflow() {
        let mut ram = [0x00; 2048];
        let register_a_value = 63u8;
        let memory_value = 64u8;

        // Program does the following:
        // - load value into register A
        // - set carry flag (for use in SBC instruction)
        // - execute SBC instruction (subtracting a positive number from a smaller positive number
        // would underflow the range of `u8`)
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let sec_opcode = 0x38;
        let sbc_immediate_addr_mode_opcode = 0xE9;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            sec_opcode,
            sbc_immediate_addr_mode_opcode,
            memory_value,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        // Underflow `u8` and wrap around to the top of `u8` range
        let expected_result = 0b1111_1111;
        assert_eq!(expected_result, cpu.register_a);
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(is_carry_flag_set, false);
    }

    #[test]
    fn txa_sets_register_a_correctly() {
        let mut ram = [0x00; 2048];
        let value = 0x13;

        // Program does the following:
        // - load value into register X
        // - execute TXA instruction
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let txa_opcode = 0x8A;
        let program = vec![ldx_immediate_addr_mode_opcode, value, txa_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(value, cpu.register_a);
    }

    #[test]
    fn tya_sets_register_a_value_correctly() {
        let mut ram = [0x00; 2048];
        let value = 0x4A;

        // Program does the following:
        // - load value into register Y
        // - execute TYA instruction
        // - break
        let ldy_immediate_addr_mode_opcode = 0xA0;
        let tya_opcode = 0x98;
        let program = vec![ldy_immediate_addr_mode_opcode, value, tya_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(value, cpu.register_a);
    }

    #[test]
    fn tay_sets_register_y_value_correctly() {
        let mut ram = [0x00; 2048];
        let value = 0x6B;

        // Program does the following:
        // - load value into register A
        // - execute TAY instruction
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let tay_opcode = 0xA8;
        let program = vec![lda_immediate_addr_mode_opcode, value, tay_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(value, cpu.register_y);
    }

    #[test]
    fn bvc_doesnt_modify_program_counter_if_overflow_flag_set() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = 0x10;
        let register_a_value = 0b0011_1111; // 63 in two's complement representation
        let memory_value = 0b0100_0001; // 65 in two's complement representation

        // Program does the following:
        // - load value into register A
        // - perform addition that sets overflow flag (which affects how the BVC instruction
        // operates)
        // - execute BVC instruction
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let adc_immediate_addr_mode_opcode = 0x69;
        let bvc_opcode = 0x50;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            adc_immediate_addr_mode_opcode,
            memory_value,
            bvc_opcode,
            offset,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions = program.len() as u16;
        cpu.load_and_run();
        assert_eq!(
            program_counter_start + no_of_instructions,
            cpu.program_counter
        );
    }

    #[test]
    fn bvc_offsets_program_counter_correctly_if_overflow_flag_clear() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -16i8;

        // Program does the following:
        // - execute BVC instruction
        let bvc_opcode = 0x50;
        let program = vec![bvc_opcode, offset as u8];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();

        let no_of_instructions_before_offset_value = 1;
        let increment_reading_next_opcode_after_offsetting = 1;
        let expected_program_counter = program_counter_start as i32
            + no_of_instructions_before_offset_value
            + offset as i32
            + increment_reading_next_opcode_after_offsetting;
        assert_eq!(expected_program_counter as u16, cpu.program_counter);
    }

    #[test]
    fn bvs_doesnt_modify_program_counter_if_overflow_flag_clear() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -16i8;

        // Program does the following:
        // - execute BVS instruction
        // - break
        let bvs_opcode = 0x70;
        let program = vec![bvs_opcode, offset as u8, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        let no_of_instructions = program.len() as u16;
        cpu.load_and_run();
        assert_eq!(
            program_counter_start + no_of_instructions,
            cpu.program_counter
        );
    }

    #[test]
    fn bvs_offsets_program_counter_correctly_if_overflow_flag_set() {
        let mut ram = [0x00; 2048];
        let program_counter_start: u16 = 0x8000;
        let offset = -16i8;
        let register_a_value = 0b0011_1111; // 63 in two's complement representation
        let memory_value = 0b0100_0001; // 65 in two's complement representation

        // Program does the following:
        // - load value into register A
        // - perform addition that sets overflow flag (which affects how the BVS instruction
        // operates)
        // - execute BVS instruction
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let adc_immediate_addr_mode_opcode = 0x69;
        let bvs_opcode = 0x70;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            adc_immediate_addr_mode_opcode,
            memory_value,
            bvs_opcode,
            offset as u8,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();

        let no_of_instructions_before_offset_value = 5;
        let increment_reading_next_opcode_after_offsetting = 1;
        let expected_program_counter = program_counter_start as i32
            + no_of_instructions_before_offset_value
            + offset as i32
            + increment_reading_next_opcode_after_offsetting;
        assert_eq!(expected_program_counter as u16, cpu.program_counter);
    }

    #[test]
    fn clv_clears_overflow_flag() {
        let mut ram = [0x00; 2048];
        let register_a_value = 0b0011_1111; // 63 in two's complement representation
        let memory_value = 0b0100_0001; // 65 in two's complement representation

        // Program does the following:
        // - load value into register A
        // - perform addition that sets overflow flag
        // - execute CLV instruction
        // - break
        let lda_immediate_addr_mode_opcode = 0xA9;
        let adc_immediate_addr_mode_opcode = 0x69;
        let clv_opcode = 0xB8;
        let program = vec![
            lda_immediate_addr_mode_opcode,
            register_a_value,
            adc_immediate_addr_mode_opcode,
            memory_value,
            clv_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(false, is_overflow_flag_set);
    }

    #[test]
    fn nop_doesnt_affect_registers_or_flags() {
        let mut ram = [0x00; 2048];

        // Program does the following:
        // - execute the NOP instruction
        // - break
        let nop_opcode = 0xEA;
        let program = vec![nop_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();

        let program_counter_start: u16 = 0x8000;
        assert_eq!(cpu.program_counter, program_counter_start + 2);
        assert_eq!(cpu.register_a, 0x00);
        assert_eq!(cpu.register_x, 0x00);
        assert_eq!(cpu.register_y, 0x00);
        assert_eq!(cpu.stack_register, STACK_REGISTER_LO_START);
        let is_zero_flag_set = cpu.status & 0b000_0010 == 0b0000_0010;
        assert_eq!(false, is_zero_flag_set);
        let is_overflow_flag_set = cpu.status & 0b0100_0000 == 0b0100_0000;
        assert_eq!(false, is_overflow_flag_set);
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(false, is_negative_flag_set);
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(false, is_carry_flag_set);
    }

    #[test]
    fn rti_first_stack_pop_to_set_status_register() {
        let mut ram = [0x00; 2048];
        let register_x_value = 0xFC;
        let status_flags = 0b1000_0001;
        let status_flags_stack_addr =
            u16::from_le_bytes([STACK_REGISTER_LO_START - 2, STACK_REGISTER_HI]);
        ram[status_flags_stack_addr as usize] = status_flags;

        // Program does the following:
        // - load value into register X
        // - put register X value into stack register
        // - execute RTI instruction (prior to this instruction being executed, the top byte of the
        // stack will have a non-zero value in it, and the stack pointer will be decremented
        // accordingly)
        // - break
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let txs_opcode = 0x9A;
        let rti_opcode = 0x40;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            txs_opcode,
            rti_opcode,
            0x00,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        let is_carry_flag_set = cpu.status & 0b0000_0001 == 0b0000_0001;
        assert_eq!(true, is_carry_flag_set);
        let is_negative_flag_set = cpu.status & 0b1000_0000 == 0b1000_0000;
        assert_eq!(true, is_negative_flag_set);
    }

    #[test]
    fn rti_second_stack_pop_to_set_program_counter() {
        let mut ram = [0x00; 2048];
        let register_x_value = 0xFC;
        let new_program_counter_lo = 0x15;
        let new_program_counter_hi = 0xAB;
        let new_program_counter =
            u16::from_le_bytes([new_program_counter_lo, new_program_counter_hi]);
        ram[(u16::from_le_bytes([STACK_REGISTER_LO_START, STACK_REGISTER_HI])) as usize] =
            new_program_counter_hi;
        ram[(u16::from_le_bytes([STACK_REGISTER_LO_START - 1, STACK_REGISTER_HI])) as usize] =
            new_program_counter_lo;

        // Program does the following:
        // - load value into register X
        // - put register X value into stack register
        // - execute RTI instruction (prior to this instruction being executed, the penultimate two
        // bytes of the stack will have non-zero values in them, and the stack pointer will be
        // decremented accordingly)
        let ldx_immediate_addr_mode_opcode = 0xA2;
        let txs_opcode = 0x9A;
        let rti_opcode = 0x40;
        let program = vec![
            ldx_immediate_addr_mode_opcode,
            register_x_value,
            txs_opcode,
            rti_opcode,
        ];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);

        cpu.load_and_run();
        assert_eq!(new_program_counter + 1, cpu.program_counter);
    }

    #[test]
    fn sed_sets_decimal_flag() {
        let mut ram = [0x00; 2048];
        let sed_opcode = 0xF8;
        let program = vec![sed_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        let is_decimal_flag_set = cpu.status & 0b0000_1000 == 0b0000_1000;
        assert_eq!(true, is_decimal_flag_set);
    }

    #[test]
    fn cld_clears_decimal_flag() {
        let mut ram = [0x00; 2048];
        let sed_opcode = 0xF8;
        let cld_opcode = 0xD8;
        let program = vec![sed_opcode, cld_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        let is_decimal_flag_set = cpu.status & 0b0000_1000 == 0b0000_1000;
        assert_eq!(false, is_decimal_flag_set);
    }

    #[test]
    fn sei_sets_interrupt_disable_flag() {
        let mut ram = [0x00; 2048];
        let sei_opcode = 0x78;
        let program = vec![sei_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        let is_interrupt_disable_flag_set = cpu.status & 0b0000_0100 == 0b0000_0100;
        assert_eq!(true, is_interrupt_disable_flag_set);
    }

    #[test]
    fn cli_clears_interrupt_disable_flag() {
        let mut ram = [0x00; 2048];
        let sei_opcode = 0x78;
        let cli_opcode = 0x58;
        let program = vec![sei_opcode, cli_opcode, 0x00];
        let bus = Bus::new(&mut ram, create_rom(&program[..]));
        let mut cpu = CPU::new(bus);
        cpu.load_and_run();
        let is_interrupt_disable_flag_set = cpu.status & 0b0000_0100 == 0b0000_0100;
        assert_eq!(false, is_interrupt_disable_flag_set);
    }
}
