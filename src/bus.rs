use crate::rom::Rom;

const RAM_SPACE_START: u16 = 0x0000;
const RAM_MIRRORS_SPACE_END: u16 = 0x1FFF;
const ROM_SPACE_START: u16 = 0x8000;
const ROM_SPACE_END: u16 = 0xFFFF;

pub struct Bus<'a> {
    ram: &'a mut [u8],
    rom: Rom,
}

impl<'a> Bus<'a> {
    pub fn new(ram: &'a mut [u8], rom: Rom) -> Bus {
        Bus { ram, rom }
    }

    fn read_prg_rom(&self, addr: u16) -> u8 {
        let offsetted_addr = addr - 0x8000;
        self.rom.prg_rom[offsetted_addr as usize]
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        match addr {
            RAM_SPACE_START..=RAM_MIRRORS_SPACE_END => {
                // Zero out bits 12 and 13 to achieve the mapping of the following regions of addresses to
                // the RAM array (often called "mirroring"):
                // - [0x0800] to[0x0FFF]
                // - [0x1000] to[0x17FF]
                // - [0x1800] to[0x1FFF]
                let mirrored_addr = addr & 0b0000_0111_1111_1111;
                self.ram[mirrored_addr as usize]
            }
            ROM_SPACE_START..=ROM_SPACE_END => self.read_prg_rom(addr),
            _ => todo!(),
        }
    }

    /// Read `u16` value stored in little-endian format, from two contiguous memory addresses each
    /// storing a single `u8` value
    pub fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos);
        let hi = self.mem_read(pos + 1);
        u16::from_le_bytes([lo, hi])
    }

    pub fn mem_write(&mut self, addr: u16, data: u8) {
        match addr {
            RAM_SPACE_START..=RAM_MIRRORS_SPACE_END => {
                // Zero out bits 12 and 13 to achieve the mapping of the following regions of addresses to
                // the RAM array (often called "mirroring"):
                // - [0x0800] to[0x0FFF]
                // - [0x1000] to[0x17FF]
                // - [0x1800] to[0x1FFF]
                let mirrored_addr = addr & 0b0000_0111_1111_1111;
                self.ram[mirrored_addr as usize] = data;
            }
            ROM_SPACE_START..=ROM_SPACE_END => panic!("ROM space addresses are read-only"),
            _ => todo!(),
        }
    }

    /// Write `u16` value to two contiguous memory addresses, in little-endian format
    pub fn mem_write_u16(&mut self, pos: u16, data: u16) {
        let bytes = u16::to_le_bytes(data);
        self.mem_write(pos, bytes[0]);
        self.mem_write(pos + 1, bytes[1]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // TODO: Duplicate of private binding in `rom.rs`, think about if that should be made public
    // for being reusable in this test module or not (or do something better)
    static ROM_HEADER_MAGIC_STRING: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];

    fn create_rom() -> Rom {
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
        let program_rom_data = [0x00; 0x8000];
        data.append(&mut program_rom_data.to_vec());
        let chr_rom_data = [0x02; 0x1FFF];
        data.append(&mut chr_rom_data.to_vec());
        Rom::new(&data[..])
    }

    #[test]
    fn mem_read_returns_correct_value() {
        let mut ram = [0x00; 2048];
        let value = 0x15;
        let addr = 0x0F;
        ram[addr as usize] = value;
        let bus = Bus::new(&mut ram, create_rom());
        let read_value = bus.mem_read(addr);
        assert_eq!(value, read_value);
    }

    #[test]
    fn mem_read_mirrored_address_returns_correct_value() {
        let mut ram = [0x00; 2048];
        let value = 0x15;
        let addr = 0x0F;
        let mirrored_addr = addr + 0x0800;
        ram[addr as usize] = value;
        let bus = Bus::new(&mut ram, create_rom());
        let read_value = bus.mem_read(mirrored_addr);
        assert_eq!(value, read_value);
    }

    #[test]
    fn mem_read_u16_returns_correct_value() {
        let mut ram = [0x00; 2048];
        let lo = 0x05;
        let hi = 0xFA;
        let expected_value = u16::from_le_bytes([lo, hi]);
        let addr = 0x24;
        ram[addr as usize] = lo;
        ram[(addr + 1) as usize] = hi;
        let bus = Bus::new(&mut ram, create_rom());
        let read_value = bus.mem_read_u16(addr);
        assert_eq!(expected_value, read_value);
    }

    #[test]
    fn mem_write_puts_correct_value_in_correct_place() {
        let mut ram = [0x00; 2048];
        let value = 0xFA;
        let addr = 0x25;
        let mut bus = Bus::new(&mut ram, create_rom());
        bus.mem_write(addr, value);
        assert_eq!(value, bus.mem_read(addr));
    }

    #[test]
    fn mem_write_mirrored_address_puts_correct_value_in_correct_place() {
        let mut ram = [0x00; 2048];
        let value = 0xFA;
        let addr = 0x25;
        let mirrored_addr = addr + 0x0800;
        let mut bus = Bus::new(&mut ram, create_rom());
        bus.mem_write(mirrored_addr, value);
        assert_eq!(value, bus.mem_read(addr));
    }

    #[test]
    fn mem_write_u16_puts_correct_value_in_correct_place() {
        let mut ram = [0x00; 2048];
        let lo = 0x05;
        let hi = 0xFA;
        let value = u16::from_le_bytes([lo, hi]);
        let addr = 0x25;
        let mut bus = Bus::new(&mut ram, create_rom());
        bus.mem_write_u16(addr, value);
        assert_eq!(value, bus.mem_read_u16(addr));
    }

    #[test]
    #[should_panic(expected = "ROM space addresses are read-only")]
    fn panic_if_attempt_write_to_rom_space() {
        let mut ram = [0x00; 2048];
        let addr_in_rom_space = 0x8000;
        let mut bus = Bus::new(&mut ram, create_rom());
        bus.mem_write(addr_in_rom_space, 0x00);
    }
}
