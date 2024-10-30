pub struct Bus<'a> {
    ram: &'a [u8],
}

impl<'a> Bus<'a> {
    pub fn new(ram: &'a [u8]) -> Bus {
        Bus { ram }
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        // Zero out bits 12 and 13 to achieve the mapping of the following regions of addresses to
        // the RAM array (often called "mirroring"):
        // - [0x0800] to[0x0FFF]
        // - [0x1000] to[0x17FF]
        // - [0x1800] to[0x1FFF]
        let mirrored_addr = addr & 0b0000_0111_1111_1111;
        self.ram[mirrored_addr as usize]
    }

    /// Read `u16` value stored in little-endian format, from two contiguous memory addresses each
    /// storing a single `u8` value
    pub fn mem_read_u16(&self, pos: u16) -> u16 {
        let lo = self.mem_read(pos);
        let hi = self.mem_read(pos + 1);
        u16::from_le_bytes([lo, hi])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mem_read_returns_correct_value() {
        let mut ram = [0x00; 2048];
        let value = 0x15;
        let addr = 0x0F;
        ram[addr as usize] = value;
        let bus = Bus::new(&ram);
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
        let bus = Bus::new(&ram);
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
        let bus = Bus::new(&ram);
        let read_value = bus.mem_read_u16(addr);
        assert_eq!(expected_value, read_value);
    }
}
