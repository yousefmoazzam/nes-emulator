pub struct Bus<'a> {
    ram: &'a [u8],
}

impl<'a> Bus<'a> {
    pub fn new(ram: &'a [u8]) -> Bus {
        Bus { ram }
    }

    pub fn mem_read(&self, addr: u16) -> u8 {
        self.ram[addr as usize]
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
}
