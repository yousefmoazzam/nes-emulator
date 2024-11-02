static HEADER_MAGIC_STRING: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];

pub struct Rom {}

impl Rom {
    pub fn new(data: &[u8]) {
        let header_beginning_string = &data[0..4];
        if header_beginning_string != &HEADER_MAGIC_STRING[..] {
            panic!("Unexpected file format (missing 'NES^Z' string at beginning of header)");
        }

        let control_byte_two_bit_three_set = &data[7] & 0b0000_1000 != 0;
        let control_byte_two_bit_two_set = &data[7] & 0b0000_0100 != 0;
        if control_byte_two_bit_three_set & control_byte_two_bit_two_set {
            panic!("Invalid iNES format version configuration");
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    #[should_panic(
        expected = "Unexpected file format (missing 'NES^Z' string at beginning of header)"
    )]
    fn panic_if_magic_string_at_start_of_header_is_missing() {
        let incorrect_string_ascii = vec![0x4E, 0x45, 0x53, 0x01];
        _ = Rom::new(&incorrect_string_ascii[..]);
    }

    #[test]
    #[should_panic(expected = "Invalid iNES format version configuration")]
    fn panic_if_both_bits_describing_ines_format_are_set() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_1111;
        let control_byte_two_with_invalid_ines_version = 0b1111_1100;
        let mut bytes_after_magic_string = vec![
            no_of_16kib_rom_banks,
            no_of_8kib_vrom_banks,
            control_byte_one,
            control_byte_two_with_invalid_ines_version,
        ];
        data.append(&mut bytes_after_magic_string);
        _ = Rom::new(&data[..]);
    }
}
