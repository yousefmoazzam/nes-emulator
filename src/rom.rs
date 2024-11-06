static HEADER_MAGIC_STRING: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
static HEADER_SIZE: usize = 16;
static TRAINER_SECTION_SIZE: usize = 512;
static PRG_ROM_PAGE_SIZE: usize = 0x8000;
static CHR_ROM_PAGE_SIZE: usize = 0x1FFF;

#[derive(Debug, PartialEq)]
enum ScreenMirroring {
    FourScreen,
    Vertical,
    Horizontal,
}

pub struct Rom {
    mapper: u8,
    screen_mirroring: ScreenMirroring,
    prg_rom: Vec<u8>,
    chr_rom: Vec<u8>,
}

impl Rom {
    pub fn new(data: &[u8]) -> Rom {
        let header_beginning_string = &data[0..4];
        if header_beginning_string != &HEADER_MAGIC_STRING[..] {
            panic!("Unexpected file format (missing 'NES^Z' string at beginning of header)");
        }

        let control_byte_two_bit_three_set = &data[7] & 0b0000_1000 != 0;
        let control_byte_two_bit_two_set = &data[7] & 0b0000_0100 != 0;
        if control_byte_two_bit_three_set & control_byte_two_bit_two_set {
            panic!("Invalid iNES format version configuration");
        }

        let is_ines_version_two = control_byte_two_bit_three_set & !control_byte_two_bit_two_set;
        if is_ines_version_two {
            panic!("iNES 2.0 isn't supprted");
        }

        let mapper_lower_bits = &data[7] >> 4;
        let mapper_upper_bits = &data[6] & 0b1111_0000;
        let mapper = mapper_lower_bits | mapper_upper_bits;

        let is_four_screen_mirroring = &data[6] & 0b0000_1000 != 0;
        let is_vertical_screen_mirroring = &data[6] & 0b0000_0001 != 0;
        let screen_mirroring = match (is_four_screen_mirroring, is_vertical_screen_mirroring) {
            // TODO: Single screen mirroring?
            (true, _) => ScreenMirroring::FourScreen,
            (false, true) => ScreenMirroring::Vertical,
            (false, false) => ScreenMirroring::Horizontal,
        };

        let includes_trainer_section = &data[6] & 0b0000_0100 != 0;
        let prg_rom_size = data[4] as usize * PRG_ROM_PAGE_SIZE;
        let prg_rom_start = HEADER_SIZE
            + if includes_trainer_section {
                TRAINER_SECTION_SIZE
            } else {
                0
            };
        let prg_rom_end = prg_rom_start + prg_rom_size;
        let prg_rom = data[prg_rom_start..prg_rom_end].to_vec();

        let chr_rom_size = data[4] as usize * CHR_ROM_PAGE_SIZE;
        let chr_rom_start = prg_rom_end;
        let chr_rom_end = chr_rom_start + chr_rom_size;
        let chr_rom = data[chr_rom_start..chr_rom_end].to_vec();

        Rom {
            mapper,
            screen_mirroring,
            prg_rom,
            chr_rom,
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

    #[test]
    #[should_panic(expected = "iNES 2.0 isn't supprted")]
    fn panic_if_ines_version_two_is_specified() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_1111;
        let control_byte_two_with_ines_version_two = 0b1111_1000;
        let mut bytes_after_magic_string = vec![
            no_of_16kib_rom_banks,
            no_of_8kib_vrom_banks,
            control_byte_one,
            control_byte_two_with_ines_version_two,
        ];
        data.append(&mut bytes_after_magic_string);
        _ = Rom::new(&data[..]);
    }

    #[test]
    fn set_mapper_type() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_1011;
        let control_byte_two = 0b1111_0000;
        let expected_mapper = 0b1111_1111;
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
        let rom = Rom::new(&data[..]);
        assert_eq!(expected_mapper, rom.mapper);
    }

    #[test]
    fn set_four_screen_mirroring() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_1000;
        let control_byte_two = 0b1111_0000;
        let expected_screen_mirroring = ScreenMirroring::FourScreen;
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
        let rom = Rom::new(&data[..]);
        assert_eq!(expected_screen_mirroring, rom.screen_mirroring);
    }

    #[test]
    fn set_vertical_screen_mirroring() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_0001;
        let control_byte_two = 0b1111_0000;
        let expected_screen_mirroring = ScreenMirroring::Vertical;
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
        let rom = Rom::new(&data[..]);
        assert_eq!(expected_screen_mirroring, rom.screen_mirroring);
    }

    #[test]
    fn set_horizontal_screen_mirroring() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_0000;
        let control_byte_two = 0b1111_0000;
        let expected_screen_mirroring = ScreenMirroring::Horizontal;
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
        let rom = Rom::new(&data[..]);
        assert_eq!(expected_screen_mirroring, rom.screen_mirroring);
    }

    #[test]
    fn program_rom_size_and_contents_without_trainer_section() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
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
        let program_rom_data = [0x01; 0x8000];
        data.append(&mut program_rom_data.to_vec());
        let chr_rom_data = [0x02; 0x1FFF];
        data.append(&mut chr_rom_data.to_vec());
        let rom = Rom::new(&data[..]);
        assert_eq!(program_rom_data.len(), rom.prg_rom.len());
        assert_eq!(&program_rom_data[..], &rom.prg_rom[..]);
    }

    #[test]
    fn program_rom_size_and_contents_with_trainer_section() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
        let no_of_16kib_rom_banks = 0x1;
        let no_of_8kib_vrom_banks = 0x1;
        let control_byte_one = 0b1111_0100;
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
        let trainer_section = [0x00; 512];
        data.append(&mut trainer_section.to_vec());
        let program_rom_data = [0x01; 0x8000];
        data.append(&mut program_rom_data.to_vec());
        let chr_rom_data = [0x02; 0x1FFF];
        data.append(&mut chr_rom_data.to_vec());
        let rom = Rom::new(&data[..]);
        assert_eq!(program_rom_data.len(), rom.prg_rom.len());
        assert_eq!(&program_rom_data[..], &rom.prg_rom[..]);
    }

    #[test]
    fn chr_rom_size_and_contents() {
        let mut data = HEADER_MAGIC_STRING.to_vec();
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
        let rom = Rom::new(&data[..]);
        assert_eq!(chr_rom_data.len(), rom.chr_rom.len());
        assert_eq!(&chr_rom_data[..], &rom.chr_rom[..]);
    }
}
