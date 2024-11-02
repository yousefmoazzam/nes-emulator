static HEADER_MAGIC_STRING: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];

pub struct Rom {}

impl Rom {
    pub fn new(data: &[u8]) {
        let header_beginning_string = &data[0..4];
        if header_beginning_string != &HEADER_MAGIC_STRING[..] {
            panic!("Unexpected file format (missing 'NES^Z' string at beginning of header)");
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
}
