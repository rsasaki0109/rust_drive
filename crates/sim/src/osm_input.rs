//! Bound the bytes actually read, including inputs without useful file metadata.
use rustdriving_routing::osm::OverpassDocument;
use std::{error::Error, io::Read};

const MAX_BYTES: u64 = 16 * 1024 * 1024;

pub(super) fn read_document(reader: impl Read) -> Result<OverpassDocument, Box<dyn Error>> {
    let mut bytes = Vec::new();
    reader.take(MAX_BYTES + 1).read_to_end(&mut bytes)?;
    if bytes.len() as u64 > MAX_BYTES {
        return Err("OSM input exceeds 16 MiB".into());
    }
    Ok(serde_json::from_slice(&bytes)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{self, Cursor};

    #[test]
    fn endless_reader_is_stopped_after_one_byte_past_the_limit() {
        struct Endless {
            consumed: u64,
        }
        impl Read for Endless {
            fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
                buffer.fill(b' ');
                self.consumed += buffer.len() as u64;
                Ok(buffer.len())
            }
        }
        let mut reader = Endless { consumed: 0 };
        assert!(
            read_document(&mut reader)
                .unwrap_err()
                .to_string()
                .contains("exceeds 16 MiB")
        );
        assert_eq!(reader.consumed, MAX_BYTES + 1);
    }

    #[test]
    fn read_failure_after_a_complete_json_prefix_is_not_accepted() {
        struct FailedEof(Cursor<&'static [u8]>);
        impl Read for FailedEof {
            fn read(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
                if self.0.position() == self.0.get_ref().len() as u64 {
                    return Err(io::Error::other("injected input failure"));
                }
                self.0.read(buffer)
            }
        }
        let error = read_document(FailedEof(Cursor::new(b"{\"elements\":[]}"))).unwrap_err();
        assert_eq!(error.to_string(), "injected input failure");
        assert_eq!(
            error.downcast_ref::<io::Error>().unwrap().kind(),
            io::ErrorKind::Other
        );
    }
}
