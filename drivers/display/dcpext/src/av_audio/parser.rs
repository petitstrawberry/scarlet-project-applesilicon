const OSSERIALIZE_MAGIC: u32 = 0xd3;
const OS_TYPE_DICTIONARY: u8 = 1;
const OS_TYPE_ARRAY: u8 = 2;
const OS_TYPE_INT64: u8 = 4;
const OS_TYPE_STRING: u8 = 9;
const OS_TYPE_BLOB: u8 = 10;
const OS_TYPE_BOOL: u8 = 11;

/// Opaque mode token returned by the display sink through DCP.
pub type DcpAvAudioCookie = [u8; 24];

#[derive(Clone, Copy)]
struct OsTag {
    size: usize,
    kind: u8,
}

struct OsObjectParser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> OsObjectParser<'a> {
    fn new(bytes: &'a [u8]) -> Result<Self, &'static str> {
        let magic = bytes
            .get(0..4)
            .and_then(|bytes| bytes.try_into().ok())
            .map(u32::from_le_bytes)
            .ok_or("apple-dcpext: truncated audio elements header")?;
        if magic != OSSERIALIZE_MAGIC {
            return Err("apple-dcpext: invalid audio elements serialization");
        }
        Ok(Self { bytes, pos: 4 })
    }

    fn align(&mut self) -> Result<(), &'static str> {
        self.pos = self
            .pos
            .checked_add(3)
            .ok_or("apple-dcpext: audio elements cursor overflow")?
            & !3;
        if self.pos > self.bytes.len() {
            return Err("apple-dcpext: truncated audio elements padding");
        }
        Ok(())
    }

    fn bytes(&mut self, count: usize) -> Result<&'a [u8], &'static str> {
        let end = self
            .pos
            .checked_add(count)
            .ok_or("apple-dcpext: audio elements cursor overflow")?;
        let bytes = self
            .bytes
            .get(self.pos..end)
            .ok_or("apple-dcpext: truncated audio elements object")?;
        self.pos = end;
        Ok(bytes)
    }

    fn tag(&mut self) -> Result<OsTag, &'static str> {
        self.align()?;
        let raw = u32::from_le_bytes(
            self.bytes(4)?
                .try_into()
                .map_err(|_| "apple-dcpext: invalid audio elements tag")?,
        );
        if (raw >> 29) & 0x3 != 0 {
            return Err("apple-dcpext: unsupported audio elements tag padding");
        }
        Ok(OsTag {
            size: (raw & 0x00ff_ffff) as usize,
            kind: ((raw >> 24) & 0x1f) as u8,
        })
    }

    fn expect_tag(&mut self, kind: u8) -> Result<OsTag, &'static str> {
        let tag = self.tag()?;
        if tag.kind != kind {
            return Err("apple-dcpext: unexpected audio elements object type");
        }
        Ok(tag)
    }

    fn string(&mut self) -> Result<&'a [u8], &'static str> {
        let tag = self.expect_tag(OS_TYPE_STRING)?;
        self.bytes(tag.size)
    }

    fn int64(&mut self) -> Result<i64, &'static str> {
        let _ = self.expect_tag(OS_TYPE_INT64)?;
        Ok(i64::from_le_bytes(self.bytes(8)?.try_into().map_err(
            |_| "apple-dcpext: invalid audio elements integer",
        )?))
    }

    fn blob_cookie(&mut self) -> Result<DcpAvAudioCookie, &'static str> {
        let tag = self.expect_tag(OS_TYPE_BLOB)?;
        let blob = self.bytes(tag.size)?;
        let mut cookie = [0u8; 24];
        let cookie_len = cookie.len();
        cookie.copy_from_slice(
            blob.get(..cookie_len)
                .ok_or("apple-dcpext: short DCP audio cookie")?,
        );
        Ok(cookie)
    }

    fn skip(&mut self) -> Result<(), &'static str> {
        let tag = self.tag()?;
        match tag.kind {
            OS_TYPE_DICTIONARY => {
                let objects = tag
                    .size
                    .checked_mul(2)
                    .ok_or("apple-dcpext: audio dictionary size overflow")?;
                for _ in 0..objects {
                    self.skip()?;
                }
            }
            OS_TYPE_ARRAY => {
                for _ in 0..tag.size {
                    self.skip()?;
                }
            }
            OS_TYPE_INT64 => {
                self.bytes(8)?;
            }
            OS_TYPE_STRING | OS_TYPE_BLOB => {
                self.bytes(tag.size)?;
            }
            OS_TYPE_BOOL => {}
            _ => return Err("apple-dcpext: unsupported audio elements object type"),
        }
        Ok(())
    }
}

pub(crate) fn select_cookie(
    elements: &[u8],
    rate_hz: u32,
    sample_bits: u32,
    channels: u32,
) -> Result<DcpAvAudioCookie, &'static str> {
    if channels == 0 || channels > 16 {
        return Err("apple-dcpext: invalid DisplayPort audio channel count");
    }
    let mut parser = OsObjectParser::new(elements)?;
    let count = parser.expect_tag(OS_TYPE_ARRAY)?.size;
    for _ in 0..count {
        if let Some(cookie) = parse_audio_element(&mut parser, rate_hz, sample_bits, channels)? {
            return Ok(cookie);
        }
    }
    Err("apple-dcpext: requested DisplayPort audio mode is unsupported")
}

fn parse_audio_element(
    parser: &mut OsObjectParser<'_>,
    wanted_rate: u32,
    wanted_bits: u32,
    wanted_channels: u32,
) -> Result<Option<DcpAvAudioCookie>, &'static str> {
    let pairs = parser.expect_tag(OS_TYPE_DICTIONARY)?.size;
    let mut rate = None;
    let mut bits = None;
    let mut channel_layout_matches = false;
    let mut cookie = None;

    for _ in 0..pairs {
        let key = parser.string()?;
        match key {
            b"StreamSampleRate" => rate = u32::try_from(parser.int64()?).ok(),
            b"SampleSize" => bits = u32::try_from(parser.int64()?).ok(),
            b"AudioChannelLayoutElements" => {
                channel_layout_matches = parse_channel_layouts(parser, wanted_channels)?;
            }
            b"ElementData" => cookie = Some(parser.blob_cookie()?),
            _ => parser.skip()?,
        }
    }

    Ok(
        (rate == Some(wanted_rate) && bits == Some(wanted_bits) && channel_layout_matches)
            .then_some(cookie)
            .flatten(),
    )
}

fn parse_channel_layouts(
    parser: &mut OsObjectParser<'_>,
    wanted_channels: u32,
) -> Result<bool, &'static str> {
    let count = parser.expect_tag(OS_TYPE_ARRAY)?.size;
    let mut matches = false;
    for _ in 0..count {
        let pairs = parser.expect_tag(OS_TYPE_DICTIONARY)?.size;
        let mut active_channels = None;
        for _ in 0..pairs {
            let key = parser.string()?;
            if key == b"ActiveChannelCount" {
                active_channels = u32::try_from(parser.int64()?).ok();
            } else {
                parser.skip()?;
            }
        }
        matches |= active_channels == Some(wanted_channels);
    }
    Ok(matches)
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use self::alloc::vec::Vec;

    use super::*;

    fn align(bytes: &mut Vec<u8>) {
        while !bytes.len().is_multiple_of(4) {
            bytes.push(0);
        }
    }

    fn tag(bytes: &mut Vec<u8>, kind: u8, size: usize) {
        align(bytes);
        bytes.extend_from_slice(&(((kind as u32) << 24) | size as u32).to_le_bytes());
    }

    fn string(bytes: &mut Vec<u8>, value: &[u8]) {
        tag(bytes, OS_TYPE_STRING, value.len());
        bytes.extend_from_slice(value);
    }

    fn int64(bytes: &mut Vec<u8>, value: i64) {
        tag(bytes, OS_TYPE_INT64, 64);
        bytes.extend_from_slice(&value.to_le_bytes());
    }

    fn fixture(cookie: DcpAvAudioCookie) -> Vec<u8> {
        let mut bytes = OSSERIALIZE_MAGIC.to_le_bytes().to_vec();
        tag(&mut bytes, OS_TYPE_ARRAY, 1);
        tag(&mut bytes, OS_TYPE_DICTIONARY, 5);

        string(&mut bytes, b"StreamSampleRate");
        int64(&mut bytes, 48_000);
        string(&mut bytes, b"SampleSize");
        int64(&mut bytes, 16);

        string(&mut bytes, b"AudioChannelLayoutElements");
        tag(&mut bytes, OS_TYPE_ARRAY, 1);
        tag(&mut bytes, OS_TYPE_DICTIONARY, 2);
        string(&mut bytes, b"ActiveChannelCount");
        int64(&mut bytes, 2);
        string(&mut bytes, b"ChannelLayout");
        tag(&mut bytes, OS_TYPE_ARRAY, 2);
        string(&mut bytes, b"Front Left");
        string(&mut bytes, b"Front Right");

        string(&mut bytes, b"ElementData");
        tag(&mut bytes, OS_TYPE_BLOB, cookie.len());
        bytes.extend_from_slice(&cookie);

        // Exercise recursive skipping of an unknown dictionary value.
        string(&mut bytes, b"Ignored");
        tag(&mut bytes, OS_TYPE_DICTIONARY, 1);
        string(&mut bytes, b"Flag");
        tag(&mut bytes, OS_TYPE_BOOL, 1);
        bytes
    }

    #[test]
    fn selects_exact_stereo_cookie() {
        let expected = core::array::from_fn(|index| index as u8 ^ 0xa5);
        let elements = fixture(expected);
        assert_eq!(select_cookie(&elements, 48_000, 16, 2), Ok(expected));
        assert!(select_cookie(&elements, 48_000, 32, 2).is_err());
        assert!(select_cookie(&elements, 44_100, 16, 2).is_err());
        assert!(select_cookie(&elements, 48_000, 16, 6).is_err());
    }

    #[test]
    fn rejects_truncated_serialization() {
        let mut elements = fixture([0x5a; 24]);
        elements.truncate(elements.len() - 8);
        assert!(select_cookie(&elements, 48_000, 16, 2).is_err());
    }
}
