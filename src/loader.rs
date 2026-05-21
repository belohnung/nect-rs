//! Firmware image parsing and bootloader protocol helpers.

/// Kinect firmware image header (little-endian).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct FirmwareHeader {
    pub magic: u32,       // e.g. 0xCA77F00D for audios, 0xF00BACCA for 2BL
    pub ver_minor: u16,   // minor comes before major (LE quirk)
    pub ver_major: u16,
    pub ver_release: u16,
    pub ver_patch: u16,
    pub base_addr: u32,   // Load address (e.g. 0x80000 for audios)
    pub size: u32,        // Image size in bytes
    pub entry_addr: u32,  // Code entry point
}

impl FirmwareHeader {
    /// Parse a firmware header from raw bytes (must be at least 24 bytes).
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < 24 {
            return None;
        }
        Some(FirmwareHeader {
            magic: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            ver_minor: u16::from_le_bytes([buf[4], buf[5]]),
            ver_major: u16::from_le_bytes([buf[6], buf[7]]),
            ver_release: u16::from_le_bytes([buf[8], buf[9]]),
            ver_patch: u16::from_le_bytes([buf[10], buf[11]]),
            base_addr: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
            size: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            entry_addr: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
        })
    }

    /// Version string like "01.02.03.04".
    pub fn version_string(&self) -> String {
        format!(
            "{:02}.{:02}.{:02}.{:02}",
            self.ver_major, self.ver_minor, self.ver_release, self.ver_patch
        )
    }

    /// Validate that the header looks sane.
    pub fn is_valid(&self) -> bool {
        self.magic == 0xCA77F00D || self.magic == 0xF00BACCA
    }
}

/// Bootloader command sent to the audio/motor device.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BootloaderCommand {
    pub magic: u32,  // 0x06022009
    pub tag: u32,    // Sequence number
    pub bytes: u32,  // Payload bytes (for cmd 0x03) or buffer size (for cmd 0)
    pub cmd: u32,    // 0 = version query, 0x03 = write page, 0x04 = jump
    pub addr: u32,   // Target address (for cmd 0x03) or register (for cmd 0)
    pub unk: u32,    // Padding / reserved
}

impl BootloaderCommand {
    /// Build a "write page" command.
    pub fn write_page(tag: u32, bytes: u32, addr: u32) -> Self {
        BootloaderCommand {
            magic: 0x06022009,
            tag,
            bytes,
            cmd: 0x03,
            addr,
            unk: 0,
        }
    }

    /// Build a "jump to entry" command.
    pub fn jump(tag: u32, entry_addr: u32) -> Self {
        BootloaderCommand {
            magic: 0x06022009,
            tag,
            bytes: 0,
            cmd: 0x04,
            addr: entry_addr,
            unk: 0,
        }
    }

    /// Build a "get version string" command.
    pub fn get_version(tag: u32) -> Self {
        BootloaderCommand {
            magic: 0x06022009,
            tag,
            bytes: 0x60,
            cmd: 0,
            addr: 0x15,
            unk: 0,
        }
    }

    /// Serialize to 24 bytes (little-endian).
    pub fn to_bytes(&self) -> [u8; 24] {
        let mut buf = [0u8; 24];
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4..8].copy_from_slice(&self.tag.to_le_bytes());
        buf[8..12].copy_from_slice(&self.bytes.to_le_bytes());
        buf[12..16].copy_from_slice(&self.cmd.to_le_bytes());
        buf[16..20].copy_from_slice(&self.addr.to_le_bytes());
        buf[20..24].copy_from_slice(&self.unk.to_le_bytes());
        buf
    }
}

/// Bootloader status reply.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BootloaderStatus {
    pub magic: u32,   // Expected: 0x0A6FE000
    pub tag: u32,     // Should match command tag
    pub status: u32,  // 0 = success
}

impl BootloaderStatus {
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < 12 {
            return None;
        }
        Some(BootloaderStatus {
            magic: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            tag: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            status: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        })
    }

    pub fn is_valid(&self, expected_tag: u32) -> bool {
        self.magic == 0x0A6FE000 && self.tag == expected_tag
    }
}

/// CEMD loader command.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CemdLoaderCommand {
    pub magic: u32,
    pub tag: u32,
    pub arg1: u32,
    pub cmd: u32,
    pub arg2: u32,
    pub zeros: [u32; 8],
}

impl CemdLoaderCommand {
    pub fn start_upload(tag: u32) -> Self {
        CemdLoaderCommand {
            magic: 0x06022009,
            tag,
            arg1: 0,
            cmd: 0x00000133,
            arg2: 0x00064014,
            zeros: [0; 8],
        }
    }

    pub fn data_block(tag: u32, bytes: u32, addr: u32) -> Self {
        CemdLoaderCommand {
            magic: 0x06022009,
            tag,
            arg1: bytes,
            cmd: 0x00000134,
            arg2: addr,
            zeros: [0; 8],
        }
    }

    pub fn finish(tag: u32) -> Self {
        CemdLoaderCommand {
            magic: 0x06022009,
            tag,
            arg1: 0,
            cmd: 0x00000135,
            arg2: 0x00064000,
            zeros: [0; 8],
        }
    }

    pub fn to_bytes(&self) -> [u8; 52] {
        let mut buf = [0u8; 52];
        let mut off = 0;
        for v in [
            self.magic, self.tag, self.arg1, self.cmd, self.arg2,
            self.zeros[0], self.zeros[1], self.zeros[2], self.zeros[3],
            self.zeros[4], self.zeros[5], self.zeros[6], self.zeros[7],
        ] {
            buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
            off += 4;
        }
        buf
    }
}

/// Parse a firmware blob and return (header, remaining_data).
pub fn parse_firmware(data: &[u8]) -> Option<(FirmwareHeader, &[u8])> {
    let hdr = FirmwareHeader::from_bytes(data)?;
    if !hdr.is_valid() {
        return None;
    }
    if data.len() < 24 + hdr.size as usize {
        return None;
    }
    Some((hdr, &data[24..]))
}

/// Compute the list of page transfers for a firmware upload.
/// Returns Vec of (addr, data_slice) for each 0x4000-byte page.
pub fn firmware_page_splits<'a>(
    base_addr: u32,
    total_size: u32,
    data: &'a [u8],
) -> Vec<(u32, &'a [u8])> {
    let page_size: usize = 0x4000;
    let mut addr = base_addr;
    let mut offset: usize = 0;
    let total = total_size.min(data.len() as u32) as usize;
    let mut pages = Vec::new();

    while offset < total {
        let end = (offset + page_size).min(total);
        pages.push((addr, &data[offset..end]));
        addr += (end - offset) as u32;
        offset = end;
    }
    pages
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_firmware_header_parse() {
        let mut buf = vec![0u8; 24];
        buf[0..4].copy_from_slice(&0xCA77F00Du32.to_le_bytes());
        buf[4..6].copy_from_slice(&0x0001u16.to_le_bytes()); // minor
        buf[6..8].copy_from_slice(&0x0002u16.to_le_bytes()); // major
        buf[12..16].copy_from_slice(&0x00080000u32.to_le_bytes()); // base_addr
        buf[16..20].copy_from_slice(&0x00010000u32.to_le_bytes()); // size

        let hdr = FirmwareHeader::from_bytes(&buf).unwrap();
        assert!(hdr.is_valid());
        assert_eq!(hdr.magic, 0xCA77F00D);
        assert_eq!(hdr.base_addr, 0x80000);
        assert_eq!(hdr.size, 0x10000);
    }

    #[test]
    fn test_bootloader_command_serialize() {
        let cmd = BootloaderCommand::write_page(7, 0x4000, 0x80000);
        let bytes = cmd.to_bytes();
        assert_eq!(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]), 0x06022009);
        assert_eq!(u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]), 7);
        assert_eq!(u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]), 0x03);
    }

    #[test]
    fn test_bootloader_status_parse() {
        let mut buf = vec![0u8; 12];
        buf[0..4].copy_from_slice(&0x0A6FE000u32.to_le_bytes());
        buf[4..8].copy_from_slice(&42u32.to_le_bytes());
        let st = BootloaderStatus::from_bytes(&buf).unwrap();
        assert!(st.is_valid(42));
        assert!(!st.is_valid(43));
    }

    #[test]
    fn test_firmware_page_splits() {
        let data = vec![0u8; 0x8001]; // 2 pages + 1 byte
        let pages = firmware_page_splits(0x80000, 0x8001, &data);
        assert_eq!(pages.len(), 3);
        assert_eq!(pages[0].0, 0x80000);
        assert_eq!(pages[0].1.len(), 0x4000);
        assert_eq!(pages[1].0, 0x84000);
        assert_eq!(pages[1].1.len(), 0x4000);
        assert_eq!(pages[2].0, 0x88000);
        assert_eq!(pages[2].1.len(), 1);
    }

    #[test]
    fn test_parse_firmware() {
        let mut data = vec![0u8; 28];
        data[0..4].copy_from_slice(&0xCA77F00Du32.to_le_bytes());
        data[16..20].copy_from_slice(&4u32.to_le_bytes()); // size = 4
        let (hdr, rest) = parse_firmware(&data).unwrap();
        assert_eq!(hdr.size, 4);
        assert_eq!(rest.len(), 4);
    }
}
