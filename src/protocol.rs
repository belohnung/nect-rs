//! Pure-Rust helpers for the Kinect camera command protocol.
//!
//! This handles the framing used by `send_cmd` / `read_register` / `write_register`
//! before requests are sent over USB control transfers.

/// Camera command header (8 bytes) used for register read/write.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CamCmdHeader {
    pub magic: [u8; 2], // 0x47, 0x4d ('G', 'M')
    pub len: u16,       // Payload length in u16 words
    pub cmd: u16,       // Command opcode (0x02=read, 0x03=write, 0x95=cmos)
    pub tag: u16,       // Sequence tag
}

impl CamCmdHeader {
    pub const SIZE: usize = 8;

    pub fn new(cmd: u16, tag: u16, payload_len_bytes: usize) -> Self {
        CamCmdHeader {
            magic: [0x47, 0x4d],
            len: (payload_len_bytes / 2) as u16,
            cmd,
            tag,
        }
    }

    pub fn to_bytes(&self) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0..2].copy_from_slice(&self.magic);
        buf[2..4].copy_from_slice(&self.len.to_le_bytes());
        buf[4..6].copy_from_slice(&self.cmd.to_le_bytes());
        buf[6..8].copy_from_slice(&self.tag.to_le_bytes());
        buf
    }

    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 8 {
            return None;
        }
        Some(CamCmdHeader {
            magic: [data[0], data[1]],
            len: u16::from_le_bytes([data[2], data[3]]),
            cmd: u16::from_le_bytes([data[4], data[5]]),
            tag: u16::from_le_bytes([data[6], data[7]]),
        })
    }
}

/// Reply header returned by the camera.
#[derive(Debug, Clone, Copy)]
pub struct CamReplyHeader {
    pub magic: [u8; 2], // Expected: 0x52, 0x42 ('R', 'B')
    pub len: u16,
    pub cmd: u16,
    pub tag: u16,
}

impl CamReplyHeader {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 8 {
            return None;
        }
        Some(CamReplyHeader {
            magic: [data[0], data[1]],
            len: u16::from_le_bytes([data[2], data[3]]),
            cmd: u16::from_le_bytes([data[4], data[5]]),
            tag: u16::from_le_bytes([data[6], data[7]]),
        })
    }

    pub fn is_valid(&self, expected_cmd: u16, expected_tag: u16) -> bool {
        self.magic[0] == 0x52
            && self.magic[1] == 0x42
            && self.cmd == expected_cmd
            && self.tag == expected_tag
    }
}

/// Build a camera read-register command payload.
pub fn read_register_cmd(reg: u16) -> [u8; 2] {
    reg.to_le_bytes()
}

/// Build a camera write-register command payload.
pub fn write_register_cmd(reg: u16, value: u16) -> [u8; 4] {
    let mut buf = [0u8; 4];
    buf[0..2].copy_from_slice(&reg.to_le_bytes());
    buf[2..4].copy_from_slice(&value.to_le_bytes());
    buf
}

/// Build a CMOS read command payload.
pub fn read_cmos_cmd(reg: u16) -> [u8; 6] {
    let mut buf = [0u8; 6];
    buf[0..2].copy_from_slice(&1u16.to_le_bytes());
    buf[2..4].copy_from_slice(&(reg & 0x7fff).to_le_bytes());
    buf[4..6].copy_from_slice(&0u16.to_le_bytes());
    buf
}

/// Build a CMOS write command payload.
pub fn write_cmos_cmd(reg: u16, value: u16) -> [u8; 6] {
    let mut buf = [0u8; 6];
    buf[0..2].copy_from_slice(&1u16.to_le_bytes());
    buf[2..4].copy_from_slice(&(reg | 0x8000).to_le_bytes());
    buf[4..6].copy_from_slice(&value.to_le_bytes());
    buf
}

/// Assemble a full command packet (header + payload).
pub fn assemble_cmd_packet(cmd: u16, tag: u16, payload: &[u8]) -> Vec<u8> {
    assert!(payload.len() % 2 == 0);
    let hdr = CamCmdHeader::new(cmd, tag, payload.len());
    let mut packet = Vec::with_capacity(8 + payload.len());
    packet.extend_from_slice(&hdr.to_bytes());
    packet.extend_from_slice(payload);
    packet
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cam_cmd_header_roundtrip() {
        let hdr = CamCmdHeader::new(0x03, 42, 4);
        let bytes = hdr.to_bytes();
        let hdr2 = CamCmdHeader::from_bytes(&bytes).unwrap();
        assert_eq!(hdr2.magic, [0x47, 0x4d]);
        assert_eq!(hdr2.cmd, 0x03);
        assert_eq!(hdr2.tag, 42);
        assert_eq!(hdr2.len, 2);
    }

    #[test]
    fn test_read_register_cmd() {
        let cmd = read_register_cmd(0x0015);
        assert_eq!(cmd, [0x15, 0x00]);
    }

    #[test]
    fn test_write_register_cmd() {
        let cmd = write_register_cmd(0x0015, 0x0007);
        assert_eq!(cmd, [0x15, 0x00, 0x07, 0x00]);
    }

    #[test]
    fn test_cmos_cmd_builders() {
        let read = read_cmos_cmd(0x0106);
        assert_eq!(read[0..2], [0x01, 0x00]); // 1
        assert_eq!(read[2..4], [0x06, 0x01]); // 0x0106 & 0x7fff = 0x0106
        assert_eq!(read[4..6], [0x00, 0x00]); // 0

        let write = write_cmos_cmd(0x0106, 0x1234);
        assert_eq!(write[2..4], [0x06, 0x81]); // 0x0106 | 0x8000 = 0x8106 (LE)
        assert_eq!(write[4..6], [0x34, 0x12]);
    }

    #[test]
    fn test_assemble_cmd_packet() {
        let payload = write_register_cmd(0x0015, 0x0007);
        let pkt = assemble_cmd_packet(0x03, 1, &payload);
        assert_eq!(pkt.len(), 12);
        assert_eq!(pkt[0..2], [0x47, 0x4d]);
        assert_eq!(pkt[4..6], [0x03, 0x00]); // cmd
    }
}
