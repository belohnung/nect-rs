//! Handles USB isochronous packet ordering, sync detection, lost-packet
//! recovery, and frame assembly for both depth and video streams.

/// 12-byte packet header prefixing every USB isoc packet.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct PacketHeader {
    pub magic: [u8; 2],     // 'R', 'B'
    pub pad: u8,
    pub flag: u8,
    pub unk1: u8,
    pub seq: u8,
    pub unk2: u8,
    pub unk3: u8,
    pub timestamp: u32,
}

impl PacketHeader {
    pub const SIZE: usize = 12;

    /// Parse from raw bytes (must be at least 12 bytes).
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(PacketHeader {
            magic: [buf[0], buf[1]],
            pad: buf[2],
            flag: buf[3],
            unk1: buf[4],
            seq: buf[5],
            unk2: buf[6],
            unk3: buf[7],
            timestamp: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        })
    }

    pub fn is_valid_magic(&self) -> bool {
        self.magic[0] == b'R' && self.magic[1] == b'B'
    }

    /// true if this is Start-of-Frame for the given stream flag.
    pub fn is_sof(&self, stream_flag: u8) -> bool {
        self.flag == (stream_flag | 1)
    }

    /// true if this is Middle-of-Frame.
    pub fn is_mof(&self, stream_flag: u8) -> bool {
        self.flag == (stream_flag | 2)
    }

    /// true if this is End-of-Frame.
    pub fn is_eof(&self, stream_flag: u8) -> bool {
        self.flag == (stream_flag | 5)
    }
}

/// State machine for reassembling a Kinect isochronous stream into frames.
pub struct PacketStream {
    pub running: bool,
    pub stream_flag: u8,
    pub synced: bool,
    pub seq: u8,
    pub pkt_num: usize,
    pub got_pkts: usize,
    pub valid_pkts: usize,
    pub pkts_per_frame: usize,
    pub pkt_size: usize,
    pub frame_size: usize,
    pub last_pkt_size: usize,
    pub variable_length: bool,
    pub valid_frames: usize,
    pub lost_pkts: usize,
    pub last_timestamp: u32,
    pub timestamp: u32,
    pub raw_buf: Vec<u8>,
    pub proc_buf: Vec<u8>,
}

impl PacketStream {
    /// Create a new stream reassembler.
    ///
    /// * `stream_flag` - 0x70 for depth, 0x80 for video
    /// * `pkt_size` - max payload per packet (DEPTH_PKTDSIZE = 1748, VIDEO_PKTDSIZE = 1908)
    /// * `frame_size` - total raw frame size in bytes
    pub fn new(stream_flag: u8, pkt_size: usize, frame_size: usize) -> Self {
        let last_pkt_size = if frame_size % pkt_size == 0 {
            pkt_size
        } else {
            frame_size % pkt_size
        };
        let pkts_per_frame = (frame_size + pkt_size - 1) / pkt_size;
        PacketStream {
            running: false,
            stream_flag,
            synced: false,
            seq: 0,
            pkt_num: 0,
            got_pkts: 0,
            valid_pkts: 0,
            pkts_per_frame,
            pkt_size,
            frame_size,
            last_pkt_size,
            variable_length: false,
            valid_frames: 0,
            lost_pkts: 0,
            last_timestamp: 0,
            timestamp: 0,
            raw_buf: vec![0u8; frame_size],
            proc_buf: vec![0u8; frame_size],
        }
    }

    /// Process a single USB isoc packet.
    ///
    /// Returns `Some(frame_size)` when a complete frame is assembled.
    /// The frame data is in `self.raw_buf`.
    pub fn process_packet(&mut self, pkt: &[u8]) -> Option<usize> {
        if pkt.len() < PacketHeader::SIZE {
            return None;
        }

        let hdr = PacketHeader::from_bytes(pkt)?;
        if !hdr.is_valid_magic() {
            return None;
        }

        let data = &pkt[PacketHeader::SIZE..];
        let datalen = data.len();

        // Sync detection: accept first valid packet, sync on SOF or EOF
        if !self.synced {
            if hdr.is_sof(self.stream_flag) {
                // Normal SOF sync
                self.synced = true;
                self.seq = hdr.seq.wrapping_add(1);
                self.pkt_num = 0;
                self.valid_pkts = 0;
                self.got_pkts = 0;
                let offset = 0;
                let copy_len = datalen.min(self.raw_buf.len());
                if copy_len > 0 {
                    self.raw_buf[offset..offset + copy_len].copy_from_slice(&data[..copy_len]);
                }
                self.pkt_num = 1;
                self.got_pkts = 1;
                self.last_timestamp = hdr.timestamp;
                if hdr.is_eof(self.stream_flag) {
                    let got = if self.variable_length {
                        offset + datalen
                    } else {
                        offset + self.last_pkt_size
                    };
                    self.pkt_num = 0;
                    self.valid_pkts = self.got_pkts;
                    self.got_pkts = 0;
                    self.timestamp = self.last_timestamp;
                    self.valid_frames += 1;
                    return Some(got);
                }
                return None;
            } else {
                // Not SOF; could be MOF or EOF from a frame already in progress.
                // Skip until we find an SOF to ensure clean frame alignment.
                return None;
            }
        }

        let mut got_frame_size: Option<usize> = None;

        // Handle lost packets
        if hdr.seq != self.seq {
            // Calculate lost packets (wrapping u8 arithmetic)
            let lost = hdr.seq.wrapping_sub(self.seq);
            self.lost_pkts += lost as usize;

            if lost > 5 || self.variable_length {
                self.synced = false;
                return None;
            }

            let left = self.pkts_per_frame.saturating_sub(self.pkt_num);
            if left <= lost as usize {
                // Frame boundary crossed by loss
                self.pkt_num = (lost as usize) - left;
                self.valid_pkts = self.got_pkts;
                self.got_pkts = 0;
                got_frame_size = Some(self.frame_size);
                self.timestamp = self.last_timestamp;
                self.valid_frames += 1;
            } else {
                self.pkt_num += lost as usize;
            }
        }

        let expected_pkt_size = if self.pkt_num == self.pkts_per_frame - 1 {
            self.last_pkt_size
        } else {
            self.pkt_size
        };

        // Flag consistency check
        if !self.variable_length {
            let sof = self.stream_flag | 1;
            let mof = self.stream_flag | 2;
            let eof = self.stream_flag | 5;
            let ok = (self.pkt_num == 0 && hdr.flag == sof)
                || (self.pkt_num == self.pkts_per_frame - 1 && hdr.flag == eof)
                || (self.pkt_num > 0 && self.pkt_num < self.pkts_per_frame - 1 && hdr.flag == mof);
            if !ok {
                self.synced = false;
                return got_frame_size;
            }
            // For the last packet, the Kinect may send more data than expected_pkt_size
            // (the firmware doesn't tightly pack the last packet). We clamp the copy
            // length below, but we don't drop the packet or lose sync.
            if datalen > expected_pkt_size && !hdr.is_eof(self.stream_flag) {
                return got_frame_size;
            }
        } else {
            let sof = self.stream_flag | 1;
            let ok = (self.pkt_num == 0 && hdr.flag == sof)
                || (self.pkt_num < self.pkts_per_frame && (hdr.flag == (self.stream_flag | 2) || hdr.flag == (self.stream_flag | 5)));
            if !ok {
                self.synced = false;
                return got_frame_size;
            }
            if datalen > expected_pkt_size {
                self.synced = false;
                return got_frame_size;
            }
            if datalen < expected_pkt_size && !hdr.is_eof(self.stream_flag) {
                self.synced = false;
                return got_frame_size;
            }
        }

        // Copy data into frame buffer
        let offset = self.pkt_num * self.pkt_size;
        let copy_len = datalen.min(self.raw_buf.len().saturating_sub(offset));
        if copy_len > 0 {
            self.raw_buf[offset..offset + copy_len].copy_from_slice(&data[..copy_len]);
        }

        self.pkt_num += 1;
        self.seq = hdr.seq.wrapping_add(1);
        self.got_pkts += 1;
        self.last_timestamp = hdr.timestamp;

        if hdr.is_eof(self.stream_flag) {
            if self.variable_length {
                got_frame_size = Some(offset + datalen);
            } else {
                got_frame_size = Some(offset + self.last_pkt_size);
            }
            self.pkt_num = 0;
            self.valid_pkts = self.got_pkts;
            self.got_pkts = 0;
            self.timestamp = self.last_timestamp;
            self.valid_frames += 1;
        }

        got_frame_size
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_packet(seq: u8, flag: u8, payload: &[u8]) -> Vec<u8> {
        let mut pkt = Vec::with_capacity(12 + payload.len());
        pkt.push(b'R');
        pkt.push(b'B');
        pkt.push(0); // pad
        pkt.push(flag);
        pkt.push(0); // unk1
        pkt.push(seq);
        pkt.push(0); // unk2
        pkt.push(0); // unk3
        pkt.extend_from_slice(&0u32.to_le_bytes()); // timestamp
        pkt.extend_from_slice(payload);
        pkt
    }

    #[test]
    fn test_stream_sync_and_frame() {
        let mut stream = PacketStream::new(0x70, 10, 30); // 3 packets per frame
        // SOF
        let r = stream.process_packet(&make_packet(0, 0x71, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]));
        assert!(r.is_none());
        // MOF
        let r = stream.process_packet(&make_packet(1, 0x72, &[11, 12, 13, 14, 15, 16, 17, 18, 19, 20]));
        assert!(r.is_none());
        // EOF
        let r = stream.process_packet(&make_packet(2, 0x75, &[21, 22, 23, 24, 25, 26, 27, 28, 29, 30]));
        assert_eq!(r, Some(30));
        assert_eq!(&stream.raw_buf[..30], &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30]);
    }

    #[test]
    fn test_invalid_magic_drops_packet() {
        let mut stream = PacketStream::new(0x70, 10, 30);
        let mut pkt = make_packet(0, 0x71, &[1; 10]);
        pkt[0] = b'X';
        let r = stream.process_packet(&pkt);
        assert!(r.is_none());
        assert!(!stream.synced);
    }

    #[test]
    fn test_lost_packet_recovery() {
        let mut stream = PacketStream::new(0x70, 10, 30);
        // SOF
        stream.process_packet(&make_packet(0, 0x71, &[1; 10]));
        // Skip seq 1 (lost), next is seq 2 with EOF
        let r = stream.process_packet(&make_packet(2, 0x75, &[3; 10]));
        // Should return a frame because loss crossed frame boundary
        assert!(r.is_some());
    }
}
