//! Bridge between isochronous packets and frame assembly.
//!
//! Wraps `stream::PacketStream` to process raw USB packets into
//! complete frames, invoking a user callback on each assembled frame.

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::stream::PacketStream;

/// Callback state for frame assembly.
pub struct FrameState {
    pub packet_stream: PacketStream,
    pub callback: Box<dyn FnMut(&[u8], u32) + 'static>,
    pub dead: AtomicBool,
    pub dead_xfers: AtomicUsize,
}

impl FrameState {
    pub fn new(
        stream_flag: u8,
        pkt_size: usize,
        frame_size: usize,
        callback: Box<dyn FnMut(&[u8], u32) + 'static>,
    ) -> Self {
        FrameState {
            packet_stream: PacketStream::new(stream_flag, pkt_size, frame_size),
            callback,
            dead: AtomicBool::new(false),
            dead_xfers: AtomicUsize::new(0),
        }
    }

    /// Process a raw USB isoc packet. Returns true if the stream is still alive.
    pub fn process_packet(&mut self, packet: &[u8]) -> bool {
        let result = self.packet_stream.process_packet(packet);
        if let Some(frame_size) = result {
            let ts = self.packet_stream.timestamp;
            let frame = &self.packet_stream.raw_buf[..frame_size];
            (self.callback)(frame, ts);
        }
        !self.dead.load(Ordering::Relaxed)
    }
}
