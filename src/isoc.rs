//! Pure-Rust isochronous USB streaming using libusb1-sys directly.
//!
//! Replaces the isoc transfer management from `usb_libusb10.c`.

use std::os::raw::{c_int, c_uint, c_void};
use std::slice;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use libusb1_sys::libusb_transfer;

use crate::usb::UsbDevice;
use crate::Error;

/// Transfers in flight per isochronous stream.
pub const NUM_XFERS: usize = 15;
/// Packets per transfer.
pub const PKTS_PER_XFER: usize = 16;
/// Depth packet buffer size.
pub const DEPTH_PKTBUF: usize = 1760;
/// Video packet buffer size.
pub const VIDEO_PKTBUF: usize = 1920;

/// State accessed exclusively by the libusb callback and managed by IsoStream.
pub struct StreamState {
    pub callback: Box<dyn FnMut(&[u8]) + 'static>,
    pub dead: AtomicBool,
    pub dead_xfers: AtomicUsize,
    pub pkts: usize,
    pub pkt_len: usize,
}

/// An in-flight isochronous stream.
pub struct IsoStream {
    pub endpoint: u8,
    pub num_xfers: usize,
    pub pkts: usize,
    pub pkt_len: usize,
    buffer: Vec<u8>,
    transfers: Vec<*mut libusb_transfer>,
    state: *mut StreamState,
}

impl IsoStream {
    /// Start an isochronous stream on the given device.
    pub fn new(
        device: &mut UsbDevice,
        endpoint: u8,
        pkts: usize,
        pkt_len: usize,
        num_xfers: usize,
        callback: Box<dyn FnMut(&[u8]) + 'static>,
    ) -> Result<Self, Error> {
        let total_buf = num_xfers * pkts * pkt_len;
        let mut buffer = vec![0u8; total_buf];
        let mut transfers = Vec::with_capacity(num_xfers);

        let raw_handle = device.handle.as_raw();

        let state = Box::into_raw(Box::new(StreamState {
            callback,
            dead: AtomicBool::new(false),
            dead_xfers: AtomicUsize::new(0),
            pkts,
            pkt_len,
        }));

        for i in 0..num_xfers {
            let buf_offset = i * pkts * pkt_len;
            let buf_ptr = unsafe { buffer.as_mut_ptr().add(buf_offset) };

            let xfer = unsafe { libusb1_sys::libusb_alloc_transfer(pkts as c_int) };
            if xfer.is_null() {
                for &t in &transfers {
                    unsafe { libusb1_sys::libusb_free_transfer(t) };
                }
                unsafe { drop(Box::from_raw(state)) };
                return Err(Error::Usb("failed to allocate libusb transfer".to_string()));
            }

            unsafe {
                libusb1_sys::libusb_fill_iso_transfer(
                    xfer,
                    raw_handle,
                    endpoint,
                    buf_ptr,
                    (pkts * pkt_len) as c_int,
                    pkts as c_int,
                    iso_callback,
                    state as *mut c_void,
                    0,
                );
                libusb1_sys::libusb_set_iso_packet_lengths(xfer, pkt_len as c_uint);
            }

            transfers.push(xfer);
        }

        let stream = IsoStream {
            endpoint,
            num_xfers,
            pkts,
            pkt_len,
            buffer,
            transfers,
            state,
        };

        for &xfer in &stream.transfers {
            let ret = unsafe { libusb1_sys::libusb_submit_transfer(xfer) };
            if ret < 0 {
                eprintln!("Warning: failed to submit isoc transfer: {}", ret);
                unsafe {
                    (*stream.state).dead_xfers.fetch_add(1, Ordering::Relaxed);
                }
            }
        }

        Ok(stream)
    }

    /// Stop the stream and clean up all transfers.
    ///
    /// Transfer scheduling strategy:
    /// 1. Mark stream dead so callbacks won't resubmit.
    /// 2. Cancel all pending transfers via libusb.
    /// 3. Wait for all transfers to return (either completed+cancelled, or just cancelled).
    /// 4. Free all transfer structures.
    pub fn stop(&mut self, ctx_raw: *mut libusb1_sys::libusb_context) {
        if self.state.is_null() {
            return;
        }

        unsafe {
            (*self.state).dead.store(true, Ordering::Relaxed);
        }

        // Cancel all pending transfers. This is safe even if a transfer has
        // already completed naturally; libusb_cancel_transfer just returns an error.
        for &xfer in &self.transfers {
            unsafe {
                let _ = libusb1_sys::libusb_cancel_transfer(xfer);
            }
        }

        // Spin until every transfer has come back through the callback.
        let mut loops = 0;
        while unsafe { (*self.state).dead_xfers.load(Ordering::Relaxed) } < self.num_xfers {
            let tv = libc::timeval {
                tv_sec: 0,
                tv_usec: 100_000,
            };
            unsafe {
                let _ = libusb1_sys::libusb_handle_events_timeout(ctx_raw, &tv);
            }
            loops += 1;
            if loops > 100 {
                eprintln!(
                    "Warning: IsoStream::stop() timeout ({} dead of {})",
                    unsafe { (*self.state).dead_xfers.load(Ordering::Relaxed) },
                    self.num_xfers
                );
                break;
            }
        }

        for &xfer in &self.transfers {
            unsafe {
                libusb1_sys::libusb_free_transfer(xfer);
            }
        }
        self.transfers.clear();

        unsafe {
            drop(Box::from_raw(self.state));
        }
        self.state = std::ptr::null_mut();
    }
}

impl Drop for IsoStream {
    fn drop(&mut self) {
        if !self.transfers.is_empty() {
            eprintln!("Warning: IsoStream dropped without calling stop() - leaking transfers");
        }
        if !self.state.is_null() {
            eprintln!("Warning: IsoStream dropped without calling stop() - leaking state");
        }
    }
}

extern "system" fn iso_callback(xfer: *mut libusb_transfer) {
    unsafe {
        let state_ptr = (*xfer).user_data as *mut StreamState;
        if state_ptr.is_null() {
            return;
        }

        let is_dead = (*state_ptr).dead.load(Ordering::Relaxed);
        if is_dead {
            // Stream is shutting down; count this xfer as dead and do NOT resubmit.
            (*state_ptr).dead_xfers.fetch_add(1, Ordering::Relaxed);
            return;
        }

        let status_int = (*xfer).status as c_int;

        if status_int == libusb1_sys::constants::LIBUSB_TRANSFER_COMPLETED {
            let buf = slice::from_raw_parts((*xfer).buffer, (*xfer).length as usize);
            let pkt_len = (*state_ptr).pkt_len;
            let num_pkts = (*xfer).num_iso_packets as usize;

            let mut total_actual = 0;
            let mut error_pkts = 0;
            for i in 0..num_pkts {
                let pkt_desc = &*((*xfer).iso_packet_desc.as_ptr().add(i));
                let actual = pkt_desc.actual_length as usize;
                let status = pkt_desc.status;
                if actual > 0 {
                    total_actual += actual;
                    let offset = i * pkt_len;
                    if offset + actual <= buf.len() {
                        let packet_data = &buf[offset..offset + actual];
                        ((*state_ptr).callback)(packet_data);
                    }
                } else if status != libusb1_sys::constants::LIBUSB_TRANSFER_COMPLETED {
                    error_pkts += 1;
                }
            }

            // After processing, check if the stream is dead BEFORE resubmitting.
            // This prevents a race where the stream is marked dead while we're
            // in the middle of processing.
            if (*state_ptr).dead.load(Ordering::Relaxed) {
                (*state_ptr).dead_xfers.fetch_add(1, Ordering::Relaxed);
                return;
            }

            // Resubmit for the next round
            let ret = libusb1_sys::libusb_submit_transfer(xfer);
            if ret != 0 {
                (*state_ptr).dead_xfers.fetch_add(1, Ordering::Relaxed);
            }
        } else {
            // Any non-COMPLETED status (CANCELLED, NO_DEVICE, ERROR, etc.)
            // means this transfer is done for good.
            (*state_ptr).dead_xfers.fetch_add(1, Ordering::Relaxed);
        }
    }
}
