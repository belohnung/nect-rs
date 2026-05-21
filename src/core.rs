//! Context lifecycle, device list, event loop, and logging.

use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

use crate::usb::{KinectUsb, UsbDevice};
use crate::{DeviceFlags, Error, VideoFormat};

/// Log level for nect-rs messages.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Fatal = 0,
    Error = 1,
    Warning = 2,
    Notice = 3,
    Info = 4,
    Debug = 5,
    Spew = 6,
}

/// A callback for log messages.
pub type LogCallback = Box<dyn Fn(LogLevel, &str)>;

/// A callback for depth/video frames.
pub type FrameCallback = Box<dyn FnMut(&[u8], u32)>;
pub type SharedFrameCallback = Rc<RefCell<FrameCallback>>;

/// Per-device state managed by the Rust core.
pub struct DeviceState {
    pub usb_cam: Option<UsbDevice>,
    pub usb_motor: Option<UsbDevice>,
    pub serial: Option<String>,
    pub user_data: Cell<Option<usize>>, // opaque pointer stored as usize
    pub dead: Cell<bool>,
    pub video_running: Cell<bool>,
    pub depth_running: Cell<bool>,
    pub video_stream: Option<crate::isoc::IsoStream>,
    pub depth_stream: Option<crate::isoc::IsoStream>,
    pub depth_callback: Option<SharedFrameCallback>,
    pub video_callback: Option<SharedFrameCallback>,
    pub video_mode: Cell<crate::FrameMode>,
    pub depth_mode: Cell<crate::FrameMode>,
}

impl DeviceState {
    pub fn new() -> Self {
        DeviceState {
            usb_cam: None,
            usb_motor: None,
            serial: None,
            user_data: Cell::new(None),
            dead: Cell::new(false),
            video_running: Cell::new(false),
            depth_running: Cell::new(false),
            video_stream: None,
            depth_stream: None,
            depth_callback: None,
            video_callback: None,
            video_mode: Cell::new(crate::modes::VIDEO_MODES[1]),
            depth_mode: Cell::new(crate::modes::DEPTH_MODES[2]),
        }
    }
}

/// Context holds the libusb context and the list of open devices.
pub struct ContextCore {
    pub usb: KinectUsb,
    pub devices: Vec<DeviceState>,
    log_level: Cell<LogLevel>,
    log_cb: RefCell<Option<LogCallback>>,
    enabled_subdevices: Cell<DeviceFlags>,
}

impl ContextCore {
    pub fn new() -> Result<Self, Error> {
        let usb = KinectUsb::new().map_err(|e| {
            Error::Usb(format!("libusb init failed: {}", e))
        })?;

        Ok(ContextCore {
            usb,
            devices: Vec::new(),
            log_level: Cell::new(LogLevel::Notice),
            log_cb: RefCell::new(None),
            enabled_subdevices: Cell::new(DeviceFlags::MOTOR.union(DeviceFlags::CAMERA)),
        })
    }

    pub fn set_log_level(&self, level: LogLevel) {
        self.log_level.set(level);
    }

    pub fn log_level(&self) -> LogLevel {
        self.log_level.get()
    }

    pub fn set_log_callback(&self, cb: LogCallback) {
        *self.log_cb.borrow_mut() = Some(cb);
    }

    pub fn clear_log_callback(&self) {
        *self.log_cb.borrow_mut() = None;
    }

    pub fn log(&self, level: LogLevel, msg: &str) {
        if level > self.log_level.get() {
            return;
        }
        if let Some(cb) = self.log_cb.borrow().as_ref() {
            cb(level, msg);
        } else {
            eprintln!("[nect-rs {:?}] {}", level, msg);
        }
    }

    pub fn enabled_subdevices(&self) -> DeviceFlags {
        self.enabled_subdevices.get()
    }

    pub fn select_subdevices(&self, subdevs: DeviceFlags) {
        let mask = DeviceFlags::MOTOR.union(DeviceFlags::CAMERA).union(DeviceFlags::AUDIO);
        self.enabled_subdevices.set(DeviceFlags(subdevs.0 & mask.0));
    }

    pub fn num_devices(&self) -> Result<usize, Error> {
        self.usb.num_devices().map_err(|e| Error::Usb(format!("{}", e)))
    }

    pub fn process_events(&self) -> Result<(), Error> {
        self.usb.process_events().map_err(|e| Error::Usb(format!("{}", e)))
    }

    pub fn process_events_timeout(&self, timeout: Duration) -> Result<(), Error> {
        self.usb.process_events_timeout(timeout).map_err(|e| Error::Usb(format!("{}", e)))
    }

    /// Open a device by index (0-based).
    pub fn open_device(&mut self, index: usize) -> Result<usize, Error> {
        let cam_device = self.usb.find_camera_by_index(index)
            .map_err(|e| Error::Usb(format!("camera not found: {}", e)))?;

        let mut cam_handle = UsbDevice::open(&cam_device)
            .map_err(|e| Error::Usb(format!("failed to open camera: {}", e)))?;

        cam_handle.claim_interface(0)
            .map_err(|e| Error::Usb(format!("claim camera interface failed: {}", e)))?;

        let motor_device = self.usb.find_motor_for_camera(&cam_device);
        let motor_handle = match motor_device {
            Ok(dev) => {
                match UsbDevice::open(&dev) {
                    Ok(mut h) => {
                        if h.claim_interface(0).is_ok() {
                            Some(h)
                        } else {
                            None
                        }
                    }
                    Err(_) => None,
                }
            }
            Err(_) => None,
        };

        let mut state = DeviceState::new();
        state.usb_cam = Some(cam_handle);
        state.usb_motor = motor_handle;
        state.serial = self.usb.list_camera_serials()
            .ok()
            .and_then(|s| s.get(index).cloned());

        self.devices.push(state);
        let handle = self.devices.len() - 1;

        self.log(LogLevel::Info, &format!("opened device index {} (handle {})", index, handle));
        Ok(handle)
    }

    /// Close a device by handle.
    pub fn close_device(&mut self, handle: usize) -> Result<(), Error> {
        if handle >= self.devices.len() {
            return Err(Error::Usb("invalid device handle".to_string()));
        }

        let (video_running, depth_running) = {
            let state = &self.devices[handle];
            (state.video_running.get(), state.depth_running.get())
        };

        if video_running {
            self.stop_video(handle)?;
        }
        if depth_running {
            self.stop_depth(handle)?;
        }
        {
            let state = &mut self.devices[handle];
            if let Some(ref mut cam) = state.usb_cam {
                let _ = cam.release_interface(0);
            }
            state.usb_cam = None;
            state.usb_motor = None;
            state.video_callback = None;
            state.depth_callback = None;
        }

        self.log(LogLevel::Info, &format!("closed device handle {}", handle));
        Ok(())
    }

    /// Start video stream.
    pub fn start_video(&mut self, handle: usize) -> Result<(), Error> {
        let state = self.devices.get_mut(handle)
            .ok_or(Error::Usb("invalid handle".to_string()))?;
        if let Some(ref mut cam) = state.usb_cam {
            let video_mode = state.video_mode.get();
            let format = video_mode.video_format();
            let resolution = video_mode.resolution();
            let width = video_mode.width() as usize;
            let height = video_mode.height() as usize;

            let wire_frame_size = match format {
                VideoFormat::Rgb | VideoFormat::Bayer => {
                    width * height // Bayer raw incoming on the wire
                }
                VideoFormat::YuvRgb | VideoFormat::YuvRaw => {
                    width * height * 2 // UYVY incoming
                }
                VideoFormat::Ir8Bit | VideoFormat::Ir10Bit | VideoFormat::Ir10BitPacked => {
                    width * height * 10 / 8 // 10-bit packed incoming
                }
            };

            let callback = state.video_callback.clone()
                .unwrap_or_else(|| Rc::new(RefCell::new(Box::new(|_frame: &[u8], _ts: u32| {}))));

            let mut proc_buf = vec![0u8; video_mode.bytes() as usize];
            let mut proc_buf_u16 = if format == VideoFormat::Ir10Bit {
                vec![0u16; width * height]
            } else {
                Vec::new()
            };

            let mut bridge = crate::stream_bridge::FrameState::new(
                0x80, // video stream flag
                1908, // VIDEO_PKTDSIZE
                wire_frame_size,
                Box::new(move |raw_frame, ts| {
                    match format {
                        VideoFormat::Rgb => {
                            if raw_frame.len() >= width * height && proc_buf.len() >= width * height * 3 {
                                crate::cameras::convert_bayer_to_rgb(raw_frame, &mut proc_buf, width, height);
                                (callback.borrow_mut())(&proc_buf, ts);
                            }
                        }
                        VideoFormat::YuvRgb => {
                            if raw_frame.len() >= width * height * 2 && proc_buf.len() >= width * height * 3 {
                                crate::cameras::convert_uyvy_to_rgb(raw_frame, &mut proc_buf, width, height);
                                (callback.borrow_mut())(&proc_buf, ts);
                            }
                        }
                        VideoFormat::Ir8Bit => {
                            let n = width * height;
                            if raw_frame.len() >= n * 10 / 8 && proc_buf.len() >= n {
                                crate::cameras::convert_packed_to_8bit(raw_frame, &mut proc_buf, 10, n);
                                (callback.borrow_mut())(&proc_buf, ts);
                            }
                        }
                        VideoFormat::Ir10Bit => {
                            let n = width * height;
                            if raw_frame.len() >= n * 10 / 8 && proc_buf_u16.len() >= n {
                                crate::cameras::convert_packed_to_16bit(raw_frame, &mut proc_buf_u16, 10, n);
                                // Safe byte view of u16 slice (since any u16 slice is valid as a u8 slice,
                                // and the size in bytes is exactly 2 * elements, this is completely defined):
                                let bytes = unsafe {
                                    std::slice::from_raw_parts(proc_buf_u16.as_ptr() as *const u8, n * 2)
                                };
                                (callback.borrow_mut())(bytes, ts);
                            }
                        }
                        _ => {
                            (callback.borrow_mut())(raw_frame, ts);
                        }
                    }
                }),
            );
            let raw_dev = unsafe { libusb1_sys::libusb_get_device(cam.handle.as_raw()) };
            let pkt_size = unsafe { libusb1_sys::libusb_get_max_iso_packet_size(raw_dev, crate::usb::EP_VIDEO) };
            let pkt_size = if pkt_size > 0 { pkt_size as usize } else { crate::isoc::VIDEO_PKTBUF };
            let stream = crate::isoc::IsoStream::new(
                cam,
                crate::usb::EP_VIDEO,
                crate::isoc::PKTS_PER_XFER,
                pkt_size,
                crate::isoc::NUM_XFERS,
                Box::new(move |packet| {
                    bridge.process_packet(packet);
                }),
            )?;
            state.video_stream = Some(stream);

            crate::camera_usb::start_video_stream(cam, format, resolution)?;
            match format {
                VideoFormat::Ir8Bit | VideoFormat::Ir10Bit | VideoFormat::Ir10BitPacked => {
                    crate::camera_usb::write_register(cam, 0x0105, 0x00)?;
                    crate::camera_usb::write_register(cam, 0x0005, 0x03)?;
                }
                _ => {
                    crate::camera_usb::write_register(cam, 0x0005, 0x01)?;
                }
            }

            state.video_running.set(true);
            Ok(())
        } else {
            Err(Error::Usb("no camera".to_string()))
        }
    }

    /// Stop video stream.
    pub fn stop_video(&mut self, handle: usize) -> Result<(), Error> {
        let state = self.devices.get_mut(handle)
            .ok_or(Error::Usb("invalid handle".to_string()))?;
        let ctx_raw = self.usb.raw_context();
        if let Some(ref mut cam) = state.usb_cam {
            let _ = crate::camera_usb::write_register(cam, 0x0005, 0x00); // STOP video stream
            let _ = crate::camera_usb::write_register(cam, 0x000c, 0x00); // Reset RGB mode register
            let _ = crate::camera_usb::write_register(cam, 0x0019, 0x00); // Reset IR mode register
        }
        if let Some(mut stream) = state.video_stream.take() {
            stream.stop(ctx_raw);
        }
        std::thread::sleep(std::time::Duration::from_millis(100)); // Let hardware settle
        state.video_running.set(false);
        Ok(())
    }

    /// Start depth stream.
    pub fn start_depth(&mut self, handle: usize) -> Result<(), Error> {
        let state = self.devices.get_mut(handle)
            .ok_or(Error::Usb("invalid handle".to_string()))?;
        if let Some(ref mut cam) = state.usb_cam {
            // Raw frames are packed 11-bit depth values; processed frames are unpacked 16-bit pixels.
            let raw_frame_size = 640 * 480 * 11 / 8; // 422400 (packed incoming)

            let callback = state.depth_callback.clone()
                .unwrap_or_else(|| Rc::new(RefCell::new(Box::new(|_frame: &[u8], _ts: u32| {}))));

            let mut bridge = crate::stream_bridge::FrameState::new(
                0x70,
                1748,
                raw_frame_size,
                Box::new(move |frame, ts| {
                    (callback.borrow_mut())(frame, ts);
                }),
            );
            let raw_dev = unsafe { libusb1_sys::libusb_get_device(cam.handle.as_raw()) };
            let pkt_size = unsafe { libusb1_sys::libusb_get_max_iso_packet_size(raw_dev, crate::usb::EP_DEPTH) };
            let pkt_size = if pkt_size > 0 { pkt_size as usize } else { crate::isoc::DEPTH_PKTBUF };

            let stream = crate::isoc::IsoStream::new(
                cam,
                crate::usb::EP_DEPTH,
                crate::isoc::PKTS_PER_XFER,
                pkt_size,
                crate::isoc::NUM_XFERS,
                Box::new(move |packet| {
                    bridge.process_packet(packet);
                }),
            )?;
            state.depth_stream = Some(stream);
            
            crate::camera_usb::write_register(cam, 0x0105, 0x00)?;
            crate::camera_usb::write_register(cam, 0x0006, 0x00)?;
            crate::camera_usb::write_register(cam, 0x0012, 0x03)?;
            crate::camera_usb::write_register(cam, 0x0013, 0x01)?;
            crate::camera_usb::write_register(cam, 0x0014, 0x1e)?;
            crate::camera_usb::write_register(cam, 0x0006, 0x02)?;
            crate::camera_usb::write_register(cam, 0x0017, 0x00)?;
            
            state.depth_running.set(true);
            Ok(())
        } else {
            Err(Error::Usb("no camera".to_string()))
        }
    }

    /// Stop depth stream.
    pub fn stop_depth(&mut self, handle: usize) -> Result<(), Error> {
        let state = self.devices.get_mut(handle)
            .ok_or(Error::Usb("invalid handle".to_string()))?;
        let ctx_raw = self.usb.raw_context();
        if let Some(ref mut cam) = state.usb_cam {
            let _ = crate::camera_usb::write_register(cam, 0x0006, 0x00); // STOP depth stream
            let _ = crate::camera_usb::write_register(cam, 0x0012, 0x00); // Reset depth format register
        }
        if let Some(mut stream) = state.depth_stream.take() {
            stream.stop(ctx_raw);
        }
        std::thread::sleep(std::time::Duration::from_millis(100)); // Let hardware settle
        state.depth_running.set(false);
        Ok(())
    }

    pub fn usb(&self) -> &KinectUsb {
        &self.usb
    }

    pub fn device_state(&self, handle: usize) -> Option<&DeviceState> {
        self.devices.get(handle)
    }

    pub fn device_state_mut(&mut self, handle: usize) -> Option<&mut DeviceState> {
        self.devices.get_mut(handle)
    }
}

impl Drop for ContextCore {
    fn drop(&mut self) {
        for i in 0..self.devices.len() {
            let _ = self.close_device(i);
        }
    }
}
