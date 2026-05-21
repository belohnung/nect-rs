use std::cell::{Cell, RefCell};
use std::rc::Rc;
use std::time::Duration;

// Explicitly mark Context and Device as !Send + !Sync because libusb device access is not thread-safe here.
type NotSendSync = std::marker::PhantomData<*const ()>;

pub mod camera_usb;
pub mod cameras;
pub mod core;
pub mod flags;
pub mod isoc;
pub mod loader;
pub mod modes;
pub mod protocol;
pub mod registration;
pub mod stream;
pub mod stream_bridge;
pub mod tilt;
pub mod usb;

/// Error type for driver operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    InitFailed,
    DeviceOpenFailed,
    DeviceNotFound,
    IoError,
    InvalidMode,
    Unsupported,
    Usb(String),
    Other(i32),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::InitFailed => write!(f, "nect-rs context initialization failed"),
            Error::DeviceOpenFailed => write!(f, "failed to open device"),
            Error::DeviceNotFound => write!(f, "device not found"),
            Error::IoError => write!(f, "USB I/O error"),
            Error::InvalidMode => write!(f, "invalid frame mode"),
            Error::Unsupported => write!(f, "unsupported operation"),
            Error::Usb(ref s) => write!(f, "USB error: {}", s),
            Error::Other(code) => write!(f, "nect-rs error code: {}", code),
        }
    }
}

impl std::error::Error for Error {}

fn check(ret: i32) -> Result<(), Error> {
    if ret == 0 {
        Ok(())
    } else if ret < 0 {
        Err(Error::Other(ret))
    } else {
        Ok(())
    }
}

pub use core::LogLevel;

/// Which subdevices to open.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceFlags(u32);

impl DeviceFlags {
    pub const MOTOR: Self = DeviceFlags(0x01);
    pub const CAMERA: Self = DeviceFlags(0x02);
    pub const AUDIO: Self = DeviceFlags(0x04);

    pub fn empty() -> Self {
        DeviceFlags(0)
    }

    pub fn union(self, other: Self) -> Self {
        DeviceFlags(self.0 | other.0)
    }

    pub fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }
}

/// LED color / pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LedOption {
    Off,
    Green,
    Red,
    Yellow,
    BlinkGreen,
    BlinkRedYellow,
}

impl LedOption {
    fn to_raw(self) -> u8 {
        match self {
            LedOption::Off => 0,
            LedOption::Green => 1,
            LedOption::Red => 2,
            LedOption::Yellow => 3,
            LedOption::BlinkGreen => 4,
            LedOption::BlinkRedYellow => 6,
        }
    }
}

/// Camera resolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Resolution {
    Low,
    Medium,
    High,
}

/// Video pixel format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoFormat {
    Rgb,
    Bayer,
    Ir8Bit,
    Ir10Bit,
    Ir10BitPacked,
    YuvRgb,
    YuvRaw,
}

/// Depth pixel format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DepthFormat {
    Bit11,
    Bit10,
    Bit11Packed,
    Bit10Packed,
    Registered,
    Mm,
}

/// Describes a frame mode (resolution, format, size, framerate).
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct FrameMode {
    pub(crate) resolution: Resolution,
    pub(crate) video_format: Option<VideoFormat>,
    pub(crate) depth_format: Option<DepthFormat>,
    pub(crate) bytes: i32,
    pub(crate) width: i16,
    pub(crate) height: i16,
    pub(crate) data_bits_per_pixel: i8,
    pub(crate) padding_bits_per_pixel: i8,
    pub(crate) framerate: i8,
    pub(crate) is_valid: bool,
}

impl std::fmt::Debug for FrameMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FrameMode")
            .field("resolution", &self.resolution())
            .field("bytes", &self.bytes())
            .field("width", &self.width())
            .field("height", &self.height())
            .field("framerate", &self.framerate())
            .field("is_valid", &self.is_valid())
            .finish()
    }
}

impl FrameMode {
    pub fn resolution(&self) -> Resolution {
        self.resolution
    }

    pub fn bytes(&self) -> i32 {
        self.bytes
    }

    pub fn width(&self) -> i16 {
        self.width
    }

    pub fn height(&self) -> i16 {
        self.height
    }

    pub fn framerate(&self) -> i8 {
        self.framerate
    }

    pub fn is_valid(&self) -> bool {
        self.is_valid
    }

    pub fn video_format(&self) -> VideoFormat {
        self.video_format.unwrap_or(VideoFormat::Rgb)
    }

    pub fn depth_format(&self) -> DepthFormat {
        self.depth_format.unwrap_or(DepthFormat::Bit11)
    }

    pub fn data_bits_per_pixel(&self) -> i8 {
        self.data_bits_per_pixel
    }

    pub fn padding_bits_per_pixel(&self) -> i8 {
        self.padding_bits_per_pixel
    }
}

/// Tilt motor status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TiltStatus {
    Stopped,
    Limit,
    Moving,
}

/// Raw accelerometer + tilt state.
#[derive(Debug, Clone, Copy)]
pub struct TiltState {
    pub accelerometer_x: i16,
    pub accelerometer_y: i16,
    pub accelerometer_z: i16,
    pub tilt_angle: i8,
    pub tilt_status: TiltStatus,
}

impl TiltState {
    pub fn accelerometer_x(&self) -> i16 {
        self.accelerometer_x
    }

    pub fn accelerometer_y(&self) -> i16 {
        self.accelerometer_y
    }

    pub fn accelerometer_z(&self) -> i16 {
        self.accelerometer_z
    }

    pub fn tilt_angle(&self) -> i8 {
        self.tilt_angle
    }

    pub fn tilt_status(&self) -> TiltStatus {
        self.tilt_status
    }

    /// Tilt in degrees with respect to the horizon.
    pub fn tilt_degs(&self) -> f64 {
        tilt::raw_tilt_to_degrees(self.tilt_angle)
    }

    /// Accelerometer values in m/s^2.
    pub fn mks_accel(&self) -> (f64, f64, f64) {
        let x = tilt::counts_to_mks(self.accelerometer_x);
        let y = tilt::counts_to_mks(self.accelerometer_y);
        let z = tilt::counts_to_mks(self.accelerometer_z);
        (x, y, z)
    }
}

/// Runtime context that owns the USB context and device list.
pub struct Context {
    core: Rc<RefCell<core::ContextCore>>,
    open_count: Rc<Cell<usize>>,
    _not_send_sync: NotSendSync,
}

impl Context {
    /// Create a new nect-rs context.
    pub fn new() -> Result<Self, Error> {
        let core = core::ContextCore::new().map_err(|_| Error::InitFailed)?;
        Ok(Context {
            core: Rc::new(RefCell::new(core)),
            open_count: Rc::new(Cell::new(0)),
            _not_send_sync: std::marker::PhantomData,
        })
    }

    /// Number of Kinect devices currently connected.
    pub fn num_devices(&self) -> Result<usize, Error> {
        self.core.borrow().num_devices()
    }

    /// Enumerate serial numbers of connected cameras.
    pub fn list_device_serials(&self) -> Result<Vec<String>, Error> {
        self.core.borrow().usb().list_camera_serials()
            .map_err(|e| Error::Usb(format!("{}", e)))
    }

    /// Select which subdevices to open for future `open_device` calls.
    pub fn select_subdevices(&self, flags: DeviceFlags) {
        self.core.borrow().select_subdevices(flags);
    }

    /// Set the global log level.
    pub fn set_log_level(&self, level: LogLevel) {
        self.core.borrow().set_log_level(level);
    }

    /// Process USB events (blocking).
    pub fn process_events(&self) -> Result<(), Error> {
        self.core.borrow().process_events()
    }

    /// Process USB events with a timeout.
    pub fn process_events_timeout(&self, timeout: Duration) -> Result<(), Error> {
        self.core.borrow().process_events_timeout(timeout)
    }

    /// Open a Kinect device by index.
    pub fn open_device(&self, index: usize) -> Result<Device, Error> {
        let handle = self.core.borrow_mut().open_device(index)
            .map_err(|_| Error::DeviceOpenFailed)?;
        self.open_count.set(self.open_count.get() + 1);
        Ok(Device {
            core: Rc::clone(&self.core),
            handle,
            callbacks: RefCell::new(Callbacks { video: None, depth: None }),
            open_count: Rc::clone(&self.open_count),
            _not_send_sync: std::marker::PhantomData,
        })
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        let n = self.open_count.get();
        if n != 0 {
            panic!(
                "Context dropped with {} device(s) still open. Close all Devices before dropping Context.",
                n
            );
        }
    }
}

/// Callbacks stored per-device.
struct Callbacks {
    video: Option<core::SharedFrameCallback>,
    depth: Option<core::SharedFrameCallback>,
}

/// A single Kinect device.
pub struct Device {
    core: Rc<RefCell<core::ContextCore>>,
    handle: usize,
    callbacks: RefCell<Callbacks>,
    open_count: Rc<Cell<usize>>,
    _not_send_sync: NotSendSync,
}

impl Device {
    /// Set the LED state.
    pub fn set_led(&mut self, option: LedOption) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        if let Some(ref mut motor) = state.usb_motor {
            camera_usb::set_led(motor, option.to_raw())
        } else {
            Err(Error::Usb("no motor device available".to_string()))
        }
    }

    /// Update and return the tilt/accelerometer state.
    pub fn update_tilt_state(&mut self) -> Result<TiltState, Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        if let Some(ref mut motor) = state.usb_motor {
            let mut buf = [0u8; 10];
            camera_usb::read_tilt_state(motor, &mut buf)?;
            // buf layout from motor firmware (10 bytes):
            // [0..1] unknown
            // [2..3] = accelerometer x (big-endian: high|low)
            // [4..5] = accelerometer y (big-endian)
            // [6..7] = accelerometer z (big-endian)
            // [8] = tilt angle (signed, half-degrees)
            // [9] = tilt status
            let accel_x = i16::from_be_bytes([buf[2], buf[3]]);
            let accel_y = i16::from_be_bytes([buf[4], buf[5]]);
            let accel_z = i16::from_be_bytes([buf[6], buf[7]]);
            let tilt_angle = buf[8] as i8;
            let tilt_status = match buf[9] {
                0 => TiltStatus::Stopped,
                1 => TiltStatus::Limit,
                4 => TiltStatus::Moving,
                _ => TiltStatus::Stopped,
            };
            Ok(TiltState {
                accelerometer_x: accel_x,
                accelerometer_y: accel_y,
                accelerometer_z: accel_z,
                tilt_angle,
                tilt_status,
            })
        } else {
            Err(Error::Usb("no motor device available".to_string()))
        }
    }

    /// Set the tilt angle in degrees.
    pub fn set_tilt_degs(&mut self, angle: f64) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        if let Some(ref mut motor) = state.usb_motor {
            let clamped = tilt::clamp_tilt_angle(angle);
            let raw = (clamped * 2.0) as i8;
            camera_usb::set_tilt_degs(motor, raw)
        } else {
            Err(Error::Usb("no motor device available".to_string()))
        }
    }

    // --- Video ---

    /// Start the video stream.
    pub fn start_video(&mut self) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let cb = self.callbacks.borrow().video.clone();
        core.device_state_mut(self.handle).unwrap().video_callback = cb;
        core.start_video(self.handle)?;
        Ok(())
    }

    /// Stop the video stream.
    pub fn stop_video(&mut self) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        core.stop_video(self.handle)?;
        Ok(())
    }

    /// Number of supported video modes.
    pub fn video_mode_count() -> usize {
        modes::num_video_modes()
    }

    /// Get the nth supported video mode.
    pub fn video_mode(index: usize) -> FrameMode {
        *modes::VIDEO_MODES.get(index).unwrap_or(&modes::VIDEO_MODES[0])
    }

    /// Find a video mode by resolution and format.
    pub fn find_video_mode(res: Resolution, fmt: VideoFormat) -> FrameMode {
        *modes::lookup_video_mode(res, fmt)
            .unwrap_or(&modes::VIDEO_MODES[0])
    }

    /// Set the video mode. Must not be called while streaming.
    pub fn set_video_mode(&mut self, mode: FrameMode) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        state.video_mode.set(mode);
        Ok(())
    }

    /// Read the current RGB/YUV exposure time in microseconds.
    pub fn exposure_us(&mut self) -> Result<f64, Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        let mode = state.video_mode.get();
        let is_rgb = match mode.video_format() {
            VideoFormat::Rgb | VideoFormat::Bayer => true,
            VideoFormat::YuvRgb | VideoFormat::YuvRaw => false,
            _ => return Err(Error::InvalidMode),
        };
        let cam = state.usb_cam.as_mut()
            .ok_or_else(|| Error::Usb("no camera device available".to_string()))?;
        let shutter = camera_usb::read_cmos_register(cam, 0x0009)?;
        Ok(flags::shutter_to_exposure_us(shutter, is_rgb))
    }

    /// Set RGB/YUV exposure time in microseconds.
    pub fn set_exposure_us(&mut self, exposure_us: f64) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        let mode = state.video_mode.get();
        let is_rgb = match mode.video_format() {
            VideoFormat::Rgb | VideoFormat::Bayer => true,
            VideoFormat::YuvRgb | VideoFormat::YuvRaw => false,
            _ => return Err(Error::InvalidMode),
        };
        let cam = state.usb_cam.as_mut()
            .ok_or_else(|| Error::Usb("no camera device available".to_string()))?;
        let shutter = flags::exposure_us_to_shutter(exposure_us.max(0.0), is_rgb);
        camera_usb::write_cmos_register(cam, 0x0009, shutter)
    }

    /// Read IR emitter brightness, in the Kinect firmware range 1..=50.
    pub fn ir_brightness(&mut self) -> Result<u16, Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        let cam = state.usb_cam.as_mut()
            .ok_or_else(|| Error::Usb("no camera device available".to_string()))?;
        camera_usb::read_register(cam, 0x0015)
    }

    /// Set IR emitter brightness, clamped to the Kinect firmware range 1..=50.
    pub fn set_ir_brightness(&mut self, brightness: u16) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        let cam = state.usb_cam.as_mut()
            .ok_or_else(|| Error::Usb("no camera device available".to_string()))?;
        camera_usb::write_register(cam, 0x0015, flags::clamp_ir_brightness(brightness))
    }

    // --- Depth ---

    /// Start the depth stream.
    pub fn start_depth(&mut self) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let cb = self.callbacks.borrow().depth.clone();
        core.device_state_mut(self.handle).unwrap().depth_callback = cb;
        core.start_depth(self.handle)?;
        Ok(())
    }

    /// Stop the depth stream.
    pub fn stop_depth(&mut self) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        core.stop_depth(self.handle)?;
        Ok(())
    }

    /// Number of supported depth modes.
    pub fn depth_mode_count() -> usize {
        modes::num_depth_modes()
    }

    /// Get the nth supported depth mode.
    pub fn depth_mode(index: usize) -> FrameMode {
        *modes::DEPTH_MODES.get(index).unwrap_or(&modes::DEPTH_MODES[0])
    }

    /// Find a depth mode by resolution and format.
    pub fn find_depth_mode(res: Resolution, fmt: DepthFormat) -> FrameMode {
        *modes::lookup_depth_mode(res, fmt)
            .unwrap_or(&modes::DEPTH_MODES[0])
    }

    /// Set the depth mode. Must not be called while streaming.
    pub fn set_depth_mode(&mut self, mode: FrameMode) -> Result<(), Error> {
        let mut core = self.core.borrow_mut();
        let state = core.device_state_mut(self.handle)
            .ok_or(Error::DeviceNotFound)?;
        state.depth_mode.set(mode);
        Ok(())
    }

    // --- Callbacks ---

    /// Set a callback to be invoked on every video frame.
    pub fn set_video_callback<F>(&mut self, cb: F)
    where
        F: FnMut(&[u8], u32) + 'static,
    {
        self.callbacks.borrow_mut().video = Some(Rc::new(RefCell::new(Box::new(cb))));
    }

    /// Set a callback to be invoked on every depth frame.
    pub fn set_depth_callback<F>(&mut self, cb: F)
    where
        F: FnMut(&[u8], u32) + 'static,
    {
        self.callbacks.borrow_mut().depth = Some(Rc::new(RefCell::new(Box::new(cb))));
    }

    /// Clear the video callback.
    pub fn clear_video_callback(&mut self) {
        self.callbacks.borrow_mut().video = None;
    }

    /// Clear the depth callback.
    pub fn clear_depth_callback(&mut self) {
        self.callbacks.borrow_mut().depth = None;
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        let _ = self.core.borrow_mut().close_device(self.handle);
        self.open_count.set(self.open_count.get().saturating_sub(1));
    }
}
