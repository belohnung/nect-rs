//! Pure-Rust USB transport layer using `rusb`.
//!
//! Handles device enumeration and control transfers.

use std::time::Duration;

use rusb::{Context, Device, DeviceHandle, UsbContext};

// ---------------------------------------------------------------------------
// USB device IDs and endpoint constants.
// ---------------------------------------------------------------------------

pub const VID_MICROSOFT: u16 = 0x045e;
pub const PID_NUI_AUDIO: u16 = 0x02ad;
pub const PID_NUI_CAMERA: u16 = 0x02ae;
pub const PID_NUI_MOTOR: u16 = 0x02b0;
pub const PID_K4W_CAMERA: u16 = 0x02bf;
pub const PID_K4W_AUDIO: u16 = 0x02be;
pub const PID_K4W_AUDIO_ALT_1: u16 = 0x02c3;
pub const PID_K4W_AUDIO_ALT_2: u16 = 0x02bb;
pub const PID_KV2_CAMERA: u16 = 0x02d9;

/// USB endpoints.
pub const EP_DEPTH: u8 = 0x82;
pub const EP_VIDEO: u8 = 0x81;

/// Isochronous transfer parameters.
pub const NUM_XFERS: i32 = 15;      // transfers in flight
pub const PKTS_PER_XFER: i32 = 16;   // packets per transfer
pub const DEPTH_PKTBUF: i32 = 1760;  // depth packet buffer size
pub const VIDEO_PKTBUF: i32 = 1920;  // video packet buffer size

// ---------------------------------------------------------------------------
// Device descriptor predicates
// ---------------------------------------------------------------------------

pub fn is_camera(desc: &rusb::DeviceDescriptor) -> bool {
    desc.vendor_id() == VID_MICROSOFT
        && (desc.product_id() == PID_NUI_CAMERA || desc.product_id() == PID_K4W_CAMERA)
}

pub fn is_motor(desc: &rusb::DeviceDescriptor) -> bool {
    desc.vendor_id() == VID_MICROSOFT && desc.product_id() == PID_NUI_MOTOR
}

pub fn is_audio(desc: &rusb::DeviceDescriptor) -> bool {
    desc.vendor_id() == VID_MICROSOFT
        && (desc.product_id() == PID_NUI_AUDIO
            || desc.product_id() == PID_K4W_AUDIO
            || desc.product_id() == PID_K4W_AUDIO_ALT_1
            || desc.product_id() == PID_K4W_AUDIO_ALT_2)
}

pub fn is_kinect(desc: &rusb::DeviceDescriptor) -> bool {
    desc.vendor_id() == VID_MICROSOFT
        && (is_camera(desc) || is_motor(desc) || is_audio(desc))
}

// ---------------------------------------------------------------------------
// USB Context wrapper
// ---------------------------------------------------------------------------

pub struct KinectUsb {
    context: Context,
    owned: bool,
}

impl KinectUsb {
    pub fn new() -> Result<Self, rusb::Error> {
        let ctx = Context::new()?;
        Ok(KinectUsb {
            context: ctx,
            owned: true,
        })
    }

    pub fn from_raw(raw: *mut rusb::ffi::libusb_context) -> Self {
        // Safety: caller must ensure raw is valid
        KinectUsb {
            context: unsafe { Context::from_raw(raw) },
            owned: false,
        }
    }

    pub fn process_events(&self) -> Result<(), rusb::Error> {
        self.context.handle_events(None)
    }

    pub fn process_events_timeout(&self, timeout: Duration) -> Result<(), rusb::Error> {
        self.context.handle_events(Some(timeout))
    }

    pub fn num_devices(&self) -> Result<usize, rusb::Error> {
        let devices = self.context.devices()?;
        let mut count = 0;
        for dev in devices.iter() {
            if let Ok(desc) = dev.device_descriptor() {
                if is_camera(&desc) {
                    count += 1;
                }
            }
        }
        Ok(count)
    }

    /// List Kinect camera serial numbers.
    pub fn list_camera_serials(&self) -> Result<Vec<String>, rusb::Error> {
        let devices = self.context.devices()?;
        let mut serials = Vec::new();

        for dev in devices.iter() {
            let desc = match dev.device_descriptor() {
                Ok(d) => d,
                Err(_) => continue,
            };

            if !is_camera(&desc) {
                continue;
            }
            if desc.serial_number_string_index().is_none() {
                continue;
            }

            let mut handle = match dev.open() {
                Ok(h) => h,
                Err(_) => continue,
            };

            let serial = match handle.read_string_descriptor_ascii(
                desc.serial_number_string_index().unwrap(),
            ) {
                Ok(s) => s,
                Err(_) => {
                    // For K4W/1473 the camera serial is "0000000000000000";
                    // try the sibling audio device instead.
                    if let Some(audio) = find_sibling(&dev, &devices, is_audio) {
                        let audio_desc = match audio.device_descriptor() {
                            Ok(d) => d,
                            Err(_) => continue,
                        };
                        let audio_handle = match audio.open() {
                            Ok(h) => h,
                            Err(_) => continue,
                        };
                        let s = match audio_handle.read_string_descriptor_ascii(
                            audio_desc.serial_number_string_index().unwrap_or(0),
                        ) {
                            Ok(s) => s,
                            Err(_) => continue,
                        };
                        s
                    } else {
                        continue;
                    }
                }
            };

            serials.push(serial);
        }

        Ok(serials)
    }

    pub fn find_camera_by_index(&self, index: usize) -> Result<Device<Context>, rusb::Error> {
        let devices = self.context.devices()?;
        let mut cam_idx = 0;
        for dev in devices.iter() {
            if let Ok(desc) = dev.device_descriptor() {
                if is_camera(&desc) {
                    if cam_idx == index {
                        return Ok(dev);
                    }
                    cam_idx += 1;
                }
            }
        }
        Err(rusb::Error::NotFound)
    }

    /// Find the motor device that shares a bus with the given camera.
    pub fn find_motor_for_camera(&self, camera: &Device<Context>) -> Result<Device<Context>, rusb::Error> {
        let devices = self.context.devices()?;
        let camera_bus = camera.bus_number();
        let camera_parent = camera.port_number();

        let mut same_bus: Option<Device<Context>> = None;
        let mut any_match: Option<Device<Context>> = None;

        for dev in devices.iter() {
            let desc = match dev.device_descriptor() {
                Ok(d) => d,
                Err(_) => continue,
            };
            if !is_motor(&desc) {
                continue;
            }

            if any_match.is_none() {
                any_match = Some(dev.clone());
            }

            if dev.bus_number() != camera_bus {
                continue;
            }
            if same_bus.is_none() {
                same_bus = Some(dev.clone());
            }

            if camera_parent != 0 && dev.port_number() == camera_parent {
                return Ok(dev);
            }
        }

        same_bus.or(any_match).ok_or(rusb::Error::NotFound)
    }

    pub fn context(&self) -> &Context {
        &self.context
    }

    /// Get the raw libusb_context pointer for advanced use.
    pub fn raw_context(&self) -> *mut libusb1_sys::libusb_context {
        self.context.as_raw()
    }
}

impl Drop for KinectUsb {
    fn drop(&mut self) {
        // Context will be dropped automatically
    }
}

// ---------------------------------------------------------------------------
// USB device handle wrapper
// ---------------------------------------------------------------------------

pub struct UsbDevice {
    pub vid: u16,
    pub pid: u16,
    pub(crate) handle: DeviceHandle<Context>,
}

impl UsbDevice {
    pub fn open(device: &Device<Context>) -> Result<Self, rusb::Error> {
        let desc = device.device_descriptor()?;
        let handle = device.open()?;
        Ok(UsbDevice {
            vid: desc.vendor_id(),
            pid: desc.product_id(),
            handle,
        })
    }

    pub fn claim_interface(&mut self, iface: u8) -> Result<(), rusb::Error> {
        // Detach kernel driver if active (Linux)
        #[cfg(not(target_os = "windows"))]
        {
            if self.handle.kernel_driver_active(iface)? {
                let _ = self.handle.detach_kernel_driver(iface);
            }
        }
        self.handle.claim_interface(iface)
    }

    pub fn release_interface(&mut self, iface: u8) -> Result<(), rusb::Error> {
        let _ = self.handle.release_interface(iface);
        #[cfg(not(target_os = "windows"))]
        {
            let _ = self.handle.attach_kernel_driver(iface);
        }
        Ok(())
    }

    pub fn set_interface_alt_setting(
        &mut self,
        iface: u8,
        alt: u8,
    ) -> Result<(), rusb::Error> {
        self.handle.set_alternate_setting(iface, alt)
    }

    /// USB control transfer.
    pub fn control_transfer(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &mut [u8],
        timeout: Duration,
    ) -> Result<usize, rusb::Error> {
        self.handle
            .write_control(request_type, request, value, index, data, timeout)
    }

    /// Read USB control transfer.
    pub fn control_read(
        &mut self,
        request_type: u8,
        request: u8,
        value: u16,
        index: u16,
        data: &mut [u8],
        timeout: Duration,
    ) -> Result<usize, rusb::Error> {
        self.handle
            .read_control(request_type, request, value, index, data, timeout)
    }

    /// Bulk transfer (for audio bootloader).
    pub fn bulk_transfer(
        &mut self,
        endpoint: u8,
        data: &mut [u8],
        timeout: Duration,
    ) -> Result<usize, rusb::Error> {
        self.handle.write_bulk(endpoint, data, timeout)
    }

    pub fn bulk_read(
        &mut self,
        endpoint: u8,
        data: &mut [u8],
        timeout: Duration,
    ) -> Result<usize, rusb::Error> {
        self.handle.read_bulk(endpoint, data, timeout)
    }

    /// Get max isochronous packet size.
    pub fn max_iso_packet_size(&self, _endpoint: u8, default: usize) -> usize {
        // rusb::Device does not expose max_iso_packet_size; use libusb default
        default
    }

    /// Number of interfaces on the active configuration.
    pub fn num_interfaces(&self) -> Result<u8, rusb::Error> {
        let device = self.handle.device();
        let config = device.active_config_descriptor()?;
        Ok(config.num_interfaces())
    }

    pub fn reset(&mut self) -> Result<(), rusb::Error> {
        self.handle.reset()
    }

    pub fn handle(&mut self) -> &mut DeviceHandle<Context> {
        &mut self.handle
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Find a sibling device on the same USB bus/hub.
fn find_sibling(
    camera: &Device<Context>,
    devices: &rusb::DeviceList<Context>,
    predicate: fn(&rusb::DeviceDescriptor) -> bool,
) -> Option<Device<Context>> {
    let camera_bus = camera.bus_number();
    let camera_parent = camera.port_number();

    let mut same_bus: Option<Device<Context>> = None;
    let mut any_match: Option<Device<Context>> = None;

    for dev in devices.iter() {
        let desc = match dev.device_descriptor() {
            Ok(d) => d,
            Err(_) => continue,
        };
        if !predicate(&desc) {
            continue;
        }

        if any_match.is_none() {
            any_match = Some(dev.clone());
        }

        if dev.bus_number() != camera_bus {
            continue;
        }
        if same_bus.is_none() {
            same_bus = Some(dev.clone());
        }

        // If same parent hub, this is the best match
        if camera_parent != 0 && dev.port_number() == camera_parent {
            return Some(dev);
        }
    }

    same_bus.or(any_match)
}
