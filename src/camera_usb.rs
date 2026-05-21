//! Camera register read/write over USB control transfers.
//!
//! Bridges `protocol.rs` command builders with the `usb.rs` `UsbDevice`
//! control transfer API to implement camera register I/O in pure Rust.

use std::time::Duration;

use crate::Error;
use crate::usb::UsbDevice;
use crate::protocol::{self, CamReplyHeader};
use crate::{Resolution, VideoFormat, DepthFormat};

const CMD_REQUEST_TYPE: u8 = 0x40; // Host to Device, Vendor, Device
const REPLY_REQUEST_TYPE: u8 = 0xC0; // Device to Host, Vendor, Device
const TIMEOUT: Duration = Duration::from_millis(500);

/// Send a camera command via USB control transfer using libusb1-sys directly.
/// This avoids borrow conflicts with rusb's safe wrapper during isoc streaming.
pub fn send_cmd(device: &mut UsbDevice, cmd: &[u8]) -> Result<(), Error> {
    let mut data = cmd.to_vec();
    let ret = unsafe {
        libusb1_sys::libusb_control_transfer(
            device.handle.as_raw(),
            CMD_REQUEST_TYPE,
            0,
            0,
            0,
            data.as_mut_ptr(),
            data.len() as u16,
            500,
        )
    };
    if ret < 0 {
        return Err(Error::Usb(format!("send_cmd failed: {}", ret)));
    }
    if ret as usize != cmd.len() {
        return Err(Error::Usb(format!("send_cmd: wrote {} of {} bytes", ret, cmd.len())));
    }
    Ok(())
}

/// Read camera reply with a polling loop.
/// Keeps reading until a non-empty, non-full reply is received.
fn read_reply(device: &mut UsbDevice) -> Result<Vec<u8>, Error> {
    let mut reply = vec![0u8; 512];
    let mut loops = 0;
    loop {
        let ret = unsafe {
            libusb1_sys::libusb_control_transfer(
                device.handle.as_raw(),
                REPLY_REQUEST_TYPE,
                0,
                0,
                0,
                reply.as_mut_ptr(),
                reply.len() as u16,
                100,
            )
        };
        if ret < 0 && ret != libusb1_sys::constants::LIBUSB_ERROR_TIMEOUT {
            return Err(Error::Usb(format!("read_reply control read failed: {}", ret)));
        }
        let res = if ret < 0 { 0 } else { ret as usize };

        if res > 0 && res < 512 {
            reply.truncate(res);
            return Ok(reply);
        }

        loops += 1;
        if loops > 100 {
            return Err(Error::Usb("read_reply: timeout waiting for camera reply".to_string()));
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

/// Read a register from the camera.
/// Sends command `0x02` and reads the register value from the reply.
pub fn read_register(device: &mut UsbDevice, addr: u16) -> Result<u16, Error> {
    let cmd = protocol::read_register_cmd(addr);
    let full_cmd = protocol::assemble_cmd_packet(0x02, 1, &cmd);
    send_cmd(device, &full_cmd)?;

    let reply = read_reply(device)?;
    let res = reply.len();

    if res < 2 {
        return Err(Error::Usb("read_register: short reply".to_string()));
    }

    if reply[0] == 0x52 && reply[1] == 0x42 {
        // ACK - success
        // Reply format: [RB][len][cmd][tag][value]
        // value is at offset 8 (after 8-byte header)
        if res >= 10 {
            let val = u16::from_le_bytes([reply[8], reply[9]]);
            return Ok(val);
        }
        return Ok(0);
    } else if reply[0] == 0x52 && reply[1] == 0x52 {
        return Err(Error::Usb("read_register: NACK".to_string()));
    } else if reply[0] == 0x52 && reply[1] == 0x53 {
        return Err(Error::Usb("read_register: NACK+bad addr".to_string()));
    }

    Err(Error::Usb(format!("read_register: unknown reply magic {:02x?}", &reply[..res.min(4)])))
}

/// Write a value to a camera register.
/// Sends command `0x03` with the register value.
pub fn write_register(device: &mut UsbDevice, addr: u16, value: u16) -> Result<(), Error> {
    let cmd = protocol::write_register_cmd(addr, value);
    let full_cmd = protocol::assemble_cmd_packet(0x03, 1, &cmd);
    send_cmd(device, &full_cmd)?;

    let reply = read_reply(device)?;
    let res = reply.len();

    if res < 2 {
        return Err(Error::Usb("write_register: short reply".to_string()));
    }

    if reply[0] == 0x52 && reply[1] == 0x42 {
        return Ok(());
    } else if reply[0] == 0x52 && reply[1] == 0x52 {
        return Err(Error::Usb("write_register: NACK".to_string()));
    } else if reply[0] == 0x52 && reply[1] == 0x53 {
        return Err(Error::Usb("write_register: NACK+bad addr".to_string()));
    }

    Err(Error::Usb(format!("write_register: unknown reply magic {:02x?}", &reply[..res.min(4)])))
}

/// Write a CMOS register (sensor-side register).
pub fn write_cmos_register(device: &mut UsbDevice, addr: u16, value: u16) -> Result<(), Error> {
    let tag = 1u16;
    let payload = protocol::write_cmos_cmd(addr, value);
    let cmd = protocol::assemble_cmd_packet(0x95, tag, &payload);
    send_cmd(device, &cmd)
}

/// Read a CMOS register (sensor-side register).
pub fn read_cmos_register(device: &mut UsbDevice, addr: u16) -> Result<u16, Error> {
    let tag = 1u16;
    let payload = protocol::read_cmos_cmd(addr);
    let cmd = protocol::assemble_cmd_packet(0x95, tag, &payload);
    send_cmd(device, &cmd)?;

    let reply = read_reply(device)?;
    if reply.len() < 14 {
        return Err(Error::Usb("read_cmos_register: short reply".to_string()));
    }
    if reply[0] != 0x52 || reply[1] != 0x42 {
        return Err(Error::Usb(format!(
            "read_cmos_register: unknown reply magic {:02x?}",
            &reply[..reply.len().min(4)]
        )));
    }

    Ok(u16::from_le_bytes([reply[12], reply[13]]))
}

/// Start the depth stream by writing the correct camera registers.
///
/// Configures depth mode registers and starts streaming.
pub fn start_depth_stream(device: &mut UsbDevice, format: DepthFormat) -> Result<(), Error> {
    write_register(device, 0x0105, 0x00)?; // Disable auto-cycle of projector
    write_register(device, 0x0006, 0x00)?; // reset depth stream

    match format {
        DepthFormat::Bit11 | DepthFormat::Bit11Packed | DepthFormat::Registered => {
            write_register(device, 0x0012, 0x03)?;
        }
        DepthFormat::Bit10 | DepthFormat::Bit10Packed => {
            write_register(device, 0x0012, 0x02)?;
        }
        _ => return Err(Error::InvalidMode),
    }

    write_register(device, 0x0013, 0x01)?; // resolution medium
    write_register(device, 0x0014, 0x1e)?; // fps 30
    write_register(device, 0x0006, 0x02)?; // start depth stream
    write_register(device, 0x0017, 0x00)?; // disable depth hflip

    Ok(())
}

/// Stop the depth stream.
pub fn stop_depth_stream(device: &mut UsbDevice) -> Result<(), Error> {
    write_register(device, 0x0006, 0x00)?; // stop depth stream
    Ok(())
}

/// Start the video stream by writing the correct camera registers.
///
/// Configures video mode registers and starts streaming.
pub fn start_video_stream(device: &mut UsbDevice, format: VideoFormat, resolution: Resolution) -> Result<(), Error> {
    let (mode_reg, res_reg, fps_reg, hflip_reg, mode_val, res_val, fps_val) = match format {
        VideoFormat::Rgb | VideoFormat::Bayer => {
            let (res_val, fps_val) = match resolution {
                Resolution::High => (0x02u16, 0x0fu16),
                Resolution::Medium => (0x01u16, 0x1eu16),
                _ => return Err(Error::InvalidMode),
            };
            (0x0cu16, 0x0du16, 0x0eu16, 0x47u16, 0x00u16, res_val, fps_val)
        }
        VideoFormat::Ir8Bit | VideoFormat::Ir10Bit | VideoFormat::Ir10BitPacked => {
            let (res_val, fps_val) = match resolution {
                Resolution::High => {
                    // Work-around: need to briefly start+stop depth stream
                    // before high-res IR will work.
                    write_register(device, 0x0013, 0x01)?;
                    write_register(device, 0x0014, 0x1e)?;
                    write_register(device, 0x0006, 0x02)?;
                    write_register(device, 0x0006, 0x00)?;
                    (0x02u16, 0x0fu16)
                }
                Resolution::Medium => (0x01u16, 0x1eu16),
                _ => return Err(Error::InvalidMode),
            };
            (0x19u16, 0x1au16, 0x1bu16, 0x48u16, 0x00u16, res_val, fps_val)
        }
        VideoFormat::YuvRgb | VideoFormat::YuvRaw => {
            if resolution != Resolution::Medium {
                return Err(Error::InvalidMode);
            }
            (0x0cu16, 0x0du16, 0x0eu16, 0x47u16, 0x05u16, 0x01u16, 0x0fu16)
        }
    };

    write_register(device, mode_reg, mode_val)?;
    write_register(device, res_reg, res_val)?;
    write_register(device, fps_reg, fps_val)?;
    write_register(device, hflip_reg, 0x00)?; // disable hflip

    Ok(())
}

/// Stop the video stream.
pub fn stop_video_stream(device: &mut UsbDevice) -> Result<(), Error> {
    // Write a reset to the mode register for the currently active format.
    // cameras.c does not have an explicit stop_video, it just resets
    // the stream via USB isoc stop. We write register 0x0c=0x00 to idle.
    let _ = write_register(device, 0x000c, 0x00);
    let _ = write_register(device, 0x0006, 0x00);
    Ok(())
}

/// Set the LED state via the motor device.
pub fn set_led(motor: &mut UsbDevice, option: u8) -> Result<(), Error> {
    // LED control is on interface 0, control transfer:
    // request_type = 0x40, request = 0x06, value = option, index = 0x00
    let mut buf = [option];
    motor.control_transfer(0x40, 0x06, option as u16, 0x00, &mut buf, TIMEOUT)
        .map_err(|e| Error::Usb(e.to_string()))?;
    Ok(())
}

/// Set the tilt motor angle via the motor device.
pub fn set_tilt_degs(motor: &mut UsbDevice, angle: i8) -> Result<(), Error> {
    // Tilt control: request_type = 0x40, request = 0x31, value = angle
    let mut buf = [angle as u8];
    motor.control_transfer(0x40, 0x31, angle as u16, 0x00, &mut buf, TIMEOUT)
        .map_err(|e| Error::Usb(e.to_string()))?;
    Ok(())
}

/// Read the tilt state / accelerometer from the motor device.
pub fn read_tilt_state(motor: &mut UsbDevice, buf: &mut [u8; 10]) -> Result<(), Error> {
    motor.control_read(0xC0, 0x32, 0x00, 0x00, buf.as_mut_slice(), TIMEOUT)
        .map_err(|e| Error::Usb(e.to_string()))?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_send_cmd_payload_roundtrip() {
        // We can't open real hardware in a unit test, but we can verify
        // that the command packet builder produces the expected bytes.
        let addr = 0x0012u16;
        let value = 0x0003u16;
        let payload = protocol::write_register_cmd(addr, value);
        let cmd = protocol::assemble_cmd_packet(0x03, 1, &payload);

        assert_eq!(cmd.len(), 12);
        assert_eq!(cmd[0..2], [0x47, 0x4d]); // magic
        assert_eq!(cmd[4..6], [0x03, 0x00]); // cmd
        assert_eq!(cmd[8..10], [0x12, 0x00]); // addr
        assert_eq!(cmd[10..12], [0x03, 0x00]); // value
    }
}
