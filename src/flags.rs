//! Pure-Rust helpers for Kinect camera flag/exposure logic.
//!
//! This module provides the high-level flag mapping and exposure math.

/// Flag bitmasks for camera options.
pub const AUTO_EXPOSURE: u32 = 1 << 14;
pub const AUTO_FLICKER: u32 = 1 << 7;
pub const AUTO_WHITE_BALANCE: u32 = 1 << 1;
pub const RAW_COLOR: u32 = 1 << 4;
pub const MIRROR_DEPTH: u32 = 1 << 16;
pub const MIRROR_VIDEO: u32 = 1 << 17;
pub const NEAR_MODE: u32 = 1 << 18; // K4W only

/// Mapping from high-level flags to hardware register addresses.
pub fn register_for_flag(flag: u32) -> Option<u16> {
    match flag {
        MIRROR_DEPTH => Some(0x17),
        MIRROR_VIDEO => Some(0x47),
        _ => None,
    }
}

/// Shutter-width to exposure conversion constants.
pub const SHUTTER_WIDTH_TO_EXP_RGB: f64 = 54.21;
pub const SHUTTER_WIDTH_TO_EXP_YUV: f64 = 63.25;

/// Convert shutter width (register value) to exposure microseconds.
pub fn shutter_to_exposure_us(shutter_width: u16, is_rgb: bool) -> f64 {
    let factor = if is_rgb {
        SHUTTER_WIDTH_TO_EXP_RGB
    } else {
        SHUTTER_WIDTH_TO_EXP_YUV
    };
    shutter_width as f64 * factor
}

/// Convert desired exposure microseconds to shutter width (register value).
pub fn exposure_us_to_shutter(exposure_us: f64, is_rgb: bool) -> u16 {
    let factor = if is_rgb {
        SHUTTER_WIDTH_TO_EXP_RGB
    } else {
        SHUTTER_WIDTH_TO_EXP_YUV
    };
    (exposure_us / factor) as u16
}

/// Clamp IR brightness to valid range [1, 50].
pub fn clamp_ir_brightness(brightness: u16) -> u16 {
    brightness.clamp(1, 50)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_for_flag() {
        assert_eq!(register_for_flag(MIRROR_DEPTH), Some(0x17));
        assert_eq!(register_for_flag(MIRROR_VIDEO), Some(0x47));
        assert_eq!(register_for_flag(AUTO_EXPOSURE), None);
    }

    #[test]
    fn test_exposure_conversion() {
        let us = shutter_to_exposure_us(100, true);
        let shutter = exposure_us_to_shutter(us, true);
        assert_eq!(shutter, 100);
    }

    #[test]
    fn test_ir_brightness_clamp() {
        assert_eq!(clamp_ir_brightness(0), 1);
        assert_eq!(clamp_ir_brightness(25), 25);
        assert_eq!(clamp_ir_brightness(100), 50);
    }
}
