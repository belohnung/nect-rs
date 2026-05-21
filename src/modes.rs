//! Defines all supported video and depth frame modes (resolution,
//! format, buffer size, framerate, etc.).

use crate::{DepthFormat, FrameMode, Resolution, VideoFormat};

const fn video_mode(
    resolution: Resolution,
    format: VideoFormat,
    bytes: i32,
    width: i16,
    height: i16,
    data_bits_per_pixel: i8,
    padding_bits_per_pixel: i8,
    framerate: i8,
    is_valid: bool,
) -> FrameMode {
    FrameMode {
        resolution,
        video_format: Some(format),
        depth_format: None,
        bytes,
        width,
        height,
        data_bits_per_pixel,
        padding_bits_per_pixel,
        framerate,
        is_valid,
    }
}

const fn depth_mode(
    resolution: Resolution,
    format: DepthFormat,
    bytes: i32,
    width: i16,
    height: i16,
    data_bits_per_pixel: i8,
    padding_bits_per_pixel: i8,
    framerate: i8,
    is_valid: bool,
) -> FrameMode {
    FrameMode {
        resolution,
        video_format: None,
        depth_format: Some(format),
        bytes,
        width,
        height,
        data_bits_per_pixel,
        padding_bits_per_pixel,
        framerate,
        is_valid,
    }
}

/// All supported video modes.
pub const VIDEO_MODES: &[FrameMode] = &[
    video_mode(Resolution::High, VideoFormat::Rgb, 1280 * 1024 * 3, 1280, 1024, 24, 0, 10, true),
    video_mode(Resolution::Medium, VideoFormat::Rgb, 640 * 480 * 3, 640, 480, 24, 0, 30, true),
    video_mode(Resolution::High, VideoFormat::Bayer, 1280 * 1024, 1280, 1024, 8, 0, 10, true),
    video_mode(Resolution::Medium, VideoFormat::Bayer, 640 * 480, 640, 480, 8, 0, 30, true),
    video_mode(Resolution::High, VideoFormat::Ir8Bit, 1280 * 1024, 1280, 1024, 8, 0, 10, true),
    video_mode(Resolution::Medium, VideoFormat::Ir8Bit, 640 * 488, 640, 488, 8, 0, 30, true),
    video_mode(Resolution::High, VideoFormat::Ir10Bit, 1280 * 1024 * 2, 1280, 1024, 10, 6, 10, true),
    video_mode(Resolution::Medium, VideoFormat::Ir10Bit, 640 * 488 * 2, 640, 488, 10, 6, 30, true),
    video_mode(Resolution::High, VideoFormat::Ir10BitPacked, 1280 * 1024 * 10 / 8, 1280, 1024, 10, 0, 10, true),
    video_mode(Resolution::Medium, VideoFormat::Ir10BitPacked, 640 * 488 * 10 / 8, 640, 488, 10, 0, 30, true),
    video_mode(Resolution::Medium, VideoFormat::YuvRgb, 640 * 480 * 3, 640, 480, 24, 0, 15, true),
    video_mode(Resolution::Medium, VideoFormat::YuvRaw, 640 * 480 * 2, 640, 480, 16, 0, 15, true),
];

/// All supported depth modes.
pub const DEPTH_MODES: &[FrameMode] = &[
    depth_mode(Resolution::Medium, DepthFormat::Bit11, 640 * 480 * 2, 640, 480, 11, 5, 30, true),
    depth_mode(Resolution::Medium, DepthFormat::Bit10, 640 * 480 * 2, 640, 480, 10, 6, 30, true),
    depth_mode(Resolution::Medium, DepthFormat::Bit11Packed, 640 * 480 * 11 / 8, 640, 480, 11, 0, 30, true),
    depth_mode(Resolution::Medium, DepthFormat::Bit10Packed, 640 * 480 * 10 / 8, 640, 480, 10, 0, 30, true),
    depth_mode(Resolution::Medium, DepthFormat::Registered, 640 * 480 * 2, 640, 480, 16, 0, 30, true),
    depth_mode(Resolution::Medium, DepthFormat::Mm, 640 * 480 * 2, 640, 480, 16, 0, 30, true),
    depth_mode(Resolution::High, DepthFormat::Bit11, 0, 0, 0, 0, 0, 0, false),
    depth_mode(Resolution::High, DepthFormat::Bit10, 0, 0, 0, 0, 0, 0, false),
    depth_mode(Resolution::High, DepthFormat::Bit11Packed, 0, 0, 0, 0, 0, 0, false),
    depth_mode(Resolution::High, DepthFormat::Bit10Packed, 0, 0, 0, 0, 0, 0, false),
    depth_mode(Resolution::High, DepthFormat::Registered, 0, 0, 0, 0, 0, 0, false),
];

/// Look up a video mode by resolution and format.
pub fn lookup_video_mode(resolution: Resolution, format: VideoFormat) -> Option<&'static FrameMode> {
    VIDEO_MODES
        .iter()
        .find(|m| m.resolution == resolution && m.video_format == Some(format))
}

/// Look up a depth mode by resolution and format.
pub fn lookup_depth_mode(resolution: Resolution, format: DepthFormat) -> Option<&'static FrameMode> {
    DEPTH_MODES
        .iter()
        .find(|m| m.resolution == resolution && m.depth_format == Some(format))
}

/// Return the number of valid video modes.
pub fn num_video_modes() -> usize {
    VIDEO_MODES.iter().filter(|m| m.is_valid).count()
}

/// Return the number of valid depth modes.
pub fn num_depth_modes() -> usize {
    DEPTH_MODES.iter().filter(|m| m.is_valid).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_video_mode_lookup_rgb_high() {
        let mode = lookup_video_mode(Resolution::High, VideoFormat::Rgb).unwrap();
        assert_eq!(mode.width(), 1280);
        assert_eq!(mode.height(), 1024);
        assert_eq!(mode.framerate(), 10);
        assert_eq!(mode.bytes(), 1280 * 1024 * 3);
        assert!(mode.is_valid());
    }

    #[test]
    fn test_video_mode_lookup_bayer_medium() {
        let mode = lookup_video_mode(Resolution::Medium, VideoFormat::Bayer).unwrap();
        assert_eq!(mode.width(), 640);
        assert_eq!(mode.height(), 480);
        assert_eq!(mode.framerate(), 30);
        assert!(mode.is_valid());
    }

    #[test]
    fn test_depth_mode_lookup_11bit_medium() {
        let mode = lookup_depth_mode(Resolution::Medium, DepthFormat::Bit11).unwrap();
        assert_eq!(mode.width(), 640);
        assert_eq!(mode.height(), 480);
        assert_eq!(mode.bytes(), 640 * 480 * 2);
        assert!(mode.is_valid());
    }

    #[test]
    fn test_depth_mode_invalid_high() {
        let mode = lookup_depth_mode(Resolution::High, DepthFormat::Bit11).unwrap();
        assert!(!mode.is_valid());
    }

    #[test]
    fn test_num_video_modes() {
        assert_eq!(num_video_modes(), 12);
    }

    #[test]
    fn test_num_depth_modes() {
        assert_eq!(num_depth_modes(), 6);
    }
}
