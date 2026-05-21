//! Pure-Rust helpers for Kinect tilt/accelerometer unit-conversion math.

/// Accelerometer counts per G (from KXSD9 datasheet).
pub const COUNTS_PER_G: f64 = 819.0;

/// Standard gravity in m/s^2.
pub const GRAVITY: f64 = 9.80665;

/// Maximum tilt angle in degrees.
pub const MAX_TILT_ANGLE: i32 = 31;
/// Minimum tilt angle in degrees.
pub const MIN_TILT_ANGLE: i32 = -31;

/// Convert raw accelerometer counts to m/s^2.
pub fn counts_to_mks(counts: i16) -> f64 {
    counts as f64 / COUNTS_PER_G * GRAVITY
}

/// Convert raw tilt encoder value to degrees.
/// The motor reports angles doubled, so divide by 2.
pub fn raw_tilt_to_degrees(raw: i8) -> f64 {
    raw as f64 / 2.0
}

/// Clamp a desired tilt angle to the safe hardware range [-31, 31].
pub fn clamp_tilt_angle(angle: f64) -> f64 {
    angle.clamp(MIN_TILT_ANGLE as f64, MAX_TILT_ANGLE as f64)
}

/// Convert a raw accelerometer + tilt state into engineering units.
#[derive(Debug, Clone, Copy)]
pub struct TiltPhysics {
    pub accel_x_mps2: f64,
    pub accel_y_mps2: f64,
    pub accel_z_mps2: f64,
    pub tilt_degrees: f64,
}

impl TiltPhysics {
    /// Build from raw sensor readings.
    pub fn from_raw(accel_x: i16, accel_y: i16, accel_z: i16, tilt_raw: i8) -> Self {
        TiltPhysics {
            accel_x_mps2: counts_to_mks(accel_x),
            accel_y_mps2: counts_to_mks(accel_y),
            accel_z_mps2: counts_to_mks(accel_z),
            tilt_degrees: raw_tilt_to_degrees(tilt_raw),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_counts_to_mks() {
        // 819 counts = 1 G = 9.80665 m/s^2
        assert!((counts_to_mks(819) - GRAVITY).abs() < 0.001);
        assert!((counts_to_mks(0) - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_raw_tilt_to_degrees() {
        assert_eq!(raw_tilt_to_degrees(12), 6.0);
        assert_eq!(raw_tilt_to_degrees(-12), -6.0);
        assert_eq!(raw_tilt_to_degrees(0), 0.0);
    }

    #[test]
    fn test_clamp_tilt_angle() {
        assert_eq!(clamp_tilt_angle(0.0), 0.0);
        assert_eq!(clamp_tilt_angle(50.0), 31.0);
        assert_eq!(clamp_tilt_angle(-50.0), -31.0);
    }

    #[test]
    fn test_tilt_physics() {
        let tp = TiltPhysics::from_raw(819, 0, 0, 12);
        assert!((tp.accel_x_mps2 - GRAVITY).abs() < 0.001);
        assert_eq!(tp.tilt_degrees, 6.0);
    }
}
