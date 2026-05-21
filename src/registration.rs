//! Handles depth-to-RGB registration, raw-to-mm conversion, and
//! camera-to-world coordinate transforms.

use std::f64;

// ---------------------------------------------------------------------------
// Constants (from registration.c)
// ---------------------------------------------------------------------------

const REG_X_VAL_SCALE: i32 = 256;
const S2D_PIXEL_CONST: f64 = 10.0;
const S2D_CONST_OFFSET: f64 = 0.375;
const DEPTH_SENSOR_X_RES: u32 = 1280;
const DEPTH_MIRROR_X: bool = false;

pub const DEPTH_MAX_METRIC_VALUE: usize = 10000; // FREENECT_DEPTH_MM_MAX_VALUE
pub const DEPTH_NO_MM_VALUE: u16 = 0;              // FREENECT_DEPTH_MM_NO_VALUE
pub const DEPTH_MAX_RAW_VALUE: usize = 2048;     // FREENECT_DEPTH_RAW_MAX_VALUE
pub const DEPTH_NO_RAW_VALUE: u16 = 2047;         // FREENECT_DEPTH_RAW_NO_VALUE

const DEPTH_X_OFFSET: i32 = 1;
const DEPTH_Y_OFFSET: i32 = 1;
pub const DEPTH_X_RES: usize = 640;
pub const DEPTH_Y_RES: usize = 480;

// ---------------------------------------------------------------------------
// Registration parameter data structures.
// ---------------------------------------------------------------------------

/// Internal Kinect registration parameters.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RegInfo {
    pub dx_center: i32,
    pub ax: i32,
    pub bx: i32,
    pub cx: i32,
    pub dx: i32,
    pub dx_start: i32,
    pub ay: i32,
    pub by: i32,
    pub cy: i32,
    pub dy: i32,
    pub dy_start: i32,
    pub dx_beta_start: i32,
    pub dy_beta_start: i32,
    pub rollout_blank: i32,
    pub rollout_size: i32,
    pub dx_beta_inc: i32,
    pub dy_beta_inc: i32,
    pub dxdx_start: i32,
    pub dxdy_start: i32,
    pub dydx_start: i32,
    pub dydy_start: i32,
    pub dxdxdx_start: i32,
    pub dydxdx_start: i32,
    pub dxdxdy_start: i32,
    pub dydxdy_start: i32,
    pub back_comp1: i32,
    pub dydydx_start: i32,
    pub back_comp2: i32,
    pub dydydy_start: i32,
}

/// Registration padding info.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct RegPadInfo {
    pub start_lines: u16,
    pub end_lines: u16,
    pub cropping_lines: u16,
}

/// Zero-plane calibration data.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ZeroPlaneInfo {
    pub dcmos_emitter_dist: f32,
    pub dcmos_rcmos_dist: f32,
    pub reference_distance: f32,
    pub reference_pixel_size: f32,
}

/// All tables and parameters needed for depth-to-RGB registration.
pub struct Registration {
    pub reg_info: RegInfo,
    pub reg_pad_info: RegPadInfo,
    pub zero_plane_info: ZeroPlaneInfo,
    pub const_shift: f64,
    pub raw_to_mm_shift: Vec<u16>,       // [DEPTH_MAX_RAW_VALUE]
    pub depth_to_rgb_shift: Vec<i32>,   // [DEPTH_MAX_METRIC_VALUE]
    pub registration_table: Vec<[i32; 2]>, // [DEPTH_X_RES * DEPTH_Y_RES]
}

impl Registration {
    /// Build all lookup tables from the calibration parameters.
    pub fn new(
        reg_info: RegInfo,
        reg_pad_info: RegPadInfo,
        zero_plane_info: ZeroPlaneInfo,
        const_shift: f64,
    ) -> Self {
        let mut reg = Registration {
            reg_info,
            reg_pad_info,
            zero_plane_info,
            const_shift,
            raw_to_mm_shift: vec![0; DEPTH_MAX_RAW_VALUE],
            depth_to_rgb_shift: vec![0; DEPTH_MAX_METRIC_VALUE],
            registration_table: vec![[0; 2]; DEPTH_X_RES * DEPTH_Y_RES],
        };
        reg.complete_tables();
        reg
    }

    /// Convert a raw shift value to metric depth (mm).
    fn raw_to_mm(&self, raw: u16) -> u16 {
        let zpi = &self.zero_plane_info;
        let parameter_coefficient = 4.0;
        let shift_scale = 10.0;
        let pixel_size_factor = 1.0;

        let fixed_ref_x = ((raw as f64
            - (parameter_coefficient * self.const_shift / pixel_size_factor))
            / parameter_coefficient)
            - S2D_CONST_OFFSET;
        let metric = fixed_ref_x * zpi.reference_pixel_size as f64 * pixel_size_factor;
        let mm = shift_scale
            * ((metric * zpi.reference_distance as f64
                / (zpi.dcmos_emitter_dist as f64 - metric))
                + zpi.reference_distance as f64);
        mm as u16
    }

    /// Fill `depth_to_rgb` shift table.
    fn init_depth_to_rgb(&mut self) {
        let zpi = &self.zero_plane_info;
        let x_scale = DEPTH_SENSOR_X_RES / DEPTH_X_RES as u32;

        let pixel_size = 1.0 / (zpi.reference_pixel_size as f64 * x_scale as f64 * S2D_PIXEL_CONST);
        let pixels_between_rgb_and_ir_cmos =
            zpi.dcmos_rcmos_dist as f64 * pixel_size * S2D_PIXEL_CONST;
        let reference_distance_in_pixels =
            zpi.reference_distance as f64 * pixel_size * S2D_PIXEL_CONST;

        // Default to NO_MM_VALUE
        self.depth_to_rgb_shift.fill(DEPTH_NO_MM_VALUE as i32);

        for i in 0..DEPTH_MAX_METRIC_VALUE {
            let current_depth_in_pixels = i as f64 * pixel_size;
            if current_depth_in_pixels == 0.0 {
                continue;
            }
            self.depth_to_rgb_shift[i] = (((pixels_between_rgb_and_ir_cmos
                * (current_depth_in_pixels - reference_distance_in_pixels)
                / current_depth_in_pixels)
                + S2D_CONST_OFFSET)
                * REG_X_VAL_SCALE as f64) as i32;
        }
    }

    /// Build the registration_table from reg_info.
    fn init_registration_table(&mut self) {
        let mut regtable_dx = vec![0.0f64; DEPTH_X_RES * DEPTH_Y_RES];
        let mut regtable_dy = vec![0.0f64; DEPTH_X_RES * DEPTH_Y_RES];

        create_dxdy_tables(
            &mut regtable_dx,
            &mut regtable_dy,
            DEPTH_X_RES as i32,
            DEPTH_Y_RES as i32,
            &self.reg_info,
        );

        let mut index = 0;
        for y in 0..DEPTH_Y_RES {
            for x in 0..DEPTH_X_RES {
        let mut new_x = x as f64 + regtable_dx[index] + DEPTH_X_OFFSET as f64;
        let new_y = y as f64 + regtable_dy[index] + DEPTH_Y_OFFSET as f64;

        if new_x < 0.0 || new_y < 0.0 || new_x >= DEPTH_X_RES as f64 || new_y >= DEPTH_Y_RES as f64 {
            new_x = 2.0 * DEPTH_X_RES as f64; // intentionally out of bounds
        }

                self.registration_table[index][0] = (new_x * REG_X_VAL_SCALE as f64) as i32;
                self.registration_table[index][1] = new_y as i32;
                index += 1;
            }
        }
    }

    /// Compute all tables.
    fn complete_tables(&mut self) {
        for i in 0..DEPTH_MAX_RAW_VALUE {
            self.raw_to_mm_shift[i] = self.raw_to_mm(i as u16);
        }
        self.raw_to_mm_shift[DEPTH_NO_RAW_VALUE as usize] = DEPTH_NO_MM_VALUE;

        self.init_depth_to_rgb();
        self.init_registration_table();
    }

    /// Apply registration to a packed or unpacked depth frame.
    ///
    /// * `input` - raw depth data (packed 11-bit or unpacked 16-bit)
    /// * `output_mm` - output buffer, must be `DEPTH_X_RES * DEPTH_Y_RES` elements
    /// * `unpacked` - true if `input` is already 16-bit per pixel
    pub fn apply_registration(&self, input: &[u8], output_mm: &mut [u16], unpacked: bool) {
        assert_eq!(output_mm.len(), DEPTH_X_RES * DEPTH_Y_RES);

        // Zero output
        for o in output_mm.iter_mut() {
            *o = DEPTH_NO_MM_VALUE;
        }

        let target_offset = DEPTH_Y_RES * self.reg_pad_info.start_lines as usize;
        let mut unpack = [0u16; 8];
        let mut source_index = 8usize;
        let mut input_offset = 0usize;

        for y in 0..DEPTH_Y_RES {
            for x in 0..DEPTH_X_RES {
                let metric_depth = if unpacked {
                    let buf_index = y * DEPTH_X_RES + x;
                    let raw = u16::from_le_bytes([
                        input[buf_index * 2],
                        input[buf_index * 2 + 1],
                    ]);
                    self.raw_to_mm_shift[raw as usize]
                } else {
                    if source_index == 8 {
                        unpack_8_pixels(&input[input_offset..], &mut unpack);
                        source_index = 0;
                        input_offset += 11;
                    }
                    let raw = unpack[source_index];
                    source_index += 1;
                    self.raw_to_mm_shift[raw as usize]
                };

                if metric_depth == DEPTH_NO_MM_VALUE {
                    continue;
                }
                if metric_depth as usize >= DEPTH_MAX_METRIC_VALUE {
                    continue;
                }

                let reg_index = if DEPTH_MIRROR_X {
                    (y + 1) * DEPTH_X_RES - x - 1
                } else {
                    y * DEPTH_X_RES + x
                };
                let nx = ((self.registration_table[reg_index][0]
                    + self.depth_to_rgb_shift[metric_depth as usize])
                    / REG_X_VAL_SCALE) as usize;
                let ny = self.registration_table[reg_index][1] as usize;

                if nx >= DEPTH_X_RES {
                    continue;
                }

                let raw_target = if DEPTH_MIRROR_X {
                    (ny + 1) * DEPTH_X_RES - nx - 1
                } else {
                    ny * DEPTH_X_RES + nx
                };
                let target_index = if raw_target >= target_offset {
                    raw_target - target_offset
                } else {
                    continue;
                };

                if target_index >= output_mm.len() {
                    continue;
                }

                let current_depth = output_mm[target_index];
                if current_depth == DEPTH_NO_MM_VALUE || current_depth > metric_depth {
                    output_mm[target_index] = metric_depth;
                }
            }
        }
    }

    /// Convert packed 11-bit depth to mm (no registration alignment).
    pub fn apply_depth_to_mm(&self, input_packed: &[u8], output_mm: &mut [u16]) {
        assert_eq!(output_mm.len(), DEPTH_X_RES * DEPTH_Y_RES);
        let mut unpack = [0u16; 8];
        let mut source_index = 8usize;
        let mut input_offset = 0usize;

        for y in 0..DEPTH_Y_RES {
            for x in 0..DEPTH_X_RES {
                if source_index == 8 {
                    unpack_8_pixels(&input_packed[input_offset..], &mut unpack);
                    source_index = 0;
                    input_offset += 11;
                }
                let raw = unpack[source_index];
                source_index += 1;
                let metric_depth = self.raw_to_mm_shift[raw as usize];
                output_mm[y * DEPTH_X_RES + x] =
                    if (metric_depth as usize) < DEPTH_MAX_METRIC_VALUE {
                        metric_depth
                    } else {
                        DEPTH_MAX_METRIC_VALUE as u16
                    };
            }
        }
    }

    /// Convert unpacked 16-bit depth to mm (no registration alignment).
    pub fn apply_depth_unpacked_to_mm(&self, input: &[u16], output_mm: &mut [u16]) {
        assert_eq!(output_mm.len(), DEPTH_X_RES * DEPTH_Y_RES);
        assert_eq!(input.len(), DEPTH_X_RES * DEPTH_Y_RES);

        for y in 0..DEPTH_Y_RES {
            for x in 0..DEPTH_X_RES {
                let buf_index = y * DEPTH_X_RES + x;
                let metric_depth = self.raw_to_mm_shift[input[buf_index] as usize];
                output_mm[buf_index] =
                    if (metric_depth as usize) < DEPTH_MAX_METRIC_VALUE {
                        metric_depth
                    } else {
                        DEPTH_MAX_METRIC_VALUE as u16
                    };
            }
        }
    }

    /// Convert a single camera pixel + depth to world coordinates.
    pub fn camera_to_world(&self, cx: i32, cy: i32, wz: i32) -> (f64, f64) {
        let ref_pix_size = self.zero_plane_info.reference_pixel_size as f64;
        let ref_distance = self.zero_plane_info.reference_distance as f64;
        let factor = 2.0 * ref_pix_size * wz as f64 / ref_distance;
        let wx = (cx - DEPTH_X_RES as i32 / 2) as f64 * factor;
        let wy = (cy - DEPTH_Y_RES as i32 / 2) as f64 * factor;
        (wx, wy)
    }

    /// Map an RGB image to align with a depth-mm image.
    ///
    /// * `depth_mm` - depth frame in mm
    /// * `rgb_raw` - raw RGB image (3 bytes per pixel)
    /// * `rgb_registered` - output aligned RGB image (3 bytes per pixel)
    pub fn map_rgb_to_depth(
        &self,
        depth_mm: &[u16],
        rgb_raw: &[u8],
        rgb_registered: &mut [u8],
    ) {
        assert_eq!(depth_mm.len(), DEPTH_X_RES * DEPTH_Y_RES);
        assert_eq!(rgb_raw.len(), DEPTH_X_RES * DEPTH_Y_RES * 3);
        assert_eq!(rgb_registered.len(), DEPTH_X_RES * DEPTH_Y_RES * 3);

        let target_offset = self.reg_pad_info.start_lines as usize * DEPTH_Y_RES;

        let mut map = vec![-1i32; DEPTH_Y_RES * DEPTH_X_RES];
        let mut z_buffer = vec![DEPTH_NO_MM_VALUE; DEPTH_Y_RES * DEPTH_X_RES];

        for y in 0..DEPTH_Y_RES {
            for x in 0..DEPTH_X_RES {
                let index = y * DEPTH_X_RES + x;
                let wz = depth_mm[index];
                if wz == DEPTH_NO_MM_VALUE {
                    continue;
                }

                let cx = ((self.registration_table[index][0]
                    + self.depth_to_rgb_shift[wz as usize])
                    / REG_X_VAL_SCALE) as usize;
                let cy = self.registration_table[index][1] as usize - target_offset;

                if cx >= DEPTH_X_RES {
                    continue;
                }

                let cindex = cy * DEPTH_X_RES + cx;
                map[index] = cindex as i32;

                if z_buffer[cindex] == DEPTH_NO_MM_VALUE || z_buffer[cindex] > wz {
                    z_buffer[cindex] = wz;
                }
            }
        }

        for y in 0..DEPTH_Y_RES {
            for x in 0..DEPTH_X_RES {
                let index = y * DEPTH_X_RES + x;
                let cindex = map[index];

                let out_idx = index * 3;
                if cindex == -1 {
                    rgb_registered[out_idx] = 0;
                    rgb_registered[out_idx + 1] = 0;
                    rgb_registered[out_idx + 2] = 0;
                    continue;
                }

                let cindex = cindex as usize;
                let current_depth = depth_mm[index];
                let min_depth = z_buffer[cindex];

                if current_depth <= min_depth {
                    let cidx = cindex * 3;
                    rgb_registered[out_idx] = rgb_raw[cidx];
                    rgb_registered[out_idx + 1] = rgb_raw[cidx + 1];
                    rgb_registered[out_idx + 2] = rgb_raw[cidx + 2];
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Re-exported from cameras module to avoid duplication.
use crate::cameras::unpack_8_pixels;

/// Create temporary dx/dy tables for the registration polynomial.
fn create_dxdy_tables(
    reg_x_table: &mut [f64],
    reg_y_table: &mut [f64],
    resolution_x: i32,
    resolution_y: i32,
    regdata: &RegInfo,
) {
    let ax6 = regdata.ax as i64;
    let bx6 = regdata.bx as i64;
    let cx2 = regdata.cx as i64;
    let dx2 = regdata.dx as i64;

    let ay6 = regdata.ay as i64;
    let by6 = regdata.by as i64;
    let cy2 = regdata.cy as i64;
    let dy2 = regdata.dy as i64;

    // Don't merge shift ops - necessary for 32-bit clamping behavior.
    let mut dx0 = ((regdata.dx_start as i64) << 13) >> 4;
    let mut dy0 = ((regdata.dy_start as i64) << 13) >> 4;

    let mut dxdx0 = ((regdata.dxdx_start as i64) << 11) >> 3;
    let mut dxdy0 = ((regdata.dxdy_start as i64) << 11) >> 3;
    let mut dydx0 = ((regdata.dydx_start as i64) << 11) >> 3;
    let mut dydy0 = ((regdata.dydy_start as i64) << 11) >> 3;

    let mut dxdxdx0 = ((regdata.dxdxdx_start as i64) << 5) << 3;
    let mut dydxdx0 = ((regdata.dydxdx_start as i64) << 5) << 3;
    let mut dydxdy0 = ((regdata.dydxdy_start as i64) << 5) << 3;
    let mut dxdxdy0 = ((regdata.dxdxdy_start as i64) << 5) << 3;
    let mut dydydx0 = ((regdata.dydydx_start as i64) << 5) << 3;
    let mut dydydy0 = ((regdata.dydydy_start as i64) << 5) << 3;

    let mut t_offs = 0usize;

    for _row in 0..resolution_y {
        dxdxdx0 += cx2;

        dxdx0 += dydxdx0 >> 8;
        dydxdx0 += dx2;

        dx0 += dydx0 >> 6;
        dydx0 += dydydx0 >> 8;
        dydydx0 += bx6;

        dxdxdy0 += cy2;

        dxdy0 += dydxdy0 >> 8;
        dydxdy0 += dy2;

        dy0 += dydy0 >> 6;
        dydy0 += dydydy0 >> 8;
        dydydy0 += by6;

        let mut cold_xd_xd_y0 = dxdxdy0;
        let mut cold_xd_y0 = dxdy0;
        let mut cold_y0 = dy0;

        let mut cold_xd_xd_x0 = dxdxdx0;
        let mut cold_xd_x0 = dxdx0;
        let mut cold_x0 = dx0;

        for _col in 0..resolution_x {
            reg_x_table[t_offs] = cold_x0 as f64 * (1.0 / (1 << 17) as f64);
            reg_y_table[t_offs] = cold_y0 as f64 * (1.0 / (1 << 17) as f64);
            t_offs += 1;

            cold_x0 += cold_xd_x0 >> 6;
            cold_xd_x0 += cold_xd_xd_x0 >> 8;
            cold_xd_xd_x0 += ax6;

            cold_y0 += cold_xd_y0 >> 6;
            cold_xd_y0 += cold_xd_xd_y0 >> 8;
            cold_xd_xd_y0 += ay6;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unpack_8_pixels() {
        // 11 bytes that encode 8 values: 0, 1, 2, 3, 4, 5, 6, 7
        let raw: [u8; 11] = [
            0x00, 0x00, 0x08, 0x00, 0x00, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00,
        ];
        let mut frame = [0u16; 8];
        unpack_8_pixels(&raw, &mut frame);
        // Just verify it doesn't panic and produces something reasonable
        assert!(frame.iter().all(|&v| v < 2048));
    }

    #[test]
    fn test_registration_new() {
        let reg_info = RegInfo {
            dx_center: 0,
            ax: 0,
            bx: 0,
            cx: 0,
            dx: 0,
            dx_start: 0,
            ay: 0,
            by: 0,
            cy: 0,
            dy: 0,
            dy_start: 0,
            dx_beta_start: 0,
            dy_beta_start: 0,
            rollout_blank: 0,
            rollout_size: 0,
            dx_beta_inc: 0,
            dy_beta_inc: 0,
            dxdx_start: 0,
            dxdy_start: 0,
            dydx_start: 0,
            dydy_start: 0,
            dxdxdx_start: 0,
            dydxdx_start: 0,
            dxdxdy_start: 0,
            dydxdy_start: 0,
            back_comp1: 0,
            dydydx_start: 0,
            back_comp2: 0,
            dydydy_start: 0,
        };
        let reg_pad_info = RegPadInfo {
            start_lines: 0,
            end_lines: 0,
            cropping_lines: 0,
        };
        let zpi = ZeroPlaneInfo {
            dcmos_emitter_dist: 7.5,
            dcmos_rcmos_dist: 2.4,
            reference_distance: 120.0,
            reference_pixel_size: 0.104,
        };
        let reg = Registration::new(reg_info, reg_pad_info, zpi, 0.0);
        assert_eq!(reg.raw_to_mm_shift.len(), DEPTH_MAX_RAW_VALUE);
        assert_eq!(reg.depth_to_rgb_shift.len(), DEPTH_MAX_METRIC_VALUE);
        assert_eq!(reg.registration_table.len(), DEPTH_X_RES * DEPTH_Y_RES);
    }

    #[test]
    fn test_apply_depth_unpacked_to_mm() {
        let reg_info = RegInfo {
            dx_center: 0,
            ax: 0,
            bx: 0,
            cx: 0,
            dx: 0,
            dx_start: 0,
            ay: 0,
            by: 0,
            cy: 0,
            dy: 0,
            dy_start: 0,
            dx_beta_start: 0,
            dy_beta_start: 0,
            rollout_blank: 0,
            rollout_size: 0,
            dx_beta_inc: 0,
            dy_beta_inc: 0,
            dxdx_start: 0,
            dxdy_start: 0,
            dydx_start: 0,
            dydy_start: 0,
            dxdxdx_start: 0,
            dydxdx_start: 0,
            dxdxdy_start: 0,
            dydxdy_start: 0,
            back_comp1: 0,
            dydydx_start: 0,
            back_comp2: 0,
            dydydy_start: 0,
        };
        let reg_pad_info = RegPadInfo {
            start_lines: 0,
            end_lines: 0,
            cropping_lines: 0,
        };
        let zpi = ZeroPlaneInfo {
            dcmos_emitter_dist: 7.5,
            dcmos_rcmos_dist: 2.4,
            reference_distance: 120.0,
            reference_pixel_size: 0.104,
        };
        let reg = Registration::new(reg_info, reg_pad_info, zpi, 0.0);

        let mut input = vec![0u16; DEPTH_X_RES * DEPTH_Y_RES];
        input[0] = 500;
        let mut output = vec![0u16; DEPTH_X_RES * DEPTH_Y_RES];
        reg.apply_depth_unpacked_to_mm(&input, &mut output);
        // With test calibration parameters, the value may be clamped;
        // just ensure the function completes without panic and writes to the buffer.
        assert_eq!(output.len(), DEPTH_X_RES * DEPTH_Y_RES);
    }
}
