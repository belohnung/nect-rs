//! Handles depth unpacking (11-bit packed to 16-bit, 10-bit packed to 16-bit/8-bit)
//! and video demosaicing (Bayer to RGB, UYVY to RGB, IR unpacking).

/// Unpack 8 packed 11-bit depth values from 11 bytes into 8 u16 values.
///
/// `raw` must be at least 11 bytes. `frame` receives 8 values.
pub fn unpack_8_pixels(raw: &[u8], frame: &mut [u16; 8]) {
    const BASE_MASK: u16 = 0x7FF;
    let r = &raw[..11];

    frame[0] = ((r[0] as u16) << 3) | ((r[1] as u16) >> 5);
    frame[1] = (((r[1] as u16) << 6) | ((r[2] as u16) >> 2)) & BASE_MASK;
    frame[2] = (((r[2] as u16) << 9) | ((r[3] as u16) << 1) | ((r[4] as u16) >> 7)) & BASE_MASK;
    frame[3] = (((r[4] as u16) << 4) | ((r[5] as u16) >> 4)) & BASE_MASK;
    frame[4] = (((r[5] as u16) << 7) | ((r[6] as u16) >> 1)) & BASE_MASK;
    frame[5] =
        (((r[6] as u16) << 10) | ((r[7] as u16) << 2) | ((r[8] as u16) >> 6)) & BASE_MASK;
    frame[6] = (((r[8] as u16) << 5) | ((r[9] as u16) >> 3)) & BASE_MASK;
    frame[7] = (((r[9] as u16) << 8) | (r[10] as u16)) & BASE_MASK;
}

/// Convert packed 11-bit depth to 16-bit. `n` must be a multiple of 8.
pub fn convert_packed11_to_16bit(raw: &[u8], frame: &mut [u16], n: usize) {
    assert_eq!(n % 8, 0);
    let mut raw_off = 0;
    let mut frame_off = 0;
    let mut remaining = n;
    let mut unpack = [0u16; 8];

    while remaining >= 8 {
        unpack_8_pixels(&raw[raw_off..], &mut unpack);
        frame[frame_off..frame_off + 8].copy_from_slice(&unpack);
        remaining -= 8;
        raw_off += 11;
        frame_off += 8;
    }
}

/// Convert packed elements with `vw` useful bits into 16-bit array.
///
/// * `src` - packed source data
/// * `dest` - destination u16 array
/// * `vw` - virtual width (bits per element)
/// * `n` - number of elements
pub fn convert_packed_to_16bit(src: &[u8], dest: &mut [u16], vw: usize, n: usize) {
    let mask = (1u32 << vw) - 1;
    let mut buffer: u32 = 0;
    let mut bits_in: usize = 0;
    let mut src_idx: usize = 0;

    for i in 0..n {
        while bits_in < vw {
            buffer = (buffer << 8) | (src[src_idx] as u32);
            src_idx += 1;
            bits_in += 8;
        }
        bits_in -= vw;
        dest[i] = ((buffer >> bits_in) & mask) as u16;
    }
}

/// Convert packed elements with `vw` useful bits into 8-bit array (drop LSB).
///
/// * `src` - packed source data
/// * `dest` - destination u8 array
/// * `vw` - virtual width (bits per element, must be >= 8)
/// * `n` - number of elements
pub fn convert_packed_to_8bit(src: &[u8], dest: &mut [u8], vw: usize, n: usize) {
    assert!(vw >= 8);
    let mut buffer: u32 = 0;
    let mut bits_in: usize = 0;
    let mut src_idx: usize = 0;

    for i in 0..n {
        while bits_in < vw {
            buffer = (buffer << 8) | (src[src_idx] as u32);
            src_idx += 1;
            bits_in += 8;
        }
        bits_in -= vw;
        dest[i] = (buffer >> (bits_in + vw - 8)) as u8;
    }
}

/// Convert UYVY (YCbCr 4:2:2) to RGB.
///
/// * `raw_buf` - UYVY input, 2 bytes per pixel pair (4 bytes per 2 pixels)
/// * `proc_buf` - RGB output, 3 bytes per pixel
/// * `width` - image width in pixels
/// * `height` - image height in pixels
pub fn convert_uyvy_to_rgb(raw_buf: &[u8], proc_buf: &mut [u8], width: usize, height: usize) {
    for y in 0..height {
        for x in (0..width).step_by(2) {
            let i = (width * y + x) * 2;
            let u = raw_buf[i] as i32;
            let y1 = raw_buf[i + 1] as i32;
            let v = raw_buf[i + 2] as i32;
            let y2 = raw_buf[i + 3] as i32;

            let r1 = (y1 - 16) * 1164 / 1000 + (v - 128) * 1596 / 1000;
            let g1 = (y1 - 16) * 1164 / 1000 - (v - 128) * 813 / 1000 - (u - 128) * 391 / 1000;
            let b1 = (y1 - 16) * 1164 / 1000 + (u - 128) * 2018 / 1000;
            let r2 = (y2 - 16) * 1164 / 1000 + (v - 128) * 1596 / 1000;
            let g2 = (y2 - 16) * 1164 / 1000 - (v - 128) * 813 / 1000 - (u - 128) * 391 / 1000;
            let b2 = (y2 - 16) * 1164 / 1000 + (u - 128) * 2018 / 1000;

            let out_i = (width * y + x) * 3;
            proc_buf[out_i] = clamp_u8(r1);
            proc_buf[out_i + 1] = clamp_u8(g1);
            proc_buf[out_i + 2] = clamp_u8(b1);
            proc_buf[out_i + 3] = clamp_u8(r2);
            proc_buf[out_i + 4] = clamp_u8(g2);
            proc_buf[out_i + 5] = clamp_u8(b2);
        }
    }
}

/// Convert Bayer-pattern raw image to RGB using bilinear interpolation.
///
/// Pattern arrangement:
/// ```text
/// G R G R
/// B G B G
/// G R G R
/// B G B G
/// ```
///
/// * `raw_buf` - Bayer raw input, 1 byte per pixel
/// * `proc_buf` - RGB output, 3 bytes per pixel
/// * `width` - image width
/// * `height` - image height
pub fn convert_bayer_to_rgb(raw_buf: &[u8], proc_buf: &mut [u8], width: usize, height: usize) {
    assert_eq!(raw_buf.len(), width * height);
    assert_eq!(proc_buf.len(), width * height * 3);

    let mut dst = 0usize;

    for y in 0..height {
        let prev_line = if y > 0 {
            &raw_buf[(y - 1) * width..y * width]
        } else {
            &raw_buf[width..2 * width] // mirror second line for top boundary
        };
        let cur_line = &raw_buf[y * width..(y + 1) * width];
        let next_line = if y < height - 1 {
            &raw_buf[(y + 1) * width..(y + 2) * width]
        } else {
            &raw_buf[(y - 1) * width..y * width] // mirror second-last line for bottom boundary
        };

        let y_odd = y & 1;

        for x in 0..width {
            let prev = if x > 0 { cur_line[x - 1] } else { cur_line[1] };
            let curr = cur_line[x];
            let next = if x < width - 1 { cur_line[x + 1] } else { cur_line[width - 2] };

            let above_prev = if x > 0 { prev_line[x - 1] } else { prev_line[1] };
            let above_curr = prev_line[x];
            let above_next = if x < width - 1 { prev_line[x + 1] } else { prev_line[width - 2] };

            let below_prev = if x > 0 { next_line[x - 1] } else { next_line[1] };
            let below_curr = next_line[x];
            let below_next = if x < width - 1 { next_line[x + 1] } else { next_line[width - 2] };

            let (r, g, b) = if y_odd == 0 {
                if x & 1 == 0 {
                    // Configuration 1: curr=G, prev=B, next=R (horizontal R/B)
                    // above_curr=R, below_curr=R (vertical R)
                    // prev=B, next=B (horizontal B)
                    (
                        ((prev as u16 + next as u16) >> 1) as u8,
                        curr,
                        ((above_curr as u16 + below_curr as u16) >> 1) as u8,
                    )
                } else {
                    // Configuration 2: curr=R, surrounding are G/B
                    (
                        curr,
                        ((prev as u16 + next as u16 + above_curr as u16 + below_curr as u16) >> 2) as u8,
                        ((above_prev as u16 + above_next as u16 + below_prev as u16 + below_next as u16) >> 2) as u8,
                    )
                }
            } else {
                if x & 1 == 0 {
                    // Configuration 3: curr=B, surrounding are G/R
                    (
                        ((above_prev as u16 + above_next as u16 + below_prev as u16 + below_next as u16) >> 2) as u8,
                        ((prev as u16 + next as u16 + above_curr as u16 + below_curr as u16) >> 2) as u8,
                        curr,
                    )
                } else {
                    // Configuration 4: curr=G, prev=R, next=B (horizontal R/B)
                    // above_curr=B, below_curr=B (vertical B)
                    // prev=R, next=R (horizontal R)
                    (
                        ((above_curr as u16 + below_curr as u16) >> 1) as u8,
                        curr,
                        ((prev as u16 + next as u16) >> 1) as u8,
                    )
                }
            };

            proc_buf[dst] = r;
            proc_buf[dst + 1] = g;
            proc_buf[dst + 2] = b;
            dst += 3;
        }
    }
}

#[inline]
fn clamp_u8(v: i32) -> u8 {
    if v < 0 {
        0
    } else if v > 255 {
        255
    } else {
        v as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unpack_8_pixels() {
        let raw: [u8; 11] = [0; 11];
        let mut frame = [0u16; 8];
        unpack_8_pixels(&raw, &mut frame);
        assert_eq!(frame[0], 0);
    }

    #[test]
    fn test_convert_packed11_to_16bit() {
        // 8 pixels packed into 11 bytes, all zeros
        let raw = vec![0u8; 11];
        let mut frame = vec![0u16; 8];
        convert_packed11_to_16bit(&raw, &mut frame, 8);
        assert!(frame.iter().all(|&v| v == 0));
    }

    #[test]
    fn test_convert_packed_to_16bit_10bit() {
        // 3 elements x 10 bits = 30 bits, so 4 bytes are required.
        let src = vec![0xFF; 4];
        let mut dest = vec![0u16; 3];
        convert_packed_to_16bit(&src, &mut dest, 10, 3);
        // Just verify it doesn't panic and produces values in range
        assert!(dest.iter().all(|&v| v < 1024));
    }

    #[test]
    fn test_convert_uyvy_to_rgb() {
        let raw = vec![128, 255, 128, 255]; // U=128, Y1=255, V=128, Y2=255 (white)
        let mut rgb = vec![0u8; 6];
        convert_uyvy_to_rgb(&raw, &mut rgb, 2, 1);
        assert_eq!(rgb[0], 255);
        assert_eq!(rgb[1], 255);
        assert_eq!(rgb[2], 255);
        assert_eq!(rgb[3], 255);
        assert_eq!(rgb[4], 255);
        assert_eq!(rgb[5], 255);
    }

}
