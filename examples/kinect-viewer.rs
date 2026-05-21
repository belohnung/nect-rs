use nect_rs::{Context, DeviceFlags, Resolution, VideoFormat, DepthFormat};
use minifb::{Key, Window, WindowOptions};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const PANELS: usize = 3;
const WIN_W: usize = WIDTH * PANELS;
const WIN_H: usize = HEIGHT;

/// Convert an 11-bit raw depth value (0..2047) to a heatmap color (u32 0x00RRGGBB).
fn raw_depth_to_color(raw: u16) -> u32 {
    // Kinect v1 11-bit raw: closer = larger value, far = smaller value
    let t = (raw as f32 / 2047.0).clamp(0.0, 1.0);
    let (r, g, b) = if t < 0.25 {
        let u = t * 4.0;
        (0, (u * 255.0) as u8, 255)
    } else if t < 0.5 {
        let u = (t - 0.25) * 4.0;
        (0, 255, ((1.0 - u) * 255.0) as u8)
    } else if t < 0.75 {
        let u = (t - 0.5) * 4.0;
        ((u * 255.0) as u8, 255, 0)
    } else {
        let u = (t - 0.75) * 4.0;
        (255, ((1.0 - u) * 255.0) as u8, 0)
    };
    ((r as u32) << 16) | ((g as u32) << 8) | (b as u32)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::new()?;
    ctx.select_subdevices(DeviceFlags::CAMERA.union(DeviceFlags::MOTOR));

    let num = ctx.num_devices()?;
    println!("Devices found: {}", num);
    if num == 0 {
        return Ok(());
    }

    let is_ir_mode = Arc::new(AtomicBool::new(false));

    let rgb_pixels: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(vec![0u32; WIDTH * HEIGHT]));
    let depth_pixels: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(vec![0u32; WIDTH * HEIGHT]));
    let ir_pixels: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(vec![0u32; WIDTH * HEIGHT]));

    {
        let mut dev = ctx.open_device(0)?;

        let rgb_mode = nect_rs::Device::find_video_mode(Resolution::Medium, VideoFormat::Rgb);
        let ir_mode = nect_rs::Device::find_video_mode(Resolution::Medium, VideoFormat::Ir10Bit);

        let mode_flag = Arc::clone(&is_ir_mode);
        let rgb_px = Arc::clone(&rgb_pixels);
        let ir_px = Arc::clone(&ir_pixels);
        dev.set_video_callback(move |data: &[u8], _ts: u32| {
            if mode_flag.load(Ordering::Relaxed) {
                let mut buf = ir_px.lock().unwrap();
                let ir_u16 = unsafe {
                    std::slice::from_raw_parts(data.as_ptr() as *const u16, WIDTH * HEIGHT)
                };
                for i in 0..(WIDTH * HEIGHT) {
                    let gray = (ir_u16[i] >> 2).min(255) as u8;
                    let c = ((gray as u32) << 16) | ((gray as u32) << 8) | (gray as u32);
                    buf[i] = c;
                }
            } else {
                let mut buf = rgb_px.lock().unwrap();
                for i in 0..(WIDTH * HEIGHT) {
                    buf[i] = ((data[i * 3] as u32) << 16)
                        | ((data[i * 3 + 1] as u32) << 8)
                        | (data[i * 3 + 2] as u32);
                }
            }
        });

        let depth_mode = nect_rs::Device::find_depth_mode(Resolution::Medium, DepthFormat::Bit11Packed);
        dev.set_depth_mode(depth_mode)?;

        let px = Arc::clone(&depth_pixels);
        dev.set_depth_callback(move |data: &[u8], _ts: u32| {
            let mut buf = px.lock().unwrap();
            let mut unpacked = vec![0u16; WIDTH * HEIGHT];
            nect_rs::cameras::convert_packed11_to_16bit(data, &mut unpacked, WIDTH * HEIGHT);
            for i in 0..(WIDTH * HEIGHT) {
                buf[i] = raw_depth_to_color(unpacked[i]);
            }
        });

        dev.set_video_mode(rgb_mode)?;
        dev.start_video()?;
        dev.start_depth()?;

        println!("Streams started. Opening window...");
        println!("  LEFT:  RGB  (press SPACE to toggle IR)");
        println!("  MID:   Depth (raw heatmap, blue=close, red=far)");
        println!("  RIGHT: mirrors LEFT");
        println!("Press UP/DOWN to tilt the Kinect motor.");
        println!("Press LEFT/RIGHT to adjust exposure (RGB) or IR brightness (IR mode).");
        println!("Hold SHIFT+LEFT/RIGHT to adjust IR brightness from any mode.");
        println!("Press ESC to quit.");

        let mut window = Window::new(
            "Kinect: RGB + Depth + RGB - SPACE toggles IR",
            WIN_W,
            WIN_H,
            WindowOptions::default(),
        )?;
        window.set_target_fps(60);

        let mut composite = vec![0u32; WIN_W * WIN_H];
        let mut frame_count = 0usize;
        let mut last_fps_update = Instant::now();
        let mut show_ir = false;
        let mut tilt_angle = 0.0f64;
        let mut last_tilt_update = Instant::now();
        let mut exposure_us = dev.exposure_us().unwrap_or(10_000.0);
        let mut ir_brightness = dev.ir_brightness().unwrap_or(25);
        let mut last_camera_setting_update = Instant::now();

        while window.is_open() && !window.is_key_down(Key::Escape) {
            let _ = ctx.process_events_timeout(Duration::from_millis(5));

            if last_tilt_update.elapsed() >= Duration::from_millis(100) {
                let tilt_step = 2.0;
                let next_angle = if window.is_key_down(Key::Up) {
                    Some((tilt_angle + tilt_step).min(30.0))
                } else if window.is_key_down(Key::Down) {
                    Some((tilt_angle - tilt_step).max(-30.0))
                } else {
                    None
                };

                if let Some(angle) = next_angle {
                    if (angle - tilt_angle).abs() > f64::EPSILON {
                        tilt_angle = angle;
                        if let Err(e) = dev.set_tilt_degs(tilt_angle) {
                            eprintln!("Failed to set tilt angle: {}", e);
                        }
                    }
                    last_tilt_update = Instant::now();
                }
            }

            if last_camera_setting_update.elapsed() >= Duration::from_millis(100) {
                let adjust_left = window.is_key_down(Key::Left);
                let adjust_right = window.is_key_down(Key::Right);
                if adjust_left || adjust_right {
                    let shift_down = window.is_key_down(Key::LeftShift)
                        || window.is_key_down(Key::RightShift);

                    if show_ir || shift_down {
                        let delta = if adjust_right { 1i32 } else { -1i32 };
                        let next = (ir_brightness as i32 + delta).clamp(1, 50) as u16;
                        if next != ir_brightness {
                            ir_brightness = next;
                            if let Err(e) = dev.set_ir_brightness(ir_brightness) {
                                eprintln!("Failed to set IR brightness: {}", e);
                            }
                        }
                    } else {
                        let delta = if adjust_right { 500.0 } else { -500.0 };
                        let next = (exposure_us + delta).clamp(100.0, 60_000.0);
                        if (next - exposure_us).abs() > f64::EPSILON {
                            exposure_us = next;
                            if let Err(e) = dev.set_exposure_us(exposure_us) {
                                eprintln!("Failed to set exposure: {}", e);
                            }
                        }
                    }

                    last_camera_setting_update = Instant::now();
                }
            }

            if window.is_key_down(Key::Space) {
                show_ir = !show_ir;
                is_ir_mode.store(show_ir, Ordering::Relaxed);

                let _ = dev.stop_video();
                if show_ir {
                    dev.set_video_mode(ir_mode)?;
                    println!("Switched to IR mode");
                } else {
                    dev.set_video_mode(rgb_mode)?;
                    println!("Switched to RGB mode");
                }
                dev.start_video()?;

                // wait until key is released
                while window.is_key_down(Key::Space) {
                    let _ = ctx.process_events_timeout(Duration::from_millis(10));
                    window.update_with_buffer(&composite, WIN_W, WIN_H)?;
                }
            }

            {
                let left = if show_ir {
                    ir_pixels.lock().unwrap()
                } else {
                    rgb_pixels.lock().unwrap()
                };
                let depth = depth_pixels.lock().unwrap();

                for y in 0..HEIGHT {
                    let src_row = y * WIDTH;
                    let dst_row = y * WIN_W;
                    composite[dst_row..dst_row + WIDTH]
                        .copy_from_slice(&left[src_row..src_row + WIDTH]);
                    composite[dst_row + WIDTH..dst_row + WIDTH * 2]
                        .copy_from_slice(&depth[src_row..src_row + WIDTH]);
                    composite[dst_row + WIDTH * 2..dst_row + WIDTH * 3]
                        .copy_from_slice(&left[src_row..src_row + WIDTH]);
                }
            }

            window.update_with_buffer(&composite, WIN_W, WIN_H)?;
            frame_count += 1;

            if last_fps_update.elapsed() >= Duration::from_secs(1) {
                let fps = frame_count as u32;
                frame_count = 0;
                last_fps_update = Instant::now();
                let mode = if show_ir { "IR" } else { "RGB" };
                window.set_title(&format!(
                    "Kinect: {} + Depth + {} - {} FPS - Tilt {:.0} deg - Exp {:.0} us - IR {}",
                    mode, mode, fps, tilt_angle, exposure_us, ir_brightness
                ));
            }
        }

        dev.stop_depth()?;
        dev.stop_video()?;
    }

    println!("Done.");
    Ok(())
}
