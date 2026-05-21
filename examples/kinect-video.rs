use nect_rs::{Context, DeviceFlags, Resolution, VideoFormat};
use minifb::{Key, Window, WindowOptions};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const WIDTH: usize = 640;
const HEIGHT: usize = 480;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::new()?;
    ctx.select_subdevices(DeviceFlags::CAMERA);

    let num = ctx.num_devices()?;
    println!("Devices found: {}", num);
    if num == 0 {
        return Ok(());
    }

    let frame_ready = Arc::new(AtomicBool::new(false));
    let latest_pixels: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(vec![0u32; WIDTH * HEIGHT]));

    {
        let mut dev = ctx.open_device(0)?;

        let video_mode = nect_rs::Device::find_video_mode(Resolution::Medium, VideoFormat::Rgb);
        dev.set_video_mode(video_mode)?;

        let ready = Arc::clone(&frame_ready);
        let pixels = Arc::clone(&latest_pixels);
        dev.set_video_callback(move |data: &[u8], _timestamp: u32| {
            let mut buf = pixels.lock().unwrap();
            for (i, pixel) in buf.iter_mut().enumerate() {
                let r = data[i * 3] as u32;
                let g = data[i * 3 + 1] as u32;
                let b = data[i * 3 + 2] as u32;
                *pixel = (r << 16) | (g << 8) | b;
            }
            ready.store(true, std::sync::atomic::Ordering::Relaxed);
        });

        dev.start_video()?;
        println!("Video stream started. Opening window...");

        let mut window = Window::new(
            "Kinect RGB Stream - Press ESC to exit",
            WIDTH,
            HEIGHT,
            WindowOptions::default(),
        )?;

        window.set_target_fps(60);

        let mut frame_count = 0usize;
        let mut last_fps_update = Instant::now();

        while window.is_open() && !window.is_key_down(Key::Escape) {
            let _ = ctx.process_events_timeout(Duration::from_millis(10));

            if frame_ready.swap(false, Ordering::Relaxed) {
                let buf = latest_pixels.lock().unwrap();
                window.update_with_buffer(&buf, WIDTH, HEIGHT)?;
                frame_count += 1;
            } else {
                window.update_with_buffer(&latest_pixels.lock().unwrap(), WIDTH, HEIGHT)?;
            }

            if last_fps_update.elapsed() >= Duration::from_secs(1) {
                let fps = frame_count as u32;
                frame_count = 0;
                last_fps_update = Instant::now();
                window.set_title(&format!("Kinect RGB - {} FPS", fps));
            }
        }

        dev.stop_video()?;
    }

    println!("Done.");
    Ok(())
}
