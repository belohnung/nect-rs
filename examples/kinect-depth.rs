//! Stream depth frames and print periodic unpacked pixel samples.
//!
//! This uses the native 11-bit packed depth format and exercises the conversion helpers
//! on live Kinect data.

use nect_rs::{Context, DeviceFlags, Resolution, DepthFormat};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::new()?;
    ctx.select_subdevices(DeviceFlags::CAMERA);

    let num = ctx.num_devices()?;
    println!("Devices found: {}", num);
    if num == 0 {
        return Ok(());
    }

    let depth_frames = Arc::new(AtomicUsize::new(0));

    {
        let mut dev = ctx.open_device(0)?;

        let depth_mode = nect_rs::Device::find_depth_mode(Resolution::Medium, DepthFormat::Bit11Packed);
        dev.set_depth_mode(depth_mode)?;

        let d_count = Arc::clone(&depth_frames);
        dev.set_depth_callback(move |data: &[u8], timestamp: u32| {
            let n = d_count.fetch_add(1, Ordering::Relaxed);
            if n % 30 == 0 {
                let mut unpack = [0u16; 8];
                nect_rs::cameras::unpack_8_pixels(&data[..11], &mut unpack);
                println!(
                    "  [depth] frame #{:4}  ts={}  first_8_unpacked={:?}",
                    n, timestamp, &unpack[..4]
                );
            }
        });

        dev.start_depth()?;
        println!("Streaming packed 11-bit depth (5 seconds)...");

        let start = Instant::now();
        while start.elapsed() < Duration::from_secs(5) {
            ctx.process_events_timeout(Duration::from_millis(100))?;
        }

        dev.stop_depth()?;
    }

    println!(
        "Total depth frames: {}",
        depth_frames.load(Ordering::Relaxed)
    );

    Ok(())
}
