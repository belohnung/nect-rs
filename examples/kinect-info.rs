use nect_rs::{Context, DeviceFlags, LedOption, Resolution, VideoFormat, DepthFormat};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = Context::new()?;
    ctx.set_log_level(nect_rs::LogLevel::Debug);

    let num = ctx.num_devices()?;
    println!("Connected Kinect devices: {}", num);

    let serials = ctx.list_device_serials()?;
    for (i, serial) in serials.iter().enumerate() {
        println!("  [{}] serial: {}", i, serial);
    }

    if num == 0 {
        println!("No Kinect found - exiting.");
        return Ok(());
    }

    ctx.select_subdevices(DeviceFlags::MOTOR.union(DeviceFlags::CAMERA));
    let mut dev = ctx.open_device(0)?;

    println!("Device opened.");

    dev.set_led(LedOption::Green)?;
    println!("LED -> Green");

    let tilt = dev.update_tilt_state()?;
    println!(
        "Tilt: angle={}, status={:?}, accel=({}, {}, {})",
        tilt.tilt_angle(),
        tilt.tilt_status(),
        tilt.accelerometer_x(),
        tilt.accelerometer_y(),
        tilt.accelerometer_z()
    );
    println!(
        "  tilt_degs={:.2}, mks_accel=({:.3}, {:.3}, {:.3}) m/s^2",
        tilt.tilt_degs(),
        tilt.mks_accel().0,
        tilt.mks_accel().1,
        tilt.mks_accel().2
    );

    let video_mode = nect_rs::Device::find_video_mode(Resolution::Medium, VideoFormat::Rgb);
    println!("Video mode (RGB 640x480): {:?}", video_mode);

    let depth_mode = nect_rs::Device::find_depth_mode(Resolution::Medium, DepthFormat::Mm);
    println!("Depth mode (MM 640x480): {:?}", depth_mode);

    dev.set_led(LedOption::Off)?;
    println!("LED -> Off");

    println!("Done.");
    Ok(())
}
