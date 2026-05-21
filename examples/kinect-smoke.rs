//! Basic hardware smoke test for device enumeration, motor, LED, and tilt I/O.
//!
//! This exercises:
//! - Context creation via `rusb`
//! - Device enumeration
//! - Camera + motor open
//! - LED control
//! - Tilt state read
//! - Tilt motor commands
//!
//! Run this first to confirm basic USB I/O works before trying streaming examples.

use nect_rs::{Context, DeviceFlags, LedOption};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Kinect hardware smoke test ===\n");

    let ctx = Context::new()?;
    ctx.select_subdevices(DeviceFlags::MOTOR.union(DeviceFlags::CAMERA));

    let num = ctx.num_devices()?;
    println!("Devices found: {}", num);

    let serials = ctx.list_device_serials()?;
    for (i, serial) in serials.iter().enumerate() {
        println!("  [{}] serial: {}", i, serial);
    }

    if num == 0 {
        println!("No Kinect detected - exiting.");
        return Ok(());
    }

    let mut dev = ctx.open_device(0)?;
    println!("\nDevice opened.\n");

    println!("LED -> Green");
    dev.set_led(LedOption::Green)?;
    std::thread::sleep(std::time::Duration::from_millis(300));

    println!("LED -> Red");
    dev.set_led(LedOption::Red)?;
    std::thread::sleep(std::time::Duration::from_millis(300));

    println!("LED -> Yellow");
    dev.set_led(LedOption::Yellow)?;
    std::thread::sleep(std::time::Duration::from_millis(300));

    let tilt = dev.update_tilt_state()?;
    println!("\nTilt state:");
    println!(
        "  angle={}  status={:?}",
        tilt.tilt_angle(),
        tilt.tilt_status()
    );
    println!(
        "  accel raw=({}, {}, {})",
        tilt.accelerometer_x(),
        tilt.accelerometer_y(),
        tilt.accelerometer_z()
    );
    println!(
        "  tilt_degs={:.2}  mks_accel=({:.3}, {:.3}, {:.3}) m/s^2",
        tilt.tilt_degs(),
        tilt.mks_accel().0,
        tilt.mks_accel().1,
        tilt.mks_accel().2
    );

    println!("\nTilt -> 15 deg");
    dev.set_tilt_degs(15.0)?;
    std::thread::sleep(std::time::Duration::from_secs(2));

    println!("Tilt -> -15 deg");
    dev.set_tilt_degs(-15.0)?;
    std::thread::sleep(std::time::Duration::from_secs(2));

    println!("Tilt -> 0 deg");
    dev.set_tilt_degs(0.0)?;
    std::thread::sleep(std::time::Duration::from_secs(2));

    println!("\nLED -> Off");
    dev.set_led(LedOption::Off)?;

    println!("\n=== Smoke test complete ===");

    Ok(())
}
