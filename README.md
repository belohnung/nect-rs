# nect-rs

`nect-rs` is an experimental Rust driver/library for the Xbox 360 Kinect / Kinect v1.

The goal is to provide a small Rust API for device discovery, motor control,
camera control, depth/video packet reassembly, and format conversion.

## Status

This project is actively experimental. It currently implements much of the Kinect camera
and motor path in Rust, using `rusb` and direct `libusb` FFI where needed for USB access
and isochronous transfers.

Longer term, the intent is to continue moving more of the stack toward pure Rust on top
of `rusb` where possible. Some lower-level USB functionality may still require `libusb`
FFI until equivalent safe Rust APIs are available.

## Features

- Device discovery and serial number enumeration
- LED and tilt motor control
- Accelerometer / tilt state reading
- Depth and video streaming (with packet reassembly)
- Bayer to RGB, YUV to RGB, IR unpacking, and depth conversion
- Example applications with real-time viewers (`minifb`)

## Examples

List devices and basic info:

```bash
cargo run --example kinect-info
```

Smoke-test basic USB I/O:

```bash
cargo run --example kinect-smoke
```

Open a video window:

```bash
cargo run --example kinect-video
```

Stream depth frames:

```bash
cargo run --example kinect-depth
```

Open the combined RGB/depth/IR viewer:

```bash
cargo run --example kinect-viewer
```

## Linux permissions

You may need udev rules or elevated permissions to access the Kinect USB devices.
If examples cannot open the device, check that your user has permission to access the
Microsoft Kinect USB camera and motor interfaces.

## Development

Build everything:

```bash
cargo build --examples
```

Run tests:

```bash
cargo test
```

## Acknowledgments

This project is heavily informed by and grateful to the OpenKinect project, especially
[`libfreenect`](https://github.com/OpenKinect/libfreenect). Many constants, packet formats,
register sequences, and conversion algorithms were ported or derived from that work.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE).
