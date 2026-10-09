use rustdrive_camera_detect::{Detector, read_image};
use std::{env, path::Path};
fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() != 6 || args[0] != "--model" || args[2] != "--image" || args[4] != "--output" {
        return Err("usage: rustdrive-camera-detect --model yolox_nano.onnx --image measured.jpg --output detections.json".into());
    }
    let detector = Detector::load(Path::new(&args[1]))?;
    let (image, sha256) = read_image(Path::new(&args[3]))?;
    let mut report = detector.detect(&image, 0.3, 0.45)?;
    report.source_image_sha256 = Some(sha256);
    let json = serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?;
    std::fs::write(&args[5], json).map_err(|e| e.to_string())?;
    println!(
        "{} boxes; CPU inference {:.3} s; {}x{}",
        report.detections.len(),
        report.inference_seconds,
        report.width,
        report.height
    );
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
