//! Separate YOLOX-S binary; never substitutes for the pinned nano baseline.
#[path = "../yolox_s.rs"]
mod yolox_s;

use rustdriving_camera_detect::read_image;
use sha2::{Digest, Sha256};
use std::{env, fs, path::Path};

fn executable_sha256() -> Result<String, String> {
    use std::io::Read;
    let executable = env::current_exe().map_err(|e| e.to_string())?;
    let file = fs::File::open(executable).map_err(|e| e.to_string())?;
    let limit = 128 * 1024 * 1024;
    if file.metadata().map_err(|e| e.to_string())?.len() > limit {
        return Err("executable exceeds receipt hashing bound".into());
    }
    let mut sha = Sha256::new();
    let mut total = 0;
    let mut input = file.take(limit + 1);
    let mut buffer = [0u8; 65536];
    loop {
        let count = input.read(&mut buffer).map_err(|e| e.to_string())?;
        if count == 0 {
            break;
        }
        total += count as u64;
        if total > limit {
            return Err("executable changed beyond receipt bound".into());
        }
        sha.update(&buffer[..count]);
    }
    Ok(format!("{:x}", sha.finalize()))
}

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.len() == 3 && args[0] == "--executable-receipt" && args[1] == "--output" {
        let file = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&args[2])
            .map_err(|e| format!("new executable receipt required: {e}"))?;
        let receipt = serde_json::json!({
            "schema_version":1,"kind":"yolox_s_executable_receipt",
            "model_profile":yolox_s::MODEL_PROFILE,
            "model_sha256":yolox_s::MODEL_SHA256,"model_bytes":yolox_s::MODEL_BYTES,
            "input_shape":[1,3,yolox_s::INPUT_SIZE,yolox_s::INPUT_SIZE],
            "output_shape":[1,yolox_s::ROWS,yolox_s::CLASSES+5],
            "compiled_sources":yolox_s::compiled_sources(),
            "executable_sha256":executable_sha256()?,
            "model_read":false,"images_read":false,"inference_run":false
        });
        serde_json::to_writer_pretty(file, &receipt).map_err(|e| e.to_string())?;
        return Ok(());
    }
    if args.len() == 3 && args[0] == "--check-model" && args[1] == "--model" {
        let _plan = yolox_s::Detector::load(Path::new(&args[2]))?;
        println!("pinned YOLOX-S CPU plan ready; no image inference performed");
        return Ok(());
    }
    if args.len() != 6 || args[0] != "--model" || args[2] != "--image" || args[4] != "--output" {
        return Err("usage: rustdriving-camera-detect-s --model yolox_s.onnx --image measured.jpg --output NEW-detections.json; or --check-model --model yolox_s.onnx".into());
    }
    // Reserve a new report before opening any image or running the model.
    // Failed attempts leave an empty/partial new file, never overwrite evidence.
    let destination = Path::new(&args[5]);
    let mut output = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(destination)
        .map_err(|e| format!("new report output required: {e}"))?;
    let detector = yolox_s::Detector::load(Path::new(&args[1]))?;
    let (image, sha256) = read_image(Path::new(&args[3]))?;
    let mut report = detector.detect(&image, 0.3, 0.45)?;
    report.source_image_sha256 = Some(sha256);
    let json = serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?;
    if json.len() > 64 * 1024 * 1024 {
        return Err("report exceeds 64 MiB bound".into());
    }
    use std::io::Write;
    output.write_all(&json).map_err(|e| e.to_string())?;
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
