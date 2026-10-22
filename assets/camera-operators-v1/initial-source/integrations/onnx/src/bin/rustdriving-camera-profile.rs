//! Serial operator diagnosis; no deadline or new image-producer admission claim.
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod camera_analysis;
#[path = "../camera_profile.rs"]
mod camera_profile;
#[allow(dead_code)]
#[path = "../yolox_s.rs"]
mod yolox_s;
#[allow(dead_code)]
#[path = "../yolox_tiles.rs"]
mod yolox_tiles;
#[allow(dead_code)]
#[path = "../yolox_vehicle_consistency.rs"]
mod yolox_vehicle_consistency;
#[allow(dead_code)]
#[path = "../yolox_vehicle_context.rs"]
mod yolox_vehicle_context;
#[allow(dead_code)]
#[path = "../yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
use sha2::{Digest, Sha256};
use std::{env, fs::File, io::Read, path::Path};

fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    match args.as_slice() {
        [m, model, i, image, o, destination]
            if m == "--model" && i == "--image" && o == "--output" =>
        {
            let output = camera_analysis::AtomicReport::reserve(Path::new(destination))?;
            let (image, sha) = camera_analysis::read_image(Path::new(image))
                .map_err(|e| format!("read image: {e}"))?;
            output.publish(&camera_profile::analyze(Path::new(model), &image, sha)?)
        }
        [m, o, destination] if m == "--executable-receipt" && o == "--output" => {
            let output = camera_analysis::AtomicReport::reserve(Path::new(destination))?;
            let mut bytes = Vec::new();
            File::open(env::current_exe().map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?
                .take(128 * 1024 * 1024 + 1)
                .read_to_end(&mut bytes)
                .map_err(|e| e.to_string())?;
            if bytes.len() > 128 * 1024 * 1024 {
                return Err("executable exceeds receipt budget".into());
            }
            output.publish(&serde_json::json!({"schema_version":1,"kind":"yolox_s_operator_profile_executable_receipt",
                "compiled_sources":camera_profile::compiled_sources(),"executable_sha256":format!("{:x}",Sha256::digest(bytes)),
                "model_read":false,"images_read":0,"model_forwards":0}))
        }
        _ => Err(
            "expected --model FILE --image FILE --output NEW; --executable-receipt --output NEW"
                .into(),
        ),
    }
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
