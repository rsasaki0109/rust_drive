//! Full-image-context vehicle reconciliation; preserves the failed vote path.
#[path = "../yolox_s.rs"]
mod yolox_s;
#[path = "../yolox_tiles.rs"]
mod yolox_tiles;
#[path = "../yolox_vehicle_context.rs"]
mod yolox_vehicle_context;
#[path = "../yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
use rustdriving_camera_detect::read_image;
use sha2::{Digest, Sha256};
use std::{
    env, fs,
    io::{Read, Write},
    path::Path,
};

fn bounded(path: &Path, limit: usize) -> Result<Vec<u8>, String> {
    let mut body = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(limit as u64 + 1)
        .read_to_end(&mut body)
        .map_err(|e| e.to_string())?;
    if body.len() > limit {
        return Err("bounded input exceeded".into());
    }
    Ok(body)
}
fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let destination=match args.as_slice() {
        [mode,output,path] if mode=="--executable-receipt" && output=="--output" => path,
        [mode,_,output,path] if mode=="--reconcile-fixture" && output=="--output" => path,
        [model,_,image,_,output,path] if model=="--model" && image=="--image" && output=="--output" => path,
        _=>return Err("expected --model FILE --image FILE --output NEW; --reconcile-fixture FILE --output NEW; --executable-receipt --output NEW".into()),
    };
    if fs::symlink_metadata(destination).is_ok() {
        return Err("fresh output required before input reads".into());
    }
    let value = if args[0] == "--executable-receipt" {
        let exe = bounded(
            &env::current_exe().map_err(|e| e.to_string())?,
            128 * 1024 * 1024,
        )?;
        serde_json::json!({"schema_version":1,"compiled_sources":yolox_vehicle_context::compiled_sources(),
            "executable_sha256":format!("{:x}",Sha256::digest(exe)),"model_sha256":yolox_s::MODEL_SHA256,
            "model_bytes":yolox_s::MODEL_BYTES,"model_read":false,"images_read":false,"inference_run":false})
    } else if args[0] == "--reconcile-fixture" {
        let fixture: yolox_tiles::Fixture =
            serde_json::from_slice(&bounded(Path::new(&args[1]), 4 * 1024 * 1024)?)
                .map_err(|e| e.to_string())?;
        serde_json::to_value(yolox_vehicle_context::reconcile(&fixture)?)
            .map_err(|e| e.to_string())?
    } else {
        let detector = yolox_s::Detector::load(Path::new(&args[1]))?;
        let (image, sha) = read_image(Path::new(&args[3]))?;
        let parent = yolox_tiles::detect(&detector, &image, sha)?;
        let fixture = yolox_tiles::Fixture {
            width: parent.width,
            height: parent.height,
            views: parent
                .views
                .iter()
                .map(|v| yolox_tiles::FixtureView {
                    rectangle: v.rectangle,
                    detections: v.prediction.detections.clone(),
                })
                .collect(),
        };
        let result = yolox_vehicle_context::reconcile(&fixture)?;
        serde_json::json!({"schema_version":1,"algorithm":"yolox_s_vehicle_full_context_v1",
            "compiled_sources":yolox_vehicle_context::compiled_sources(),"base_prediction":parent,"result":result})
    };
    let body = serde_json::to_vec_pretty(&value).map_err(|e| e.to_string())?;
    if body.len() > 4 * 1024 * 1024 {
        return Err("bounded report exceeded".into());
    }
    let mut output = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(destination)
        .map_err(|e| e.to_string())?;
    output.write_all(&body).map_err(|e| e.to_string())?;
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
