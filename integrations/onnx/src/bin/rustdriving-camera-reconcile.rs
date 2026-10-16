//! Report-only adapter. Shared producer modules supply receipt/merge code only.
#[allow(dead_code)]
#[path = "../yolox_s.rs"]
mod yolox_s;
#[allow(dead_code)]
#[path = "../yolox_tiles.rs"]
mod yolox_tiles;
#[path = "../yolox_vehicle_consistency.rs"]
mod yolox_vehicle_consistency;
#[path = "../yolox_vehicle_context.rs"]
mod yolox_vehicle_context;
#[path = "../yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
use sha2::{Digest, Sha256};
use std::{
    env, fs,
    io::{Read, Write},
    path::Path,
};
fn read(path: &Path, limit: usize) -> Result<Vec<u8>, String> {
    let mut b = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(limit as u64 + 1)
        .read_to_end(&mut b)
        .map_err(|e| e.to_string())?;
    if b.len() > limit {
        return Err("bounded input exceeded".into());
    }
    Ok(b)
}
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args().skip(1).collect();
    let destination=match args.as_slice() {
        [m,o,p] if m=="--executable-receipt"&&o=="--output"=>p,
        [m,_,o,p] if (m=="--input"||m=="--reconcile-fixture")&&o=="--output"=>p,
        _=>return Err("expected --input PRODUCER_JSON --output NEW; --reconcile-fixture FILE --output NEW; --executable-receipt --output NEW".into()),
    };
    if fs::symlink_metadata(destination).is_ok() {
        return Err("fresh output required before input reads".into());
    }
    let value = if args[0] == "--executable-receipt" {
        let bytes = read(
            &env::current_exe().map_err(|e| e.to_string())?,
            128 * 1024 * 1024,
        )?;
        serde_json::json!({"compiled_sources":yolox_vehicle_consistency::compiled_sources(),"executable_sha256":format!("{:x}",Sha256::digest(bytes)),"model_read":false,"images_read":false,"inference_run":false})
    } else {
        let bytes = read(Path::new(&args[1]), 4 * 1024 * 1024)?;
        if args[0] == "--reconcile-fixture" {
            let fixture: yolox_tiles::Fixture =
                serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
            serde_json::to_value(yolox_vehicle_consistency::reconcile(&fixture)?)
                .map_err(|e| e.to_string())?
        } else {
            let parent: serde_json::Value =
                serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
            let fixture = yolox_vehicle_consistency::producer_fixture(&parent)?;
            let result = yolox_vehicle_consistency::reconcile(&fixture)?;
            serde_json::json!({"schema_version":1,"algorithm":"yolox_s_vehicle_consistency_v1",
                "compiled_sources":yolox_vehicle_consistency::compiled_sources(),"source_report_sha256":format!("{:x}",Sha256::digest(bytes)),
                "source_image_sha256":parent["source_image_sha256"],"result":result,"model_read":false,"images_read":false,"inference_run":false})
        }
    };
    let bytes = serde_json::to_vec_pretty(&value).map_err(|e| e.to_string())?;
    if bytes.len() > 4 * 1024 * 1024 {
        return Err("bounded report exceeded".into());
    }
    fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(destination)
        .map_err(|e| e.to_string())?
        .write_all(&bytes)
        .map_err(|e| e.to_string())?;
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
