//! Separate bounded multi-view research binary, with inspectable crop/merge paths.
#[path = "../yolox_s.rs"]
mod yolox_s;
#[path = "../yolox_tiles.rs"]
mod yolox_tiles;
use rustdriving_camera_detect::read_image;
use sha2::{Digest, Sha256};
use std::{
    env, fs,
    io::{Read, Write},
    path::Path,
};

fn run() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    let (destination, value) = if args.len() == 3
        && args[0] == "--executable-receipt"
        && args[1] == "--output"
    {
        let executable = env::current_exe().map_err(|e| e.to_string())?;
        let mut bytes = Vec::new();
        fs::File::open(executable)
            .map_err(|e| e.to_string())?
            .take(128 * 1024 * 1024 + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() > 128 * 1024 * 1024 {
            return Err("executable exceeds receipt bound".into());
        }
        (
            &args[2],
            serde_json::json!({"schema_version":1,"kind":"yolox_s_tiles_executable_receipt",
            "compiled_sources":yolox_tiles::compiled_sources(),"executable_sha256":format!("{:x}",Sha256::digest(bytes)),
            "model_sha256":yolox_s::MODEL_SHA256,"model_bytes":yolox_s::MODEL_BYTES,
            "model_read":false,"images_read":false,"inference_run":false}),
        )
    } else if args.len() == 4 && args[0] == "--merge-fixture" && args[2] == "--output" {
        let mut bytes = Vec::new();
        fs::File::open(&args[1])
            .map_err(|e| e.to_string())?
            .take(512 * 1024 + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() > 512 * 1024 {
            return Err("fixture exceeds 512 KiB".into());
        }
        let fixture: yolox_tiles::Fixture =
            serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
        (
            &args[3],
            serde_json::to_value(yolox_tiles::merge(&fixture)?).map_err(|e| e.to_string())?,
        )
    } else if args.len() == 5
        && args[0] == "--inspect-views"
        && args[1] == "--image"
        && args[3] == "--output"
    {
        let (image, sha) = read_image(Path::new(&args[2]))?;
        let views: Vec<_> = yolox_tiles::rectangles(image.width(),image.height())?.into_iter().map(|r| {
            let crop=image::imageops::crop_imm(&image,r.x,r.y,r.width,r.height).to_image();
            serde_json::json!({"rectangle":r,"rgb_sha256":format!("{:x}",Sha256::digest(crop.as_raw()))})
        }).collect();
        (
            &args[4],
            serde_json::json!({"source_image_sha256":sha,"width":image.width(),"height":image.height(),"views":views,"model_read":false,"inference_run":false}),
        )
    } else if args.len() == 6
        && args[0] == "--model"
        && args[2] == "--image"
        && args[4] == "--output"
    {
        if Path::new(&args[5]).exists() {
            return Err("fresh report required before inference".into());
        }
        let detector = yolox_s::Detector::load(Path::new(&args[1]))?;
        let (image, sha) = read_image(Path::new(&args[3]))?;
        (
            &args[5],
            serde_json::to_value(yolox_tiles::detect(&detector, &image, sha)?)
                .map_err(|e| e.to_string())?,
        )
    } else {
        return Err("expected --model FILE --image FILE --output NEW; --inspect-views --image FILE --output NEW; --merge-fixture FILE --output NEW; --executable-receipt --output NEW".into());
    };
    let bytes = serde_json::to_vec_pretty(&value).map_err(|e| e.to_string())?;
    if bytes.len() > 64 * 1024 * 1024 {
        return Err("bounded report exceeded".into());
    }
    let mut output = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(destination)
        .map_err(|e| e.to_string())?;
    output.write_all(&bytes).map_err(|e| e.to_string())?;
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(2);
    }
}
