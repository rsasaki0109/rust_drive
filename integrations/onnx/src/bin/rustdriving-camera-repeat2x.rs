//! Lowered nearest upsampling with existing bounded parallel views and clocks.
extern crate self as rustdriving_image_tracking;
#[allow(dead_code)]
#[path = "../camera_parallel.rs"]
mod baseline_parallel;
#[allow(dead_code)]
#[path = "../yolox_s.rs"]
mod baseline_yolox_s;
#[path = "../camera_repeat_analysis.rs"]
mod camera_analysis;
#[path = "../../../camera-tracking/src/freshness.rs"]
mod camera_freshness;
#[path = "../camera_latency.rs"]
mod camera_latency;
#[path = "../camera_repeat2.rs"]
mod camera_repeat2;
#[allow(dead_code)]
#[path = "../camera_sequence.rs"]
mod camera_sequence;
#[path = "../../../camera-tracking/src/lib.rs"]
mod image_tracking;
#[allow(dead_code)]
#[path = "../camera_analysis.rs"]
mod serial_camera_analysis;
#[path = "../camera_repeat_detector.rs"]
mod yolox_s;
#[path = "../yolox_tiles.rs"]
mod yolox_tiles;
#[path = "../yolox_vehicle_consistency.rs"]
mod yolox_vehicle_consistency;
#[path = "../yolox_vehicle_context.rs"]
mod yolox_vehicle_context;
#[path = "../yolox_vehicle_vote.rs"]
mod yolox_vehicle_vote;
pub use image_tracking::{Acquisition, Config, Detection, Frame, Phase, Snapshot, Tracker};
use sha2::{Digest, Sha256};
use std::{env, fs, io::Read, path::Path, time::Instant};
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let destination=match args.as_slice() {
        [m,_,s,_,o,d] if m=="--model" && (s=="--image" || s=="--sequence") && o=="--output"=>d,
        [m,_,o,d] if m=="--inspect-sequence" && o=="--output"=>d,
        [m,o,d] if m=="--executable-receipt" && o=="--output"=>d,
        _=>return Err("expected --model FILE --image FILE --output NEW; --model FILE --sequence FILE --output NEW; --inspect-sequence FILE --output NEW; --executable-receipt --output NEW".into()),
    };
    let output = camera_analysis::AtomicReport::reserve(Path::new(destination))?;
    let startup = Instant::now();
    if args[0] == "--executable-receipt" {
        let mut bytes = Vec::new();
        fs::File::open(env::current_exe().map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?
            .take(128 * 1024 * 1024 + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() > 128 * 1024 * 1024 {
            return Err("executable exceeds receipt budget".into());
        }
        return output.publish(&serde_json::json!({"compiled_sources":camera_latency::compiled_sources(),"executable_sha256":format!("{:x}",Sha256::digest(bytes)),
            "model_sha256":yolox_s::MODEL_SHA256,"model_bytes":yolox_s::MODEL_BYTES,"model_read":false,"images_read":false,"inference_run":false}));
    }
    if args[0] == "--model" && args[2] == "--image" {
        let (image, sha) = camera_analysis::read_image(Path::new(&args[3]))
            .map_err(|e| format!("read image: {e}"))?;
        let detector =
            yolox_s::Detector::load(Path::new(&args[1])).map_err(|e| format!("load model: {e}"))?;
        return output.publish(&camera_analysis::analyze(&detector, &image, sha)?);
    }
    let path = Path::new(if args[0] == "--model" {
        &args[3]
    } else {
        &args[1]
    });
    let (sequence, sha) = camera_sequence::read_sequence(path)?;
    let base = path.parent().unwrap_or(Path::new("."));
    let checked = camera_sequence::preflight(&sequence, base)?;
    let slots = camera_latency::plan(&sequence, &checked)?;
    let preflight_wall = startup.elapsed().as_secs_f64();
    if args[0] == "--inspect-sequence" {
        output.publish(&serde_json::json!({"schema_version":1,"algorithm":"yolox_s_measured_schedule_validation_v1","compiled_sources":camera_latency::compiled_sources(),
            "source_sequence_sha256":sha,"camera_id":sequence.camera_id,"width":sequence.width,"height":sequence.height,"config":sequence.config,
            "policy":camera_freshness::Policy::default(),"slots":slots,"images_read":camera_latency::image_count(&slots),"model_read":false,"inference_run":false,"control_permission":false}))
    } else {
        output.publish(&camera_latency::run(
            &sequence,
            slots,
            base,
            Path::new(&args[1]),
            sha,
            startup,
            preflight_wall,
        )?)
    }
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
