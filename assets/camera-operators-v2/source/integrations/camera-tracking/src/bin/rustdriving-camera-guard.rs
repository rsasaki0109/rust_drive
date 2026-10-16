//! Offline, two-clock admission of camera results. No vehicle-control permit.
#[path = "../freshness.rs"]
mod freshness;
#[path = "../report_input.rs"]
mod report_input;
use freshness::{Guard, Policy};
use rustdriving_image_tracking::{Config, Detection, valid_sha256};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    env, fs,
    io::{Read, Write},
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Sequence {
    schema_version: u32,
    camera_id: String,
    width: u32,
    height: u32,
    #[serde(default)]
    config: Config,
    #[serde(default)]
    policy: Policy,
    events: Vec<Event>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Event {
    now_s: f64,
    input: Input,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Input {
    Analysis {
        acquired_s: f64,
        report: PathBuf,
        sha256: String,
    },
    Detections {
        acquired_s: f64,
        image_sha256: String,
        detections: Vec<Detection>,
    },
    Tick,
    Failed,
}
#[derive(Serialize)]
struct SourceReport {
    event_index: usize,
    sha256: String,
}
fn digest(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn read(path: &Path, limit: usize) -> Result<Vec<u8>, String> {
    if !fs::symlink_metadata(path)
        .map_err(|e| e.to_string())?
        .is_file()
    {
        return Err("regular non-symlink input required".into());
    }
    let mut bytes = vec![];
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(limit as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > limit {
        return Err("bounded JSON input exceeded".into());
    }
    Ok(bytes)
}
fn sources() -> BTreeMap<&'static str, String> {
    [
        (
            "integrations/camera-tracking/src/lib.rs",
            include_bytes!("../lib.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/assignment.rs",
            include_bytes!("../assignment.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/report_input.rs",
            include_bytes!("../report_input.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/freshness.rs",
            include_bytes!("../freshness.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/bin/rustdriving-camera-guard.rs",
            include_bytes!("rustdriving-camera-guard.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/producer-sources.json",
            include_bytes!("../../producer-sources.json").as_slice(),
        ),
        (
            "integrations/camera-tracking/profile-freshness.json",
            include_bytes!("../../profile-freshness.json").as_slice(),
        ),
        (
            "integrations/camera-tracking/Cargo.toml",
            include_bytes!("../../Cargo.toml").as_slice(),
        ),
        (
            "integrations/camera-tracking/Cargo.lock",
            include_bytes!("../../Cargo.lock").as_slice(),
        ),
        (
            "rust-toolchain.toml",
            include_bytes!("../../../../rust-toolchain.toml").as_slice(),
        ),
    ]
    .into_iter()
    .map(|(name, bytes)| (name, digest(bytes)))
    .collect()
}

struct Output {
    destination: PathBuf,
    stage: PathBuf,
    file: Option<fs::File>,
}
impl Output {
    fn reserve(destination: &Path) -> Result<Self, String> {
        match fs::symlink_metadata(destination) {
            Ok(_) => return Err("fresh output required before input reads".into()),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => (),
            Err(e) => return Err(e.to_string()),
        }
        let parent = destination
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        for attempt in 0..128 {
            let stage = parent.join(format!(
                ".rustdriving-guard-{}-{attempt}.partial",
                std::process::id()
            ));
            if stage.file_name() == destination.file_name() {
                continue;
            }
            match fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&stage)
            {
                Ok(file) => {
                    return Ok(Self {
                        destination: destination.to_path_buf(),
                        stage,
                        file: Some(file),
                    });
                }
                Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(e) => return Err(format!("prepare output: {e}")),
            }
        }
        Err("output staging names exhausted".into())
    }
    fn publish(mut self, value: &impl Serialize) -> Result<(), String> {
        let bytes = serde_json::to_vec_pretty(value).map_err(|e| e.to_string())?;
        if bytes.len() > 16 * 1024 * 1024 {
            return Err("report byte budget exceeded".into());
        }
        let file = self.file.as_mut().ok_or("output staging closed")?;
        file.write_all(&bytes).map_err(|e| e.to_string())?;
        file.sync_all().map_err(|e| e.to_string())?;
        self.file.take();
        fs::hard_link(&self.stage, &self.destination)
            .map_err(|e| format!("publish report: {e}"))?;
        Ok(())
    }
}
impl Drop for Output {
    fn drop(&mut self) {
        self.file.take();
        let _ = fs::remove_file(&self.stage);
    }
}

fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let [mode, input, flag, destination] = args.as_slice() else {
        return Err("expected --sequence FILE --output NEW".into());
    };
    if mode != "--sequence" || flag != "--output" {
        return Err("expected --sequence FILE --output NEW".into());
    }
    let output = Output::reserve(Path::new(destination))?;
    let input = Path::new(input);
    let bytes = read(input, 8 * 1024 * 1024)?;
    let sequence: Sequence = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    if sequence.schema_version != 1
        || sequence.events.is_empty()
        || sequence.events.len() > 128
        || sequence.camera_id.is_empty()
        || sequence.camera_id.len() > 64
        || !sequence
            .camera_id
            .bytes()
            .all(|v| v.is_ascii_alphanumeric() || b"._-".contains(&v))
    {
        return Err("invalid sequence version, camera ID or event count".into());
    }
    let mut guard = Guard::new(
        sequence.width,
        sequence.height,
        sequence.config.clone(),
        sequence.policy.clone(),
    )?;
    let mut results = vec![];
    let mut reports = vec![];
    let mut inline_events = 0;
    for (index, event) in sequence.events.into_iter().enumerate() {
        let admitted = match event.input {
            Input::Analysis {
                acquired_s,
                report,
                sha256,
            } => {
                if !valid_sha256(&sha256) {
                    return Err(format!("event {index}: invalid report hash"));
                }
                let path = input.parent().unwrap_or(Path::new(".")).join(report);
                let report_bytes =
                    read(&path, 4 * 1024 * 1024).map_err(|e| format!("event {index}: {e}"))?;
                if digest(&report_bytes) != sha256 {
                    return Err(format!("event {index}: source report SHA-256 mismatch"));
                }
                let value = serde_json::from_slice(&report_bytes)
                    .map_err(|e| format!("event {index}: {e}"))?;
                let (image_sha256, detections) =
                    report_input::detections(&value, sequence.width, sequence.height)
                        .map_err(|e| format!("event {index}: {e}"))?;
                reports.push(SourceReport {
                    event_index: index,
                    sha256,
                });
                freshness::Input::Sample {
                    acquired_s,
                    image_sha256,
                    detections,
                }
            }
            Input::Detections {
                acquired_s,
                image_sha256,
                detections,
            } => {
                inline_events += 1;
                freshness::Input::Sample {
                    acquired_s,
                    image_sha256,
                    detections,
                }
            }
            Input::Tick => freshness::Input::Tick,
            Input::Failed => freshness::Input::Failed,
        };
        results.push(
            guard
                .step(&freshness::Event {
                    now_s: event.now_s,
                    input: admitted,
                })
                .map_err(|e| format!("event {index}: {e}"))?,
        );
    }
    output.publish(&serde_json::json!({
        "schema_version":1,"algorithm":"image_tracking_freshness_v1","compiled_sources":sources(),
        "source_sequence_sha256":digest(&bytes),"camera_id":sequence.camera_id,"width":sequence.width,"height":sequence.height,
        "config":sequence.config,"policy":sequence.policy,"source_reports":reports,"inline_detection_events":inline_events,
        "model_read":false,"images_read":false,"inference_run":false,"control_permission":false,"results":results
    }))
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
