//! Stream bounded recorded images through one reusable model plan and tracker.
use super::{camera_analysis, image_tracking, yolox_s};
use image_tracking::{Acquisition, Config, Detection, Frame, Snapshot, Tracker};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::Read,
    path::{Path, PathBuf},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sequence {
    pub schema_version: u32,
    pub camera_id: String,
    pub width: u32,
    pub height: u32,
    #[serde(default)]
    pub config: Config,
    pub events: Vec<Event>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Event {
    /// Original decimal seconds, parsed with Rust's correctly rounded parser.
    pub timestamp_s: String,
    pub input: Input,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Input {
    Image { path: PathBuf, sha256: String },
    Missing,
    Failed,
}
#[derive(Serialize)]
pub struct CheckedEvent {
    pub event_index: usize,
    pub timestamp_text: String,
    pub timestamp_s: f64,
    pub image_sha256: Option<String>,
}
#[derive(Serialize)]
pub struct EventReport {
    pub event_index: usize,
    pub timestamp_text: String,
    pub timestamp_s: f64,
    pub analysis: Option<camera_analysis::Report>,
    pub tracking: Snapshot,
}

pub fn read_sequence(path: &Path) -> Result<(Sequence, String), String> {
    if !fs::symlink_metadata(path)
        .map_err(|e| e.to_string())?
        .is_file()
    {
        return Err("sequence must be a regular non-symlink file".into());
    }
    let mut bytes = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(512 * 1024 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > 512 * 1024 {
        return Err("sequence exceeds 512 KiB".into());
    }
    let sequence = serde_json::from_slice(&bytes).map_err(|e| format!("sequence JSON: {e}"))?;
    Ok((sequence, format!("{:x}", Sha256::digest(bytes))))
}

pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = camera_analysis::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/camera_sequence.rs",
            include_bytes!("camera_sequence.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-sequence.rs",
            include_bytes!("bin/rustdriving-camera-sequence.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-sequence.json",
            include_bytes!("../profile-sequence.json").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/lib.rs",
            include_bytes!("../../camera-tracking/src/lib.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/assignment.rs",
            include_bytes!("../../camera-tracking/src/assignment.rs").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

fn timestamp(event: &Event) -> Result<f64, String> {
    if event.timestamp_s.is_empty() || event.timestamp_s.len() > 64 {
        return Err("timestamp text exceeds bound".into());
    }
    event
        .timestamp_s
        .parse()
        .map_err(|_| "invalid decimal timestamp".into())
}
fn check_image(
    sequence: &Sequence,
    path: &Path,
    expected: &str,
) -> Result<(image::RgbImage, String), String> {
    let (image, sha) = camera_analysis::read_image(path)?;
    if sha != expected {
        return Err("image SHA-256 mismatch".into());
    }
    if (image.width(), image.height()) != (sequence.width, sequence.height) {
        return Err("image dimensions differ from fixed camera dimensions".into());
    }
    Ok((image, sha))
}

/// Validate every image before model loading, keeping only one decoded image
/// in memory. This dry tracker validates clock/cache/config, never emits tracks.
pub fn preflight(sequence: &Sequence, base: &Path) -> Result<Vec<CheckedEvent>, String> {
    if sequence.schema_version != 1
        || !(1..=32).contains(&sequence.events.len())
        || sequence.camera_id.is_empty()
        || sequence.camera_id.len() > 64
        || !sequence
            .camera_id
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(&b))
    {
        return Err("invalid sequence version, camera ID or 1..32 event count".into());
    }
    let mut validator = Tracker::new(sequence.width, sequence.height, sequence.config.clone())?;
    let mut checked = Vec::new();
    for (i, event) in sequence.events.iter().enumerate() {
        let time = timestamp(event).map_err(|e| format!("event {i}: {e}"))?;
        let acquisition = match &event.input {
            Input::Image { sha256, .. } => Acquisition::Detected {
                image_sha256: sha256.clone(),
                detections: vec![],
            },
            Input::Missing => Acquisition::Missing,
            Input::Failed => Acquisition::Failed,
        };
        validator
            .step(&Frame {
                timestamp_s: time,
                acquisition,
            })
            .map_err(|e| format!("event {i}: {e}"))?;
        let image_sha256 = if let Input::Image { path, sha256 } = &event.input {
            let (_, sha) = check_image(sequence, &base.join(path), sha256)
                .map_err(|e| format!("event {i}: {e}"))?;
            Some(sha)
        } else {
            None
        };
        checked.push(CheckedEvent {
            event_index: i,
            timestamp_text: event.timestamp_s.clone(),
            timestamp_s: time,
            image_sha256,
        });
    }
    Ok(checked)
}

#[derive(Serialize)]
pub struct Execution {
    pub model_loads: usize,
    pub preflight_image_reads: usize,
    pub inference_image_reads: usize,
    pub model_forwards: usize,
}
#[derive(Serialize)]
pub struct Report {
    pub schema_version: u32,
    pub algorithm: &'static str,
    pub compiled_sources: BTreeMap<&'static str, String>,
    pub source_sequence_sha256: String,
    pub camera_id: String,
    pub width: u32,
    pub height: u32,
    pub config: Config,
    pub execution: Execution,
    pub events: Vec<EventReport>,
}

pub fn run(
    sequence: &Sequence,
    base: &Path,
    model: &Path,
    source_sha: String,
    checked: &[CheckedEvent],
) -> Result<Report, String> {
    let images = checked.iter().filter(|e| e.image_sha256.is_some()).count();
    let detector = if images > 0 {
        Some(yolox_s::Detector::load(model).map_err(|e| format!("load model: {e}"))?)
    } else {
        None
    };
    let mut tracker = Tracker::new(sequence.width, sequence.height, sequence.config.clone())?;
    let mut events = Vec::new();
    let mut model_forwards = 0;
    for (i, (event, check)) in sequence.events.iter().zip(checked).enumerate() {
        let (analysis, acquisition) = match &event.input {
            Input::Image { path, sha256 } => {
                // Recheck content/dimensions after preflight instead of retaining
                // every RGB image or accepting a changed file at inference time.
                let (image, sha) = check_image(sequence, &base.join(path), sha256)
                    .map_err(|e| format!("event {i} inference read: {e}"))?;
                let analysis = camera_analysis::analyze(
                    detector.as_ref().ok_or("missing model plan")?,
                    &image,
                    sha.clone(),
                )?;
                model_forwards += analysis.execution.model_forwards;
                let detections = analysis
                    .result
                    .detections
                    .iter()
                    .map(|d| Detection {
                        class_index: d.class_index,
                        confidence: f64::from(d.confidence),
                        bbox: d.bbox.map(f64::from),
                    })
                    .collect();
                (
                    Some(analysis),
                    Acquisition::Detected {
                        image_sha256: sha,
                        detections,
                    },
                )
            }
            Input::Missing => (None, Acquisition::Missing),
            Input::Failed => (None, Acquisition::Failed),
        };
        let tracking = tracker
            .step(&Frame {
                timestamp_s: check.timestamp_s,
                acquisition,
            })
            .map_err(|e| format!("event {i}: {e}"))?;
        events.push(EventReport {
            event_index: i,
            timestamp_text: event.timestamp_s.clone(),
            timestamp_s: check.timestamp_s,
            analysis,
            tracking,
        });
    }
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_s_sequence_tracking_v1",
        compiled_sources: compiled_sources(),
        source_sequence_sha256: source_sha,
        camera_id: sequence.camera_id.clone(),
        width: sequence.width,
        height: sequence.height,
        config: sequence.config.clone(),
        execution: Execution {
            model_loads: usize::from(detector.is_some()),
            preflight_image_reads: images,
            inference_image_reads: images,
            model_forwards,
        },
        events,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sequence(events: &str) -> Sequence {
        serde_json::from_str(&format!(r#"{{"schema_version":1,"camera_id":"clock","width":640,"height":480,"events":{events}}}"#)).unwrap()
    }
    #[test]
    fn source_epoch_timestamp_text_keeps_its_rust_float_identity() {
        let input = sequence(
            r#"[{"timestamp_s":"1341845824.197930","input":{"kind":"missing"}},{"timestamp_s":"1341845824.229907","input":{"kind":"failed"}}]"#,
        );
        let checked = preflight(&input, Path::new("missing")).unwrap();
        assert_eq!(
            checked[0].timestamp_s.to_bits(),
            "1341845824.197930".parse::<f64>().unwrap().to_bits()
        );
        let report = run(
            &input,
            Path::new("missing"),
            Path::new("missing-model"),
            "0".repeat(64),
            &checked,
        )
        .unwrap();
        assert_eq!(report.execution.model_loads, 0);
        assert_eq!(
            report.events[1].tracking.health,
            image_tracking::Health::AcquisitionFailed
        );
    }
    #[test]
    fn duplicate_clock_and_nonfinite_timestamp_fail_before_model_loading() {
        for time in ["0", "-1", "NaN", "inf", "bad"] {
            let input = sequence(&format!(
                r#"[{{"timestamp_s":"0","input":{{"kind":"missing"}}}},{{"timestamp_s":"{time}","input":{{"kind":"missing"}}}}]"#
            ));
            assert!(preflight(&input, Path::new("missing")).is_err());
        }
    }
}
