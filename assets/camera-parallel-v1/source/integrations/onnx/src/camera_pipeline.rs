//! Images through unchanged analysis and freshness, at caller-supplied replay times.
use super::{camera_analysis, camera_freshness, image_tracking, yolox_s};
use camera_freshness::{Decision, Guard, Policy};
use image_tracking::{Config, Detection, valid_sha256};
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
    #[serde(default)]
    pub policy: Policy,
    pub events: Vec<Event>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Event {
    pub now_s: String,
    pub input: Input,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Input {
    Image {
        acquired_s: String,
        path: PathBuf,
        sha256: String,
    },
    Tick,
    Failed,
}
#[derive(Serialize)]
pub struct CheckedEvent {
    event_index: usize,
    now_text: String,
    now_s: f64,
    acquired_text: Option<String>,
    acquired_s: Option<f64>,
    source_image_sha256: Option<String>,
    admission_without_model: Decision,
}
#[derive(Serialize)]
pub struct EventReport {
    event_index: usize,
    now_text: String,
    now_s: f64,
    acquired_text: Option<String>,
    acquired_s: Option<f64>,
    source_image_sha256: Option<String>,
    inference_skipped: bool,
    analysis: Option<camera_analysis::Report>,
    freshness: camera_freshness::Output,
}
#[derive(Serialize)]
pub struct Execution {
    model_loads: usize,
    preflight_image_reads: usize,
    inference_image_reads: usize,
    skipped_image_events: usize,
    model_forwards: usize,
}
#[derive(Serialize)]
pub struct Report {
    schema_version: u32,
    algorithm: &'static str,
    compiled_sources: BTreeMap<&'static str, String>,
    source_sequence_sha256: String,
    camera_id: String,
    width: u32,
    height: u32,
    config: Config,
    policy: Policy,
    freshness_clock: &'static str,
    control_permission: bool,
    execution: Execution,
    events: Vec<EventReport>,
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
            "integrations/onnx/src/camera_pipeline.rs",
            include_bytes!("camera_pipeline.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-pipeline.rs",
            include_bytes!("bin/rustdriving-camera-pipeline.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-pipeline.json",
            include_bytes!("../profile-pipeline.json").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/lib.rs",
            include_bytes!("../../camera-tracking/src/lib.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/assignment.rs",
            include_bytes!("../../camera-tracking/src/assignment.rs").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/freshness.rs",
            include_bytes!("../../camera-tracking/src/freshness.rs").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}
fn decimal(text: &str) -> Result<f64, String> {
    if text.is_empty() || text.len() > 64 {
        return Err("clock text exceeds bound".into());
    }
    let value: f64 = text.parse().map_err(|_| "invalid decimal clock")?;
    if !value.is_finite() {
        return Err("nonfinite decimal clock".into());
    }
    Ok(value)
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
pub fn image_count(checked: &[CheckedEvent]) -> usize {
    checked
        .iter()
        .filter(|event| event.source_image_sha256.is_some())
        .count()
}

/// Decode/verify every image before model access, keeping only one in memory.
/// Empty payloads probe clocks/hash admission only; no object output is emitted.
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
    let mut validator = Guard::new(
        sequence.width,
        sequence.height,
        sequence.config.clone(),
        sequence.policy.clone(),
    )?;
    let mut checked = Vec::new();
    for (index, event) in sequence.events.iter().enumerate() {
        let now_s = decimal(&event.now_s).map_err(|e| format!("event {index}: {e}"))?;
        let (acquired_text, acquired_s, source_image_sha256, input) = match &event.input {
            Input::Image {
                acquired_s,
                path,
                sha256,
            } => {
                let source = decimal(acquired_s).map_err(|e| format!("event {index}: {e}"))?;
                if !valid_sha256(sha256) {
                    return Err(format!("event {index}: invalid image hash"));
                }
                let (_, sha) = check_image(sequence, &base.join(path), sha256)
                    .map_err(|e| format!("event {index}: {e}"))?;
                (
                    Some(acquired_s.clone()),
                    Some(source),
                    Some(sha.clone()),
                    camera_freshness::Input::Sample {
                        acquired_s: source,
                        image_sha256: sha,
                        detections: vec![],
                    },
                )
            }
            Input::Tick => (None, None, None, camera_freshness::Input::Tick),
            Input::Failed => (None, None, None, camera_freshness::Input::Failed),
        };
        let admission = validator
            .step(&camera_freshness::Event { now_s, input })
            .map_err(|e| format!("event {index}: {e}"))?;
        checked.push(CheckedEvent {
            event_index: index,
            now_text: event.now_s.clone(),
            now_s,
            acquired_text,
            acquired_s,
            source_image_sha256,
            admission_without_model: admission.decision,
        });
    }
    Ok(checked)
}

pub fn run(
    sequence: &Sequence,
    base: &Path,
    model: &Path,
    source_sha: String,
    checked: &[CheckedEvent],
) -> Result<Report, String> {
    if sequence.events.len() != checked.len() {
        return Err("complete preflight required".into());
    }
    let admitted = checked
        .iter()
        .filter(|event| event.admission_without_model == Decision::Accepted)
        .count();
    let detector = if admitted > 0 {
        Some(yolox_s::Detector::load(model).map_err(|e| format!("load model: {e}"))?)
    } else {
        None
    };
    let mut guard = Guard::new(
        sequence.width,
        sequence.height,
        sequence.config.clone(),
        sequence.policy.clone(),
    )?;
    let mut events = Vec::new();
    let mut model_forwards = 0;
    for (index, (event, check)) in sequence.events.iter().zip(checked).enumerate() {
        let mut analysis = None;
        let mut inference_skipped = false;
        let input = match &event.input {
            Input::Image { path, sha256, .. } => {
                let mut detections = Vec::new();
                if check.admission_without_model == Decision::Accepted {
                    // Verify again instead of trusting preflight file stability.
                    let (image, sha) = check_image(sequence, &base.join(path), sha256)
                        .map_err(|e| format!("event {index} inference read: {e}"))?;
                    let report = camera_analysis::analyze(
                        detector.as_ref().ok_or("missing model plan")?,
                        &image,
                        sha,
                    )?;
                    model_forwards += report.execution.model_forwards;
                    detections = report
                        .result
                        .detections
                        .iter()
                        .map(|d| Detection {
                            class_index: d.class_index,
                            confidence: f64::from(d.confidence),
                            bbox: d.bbox.map(f64::from),
                        })
                        .collect();
                    analysis = Some(report);
                } else {
                    inference_skipped = true;
                }
                camera_freshness::Input::Sample {
                    acquired_s: check
                        .acquired_s
                        .ok_or("missing checked acquisition clock")?,
                    image_sha256: check
                        .source_image_sha256
                        .clone()
                        .ok_or("missing checked image identity")?,
                    detections,
                }
            }
            Input::Tick => camera_freshness::Input::Tick,
            Input::Failed => camera_freshness::Input::Failed,
        };
        let freshness = guard
            .step(&camera_freshness::Event {
                now_s: check.now_s,
                input,
            })
            .map_err(|e| format!("event {index}: {e}"))?;
        if freshness.decision != check.admission_without_model {
            return Err(format!(
                "event {index}: runtime admission differs from preflight"
            ));
        }
        events.push(EventReport {
            event_index: index,
            now_text: check.now_text.clone(),
            now_s: check.now_s,
            acquired_text: check.acquired_text.clone(),
            acquired_s: check.acquired_s,
            source_image_sha256: check.source_image_sha256.clone(),
            inference_skipped,
            analysis,
            freshness,
        });
    }
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_s_camera_pipeline_v1",
        compiled_sources: compiled_sources(),
        source_sequence_sha256: source_sha,
        camera_id: sequence.camera_id.clone(),
        width: sequence.width,
        height: sequence.height,
        config: sequence.config.clone(),
        policy: sequence.policy.clone(),
        freshness_clock: "caller_supplied_replay",
        control_permission: false,
        execution: Execution {
            model_loads: usize::from(detector.is_some()),
            preflight_image_reads: image_count(checked),
            inference_image_reads: admitted,
            skipped_image_events: image_count(checked) - admitted,
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
    fn both_epoch_clock_strings_keep_correctly_rounded_identity() {
        let acquired = decimal("1341845824.197930").unwrap();
        let now = decimal("1341845824.197935").unwrap();
        assert_eq!(
            acquired.to_bits(),
            "1341845824.197930".parse::<f64>().unwrap().to_bits()
        );
        assert_eq!(
            now.to_bits(),
            "1341845824.197935".parse::<f64>().unwrap().to_bits()
        );
        assert!(now > acquired && now - acquired < 0.000006);
    }
    #[test]
    fn polls_and_failure_never_open_a_model_or_invent_objects() {
        let input = sequence(
            r#"[{"now_s":"0","input":{"kind":"tick"}},{"now_s":"0.1","input":{"kind":"failed"}},{"now_s":"0.2","input":{"kind":"tick"}}]"#,
        );
        let checked = preflight(&input, Path::new("missing")).unwrap();
        let report = run(
            &input,
            Path::new("missing"),
            Path::new("missing-model"),
            "0".repeat(64),
            &checked,
        )
        .unwrap();
        assert_eq!(report.execution.model_loads, 0);
        assert_eq!(report.execution.model_forwards, 0);
        assert!(
            report.events.iter().all(|e| e.analysis.is_none()
                && e.freshness.tracks.is_empty()
                && !e.freshness.ready)
        );
        assert_eq!(report.events[2].freshness.fault_epoch_s, Some(0.1));
    }
    #[test]
    fn delivery_clock_and_policy_errors_fail_preflight() {
        for time in ["0", "-1", "NaN", "inf", "bad", "1000000000001"] {
            let input = sequence(&format!(
                r#"[{{"now_s":"0","input":{{"kind":"tick"}}}},{{"now_s":"{time}","input":{{"kind":"tick"}}}}]"#
            ));
            assert!(preflight(&input, Path::new("missing")).is_err());
        }
        let mut input = sequence(r#"[{"now_s":"0","input":{"kind":"tick"}}]"#);
        input.policy.max_age_s = 0.0001;
        assert!(preflight(&input, Path::new("missing")).is_err());
    }
    #[test]
    fn run_cannot_silently_truncate_an_unchecked_sequence() {
        let input = sequence(r#"[{"now_s":"0","input":{"kind":"tick"}}]"#);
        let error = run(
            &input,
            Path::new("missing"),
            Path::new("missing-model"),
            "0".repeat(64),
            &[],
        )
        .err()
        .unwrap();
        assert_eq!(error, "complete preflight required");
    }
}
