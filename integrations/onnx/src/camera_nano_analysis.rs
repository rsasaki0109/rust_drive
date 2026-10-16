//! Separate Nano producer over unchanged fixed-view merge/consistency rules.
use super::{baseline_parallel, yolox_s, yolox_tiles, yolox_vehicle_consistency};
pub use baseline_parallel::{AtomicReport, read_image};
use image::RgbImage;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, time::Instant};

pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = baseline_parallel::compiled_sources();
    sources.insert(
        "integrations/onnx/src/camera_nano_analysis.rs",
        format!(
            "{:x}",
            Sha256::digest(include_bytes!("camera_nano_analysis.rs"))
        ),
    );
    sources
}

#[derive(Serialize)]
pub struct Execution {
    model_read: bool,
    images_read: usize,
    pub model_forwards: usize,
    inference_run: bool,
    view_workers: usize,
    shared_model_plans: usize,
    analysis_wall_s: f64,
}

#[derive(Serialize)]
pub struct Report {
    schema_version: u32,
    algorithm: &'static str,
    compiled_sources: BTreeMap<&'static str, String>,
    source_image_sha256: String,
    pub execution: Execution,
    control_permission: bool,
    model_sha256: &'static str,
    base_prediction: serde_json::Value,
    pub result: yolox_vehicle_consistency::ConsistencyResult,
}

fn report(
    mut base: serde_json::Value,
    result: yolox_vehicle_consistency::ConsistencyResult,
    sha: String,
    started: Instant,
    parallel: bool,
) -> Result<Report, String> {
    let count = base["views"]
        .as_array()
        .ok_or("missing computed views")?
        .len();
    // These values come from internal typed inference, never external reports.
    base["algorithm"] = serde_json::json!("pinned_yolox_nano_fixed_views_v1");
    base["compiled_sources"] =
        serde_json::to_value(compiled_sources()).map_err(|e| e.to_string())?;
    Ok(Report {
        schema_version: 1,
        algorithm: if parallel {
            "yolox_nano_parallel_image_analysis_v1"
        } else {
            "yolox_nano_serial_image_analysis_v1"
        },
        compiled_sources: compiled_sources(),
        source_image_sha256: sha,
        execution: Execution {
            model_read: true,
            images_read: 1,
            model_forwards: count,
            inference_run: true,
            view_workers: if parallel { count } else { 1 },
            shared_model_plans: 1,
            analysis_wall_s: started.elapsed().as_secs_f64(),
        },
        control_permission: false,
        model_sha256: yolox_s::MODEL_SHA256,
        base_prediction: base,
        result,
    })
}

pub fn analyze(
    detector: &yolox_s::Detector,
    image: &RgbImage,
    sha: String,
) -> Result<Report, String> {
    let started = Instant::now();
    let computed = baseline_parallel::analyze(detector, image, sha.clone())?;
    let mut serialized = serde_json::to_value(&computed).map_err(|e| e.to_string())?;
    let base = serialized
        .get_mut("base_prediction")
        .ok_or("missing internal prediction")?
        .take();
    report(base, computed.result, sha, started, true)
}

pub fn analyze_serial(
    detector: &yolox_s::Detector,
    image: &RgbImage,
    sha: String,
) -> Result<Report, String> {
    let started = Instant::now();
    let base = yolox_tiles::detect(detector, image, sha.clone())?;
    let fixture = yolox_tiles::Fixture {
        width: image.width(),
        height: image.height(),
        views: base
            .views
            .iter()
            .map(|view| yolox_tiles::FixtureView {
                rectangle: view.rectangle,
                detections: view.prediction.detections.clone(),
            })
            .collect(),
    };
    let result = yolox_vehicle_consistency::reconcile(&fixture)?;
    report(
        serde_json::to_value(base).map_err(|e| e.to_string())?,
        result,
        sha,
        started,
        false,
    )
}
