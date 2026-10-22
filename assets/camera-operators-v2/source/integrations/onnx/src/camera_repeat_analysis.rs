//! A distinct producer built from internally measured lowered-plan detections.
use super::{baseline_parallel, yolox_s, yolox_vehicle_consistency};
pub use baseline_parallel::{AtomicReport, read_image};
use image::RgbImage;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, time::Instant};
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = baseline_parallel::compiled_sources();
    sources.insert(
        "integrations/onnx/src/camera_repeat_analysis.rs",
        format!(
            "{:x}",
            Sha256::digest(include_bytes!("camera_repeat_analysis.rs"))
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
    graph_optimization: &'static str,
    base_prediction: serde_json::Value,
    pub result: yolox_vehicle_consistency::ConsistencyResult,
}
pub fn analyze(
    detector: &yolox_s::Detector,
    image: &RgbImage,
    sha: String,
) -> Result<Report, String> {
    let started = Instant::now();
    let computed = baseline_parallel::analyze(detector, image, sha.clone())?;
    // This constructs our own report around internal typed inference, not an
    // external report admitted under a rewritten serial producer identity.
    let mut serialized = serde_json::to_value(&computed).map_err(|e| e.to_string())?;
    let mut base = serialized
        .get_mut("base_prediction")
        .ok_or("missing internal base prediction")?
        .take();
    base["algorithm"] = serde_json::json!("pinned_yolox_s_repeat2x_parallel_fixed_views_v1");
    base["compiled_sources"] =
        serde_json::to_value(compiled_sources()).map_err(|e| e.to_string())?;
    let sources = compiled_sources();
    let n = computed.execution.model_forwards;
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_s_repeat2x_parallel_image_analysis_v1",
        compiled_sources: sources,
        source_image_sha256: sha,
        execution: Execution {
            model_read: true,
            images_read: 1,
            model_forwards: n,
            inference_run: true,
            view_workers: n,
            shared_model_plans: 1,
            analysis_wall_s: started.elapsed().as_secs_f64(),
        },
        graph_optimization: "validated_asymmetric_floor_nearest_2x_only",
        base_prediction: base,
        result: computed.result,
    })
}
