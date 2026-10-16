//! Admit only the pinned analyzer profile and unchanged retained measurements.
use rustdriving_image_tracking::{Detection, valid_sha256};
use serde_json::Value;

pub fn detections(
    report: &Value,
    width: u32,
    height: u32,
) -> Result<(String, Vec<Detection>), String> {
    let fail = || "invalid pinned camera analysis report".to_string();
    let sources: Value =
        serde_json::from_str(include_str!("../producer-sources.json")).map_err(|_| fail())?;
    let base = &report["base_prediction"];
    let image_sha = report["source_image_sha256"].as_str().ok_or_else(fail)?;
    if report["schema_version"] != 1
        || report["algorithm"] != "yolox_s_image_analysis_v1"
        || report["compiled_sources"] != sources
        || !valid_sha256(image_sha)
        || base["source_image_sha256"] != image_sha
        || base["algorithm"] != "pinned_yolox_s_full_plus_three_squares_v1"
        || base["model_sha256"]
            != "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063"
        || base["width"] != width
        || base["height"] != height
    {
        return Err(fail());
    }
    let views = base["views"].as_array().ok_or_else(fail)?;
    if !(1..=4).contains(&views.len())
        || report["execution"]
            != serde_json::json!({"model_read":true,"images_read":1,"model_forwards":views.len(),"inference_run":true})
    {
        return Err(fail());
    }
    let result = &report["result"];
    let mut selected: Vec<Detection> =
        serde_json::from_value(result["detections"].clone()).map_err(|_| fail())?;
    let original: Vec<Detection> =
        serde_json::from_value(base["detections"].clone()).map_err(|_| fail())?;
    let ranks: Vec<usize> =
        serde_json::from_value(result["retained_base_ranks"].clone()).map_err(|_| fail())?;
    let origins = result["detection_origins"].as_array().ok_or_else(fail)?;
    let base_origins = base["detection_origins"].as_array().ok_or_else(fail)?;
    if selected.len() > 300
        || original.len() > 300
        || selected.len() != ranks.len()
        || origins.len() != selected.len()
        || base_origins.len() != original.len()
        || ranks.windows(2).any(|v| v[0] >= v[1])
    {
        return Err(fail());
    }
    for (i, (&rank, candidate)) in ranks.iter().zip(&selected).enumerate() {
        let old = original.get(rank).ok_or_else(fail)?;
        let same = |a: f64, b: f64| {
            a.is_finite() && b.is_finite() && (a as f32).to_bits() == (b as f32).to_bits()
        };
        if old.class_index != candidate.class_index
            || !same(old.confidence, candidate.confidence)
            || old
                .bbox
                .iter()
                .zip(candidate.bbox)
                .any(|(&a, b)| !same(a, b))
            || origins[i] != base_origins[rank]
        {
            return Err(fail());
        }
    }
    // The analyzer stores original f32 model measurements through two JSON
    // representations. Canonicalize their ABI before doing f64 tracking math.
    for detection in &mut selected {
        detection.confidence = f64::from(detection.confidence as f32);
        detection.bbox = detection.bbox.map(|v| f64::from(v as f32));
    }
    Ok((image_sha.to_string(), selected))
}
