//! Fixed Nano arithmetic; independently executable original ONNX plan reference.
pub use super::baseline_yolox_s::Report;
pub const MODEL_BYTES: u64 = super::frozen_detector::MODEL_BYTES;
use super::frozen_detector;
use image::RgbImage;
pub use rustdriving_camera_detect::{CLASSES, INPUT_SIZE, MODEL_SHA256, ROWS};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, path::Path};
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    super::compiled_sources()
}
enum Plan {
    Optimized(Box<frozen_detector::Detector>),
    Original(Box<rustdriving_camera_detect::Detector>),
}
pub struct Detector {
    plan: Plan,
}
impl Detector {
    pub fn load(path: &Path) -> Result<Self, String> {
        Self::comparison(path, false)
    }
    pub fn comparison(path: &Path, reference: bool) -> Result<Self, String> {
        let plan = if reference {
            frozen_detector::verify_model(path)?;
            Plan::Original(Box::new(rustdriving_camera_detect::Detector::load(path)?))
        } else {
            Plan::Optimized(Box::new(frozen_detector::Detector::load(path)?))
        };
        Ok(Self { plan })
    }
    pub fn is_reference(&self) -> bool {
        matches!(self.plan, Plan::Original(_))
    }
    pub fn detect(&self, image: &RgbImage, score: f32, nms: f32) -> Result<Report, String> {
        let mut report = match &self.plan {
            Plan::Optimized(d) => d.detect(image, score, nms)?,
            Plan::Original(d) => {
                let p = d.detect(image, score, nms)?;
                Report {
                    schema_version: p.schema_version,
                    algorithm: "",
                    model_profile: "",
                    compiled_sources: compiled_sources(),
                    model_profile_sha256: String::new(),
                    input_shape: [1, 3, INPUT_SIZE, INPUT_SIZE],
                    output_shape: [1, ROWS, CLASSES + 5],
                    model_sha256: p.model_sha256,
                    source_image_sha256: p.source_image_sha256,
                    width: p.width,
                    height: p.height,
                    score_threshold: p.score_threshold,
                    nms_iou_threshold: p.nms_iou_threshold,
                    model_load_seconds: p.model_load_seconds,
                    inference_seconds: p.inference_seconds,
                    detections: p.detections,
                }
            }
        };
        // Only our internally computed typed measurement receives our identity.
        report.algorithm = if self.is_reference() {
            "pinned_nano_grid_original_plan_triangle_v1"
        } else {
            "pinned_nano_grid_depthwise_triangle_v1"
        };
        report.model_profile = "yolox-nano-grid-v1";
        report.compiled_sources = compiled_sources();
        report.model_profile_sha256 = format!(
            "{:x}",
            Sha256::digest(include_bytes!("../profile-grid.json"))
        );
        Ok(report)
    }
}
