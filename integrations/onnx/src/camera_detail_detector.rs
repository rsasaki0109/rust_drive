//! Pinned YOLOX-S640: guarded Resize2x plan and unchanged original graph reference.
pub use super::baseline_yolox_s::{MODEL_SHA256, Report};
pub const MODEL_BYTES: u64 = super::repeat_detector::MODEL_BYTES;
use image::RgbImage;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, path::Path};
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    super::compiled_sources()
}
enum Plan {
    Optimized(Box<super::repeat_detector::Detector>),
    Original(Box<super::baseline_yolox_s::Detector>),
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
            Plan::Original(Box::new(super::baseline_yolox_s::Detector::load(path)?))
        } else {
            Plan::Optimized(Box::new(super::repeat_detector::Detector::load(path)?))
        };
        Ok(Self { plan })
    }
    pub fn is_reference(&self) -> bool {
        matches!(self.plan, Plan::Original(_))
    }
    pub fn detect(&self, image: &RgbImage, score: f32, nms: f32) -> Result<Report, String> {
        let mut report = match &self.plan {
            Plan::Optimized(d) => d.detect(image, score, nms)?,
            Plan::Original(d) => d.detect(image, score, nms)?,
        };
        // Identity describes this internally executed measurement, not imported JSON.
        report.algorithm = if self.is_reference() {
            "pinned_s_detail_original_plan_triangle_v1"
        } else {
            "pinned_s_detail_repeat2x_triangle_v1"
        };
        report.model_profile = "yolox-s-detail-v1";
        report.compiled_sources = compiled_sources();
        report.model_profile_sha256 = format!(
            "{:x}",
            Sha256::digest(include_bytes!("../profile-detail.json"))
        );
        Ok(report)
    }
}
