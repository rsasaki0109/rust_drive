//! Adapt the unchanged pinned Nano detector to the existing typed view interface.
pub use super::baseline_yolox_s::Report;
use image::RgbImage;
use rustdriving_camera_detect as nano;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, fs, path::Path};
pub const MODEL_SHA256: &str = nano::MODEL_SHA256;
pub const MODEL_BYTES: u64 = nano::MODEL_BYTES;

pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    // The old S source supplies only the common report type. The actual model,
    // preprocessing, inference and decoder below are the original Nano library.
    let mut sources = super::baseline_yolox_s::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/camera_nano_detector.rs",
            include_bytes!("camera_nano_detector.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-nano.rs",
            include_bytes!("bin/rustdriving-camera-nano.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-nano-parallel.json",
            include_bytes!("../profile-nano-parallel.json").as_slice(),
        ),
        (
            "integrations/onnx/fetch.py",
            include_bytes!("../fetch.py").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

pub struct Detector(nano::Detector);

impl Detector {
    pub fn load(path: &Path) -> Result<Self, String> {
        let metadata = fs::symlink_metadata(path).map_err(|e| e.to_string())?;
        if !metadata.is_file() || metadata.file_type().is_symlink() {
            return Err("model must be a regular non-symlink file".into());
        }
        if metadata.len() != MODEL_BYTES {
            return Err("model size differs from pinned YOLOX nano artifact".into());
        }
        // Nano verifies its original byte/hash pin again before ONNX parsing.
        nano::Detector::load(path).map(Self)
    }

    pub fn detect(&self, image: &RgbImage, score: f32, nms: f32) -> Result<Report, String> {
        let original = self.0.detect(image, score, nms)?;
        Ok(Report {
            schema_version: 1,
            algorithm: "pinned_yolox_nano_cpu_image_triangle_argmax_classwise",
            model_profile: "yolox-nano-parallel-v1",
            compiled_sources: compiled_sources(),
            model_profile_sha256: format!(
                "{:x}",
                Sha256::digest(include_bytes!("../profile-nano-parallel.json"))
            ),
            input_shape: [1, 3, nano::INPUT_SIZE, nano::INPUT_SIZE],
            output_shape: [1, nano::ROWS, nano::CLASSES + 5],
            model_sha256: MODEL_SHA256,
            source_image_sha256: None,
            width: original.width,
            height: original.height,
            score_threshold: original.score_threshold,
            nms_iou_threshold: original.nms_iou_threshold,
            model_load_seconds: original.model_load_seconds,
            inference_seconds: original.inference_seconds,
            detections: original.detections,
        })
    }
}
