//! Executable detector using the already bit-checked, guarded Resize lowering.
pub use super::baseline_yolox_s::{
    CLASSES, INPUT_SIZE, MODEL_BYTES, MODEL_SHA256, ROWS, Report, decode, verify_model,
};
use super::{baseline_yolox_s, camera_repeat2};
use image::{RgbImage, imageops::FilterType};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, io::Cursor, path::Path, time::Instant};
use tract_onnx::prelude::*;
const MODEL_PROFILE: &str = "yolox-s-repeat2x-v1";
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut result = baseline_yolox_s::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/camera_repeat_detector.rs",
            include_bytes!("camera_repeat_detector.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/camera_repeat2.rs",
            include_bytes!("camera_repeat2.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-repeat2x.rs",
            include_bytes!("bin/rustdriving-camera-repeat2x.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-repeat2x.json",
            include_bytes!("../profile-repeat2x.json").as_slice(),
        ),
    ] {
        result.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    result
}
pub struct Detector {
    plan: SimplePlan<TypedFact, Box<dyn TypedOp>, TypedModel>,
    load_seconds: f64,
}
impl Detector {
    pub fn load(path: &Path) -> Result<Self, String> {
        let started = Instant::now();
        let bytes = verify_model(path)?;
        let onnx = tract_onnx::onnx();
        let proto = onnx
            .proto_model_for_read(&mut Cursor::new(bytes))
            .map_err(|e| e.to_string())?;
        let mut model = onnx
            .model_for_proto_model(&proto)
            .and_then(|m| m.with_input_fact(0, f32::fact([1, 3, INPUT_SIZE, INPUT_SIZE]).into()))
            .and_then(|m| m.into_optimized())
            .map_err(|e| e.to_string())?;
        camera_repeat2::rewrite(&mut model, &proto)?;
        Ok(Self {
            plan: model.into_runnable().map_err(|e| e.to_string())?,
            load_seconds: started.elapsed().as_secs_f64(),
        })
    }
    pub fn detect(&self, image: &RgbImage, score: f32, nms: f32) -> Result<Report, String> {
        if !score.is_finite()
            || !nms.is_finite()
            || !(0.01..=1.0).contains(&score)
            || !(0.0..=1.0).contains(&nms)
        {
            return Err("invalid score/NMS thresholds".into());
        }
        let (w, h) = image.dimensions();
        if w == 0 || h == 0 || u64::from(w) * u64::from(h) > 16_000_000 {
            return Err("image pixel budget exceeded".into());
        }
        let ratio = (INPUT_SIZE as f32 / w as f32).min(INPUT_SIZE as f32 / h as f32);
        let resized = image::imageops::resize(
            image,
            (w as f32 * ratio).floor().max(1.0) as u32,
            (h as f32 * ratio).floor().max(1.0) as u32,
            FilterType::Triangle,
        );
        let mut values = vec![114.0_f32; 3 * INPUT_SIZE * INPUT_SIZE];
        for (x, y, p) in resized.enumerate_pixels() {
            for c in 0..3 {
                values[c * INPUT_SIZE * INPUT_SIZE + y as usize * INPUT_SIZE + x as usize] =
                    f32::from(p[2 - c]);
            }
        }
        let input = Tensor::from_shape(&[1, 3, INPUT_SIZE, INPUT_SIZE], &values)
            .map_err(|e| e.to_string())?;
        let started = Instant::now();
        let outputs = self
            .plan
            .run(tvec!(input.into()))
            .map_err(|e| e.to_string())?;
        let inference_seconds = started.elapsed().as_secs_f64();
        if outputs.len() != 1 || outputs[0].shape() != [1, ROWS, CLASSES + 5] {
            return Err("unexpected pinned output shape".into());
        }
        let detections = decode(
            outputs[0].as_slice::<f32>().map_err(|e| e.to_string())?,
            w,
            h,
            ratio,
            score,
            nms,
        )?;
        Ok(Report {
            schema_version: 1,
            algorithm: "pinned_yolox_s_repeat2x_cpu_image_triangle_argmax_classwise",
            model_profile: MODEL_PROFILE,
            compiled_sources: compiled_sources(),
            model_profile_sha256: format!(
                "{:x}",
                Sha256::digest(include_bytes!("../profile-repeat2x.json"))
            ),
            input_shape: [1, 3, INPUT_SIZE, INPUT_SIZE],
            output_shape: [1, ROWS, CLASSES + 5],
            model_sha256: MODEL_SHA256,
            source_image_sha256: None,
            width: w,
            height: h,
            score_threshold: score,
            nms_iou_threshold: nms,
            model_load_seconds: self.load_seconds,
            inference_seconds,
            detections,
        })
    }
}
