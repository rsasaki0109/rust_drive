//! Controlled preprocessing comparison; unchanged guarded Nano inference and decoder.
use super::camera_linear::{self, Mode};
use super::{camera_depthwise, camera_repeat2};
use image::RgbImage;
pub use rustdriving_camera_detect::{CLASSES, INPUT_SIZE, MODEL_SHA256, ROWS, decode};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, io::Cursor, path::Path, time::Instant};
use tract_onnx::prelude::*;
#[derive(Serialize)]
pub struct Report {
    pub schema_version: u32,
    pub algorithm: &'static str,
    pub preprocessing: Mode,
    pub model_profile: &'static str,
    pub compiled_sources: BTreeMap<&'static str, String>,
    pub model_profile_sha256: String,
    pub input_shape: [usize; 4],
    pub output_shape: [usize; 3],
    pub model_sha256: &'static str,
    pub source_image_sha256: Option<String>,
    pub width: u32,
    pub height: u32,
    pub score_threshold: f32,
    pub nms_iou_threshold: f32,
    pub model_load_seconds: f64,
    pub inference_seconds: f64,
    pub input_tensor_f32le_sha256: String,
    pub resized: [u32; 2],
    pub original_graph_reference: bool,
    pub output_tensor_f32le_sha256: String,
    pub detections: Vec<rustdriving_camera_detect::Detection>,
}
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    super::compiled_sources()
}
pub fn verify_model(path: &Path) -> Result<Vec<u8>, String> {
    if !std::fs::symlink_metadata(path)
        .map_err(|e| e.to_string())?
        .is_file()
    {
        return Err("model must be a regular non-symlink file".into());
    }
    rustdriving_camera_detect::verify_model(path)
}
pub struct Detector {
    plan: SimplePlan<TypedFact, Box<dyn TypedOp>, TypedModel>,
    load_seconds: f64,
    mode: Mode,
    depthwise: Option<camera_depthwise::Bindings>,
}
impl Detector {
    pub fn load(path: &Path) -> Result<Self, String> {
        Self::with_mode(path, Mode::Linear)
    }
    pub fn with_mode(path: &Path, mode: Mode) -> Result<Self, String> {
        Self::comparison(path, mode, false)
    }
    pub fn comparison(path: &Path, mode: Mode, original_graph: bool) -> Result<Self, String> {
        let started = Instant::now();
        let _ = compiled_sources();
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
        let depthwise = if original_graph {
            None
        } else {
            camera_repeat2::rewrite(&mut model, &proto)?;
            Some(camera_depthwise::bind(&model, &proto)?)
        };
        Ok(Self {
            plan: model.into_runnable().map_err(|e| e.to_string())?,
            depthwise,
            load_seconds: started.elapsed().as_secs_f64(),
            mode,
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
        let prepared = camera_linear::prepare(image, INPUT_SIZE, self.mode)?;
        let ratio = prepared.ratio;
        let values = prepared.values;
        let mut hasher = Sha256::new();
        for value in &values {
            hasher.update(value.to_le_bytes());
        }
        let input_tensor_f32le_sha256 = format!("{:x}", hasher.finalize());
        let input = Tensor::from_shape(&[1, 3, INPUT_SIZE, INPUT_SIZE], &values)
            .map_err(|e| e.to_string())?;
        let started = Instant::now();
        let outputs = match &self.depthwise {
            Some(bindings) => camera_depthwise::evaluate(&self.plan, bindings, input),
            None => self.plan.run(tvec!(input.into())),
        }
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
        let mut output_hash = Sha256::new();
        for value in outputs[0].as_slice::<f32>().map_err(|e| e.to_string())? {
            output_hash.update(value.to_le_bytes());
        }
        Ok(Report {
            schema_version: 1,
            algorithm: "pinned_yolox_nano_controlled_preprocessing_v1",
            preprocessing: self.mode,
            model_profile: "yolox-nano-controlled-preprocessing-v1",
            compiled_sources: compiled_sources(),
            model_profile_sha256: format!(
                "{:x}",
                Sha256::digest(include_bytes!("../profile-linear.json"))
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
            input_tensor_f32le_sha256,
            resized: prepared.resized,
            original_graph_reference: self.depthwise.is_none(),
            output_tensor_f32le_sha256: format!("{:x}", output_hash.finalize()),
            detections,
        })
    }
}
