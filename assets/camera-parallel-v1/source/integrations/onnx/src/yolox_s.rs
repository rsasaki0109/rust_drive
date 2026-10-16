//! Additive pinned YOLOX-S CPU inference, separate from the immutable nano baseline.
use image::{RgbImage, imageops::FilterType};
use rustdriving_camera_detect::{Detection, intersection_over_union};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    fs,
    io::{Cursor, Read},
    path::Path,
    time::Instant,
};
use tract_onnx::prelude::*;

pub const MODEL_PROFILE: &str = "yolox-s-v1";

pub fn compiled_sources() -> std::collections::BTreeMap<&'static str, String> {
    [
        (
            "integrations/onnx/src/bin/rustdriving-camera-detect-s.rs",
            include_bytes!("bin/rustdriving-camera-detect-s.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/yolox_s.rs",
            include_bytes!("yolox_s.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/lib.rs",
            include_bytes!("lib.rs").as_slice(),
        ),
        (
            "integrations/onnx/Cargo.toml",
            include_bytes!("../Cargo.toml").as_slice(),
        ),
        (
            "integrations/onnx/Cargo.lock",
            include_bytes!("../Cargo.lock").as_slice(),
        ),
        (
            "integrations/onnx/profile-s.json",
            include_bytes!("../profile-s.json").as_slice(),
        ),
        (
            "integrations/onnx/fetch_s.py",
            include_bytes!("../fetch_s.py").as_slice(),
        ),
        (
            "rust-toolchain.toml",
            include_bytes!("../../../rust-toolchain.toml").as_slice(),
        ),
    ]
    .into_iter()
    .map(|(name, bytes)| (name, format!("{:x}", Sha256::digest(bytes))))
    .collect()
}

pub const MODEL_SHA256: &str = "c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063";
pub const MODEL_BYTES: u64 = 35_858_002;
pub const INPUT_SIZE: usize = 640;
pub const CLASSES: usize = 80;
pub const ROWS: usize = 8400;
const MAX_IMAGE_PIXELS: u64 = 16_000_000;
const MAX_DETECTIONS: usize = 300;

#[derive(Serialize)]
pub struct Report {
    pub schema_version: u32,
    pub algorithm: &'static str,
    pub model_profile: &'static str,
    pub compiled_sources: std::collections::BTreeMap<&'static str, String>,
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
    pub detections: Vec<Detection>,
}

type CpuPlan = SimplePlan<TypedFact, Box<dyn TypedOp>, TypedModel>;
pub struct Detector {
    plan: CpuPlan,
    model_load_seconds: f64,
}

pub fn verify_model(path: &Path) -> Result<Vec<u8>, String> {
    let metadata = fs::symlink_metadata(path).map_err(|e| e.to_string())?;
    if !metadata.is_file() || metadata.file_type().is_symlink() {
        return Err("model must be a regular non-symlink file".into());
    }
    let len = metadata.len();
    if len != MODEL_BYTES {
        return Err("model size differs from pinned YOLOX-S artifact".into());
    }
    let mut bytes = Vec::with_capacity(MODEL_BYTES as usize);
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(MODEL_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() as u64 != MODEL_BYTES {
        return Err("model size changed while reading pinned artifact".into());
    }
    if format!("{:x}", Sha256::digest(&bytes)) != MODEL_SHA256 {
        return Err("model SHA-256 differs from pinned YOLOX-S artifact".into());
    }
    Ok(bytes)
}

impl Detector {
    pub fn load(path: &Path) -> Result<Self, String> {
        let started = Instant::now();
        let bytes = verify_model(path)?;
        let plan = tract_onnx::onnx()
            .model_for_read(&mut Cursor::new(bytes))
            .and_then(|m| m.with_input_fact(0, f32::fact([1, 3, INPUT_SIZE, INPUT_SIZE]).into()))
            .and_then(|m| m.into_optimized())
            .and_then(|m| m.into_runnable())
            .map_err(|e| format!("ONNX plan failed: {e}"))?;
        Ok(Self {
            plan,
            model_load_seconds: started.elapsed().as_secs_f64(),
        })
    }
    pub fn detect(
        &self,
        image: &RgbImage,
        score_threshold: f32,
        nms_iou_threshold: f32,
    ) -> Result<Report, String> {
        validate_thresholds(score_threshold, nms_iou_threshold)?;
        let (input, ratio) = preprocess(image)?;
        let started = Instant::now();
        let outputs = self
            .plan
            .run(tvec!(input.into()))
            .map_err(|e| format!("CPU inference failed: {e}"))?;
        let inference_seconds = started.elapsed().as_secs_f64();
        if outputs.len() != 1 {
            return Err("expected one YOLOX output".into());
        }
        let output = outputs[0]
            .to_array_view::<f32>()
            .map_err(|e| e.to_string())?;
        if output.shape() != [1, ROWS, CLASSES + 5] {
            return Err(format!("unexpected YOLOX shape: {:?}", output.shape()));
        }
        let values = output.as_slice().ok_or("non-contiguous model output")?;
        let detections = decode(
            values,
            image.width(),
            image.height(),
            ratio,
            score_threshold,
            nms_iou_threshold,
        )?;
        Ok(Report {
            schema_version: 1,
            algorithm: "pinned_yolox_s_cpu_image_triangle_argmax_classwise",
            model_profile: MODEL_PROFILE,
            compiled_sources: compiled_sources(),
            model_profile_sha256: format!(
                "{:x}",
                Sha256::digest(include_bytes!("../profile-s.json"))
            ),
            input_shape: [1, 3, INPUT_SIZE, INPUT_SIZE],
            output_shape: [1, ROWS, CLASSES + 5],
            model_sha256: MODEL_SHA256,
            source_image_sha256: None,
            width: image.width(),
            height: image.height(),
            score_threshold,
            nms_iou_threshold,
            model_load_seconds: self.model_load_seconds,
            inference_seconds,
            detections,
        })
    }
}

fn validate_thresholds(score: f32, nms: f32) -> Result<(), String> {
    if !score.is_finite()
        || !nms.is_finite()
        || !(0.01..=1.0).contains(&score)
        || !(0.0..=1.0).contains(&nms)
    {
        return Err("score must be finite in [0.01,1], NMS IoU finite in [0,1]".into());
    }
    Ok(())
}

fn preprocess(image: &RgbImage) -> Result<(Tensor, f32), String> {
    let (w, h) = image.dimensions();
    if w == 0 || h == 0 || u64::from(w) * u64::from(h) > MAX_IMAGE_PIXELS {
        return Err("image pixel budget exceeded".into());
    }
    let ratio = (INPUT_SIZE as f32 / w as f32).min(INPUT_SIZE as f32 / h as f32);
    let resized = image::imageops::resize(
        image,
        (w as f32 * ratio).floor().max(1.0) as u32,
        (h as f32 * ratio).floor().max(1.0) as u32,
        FilterType::Triangle,
    );
    // YOLOX non-legacy preprocessing: raw BGR [0,255], top-left letterbox, 114 padding.
    let mut values = vec![114.0_f32; 3 * INPUT_SIZE * INPUT_SIZE];
    for (x, y, p) in resized.enumerate_pixels() {
        for c in 0..3 {
            values[c * INPUT_SIZE * INPUT_SIZE + y as usize * INPUT_SIZE + x as usize] =
                f32::from(p[2 - c]);
        }
    }
    let tensor =
        Tensor::from_shape(&[1, 3, INPUT_SIZE, INPUT_SIZE], &values).map_err(|e| e.to_string())?;
    Ok((tensor, ratio))
}

pub fn decode(
    values: &[f32],
    width: u32,
    height: u32,
    ratio: f32,
    score: f32,
    nms: f32,
) -> Result<Vec<Detection>, String> {
    validate_thresholds(score, nms)?;
    if values.len() != ROWS * (CLASSES + 5)
        || width == 0
        || height == 0
        || u64::from(width) * u64::from(height) > MAX_IMAGE_PIXELS
        || !ratio.is_finite()
        || ratio <= 0.0
    {
        return Err("invalid YOLOX output length or image geometry".into());
    }
    if values.iter().any(|v| !v.is_finite()) {
        return Err("non-finite model output".into());
    }
    let mut candidates = Vec::new();
    let mut row_index = 0;
    for stride in [8, 16, 32] {
        for gy in 0..INPUT_SIZE / stride {
            for gx in 0..INPUT_SIZE / stride {
                let row = &values[row_index * (CLASSES + 5)..(row_index + 1) * (CLASSES + 5)];
                row_index += 1;
                if !(-1e-6..=1.0 + 1e-6).contains(&row[4])
                    || row[5..].iter().any(|p| !(-1e-6..=1.0 + 1e-6).contains(p))
                {
                    return Err(format!(
                        "model probability outside [0,1]: row {row_index}, objectness {}, minclass {}, maxclass {}",
                        row[4],
                        row[5..].iter().copied().fold(f32::INFINITY, f32::min),
                        row[5..].iter().copied().fold(f32::NEG_INFINITY, f32::max)
                    ));
                }
                let (class_index, class_score) = row[5..]
                    .iter()
                    .enumerate()
                    .max_by(|a, b| a.1.total_cmp(b.1).then(b.0.cmp(&a.0)))
                    .unwrap();
                let confidence = row[4].clamp(0.0, 1.0) * class_score.clamp(0.0, 1.0);
                if confidence < score {
                    continue;
                }
                // Bounding exponent prevents overflow from corrupt/model-incompatible output.
                if row[2].abs() > 20.0 || row[3].abs() > 20.0 {
                    return Err("model box exponent exceeds bound".into());
                }
                let cx = (row[0] + gx as f32) * stride as f32 / ratio;
                let cy = (row[1] + gy as f32) * stride as f32 / ratio;
                let bw = row[2].exp() * stride as f32 / ratio;
                let bh = row[3].exp() * stride as f32 / ratio;
                if [cx, cy, bw, bh].iter().any(|value| !value.is_finite()) {
                    return Err("decoded model box arithmetic is non-finite".into());
                }
                let bbox = [
                    (cx - bw * 0.5).clamp(0.0, width as f32),
                    (cy - bh * 0.5).clamp(0.0, height as f32),
                    (cx + bw * 0.5).clamp(0.0, width as f32),
                    (cy + bh * 0.5).clamp(0.0, height as f32),
                ];
                if bbox[2] > bbox[0] && bbox[3] > bbox[1] {
                    candidates.push(Detection {
                        class_index,
                        confidence,
                        bbox,
                    });
                }
            }
        }
    }
    candidates.sort_by(|a, b| {
        b.confidence
            .total_cmp(&a.confidence)
            .then(a.class_index.cmp(&b.class_index))
            .then(a.bbox[0].total_cmp(&b.bbox[0]))
    });
    let mut kept: Vec<Detection> = Vec::new();
    for candidate in candidates {
        if kept.iter().any(|k| {
            k.class_index == candidate.class_index
                && intersection_over_union(k.bbox, candidate.bbox) > nms
        }) {
            continue;
        }
        kept.push(candidate);
        if kept.len() == MAX_DETECTIONS {
            break;
        }
    }
    Ok(kept)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_head() -> Vec<f32> {
        vec![0.; ROWS * (CLASSES + 5)]
    }

    fn candidate(values: &mut [f32], index: usize, class: usize, xy: [f32; 2]) {
        let row = &mut values[index * 85..(index + 1) * 85];
        row[0] = xy[0];
        row[1] = xy[1];
        row[2] = 2_f32.ln();
        row[3] = 4_f32.ln();
        row[4] = 0.9;
        row[5 + class] = 0.8;
    }

    #[test]
    fn independent_three_stride_geometry_restores_original_image_coordinates() {
        let mut values = empty_head();
        for (index, class) in [(163, 0), (6483, 1), (8043, 2)] {
            candidate(&mut values, index, class, [0.5, 0.25]);
        }
        let boxes = decode(&values, 1280, 720, 0.5, 0.3, 0.45).unwrap();
        assert_eq!(boxes.len(), 3);
        // Hand-derived boxes: stride8/16/32 at grid(3,2), offsets(.5,.25),
        // widths2*stride, heights4*stride and source scale2.
        for (box_, expected) in boxes.iter().zip([
            [40., 4., 72., 68.],
            [80., 8., 144., 136.],
            [160., 16., 288., 272.],
        ]) {
            assert_eq!(box_.bbox, expected);
            assert!((box_.confidence - 0.72).abs() < 1e-6);
        }
    }

    #[test]
    fn overlapping_boxes_are_classwise_suppressed_with_deterministic_class_ties() {
        let mut values = empty_head();
        candidate(&mut values, 163, 0, [0.5, 0.25]);
        values[163 * 85 + 6] = 0.8; // Equal class0/class1: class0 wins.
        candidate(&mut values, 164, 0, [-0.5, 0.25]);
        candidate(&mut values, 165, 2, [-1.5, 0.25]);
        let boxes = decode(&values, 640, 640, 1., 0.3, 0.45).unwrap();
        assert_eq!(boxes.len(), 2);
        assert_eq!(boxes[0].class_index, 0);
        assert_eq!(boxes[1].class_index, 2);
        assert_eq!(boxes[0].bbox, boxes[1].bbox);
    }

    #[test]
    fn raw_bgr_constant_pixels_and_letterbox_padding_are_not_normalized() {
        let image = RgbImage::from_pixel(1280, 640, image::Rgb([10, 20, 30]));
        let (input, ratio) = preprocess(&image).unwrap();
        assert_eq!(ratio, 0.5);
        let array = input.to_array_view::<f32>().unwrap();
        assert_eq!(array.shape(), &[1, 3, 640, 640]);
        for (channel, raw) in [(0, 30.), (1, 20.), (2, 10.)] {
            assert_eq!(array[[0, channel, 0, 0]], raw);
            assert_eq!(array[[0, channel, 319, 639]], raw);
            assert_eq!(array[[0, channel, 320, 0]], 114.);
        }
        assert!(preprocess(&RgbImage::new(0, 1)).is_err());
    }

    #[test]
    fn hidden_corruption_shape_and_image_resource_violations_fail_closed() {
        let mut values = empty_head();
        assert!(decode(&values[..values.len() - 1], 640, 640, 1., 0.3, 0.45).is_err());
        for corrupt in [f32::NAN, f32::INFINITY, 1.01, -0.01] {
            values[8399 * 85 + 84] = corrupt; // Even a row below score gate.
            assert!(decode(&values, 640, 640, 1., 0.3, 0.45).is_err());
        }
        values[8399 * 85 + 84] = 0.;
        assert!(decode(&values, 16000001, 1, 1., 0.3, 0.45).is_err());
        assert!(decode(&values, 640, 640, 0., 0.3, 0.45).is_err());
        assert!(decode(&values, 640, 640, 1., f32::NAN, 0.45).is_err());
        candidate(&mut values, 163, 0, [0.5, 0.25]);
        values[163 * 85 + 2] = 20.01;
        assert!(decode(&values, 640, 640, 1., 0.3, 0.45).is_err());
        values[163 * 85 + 2] = 0.;
        values[163 * 85] = f32::MAX;
        assert!(decode(&values, 640, 640, 1., 0.3, 0.45).is_err());
    }

    #[test]
    fn roundoff_threshold_and_retained_box_budget_remain_bounded() {
        let mut values = empty_head();
        values[4] = -f32::EPSILON;
        values[5] = 1. + f32::EPSILON;
        assert!(decode(&values, 640, 640, 1., 0.3, 0.45).unwrap().is_empty());
        values[4] = 0.;
        values[5] = 0.;
        for index in 0..301 {
            let row = &mut values[index * 85..(index + 1) * 85];
            row[0] = 0.5;
            row[1] = 0.5;
            row[4] = 0.9;
            row[5] = 0.8;
        }
        assert_eq!(decode(&values, 640, 640, 1., 0.3, 0.45).unwrap().len(), 300);
    }
}
