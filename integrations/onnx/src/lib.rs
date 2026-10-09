//! Offline measured-camera inference. Pixel boxes are never vehicle-control inputs.
use image::{ImageReader, RgbImage, imageops::FilterType};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{fs, io::Cursor, path::Path, time::Instant};
use tract_onnx::prelude::*;

pub const MODEL_SHA256: &str = "c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d";
pub const MODEL_BYTES: u64 = 3_659_407;
pub const INPUT_SIZE: usize = 416;
pub const CLASSES: usize = 80;
pub const ROWS: usize = 3549;
const MAX_IMAGE_BYTES: u64 = 16 * 1024 * 1024;
const MAX_IMAGE_PIXELS: u64 = 16_000_000;
const MAX_DETECTIONS: usize = 300;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Detection {
    /// Zero-based COCO contiguous class index (not sparse COCO category ID).
    pub class_index: usize,
    pub confidence: f32,
    /// Pixel [left, top, right, bottom] in original image coordinates.
    pub bbox: [f32; 4],
}
#[derive(Serialize)]
pub struct Report {
    pub schema_version: u32,
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
    let len = fs::metadata(path).map_err(|e| e.to_string())?.len();
    if len != MODEL_BYTES {
        return Err("model size differs from pinned YOLOX nano artifact".into());
    }
    let bytes = fs::read(path).map_err(|e| e.to_string())?;
    if format!("{:x}", Sha256::digest(&bytes)) != MODEL_SHA256 {
        return Err("model SHA-256 differs from pinned YOLOX nano artifact".into());
    }
    Ok(bytes)
}

pub fn read_image(path: &Path) -> Result<(RgbImage, String), String> {
    if fs::metadata(path).map_err(|e| e.to_string())?.len() > MAX_IMAGE_BYTES {
        return Err("image encoded-byte budget exceeded".into());
    }
    let bytes = fs::read(path).map_err(|e| e.to_string())?;
    let reader = ImageReader::new(Cursor::new(&bytes))
        .with_guessed_format()
        .map_err(|e| e.to_string())?;
    let (w, h) = reader.into_dimensions().map_err(|e| e.to_string())?;
    if w == 0 || h == 0 || u64::from(w) * u64::from(h) > MAX_IMAGE_PIXELS {
        return Err("image pixel budget exceeded".into());
    }
    image::load_from_memory(&bytes)
        .map(|i| (i.into_rgb8(), format!("{:x}", Sha256::digest(&bytes))))
        .map_err(|e| e.to_string())
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

pub fn intersection_over_union(a: [f32; 4], b: [f32; 4]) -> f32 {
    let iw = (a[2].min(b[2]) - a[0].max(b[0])).max(0.0);
    let ih = (a[3].min(b[3]) - a[1].max(b[1])).max(0.0);
    let aa = (a[2] - a[0]).max(0.0) * (a[3] - a[1]).max(0.0);
    let ba = (b[2] - b[0]).max(0.0) * (b[3] - b[1]).max(0.0);
    let union = aa + ba - iw * ih;
    if union > 0.0 { iw * ih / union } else { 0.0 }
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
    #[test]
    fn malformed_outputs_and_budgets_fail_closed() {
        let mut output = vec![0.0; ROWS * (CLASSES + 5)];
        assert!(decode(&output[..10], 100, 100, 1.0, 0.3, 0.45).is_err());
        output[50] = f32::NAN;
        assert!(decode(&output, 100, 100, 1.0, 0.3, 0.45).is_err());
        assert!(validate_thresholds(f32::NAN, 0.45).is_err());
        assert!(preprocess(&RgbImage::new(0, 0)).is_err());
    }
    #[test]
    fn pinned_model_and_image_decoding_reject_bad_assets() {
        let dir = std::env::temp_dir().join(format!(
            "rustdrive-camera-bad-assets-{}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let model = dir.join("model.onnx");
        let file = std::fs::File::create(&model).unwrap();
        file.set_len(MODEL_BYTES).unwrap();
        assert!(verify_model(&model).unwrap_err().contains("SHA-256"));
        file.set_len(12).unwrap();
        assert!(verify_model(&model).unwrap_err().contains("size"));
        let image = dir.join("image.jpg");
        std::fs::write(&image, b"not a JPEG").unwrap();
        assert!(read_image(&image).is_err());
        std::fs::File::create(&image)
            .unwrap()
            .set_len(MAX_IMAGE_BYTES + 1)
            .unwrap();
        assert!(
            read_image(&image)
                .unwrap_err()
                .contains("encoded-byte budget")
        );
        std::fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn roundoff_is_bounded_and_probability_corruption_rejected() {
        let mut output = vec![0.0; ROWS * (CLASSES + 5)];
        output[4] = -f32::EPSILON;
        assert!(
            decode(&output, 100, 100, 1.0, 0.3, 0.45)
                .unwrap()
                .is_empty()
        );
        output[4] = -0.001;
        assert!(decode(&output, 100, 100, 1.0, 0.3, 0.45).is_err());
        output[4] = 0.9;
        output[5] = 1.001;
        assert!(decode(&output, 100, 100, 1.0, 0.3, 0.45).is_err());
    }
    #[test]
    fn independent_box_geometry_and_classwise_nms() {
        assert!(
            (intersection_over_union([0., 0., 10., 10.], [5., 0., 15., 10.]) - 1.0 / 3.0).abs()
                < 1e-6
        );
        let mut output = vec![0.0; ROWS * (CLASSES + 5)];
        for (index, c) in [(53, 2), (54, 2), (55, 0)] {
            let row = &mut output[index * 85..(index + 1) * 85];
            // Shift neighboring grid centers to identical 16x16 boxes centered at (12,12).
            row[0] = 1.5 - (index % 52) as f32;
            row[1] = 0.5;
            row[2] = 2.0_f32.ln();
            row[3] = 2.0_f32.ln();
            row[4] = 0.9;
            row[5 + c] = 0.8;
        }
        let boxes = decode(&output, 100, 100, 1.0, 0.3, 0.45).unwrap();
        assert_eq!(boxes.len(), 2);
        assert_eq!(boxes[0].bbox, [4., 4., 20., 20.]);
        assert_ne!(boxes[0].class_index, boxes[1].class_index);
    }
}
