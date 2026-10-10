//! New continuous evaluator. Measurement functions below are minimally copied
//! from the immutable pair prototype because its private APIs cannot be imported.
use rustdriving_core::Vec3;
use rustdriving_localization::{
    registration3d::{Pose3, Quaternion},
    reprojection3d::{
        CameraIntrinsics, ReprojectionConfig3d, ReprojectionObservation3d, refine_reprojection,
    },
    visual_odometry3d::{
        Correspondence3d, VisualOdometry3dConfig, register_correspondences, validate_geometry,
    },
};
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::{Cursor, Read, Write},
    path::{Component, Path},
    sync::Arc,
};
mod image_features {
    pub use rustdriving_perception::image_features::*;
}
#[path = "../../../crates/perception/src/multiscale_features.rs"]
mod multiscale_features;
use image_features::{GrayImage, extract_features};
use multiscale_features::{
    MultiscaleFeature, downsample_half, extract_multiscale, match_multiscale,
};

#[derive(Clone, Deserialize)]
struct Calibration {
    width: usize,
    height: usize,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    units_per_metre: f64,
    invalid_depth: u16,
}
#[derive(Deserialize)]
struct Frame {
    source_index: usize,
    depth_file: String,
    depth_timestamp: f64,
    rgb_file: String,
    rgb_timestamp: f64,
}
#[derive(Deserialize)]
struct SourceFile {
    file: String,
    bytes: usize,
    sha256: String,
    role: String,
}
#[derive(Deserialize)]
struct Manifest {
    dataset: String,
    depth_calibration: Calibration,
    frames: Vec<Frame>,
    files: Vec<SourceFile>,
}
struct Sensor {
    native: Vec<MultiscaleFeature>,
    multi: Vec<MultiscaleFeature>,
    depth: Vec<u8>,
    witness: Value,
}
fn sha(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn bounded(path: &Path, max: usize) -> Result<Vec<u8>, String> {
    let mut out = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(max as u64 + 1)
        .read_to_end(&mut out)
        .map_err(|e| e.to_string())?;
    if out.len() > max {
        return Err("byte bound exceeded".into());
    }
    Ok(out)
}
fn safe(name: &str) -> bool {
    let mut parts = Path::new(name).components();
    !name.contains(['/', '\\'])
        && matches!(parts.next(), Some(Component::Normal(_)))
        && parts.next().is_none()
}
fn vjson(p: Vec3) -> Value {
    json!([p.x, p.y, p.z])
}
fn pose_json(p: Pose3) -> Value {
    json!({"translation_m":vjson(p.translation),"quaternion_wxyz":[p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z]})
}
fn feature_json(f: &MultiscaleFeature) -> Value {
    let p = f.feature;
    json!({"x":p.x,"y":p.y,"level":f.level,"level_x":f.level_x,"level_y":f.level_y,
        "score":p.score,"orientation":p.orientation,"descriptor":p.descriptor})
}
fn native(w: usize, h: usize, gray: &[u8]) -> Result<Vec<MultiscaleFeature>, String> {
    Ok(
        extract_features(GrayImage::new(w, h, gray).map_err(|e| format!("{e:?}"))?)
            .into_iter()
            .map(|feature| MultiscaleFeature {
                level: 0,
                level_x: feature.x,
                level_y: feature.y,
                feature,
            })
            .collect(),
    )
}
fn depth_point(
    depth: &[u8],
    f: &MultiscaleFeature,
    c: &Calibration,
) -> (Result<Vec3, String>, Value) {
    let (x, y) = (f.feature.x.round() as usize, f.feature.y.round() as usize);
    let mut witness = json!({"sample_x":x,"sample_y":y,"raw_stencil":[]});
    let result = (|| {
        if depth.len() != 2 * c.width * c.height
            || x < 1
            || y < 1
            || x + 1 >= c.width
            || y + 1 >= c.height
        {
            return Err("depth patch exceeds image".into());
        }
        let mut samples = Vec::new();
        for py in y - 1..=y + 1 {
            for px in x - 1..=x + 1 {
                let i = 2 * (py * c.width + px);
                samples.push(u16::from_be_bytes([depth[i], depth[i + 1]]));
            }
        }
        witness["raw_stencil"] = json!(samples);
        let z: Vec<_> = samples
            .iter()
            .map(|&raw| raw as f64 / c.units_per_metre)
            .collect();
        if samples.contains(&c.invalid_depth) || z.iter().any(|z| !(0.3..=5.).contains(z)) {
            return Err("invalid or range-limited measured depth patch".into());
        }
        let spread = z.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - z.iter().copied().fold(f64::INFINITY, f64::min);
        if spread > 0.05 {
            return Err("measured depth discontinuity exceeds patch gate".into());
        }
        // The depth measurement is sampled from the ORIGINAL image, never a
        // pyramid depth map. The ray follows the fractional feature centre.
        Ok(Vec3::new(
            (f.feature.x - c.cx) * z[4] / c.fx,
            (f.feature.y - c.cy) * z[4] / c.fy,
            z[4],
        ))
    })();
    match &result {
        Ok(p) => {
            witness["xyz"] = vjson(*p);
        }
        Err(e) => {
            witness["rejection"] = json!(e);
        }
    }
    (result, witness)
}
fn pair(
    a: &Sensor,
    b: &Sensor,
    c: &Calibration,
    multi: bool,
    clock: Result<(), String>,
) -> (Option<Pose3>, Value) {
    let mut row =
        json!({"accepted":false,"matches":[],"correspondences":[],"refinement_observations":[]});
    let result: Result<Pose3, String> = (|| {
        clock?;
        let (fa, fb) = if multi {
            (&a.multi, &b.multi)
        } else {
            (&a.native, &b.native)
        };
        let matches = match_multiscale(fa, fb).map_err(|e| format!("{e:?}"))?;
        let mut rows = Vec::new();
        let mut pairs = Vec::new();
        let mut pixels = Vec::new();
        for m in matches {
            let (pa, wa) = depth_point(&a.depth, &fa[m.previous_index], c);
            let (pb, wb) = depth_point(&b.depth, &fb[m.current_index], c);
            let mut entry = json!({"previous_index":m.previous_index,"current_index":m.current_index,
                "hamming_distance":m.hamming_distance,"previous_depth":wa,"current_depth":wb});
            if let (Ok(previous), Ok(current)) = (pa, pb) {
                entry["correspondence_index"] = json!(pairs.len());
                pairs.push(Correspondence3d { previous, current });
                pixels.push([fb[m.current_index].feature.x, fb[m.current_index].feature.y]);
            }
            rows.push(entry);
        }
        row["matches"] = json!(rows);
        row["correspondences"] = json!(
            pairs
                .iter()
                .map(|p| json!({"previous":vjson(p.previous),"current":vjson(p.current)}))
                .collect::<Vec<_>>()
        );
        let fit = register_correspondences(&pairs, &VisualOdometry3dConfig::default())?;
        row["coarse_pose"] = pose_json(fit.pose);
        row["coarse_fit"] = json!({"rms_m":fit.rms_m,"inlier_indices":fit.inlier_indices,
            "inlier_count":fit.inlier_count,"inlier_ratio":fit.inlier_ratio,"hypotheses_evaluated":fit.hypotheses_evaluated,
            "point_checks":fit.point_checks,"refits":fit.refits,"geometry_ratio_current":fit.geometry_ratio_current,
            "geometry_ratio_previous":fit.geometry_ratio_previous,"candidate_models":fit.candidate_models,"competing_models":fit.competing_models});
        let observations: Vec<_> = fit
            .inlier_indices
            .iter()
            .map(|&i| ReprojectionObservation3d {
                previous: pairs[i].previous,
                current_pixel: pixels[i],
            })
            .collect();
        row["refinement_observations"]=json!(fit.inlier_indices.iter().map(|&i|json!({"correspondence_index":i,"previous":vjson(pairs[i].previous),"current_pixel":pixels[i]})).collect::<Vec<_>>());
        let refined = refine_reprojection(
            &observations,
            &CameraIntrinsics {
                fx: c.fx,
                fy: c.fy,
                cx: c.cx,
                cy: c.cy,
            },
            fit.pose,
            &ReprojectionConfig3d::default(),
        )?;
        row["refinement"] = json!({"initial_rms_px":refined.initial_rms_px,"final_rms_px":refined.final_rms_px,
            "initial_huber_cost":refined.initial_huber_cost,"final_huber_cost":refined.final_huber_cost,
            "iterations":refined.iterations,"accepted_steps":refined.accepted_steps,"point_checks":refined.point_checks,
            "valid_support":refined.valid_support,"converged":refined.converged,"max_normal_condition_number":refined.max_normal_condition_number,
            "trace":refined.trace.iter().map(|s|json!({"current_from_previous":pose_json(s.current_from_previous),
                "huber_cost_before":s.huber_cost_before,"huber_cost_after":s.huber_cost_after,"scale":s.scale,
                "increment":s.increment,"normal_condition_number":s.normal_condition_number,"line_search_trials":s.line_search_trials})).collect::<Vec<_>>()});
        Ok(refined.pose)
    })();
    match result {
        Ok(p) => {
            row["accepted"] = json!(true);
            row["relative_estimate"] = pose_json(p);
            (Some(p), row)
        }
        Err(e) => {
            row["rejection"] = json!(e);
            (None, row)
        }
    }
}
fn decode(bytes: &[u8], depth: bool, c: &Calibration) -> Result<Vec<u8>, String> {
    if bytes.len() > 4 * 1024 * 1024 {
        return Err("PNG byte bound".into());
    }
    let mut decoder = png::Decoder::new(Cursor::new(bytes));
    decoder.set_limits(png::Limits {
        bytes: 8 * 1024 * 1024,
    });
    decoder.ignore_checksums(false);
    decoder.set_transformations(png::Transformations::IDENTITY);
    let mut reader = decoder.read_info().map_err(|e| e.to_string())?;
    let info = reader.info();
    let valid = if depth {
        info.color_type == png::ColorType::Grayscale && info.bit_depth == png::BitDepth::Sixteen
    } else {
        matches!(info.color_type, png::ColorType::Rgb | png::ColorType::Rgba)
            && info.bit_depth == png::BitDepth::Eight
    };
    if info.width as usize != c.width
        || info.height as usize != c.height
        || info.animation_control.is_some()
        || !valid
    {
        return Err("invalid static sensor PNG".into());
    }
    let channels = if depth {
        2
    } else if info.color_type == png::ColorType::Rgb {
        3
    } else {
        4
    };
    let size = reader.output_buffer_size().ok_or("PNG output overflow")?;
    if size != c.width * c.height * channels {
        return Err("PNG layout mismatch".into());
    }
    let mut out = vec![0; size];
    if reader
        .next_frame(&mut out)
        .map_err(|e| e.to_string())?
        .buffer_size()
        != size
    {
        return Err("incomplete PNG".into());
    }
    reader.finish().map_err(|e| e.to_string())?;
    if depth {
        return Ok(out);
    }
    out.chunks_exact(channels)
        .map(|p| {
            if channels == 4 && p[3] != 255 {
                return Err("nonopaque sensor RGB".into());
            }
            Ok(((77 * p[0] as u32 + 150 * p[1] as u32 + 29 * p[2] as u32) >> 8) as u8)
        })
        .collect()
}
fn verified(
    raw: &Path,
    file: &str,
    inventory: &BTreeMap<&str, &SourceFile>,
) -> Result<Vec<u8>, String> {
    if !safe(file) {
        return Err("unsafe raw filename".into());
    }
    let pin = inventory.get(file).ok_or("missing inventory pin")?;
    let bytes = bounded(&raw.join(file), 4 * 1024 * 1024)?;
    if bytes.len() != pin.bytes || sha(&bytes) != pin.sha256 {
        return Err("raw pin mismatch".into());
    }
    Ok(bytes)
}

// Evaluation ONLY: called after the entire sensor-pair loop has completed.
fn truth(bytes: &[u8]) -> Result<Vec<(f64, Pose3)>, String> {
    let text = std::str::from_utf8(bytes).map_err(|e| e.to_string())?;
    let mut out = Vec::new();
    for line in text
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.trim_start().starts_with('#'))
    {
        if out.len() >= 30_000 {
            return Err("truth row bound".into());
        }
        let v: Vec<f64> = line
            .split_whitespace()
            .map(|x| x.parse::<f64>().map_err(|e| e.to_string()))
            .collect::<Result<_, _>>()?;
        if v.len() != 8
            || v.iter().any(|x| !x.is_finite())
            || out.last().is_some_and(|(t, _)| v[0] <= *t)
        {
            return Err("invalid chronological labels".into());
        }
        out.push((
            v[0],
            Pose3 {
                translation: Vec3::new(v[1], v[2], v[3]),
                rotation: Quaternion {
                    w: v[7],
                    x: v[4],
                    y: v[5],
                    z: v[6],
                }
                .normalized()?,
            },
        ));
    }
    Ok(out)
}
fn interpolate(gt: &[(f64, Pose3)], stamp: f64) -> Result<Pose3, String> {
    let right = gt.partition_point(|(t, _)| *t < stamp);
    if right < gt.len() && gt[right].0 == stamp {
        return Ok(gt[right].1);
    }
    if right == 0 || right == gt.len() {
        return Err("truth cannot bracket; no extrapolation".into());
    }
    let (ta, a) = gt[right - 1];
    let (tb, b) = gt[right];
    if tb - ta > 0.02 {
        return Err("truth bracket exceeds .02s".into());
    }
    let t = (stamp - ta) / (tb - ta);
    let qa = a.rotation;
    let mut qb = b.rotation;
    let mut dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
    if dot < 0. {
        dot = -dot;
        qb = Quaternion {
            w: -qb.w,
            x: -qb.x,
            y: -qb.y,
            z: -qb.z,
        };
    }
    let (sa, sb) = if dot > 0.9995 {
        (1. - t, t)
    } else {
        let angle = dot.clamp(-1., 1.).acos();
        (
            ((1. - t) * angle).sin() / angle.sin(),
            (t * angle).sin() / angle.sin(),
        )
    };
    Ok(Pose3 {
        translation: Vec3::new(
            a.translation.x + (b.translation.x - a.translation.x) * t,
            a.translation.y + (b.translation.y - a.translation.y) * t,
            a.translation.z + (b.translation.z - a.translation.z) * t,
        ),
        rotation: Quaternion {
            w: sa * qa.w + sb * qb.w,
            x: sa * qa.x + sb * qb.x,
            y: sa * qa.y + sb * qb.y,
            z: sa * qa.z + sb * qb.z,
        }
        .normalized()?,
    })
}
fn score(estimate: Option<Pose3>, reference: Result<Pose3, String>) -> Value {
    match reference {
        Err(e) => {
            json!({"reference_valid":false,"reference_rejection":e,"estimate_present":estimate.is_some(),"within_accuracy_gates":false})
        }
        Ok(t) => {
            let mut row = json!({"reference_valid":true,"evaluation_only_truth":pose_json(t),"estimate_present":estimate.is_some(),"within_accuracy_gates":false});
            if let Some(p) = estimate {
                let error = (p.translation.x - t.translation.x)
                    .hypot(p.translation.y - t.translation.y)
                    .hypot(p.translation.z - t.translation.z);
                let angle = p.rotation.angular_distance(t.rotation);
                row["translation_error_m"] = json!(error);
                row["rotation_error_rad"] = json!(angle);
                row["within_accuracy_gates"] = json!(error <= 0.1 && angle <= 0.1);
            }
            row
        }
    }
}
fn texture(width: usize, height: usize) -> Vec<u8> {
    (0..width * height)
        .map(|i| {
            let (x, y) = (i % width, i / width);
            let base = if (x / 7 + y / 11) % 2 == 0 { 40 } else { 170 };
            base + ((x * 97 + y * 193 + x * y * 17) % 61) as u8
        })
        .collect()
}

const DESIGN_SHA: &str = "d0b6a8f21ded03ce8125e30e47440d223c77567d0f2e41b6d9031c32e1ea34df";
fn sources() -> Value {
    json!({
        "temporal_binary":sha(include_bytes!("bin/rustdriving-rgbd-multiscale-temporal.rs")),
        "temporal_support":sha(include_bytes!("multiscale_temporal_support.rs")),
        "design":sha(include_bytes!("../../../assets/multiscale-temporal-v1/design.json")),
        "pair_binary":sha(include_bytes!("bin/rustdriving-rgbd-multiscale-pairs.rs")),
        "multiscale_features":sha(include_bytes!("../../../crates/perception/src/multiscale_features.rs")),
        "image_features":sha(include_bytes!("../../../crates/perception/src/image_features.rs")),
        "registration3d":sha(include_bytes!("../../../crates/localization/src/registration3d.rs")),
        "visual_odometry3d":sha(include_bytes!("../../../crates/localization/src/visual_odometry3d.rs")),
        "reprojection3d":sha(include_bytes!("../../../crates/localization/src/reprojection3d.rs")),
        "localization_lib":sha(include_bytes!("../../../crates/localization/src/lib.rs")),
        "perception_lib":sha(include_bytes!("../../../crates/perception/src/lib.rs")),
        "core_lib":sha(include_bytes!("../../../crates/core/src/lib.rs")),
        "cargo_lock":sha(include_bytes!("../Cargo.lock")),
        "cargo_manifest":sha(include_bytes!("../Cargo.toml")),
        "rust_toolchain":sha(include_bytes!("../../../rust-toolchain.toml")),
        "core_manifest":sha(include_bytes!("../../../crates/core/Cargo.toml")),
        "perception_manifest":sha(include_bytes!("../../../crates/perception/Cargo.toml")),
        "localization_manifest":sha(include_bytes!("../../../crates/localization/Cargo.toml"))
    })
}
fn design() -> Result<Value, String> {
    let bytes = include_bytes!("../../../assets/multiscale-temporal-v1/design.json");
    if sha(bytes) != DESIGN_SHA {
        return Err("fixed design SHA differs".into());
    }
    serde_json::from_slice(bytes).map_err(|e| e.to_string())
}
fn protocol(bytes: &[u8]) -> Result<Value, String> {
    let metadata: Value = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
    let name = metadata["dataset"].as_str().ok_or("dataset missing")?;
    let d = design()?;
    if d["datasets"][name]["manifest_sha256"] != sha(bytes) {
        return Err("manifest not one of two fixed viewed intervals".into());
    }
    Ok(
        json!({"schema_version":1,"algorithm":"bounded_multiscale_continuous_viewed_comparison",
        "dataset":name,"manifest_sha256":sha(bytes),"design_sha256":DESIGN_SHA,"design":d,
        "sources":sources(),
        "source_reuse":"Measurement functions feature_json/native/depth_point/pair/decode/verified/truth/interpolate/score/texture copied byte-for-byte from immutable pair prototype; private API prevented import. New sensor/state/evaluation/CLI functions are separately bound.",
        "indices":"ordinal0..179; original source_index100..279 separately retained",
        "no_ground_truth_operational":true}),
    )
}
fn sensor_mask(
    c: &Calibration,
    gray: &[u8],
    depth: Vec<u8>,
    allow: [bool; 2],
) -> Result<Sensor, String> {
    let a = if allow[0] {
        native(c.width, c.height, gray)?
    } else {
        Vec::new()
    };
    let b = if allow[1] {
        extract_multiscale(c.width, c.height, gray).map_err(|e| format!("{e:?}"))?
    } else {
        Vec::new()
    };
    let mut levels =
        vec![json!({"level":0,"width":c.width,"height":c.height,"gray_sha256":sha(gray)})];
    if allow[1] {
        let (mut w, mut h, mut pixels) = (c.width, c.height, gray.to_vec());
        for level in 1..3 {
            if w <= 34 || h <= 34 {
                break;
            }
            (w, h, pixels) = downsample_half(w, h, &pixels).map_err(|e| format!("{e:?}"))?;
            levels.push(json!({"level":level,"width":w,"height":h,"gray_sha256":sha(&pixels)}));
        }
    }
    let witness = json!({"levels":levels,"native_features":a.iter().map(feature_json).collect::<Vec<_>>(),"multiscale_features":b.iter().map(feature_json).collect::<Vec<_>>()});
    Ok(Sensor {
        native: a,
        multi: b,
        depth,
        witness,
    })
}
struct Reference {
    index: usize,
    sensor: Arc<Sensor>,
    root: Pose3,
}
#[derive(Default)]
struct State {
    reference: Option<Reference>,
    accepted_stamp: Option<f64>,
    observed_rgb: Option<f64>,
    lost: bool,
}
impl State {
    fn witness(&self) -> Value {
        json!({"reference_frame_index":self.reference.as_ref().map(|r|r.index),
            "last_accepted_stamp":self.accepted_stamp,"last_observed_rgb_stamp":self.observed_rgb,
            "lost":self.lost,"root_at_reference":self.reference.as_ref().map(|r|pose_json(r.root))})
    }
    // This runs BEFORE this frame's PNG decoding/features/fitting. The startup
    // inventory pass hashes opaque bytes only, including numeric-label bytes.
    fn observe(&mut self, f: &Frame) -> Result<(), String> {
        if self.lost {
            return Err("visual odometry lost; explicit new origin required".into());
        }
        if self
            .accepted_stamp
            .is_some_and(|t| f.depth_timestamp - t > 0.20 + 1e-9)
        {
            self.lost = true;
            return Err("visual accepted-pose age exceeded; localization lost".into());
        }
        if self.observed_rgb.is_some_and(|t| f.rgb_timestamp <= t) {
            return Err("duplicate or stale RGB acquisition; no pose permission renewal".into());
        }
        self.observed_rgb = Some(f.rgb_timestamp);
        Ok(())
    }
    #[allow(clippy::too_many_arguments)] // One atomic state transition and its audit context.
    fn finish(
        &mut self,
        index: usize,
        f: &Frame,
        c: &Calibration,
        current: Option<Arc<Sensor>>,
        multi: bool,
        before: Value,
        clock: Result<(), String>,
        input_error: Option<&str>,
    ) -> Value {
        let ref_before = self.reference.as_ref().map(|r| r.index);
        let mut row = json!({"state_before":before,"clock_accepted":clock.is_ok(),"sensor_attempted":current.is_some(),"features_computed":current.is_some(),
            "accepted":false,"initialized":false,"reference_frame_index_before":ref_before,
            "matches":[],"correspondences":[],"refinement_observations":[],"initialization_depth_features":[]});
        let result: Result<Pose3, String> = (|| {
            clock?;
            if let Some(e) = input_error {
                return Err(e.to_string());
            }
            let sensor = current.as_ref().ok_or("sensor unavailable")?;
            if (f.rgb_timestamp - f.depth_timestamp).abs() > 0.02 {
                return Err("sensor association gap exceeds .02s".into());
            }
            if let Some(reference) = &self.reference {
                row["reference_frame_index"] = json!(reference.index);
                row["reference_stamp"] = json!(self.accepted_stamp);
                row["root_from_reference"] = pose_json(reference.root);
                let (relative, mut measurement) = pair(&reference.sensor, sensor, c, multi, Ok(()));
                let object = measurement.as_object_mut().ok_or("invalid pair witness")?;
                for (key, value) in std::mem::take(object) {
                    row[key] = value;
                }
                let relative = relative.ok_or_else(|| {
                    row["rejection"]
                        .as_str()
                        .unwrap_or("pair fit rejected")
                        .to_string()
                })?;
                Ok(reference.root.compose(relative))
            } else {
                if index != 0 {
                    self.lost = true;
                    return Err("initial origin unavailable; no later initialization".into());
                }
                let features = if multi { &sensor.multi } else { &sensor.native };
                let mut points = Vec::new();
                let mut rows = Vec::new();
                for (feature_index, feature) in features.iter().enumerate() {
                    if points.len() >= 256 {
                        break;
                    }
                    let (point, depth) = depth_point(&sensor.depth, feature, c);
                    let mut witness = json!({"feature_index":feature_index,"accepted":point.is_ok(),"depth":depth});
                    match point {
                        Ok(point) => {
                            witness["point"] = vjson(point);
                            points.push(point);
                        }
                        Err(reason) => {
                            witness["rejection"] = json!(reason);
                        }
                    }
                    rows.push(witness);
                }
                row["initialization_depth_features"] = json!(rows);
                let geometry = validate_geometry(&points, &VisualOdometry3dConfig::default())
                    .inspect_err(|_| self.lost = true)?;
                row["initialization_geometry_ratio"] = json!(geometry);
                row["initialized"] = json!(true);
                Ok(Pose3::identity())
            }
        })();
        match result {
            Ok(root) => {
                row["accepted"] = json!(true);
                row["root_estimate"] = pose_json(root);
                self.reference = Some(Reference {
                    index,
                    sensor: current.expect("accepted sensor was checked"),
                    root,
                });
                self.accepted_stamp = Some(f.depth_timestamp);
            }
            Err(reason) => {
                row["accepted"] = json!(false);
                row["rejection"] = json!(reason);
                if index == 0 {
                    self.lost = true;
                }
                if let Some(object) = row.as_object_mut() {
                    object.remove("root_estimate");
                    object.remove("relative_estimate");
                }
            }
        }
        row["reference_frame_index_after"] = json!(self.reference.as_ref().map(|r| r.index));
        row["state_after"] = self.witness();
        row
    }
}
fn validate_manifest(m: &Manifest) -> Result<BTreeMap<&str, &SourceFile>, String> {
    let c = &m.depth_calibration;
    if m.frames.len() != 180 || c.width != 640 || c.height != 480 {
        return Err("fixed180frame VGA interval required".into());
    }
    for (i, f) in m.frames.iter().enumerate() {
        if f.source_index != 100 + i
            || !f.depth_timestamp.is_finite()
            || !f.rgb_timestamp.is_finite()
            || i > 0
                && (f.depth_timestamp <= m.frames[i - 1].depth_timestamp
                    || f.rgb_timestamp < m.frames[i - 1].rgb_timestamp)
        {
            return Err("invalid source chronology".into());
        }
    }
    let mut inventory = BTreeMap::new();
    for pin in &m.files {
        if !safe(&pin.file) || inventory.insert(pin.file.as_str(), pin).is_some() {
            return Err("invalid inventory".into());
        }
    }
    let mut total = 0usize;
    for f in &m.frames {
        for (name, role) in [(&f.rgb_file, "rgb_frame"), (&f.depth_file, "depth_frame")] {
            let pin = inventory.get(name.as_str()).ok_or("missing sensor pin")?;
            if pin.role != role || pin.bytes > 4 * 1024 * 1024 {
                return Err("sensor pin role or bytes".into());
            }
            total = total.checked_add(pin.bytes).ok_or("sensor size overflow")?;
        }
    }
    if total > 128 * 1024 * 1024 {
        return Err("total sensor byte bound".into());
    }
    Ok(inventory)
}
fn summary(rows: &[Value], mode: &str, state: &State) -> Value {
    let updates = &rows[1..];
    let initialization = rows
        .iter()
        .filter(|r| r[mode]["initialized"] == true)
        .count();
    let accepted = updates
        .iter()
        .filter(|r| r[mode]["accepted"] == true)
        .count();
    let accurate = updates
        .iter()
        .filter(|r| r[mode]["evaluation"]["within_accuracy_gates"] == true)
        .count();
    let scorable = updates
        .iter()
        .filter(|r| r[mode]["evaluation"]["reference_valid"] == true)
        .count();
    json!({"frames":rows.len(),"updates":updates.len(),"initialized_frames":initialization,"accepted_updates":accepted,
        "rejected_updates":updates.len()-accepted,"accurate_root_updates":accurate,"reference_valid_updates":scorable,
        "lost":state.lost,"all_updates_passed":initialization==1&&accepted==updates.len()&&accurate==updates.len()&&scorable==updates.len()})
}
fn root_from_json(row: &Value) -> Option<Pose3> {
    if row["accepted"] != true {
        return None;
    }
    let p = &row["root_estimate"];
    let t = &p["translation_m"];
    let q = &p["quaternion_wxyz"];
    Some(Pose3 {
        translation: Vec3::new(t[0].as_f64()?, t[1].as_f64()?, t[2].as_f64()?),
        rotation: Quaternion {
            w: q[0].as_f64()?,
            x: q[1].as_f64()?,
            y: q[2].as_f64()?,
            z: q[3].as_f64()?,
        },
    })
}
fn verify_inventory(
    raw: &Path,
    m: &Manifest,
    inventory: &BTreeMap<&str, &SourceFile>,
) -> Result<Value, String> {
    let mut total = 0usize;
    for pin in &m.files {
        if pin.bytes > 4 * 1024 * 1024 {
            return Err(format!("raw inventory per-file bound: {}", pin.file));
        }
        total = total
            .checked_add(pin.bytes)
            .ok_or("raw inventory byte overflow")?;
    }
    if total > 128 * 1024 * 1024 {
        return Err("raw inventory total byte bound".into());
    }
    for pin in &m.files {
        // Hash only. Do not decode a PNG, interpret calibration/index text or
        // parse numeric motion-capture labels during source-integrity preflight.
        let _opaque = verified(raw, &pin.file, inventory)
            .map_err(|e| format!("raw inventory verification {}: {e}", pin.file))?;
    }
    Ok(
        json!({"all_files_verified":true,"files_verified":m.files.len(),"bytes_verified":total,
        "no_pixels_decoded":true,"no_numeric_labels_parsed":true}),
    )
}
fn evaluate(bytes: &[u8], raw: &Path, freeze: &Value) -> Result<Value, String> {
    let m: Manifest = serde_json::from_slice(bytes).map_err(|e| e.to_string())?;
    let inventory = validate_manifest(&m)?;
    let inventory_integrity = verify_inventory(raw, &m, &inventory)?;
    let c = &m.depth_calibration;
    let mut native_state = State::default();
    let mut multi_state = State::default();
    let mut rows = Vec::new();
    let mut input_failure: Option<String> = None;
    for (index, f) in m.frames.iter().enumerate() {
        let before = [native_state.witness(), multi_state.witness()];
        let clocks = [native_state.observe(f), multi_state.observe(f)];
        let allow = [clocks[0].is_ok(), clocks[1].is_ok()];
        let mut current = None;
        let mut frame = json!({"index":index,"source_index":f.source_index,
            "rgb_file":f.rgb_file,"depth_file":f.depth_file,"rgb_timestamp":f.rgb_timestamp,"depth_timestamp":f.depth_timestamp,
            "rgb_sha256":inventory[f.rgb_file.as_str()].sha256,"depth_sha256":inventory[f.depth_file.as_str()].sha256,
            "sensor_decoded":false,"frontend":{"levels":[],"native_features":[],"multiscale_features":[]}});
        if input_failure.is_none() && (allow[0] || allow[1]) {
            let read = (|| {
                let rgb = verified(raw, &f.rgb_file, &inventory)?;
                let dep = verified(raw, &f.depth_file, &inventory)?;
                let gray = decode(&rgb, false, c)?;
                let depth = decode(&dep, true, c)?;
                sensor_mask(c, &gray, depth, allow)
            })();
            match read {
                Ok(sensor) => {
                    frame["sensor_decoded"] = json!(true);
                    frame["frontend"] = sensor.witness.clone();
                    current = Some(Arc::new(sensor));
                }
                Err(e) => {
                    input_failure = Some(format!(
                        "sensor input failed at ordinal{index}: {e}; further sensor work aborted"
                    ));
                }
            }
        }
        frame["native"] = native_state.finish(
            index,
            f,
            c,
            if allow[0] { current.clone() } else { None },
            false,
            before[0].clone(),
            clocks[0].clone(),
            input_failure.as_deref(),
        );
        frame["multiscale"] = multi_state.finish(
            index,
            f,
            c,
            if allow[1] { current } else { None },
            true,
            before[1].clone(),
            clocks[1].clone(),
            input_failure.as_deref(),
        );
        rows.push(frame);
    }
    // The complete native AND multiscale sensor histories are now final.
    let labels = (|| {
        let pins: Vec<_> = m
            .files
            .iter()
            .filter(|p| p.role == "evaluation_only_mocap_ground_truth")
            .collect();
        if pins.len() != 1 {
            return Err("expected one evaluation-only label source".into());
        }
        truth(&verified(raw, &pins[0].file, &inventory)?)
    })();
    for (index, row) in rows.iter_mut().enumerate() {
        let reference = match &labels {
            Ok(gt) => interpolate(gt, m.frames[0].depth_timestamp).and_then(|origin| {
                interpolate(gt, m.frames[index].depth_timestamp)
                    .map(|current| origin.inverse().compose(current))
            }),
            Err(e) => Err(e.clone()),
        };
        for mode in ["native", "multiscale"] {
            row[mode]["evaluation"] = score(root_from_json(&row[mode]), reference.clone());
        }
    }
    Ok(
        json!({"schema_version":1,"algorithm":"bounded_multiscale_continuous_viewed_comparison","dataset":m.dataset,
        "freeze":freeze,"manifest_sha256":sha(bytes),"ground_truth_operational":false,"raw_redistributed":false,
        "frames":rows,"inventory_integrity":inventory_integrity,"summary":{"native":summary(&rows,"native",&native_state),"multiscale":summary(&rows,"multiscale",&multi_state)},
        "evaluation_label_failure":labels.as_ref().err(),"input_failure":input_failure,
        "limits":["Viewed indoor regression only, not fresh holdout or vehicle generalization.","Continuous root accumulation uses one initial origin and last accepted measured reference; loss cannot reset.","Unchanged gates can reject or accumulate error; no calibrated covariance or RNE pose permission integration."]}),
    )
}

fn control_camera() -> Calibration {
    Calibration {
        width: 320,
        height: 240,
        fx: 240.,
        fy: 240.,
        cx: 159.5,
        cy: 119.5,
        units_per_metre: 5000.,
        invalid_depth: 0,
    }
}
fn rendered(c: &Calibration, shift: [usize; 2], raw_depth: u16) -> (Vec<u8>, Vec<u8>) {
    let base = texture(c.width, c.height);
    let mut pixels = vec![0; c.width * c.height];
    for y in shift[1]..c.height {
        for x in shift[0]..c.width {
            pixels[y * c.width + x] = base[(y - shift[1]) * c.width + x - shift[0]];
        }
    }
    (
        pixels,
        (0..c.width * c.height)
            .flat_map(|_| raw_depth.to_be_bytes())
            .collect(),
    )
}
fn temporal_control(kind: &str) -> Result<Value, String> {
    let c = control_camera();
    let specifications: Vec<(f64, f64, [usize; 2], u16, bool)> = match kind {
        "healthy" => vec![
            (0., 0., [0, 0], 7500, false),
            (0.04, 0.04, [8, 4], 7500, false),
            (0.08, 0.08, [16, 8], 7500, false),
            (0.12, 0.12, [24, 12], 7500, false),
        ],
        "depth_failure_duplicate_expiry_latch" => vec![
            (0., 0., [0, 0], 7500, false),
            (0.04, 0.04, [8, 4], 7500, false),
            (0.06, 0.06, [16, 8], 7500, false),
            (0.08, 0.08, [24, 12], 0, false),
            (0.10, 0.08, [24, 12], 7500, false),
            (0.261, 0.261, [24, 12], 7500, false),
            (0.28, 0.28, [32, 16], 7500, false),
        ],
        "refinement_failure_no_fallback" => vec![
            (0., 0., [0, 0], 7500, false),
            // Deliberate depth bias: healthy texture motion would require more
            // than0.5m translation. The bounded coarse fit accepts a biased
            // model, but the unchanged pixel refinement fails its line search.
            // Chosen from a SYNTHETIC development sweep, never recorded scenes.
            (0.04, 0.04, [81, 0], 7000, false),
            (0.06, 0.06, [8, 4], 7500, false),
        ],
        "initialization_failure_latch" => vec![
            (0., 0., [0, 0], 7500, true),
            (0.04, 0.04, [8, 4], 7500, false),
        ],
        _ => return Err("unknown synthetic temporal control".into()),
    };
    let mut states = [State::default(), State::default()];
    let mut rows = Vec::new();
    for (index, &(depth_stamp, rgb_stamp, shift, raw_depth, blank)) in
        specifications.iter().enumerate()
    {
        let f = Frame {
            source_index: index,
            depth_file: format!("synthetic-depth-{index}"),
            depth_timestamp: depth_stamp,
            rgb_file: format!("synthetic-rgb-{index}"),
            rgb_timestamp: rgb_stamp,
        };
        let before = [states[0].witness(), states[1].witness()];
        let clocks = [states[0].observe(&f), states[1].observe(&f)];
        let allow = [clocks[0].is_ok(), clocks[1].is_ok()];
        let mut current = None;
        let mut frame = json!({"index":index,"source_index":index,"depth_timestamp":depth_stamp,"rgb_timestamp":rgb_stamp,
            "render":{"shift_pixels":shift,"raw_depth":raw_depth,"blank":blank},"sensor_decoded":false,
            "frontend":{"levels":[],"native_features":[],"multiscale_features":[]}});
        if allow[0] || allow[1] {
            let (mut gray, depth) = rendered(&c, shift, raw_depth);
            if blank {
                gray.fill(127);
            }
            let sensor = sensor_mask(&c, &gray, depth, allow)?;
            frame["sensor_decoded"] = json!(true);
            frame["frontend"] = sensor.witness.clone();
            current = Some(Arc::new(sensor));
        }
        for (k, name) in ["native", "multiscale"].into_iter().enumerate() {
            frame[name] = states[k].finish(
                index,
                &f,
                &c,
                if allow[k] { current.clone() } else { None },
                k == 1,
                before[k].clone(),
                clocks[k].clone(),
                None,
            );
        }
        rows.push(frame);
    }
    // Synthetic evaluation-only expected roots are derived AFTER all fits.
    for (index, row) in rows.iter_mut().enumerate() {
        let shift = specifications[index].2;
        let expected = Pose3 {
            translation: Vec3::new(
                -(shift[0] as f64) * 1.5 / c.fx,
                -(shift[1] as f64) * 1.5 / c.fy,
                0.,
            ),
            rotation: Quaternion::identity(),
        };
        for mode in ["native", "multiscale"] {
            row[mode]["evaluation"] = score(root_from_json(&row[mode]), Ok(expected));
        }
    }
    let expected = if kind == "healthy" {
        vec![true, true, true, true]
    } else if kind == "initialization_failure_latch" {
        vec![false, false]
    } else if kind == "refinement_failure_no_fallback" {
        vec![true, false, true]
    } else {
        vec![true, true, true, false, false, false, false]
    };
    let pass = rows.iter().enumerate().all(|(i, row)| {
        ["native", "multiscale"].into_iter().all(|m| {
            row[m]["accepted"] == expected[i]
                && (row[m]["accepted"] != true
                    || row[m]["evaluation"]["within_accuracy_gates"] == true)
        })
    });
    Ok(
        json!({"kind":kind,"frames":rows,"summary":{"native":summary(&rows,"native",&states[0]),"multiscale":summary(&rows,"multiscale",&states[1])},
        "expected_accepted":expected,"expected_behavior_passed":pass}),
    )
}
fn control() -> Result<Value, String> {
    let camera = control_camera();
    let cases = [
        "healthy",
        "depth_failure_duplicate_expiry_latch",
        "initialization_failure_latch",
        "refinement_failure_no_fallback",
    ]
    .into_iter()
    .map(temporal_control)
    .collect::<Result<Vec<_>, _>>()?;
    Ok(
        json!({"schema_version":1,"kind":"analytic_continuous_measured_controls","sources":sources(),"design_sha256":DESIGN_SHA,
        "calibration":{"width":camera.width,"height":camera.height,"fx":camera.fx,"fy":camera.fy,"cx":camera.cx,"cy":camera.cy,"units_per_metre":camera.units_per_metre,"invalid_depth":camera.invalid_depth},
        "texture":"aperiodic checker: base40/170 plus (x*97+y*193+x*y*17)%61","cases":cases}),
    )
}
fn fresh(path: &Path) -> Result<fs::File, String> {
    if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| format!("fresh output admission: {e}"))
}
fn run() -> Result<i32, String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let mut options = BTreeMap::new();
    let mut mode = "evaluate";
    let mut modes = 0;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            flag @ ("--control" | "--prepare-freeze" | "--executable-receipt") => {
                modes += 1;
                mode = match flag {
                    "--control" => "control",
                    "--prepare-freeze" => "freeze",
                    _ => "receipt",
                };
                i += 1;
            }
            key @ ("--manifest" | "--raw" | "--freeze" | "--output") => {
                let value = args.get(i + 1).ok_or("missing argument")?;
                if options.insert(key, value.as_str()).is_some() {
                    return Err("duplicate argument".into());
                }
                i += 2;
            }
            _ => return Err("unknown argument".into()),
        }
    }
    if modes > 1 {
        return Err("mutually exclusive modes".into());
    }
    let output = Path::new(*options.get("--output").ok_or("--output required")?);
    let mut file = fresh(output)?;
    let report = match mode {
        "control" => control()?,
        "receipt" => {
            let executable = std::env::current_exe().map_err(|e| e.to_string())?;
            json!({"schema_version":1,"executable_sha256":sha(&bounded(&executable,20*1024*1024)?),"sources":sources(),"scope":"Local executable receipt; excluded from deterministic source/protocol freeze and physical report."})
        }
        _ => {
            let bytes = bounded(
                Path::new(*options.get("--manifest").ok_or("--manifest required")?),
                512 * 1024,
            )?;
            let expected = protocol(&bytes)?;
            if mode == "freeze" {
                expected
            } else {
                let frozen: Value = serde_json::from_slice(&bounded(
                    Path::new(*options.get("--freeze").ok_or("--freeze required")?),
                    512 * 1024,
                )?)
                .map_err(|e| e.to_string())?;
                if frozen != expected {
                    return Err("source/design/manifest/protocol freeze mismatch".into());
                }
                evaluate(
                    &bytes,
                    Path::new(*options.get("--raw").ok_or("--raw required")?),
                    &frozen,
                )?
            }
        }
    };
    let bytes = serde_json::to_vec(&report).map_err(|e| e.to_string())?;
    if bytes.len() > 64 * 1024 * 1024 {
        return Err("report byte bound exceeded".into());
    }
    file.write_all(&bytes).map_err(|e| e.to_string())?;
    if report["input_failure"].is_string() || report["evaluation_label_failure"].is_string() {
        return Ok(2);
    }
    if mode == "evaluate" {
        println!("{}", report["summary"]);
        Ok(
            if report["summary"]["native"]["all_updates_passed"] == true
                && report["summary"]["multiscale"]["all_updates_passed"] == true
            {
                0
            } else {
                1
            },
        )
    } else if mode == "control" {
        Ok(
            if report["cases"]
                .as_array()
                .ok_or("missing control cases")?
                .iter()
                .all(|c| c["expected_behavior_passed"] == true)
            {
                0
            } else {
                2
            },
        )
    } else {
        Ok(0)
    }
}
pub fn main_entry() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("continuous multiscale diagnostic: {e}");
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn actual_healthy_sensor_sequence_composes_one_origin() {
        let c = temporal_control("healthy").unwrap();
        assert_eq!(c["expected_behavior_passed"], true);
        for mode in ["native", "multiscale"] {
            assert_eq!(c["summary"][mode]["initialized_frames"], 1);
            assert_eq!(c["summary"][mode]["all_updates_passed"], true);
        }
    }
    #[test]
    fn actual_depth_rejection_duplicate_and_expiry_never_renew() {
        let c = temporal_control("depth_failure_duplicate_expiry_latch").unwrap();
        assert_eq!(c["expected_behavior_passed"], true);
        for mode in ["native", "multiscale"] {
            let rows = c["frames"].as_array().unwrap();
            let expected = &rows[2][mode]["state_after"];
            for row in &rows[3..] {
                assert_eq!(
                    row[mode]["state_after"]["reference_frame_index"],
                    expected["reference_frame_index"]
                );
                assert_eq!(
                    row[mode]["state_after"]["last_accepted_stamp"],
                    expected["last_accepted_stamp"]
                );
                assert_eq!(
                    row[mode]["state_after"]["root_at_reference"],
                    expected["root_at_reference"]
                );
                assert!(row[mode].get("root_estimate").is_none());
            }
            assert_eq!(
                rows[3][mode]["state_after"]["last_observed_rgb_stamp"],
                0.08
            );
            assert_eq!(rows[4][mode]["sensor_attempted"], false);
            assert_eq!(rows[5][mode]["state_after"]["lost"], true);
            assert_eq!(rows[6][mode]["state_after"]["lost"], true);
            assert_eq!(rows[6]["sensor_decoded"], false);
        }
    }
    #[test]
    fn actual_empty_geometry_latches_initialization_failure() {
        let c = temporal_control("initialization_failure_latch").unwrap();
        assert_eq!(c["expected_behavior_passed"], true);
        for mode in ["native", "multiscale"] {
            assert_eq!(c["summary"][mode]["initialized_frames"], 0);
            assert_eq!(c["summary"][mode]["lost"], true);
            assert_eq!(c["frames"][1]["sensor_decoded"], false);
        }
    }
    #[test]
    fn actual_refinement_failure_never_falls_back_to_coarse_pose() {
        let control = temporal_control("refinement_failure_no_fallback").unwrap();
        assert_eq!(control["expected_behavior_passed"], true);
        for mode in ["native", "multiscale"] {
            let failed = &control["frames"][1][mode];
            assert!(failed.get("coarse_pose").is_some());
            assert_eq!(failed["rejection"], "reprojection line search failed");
            assert!(failed.get("relative_estimate").is_none());
            assert!(failed.get("root_estimate").is_none());
            assert_eq!(failed["state_after"]["reference_frame_index"], 0);
            assert_eq!(failed["state_after"]["last_accepted_stamp"], 0.);
            assert_eq!(failed["state_after"]["last_observed_rgb_stamp"], 0.04);
            assert_eq!(
                control["frames"][2][mode]["reference_frame_index_before"],
                0
            );
            assert_eq!(control["frames"][2][mode]["accepted"], true);
        }
    }
}
