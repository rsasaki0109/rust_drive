//! Additive recorded RGB-D pixel-reprojection refinement. Feature associations and
//! rigid fits finish before any evaluation-only motion-capture labels are parsed.
use super::*;
use rustdriving_localization::reprojection3d::{
    CameraIntrinsics, ReprojectionConfig3d, ReprojectionObservation3d, refine_reprojection,
};
use rustdriving_localization::visual_odometry3d::{
    Correspondence3d, VisualOdometry3dConfig, register_correspondences, validate_geometry,
};
use rustdriving_perception::image_features::{
    GrayImage, ImageFeature, extract_features, match_features,
};

const MAX_GT_ROWS: usize = 30_000;
const MAX_GT_BYTES: usize = 4 * 1024 * 1024;
const MAX_AGE_S: f64 = 0.20;
const PAIR_GAP_S: f64 = 0.02;
const DEPTH_PATCH_SPREAD_M: f64 = 0.05;
type MeasuredReference = (usize, Vec<ImageFeature>, Vec<u8>, Pose3);
#[derive(Default)]
struct AcceptedState {
    reference: Option<MeasuredReference>,
    stamp: Option<f64>,
}
impl AcceptedState {
    fn finish(
        &mut self,
        result: Result<Pose3, String>,
        row: &mut Value,
        index: usize,
        stamp: f64,
        features: Vec<ImageFeature>,
        depth: Vec<u8>,
    ) {
        match result {
            Ok(root) => {
                row["accepted"] = json!(true);
                row["root_estimate"] = pose_json(root);
                self.reference = Some((index, features, depth, root));
                self.stamp = Some(stamp);
            }
            Err(error) => {
                row["accepted"] = json!(false);
                row["rejection"] = json!(error);
                // A failed refinement cannot authorize its coarse pose, even
                // if a caller accidentally populated an output field early.
                if let Some(object) = row.as_object_mut() {
                    object.remove("root_estimate");
                    object.remove("relative_estimate");
                }
            }
        }
    }
}
// This gate runs before pixel decoding. Observing an image is distinct from
// accepting its pose: a failed fit must not let the same acquisition retry.
fn observe_clock(
    depth_stamp: f64,
    rgb_stamp: f64,
    last_accepted: Option<f64>,
    last_observed_rgb: &mut Option<f64>,
    lost: &mut bool,
) -> Result<(), String> {
    if *lost {
        return Err("visual odometry lost; explicit new origin required".into());
    }
    if last_accepted.is_some_and(|t| depth_stamp - t > MAX_AGE_S + 1e-9) {
        *lost = true;
        return Err("visual accepted-pose age exceeded; localization lost".into());
    }
    if last_observed_rgb.is_some_and(|t| rgb_stamp <= t) {
        return Err("duplicate or stale RGB acquisition; no pose permission renewal".into());
    }
    *last_observed_rgb = Some(rgb_stamp);
    Ok(())
}
#[derive(Deserialize)]
struct VisualFrame {
    source_index: usize,
    depth_file: String,
    depth_timestamp: f64,
    rgb_file: String,
    rgb_timestamp: f64,
    rgb_source_index: usize,
    pair_gap_seconds: f64,
    split: String,
}
#[derive(Deserialize)]
struct VisualManifest {
    schema_version: u32,
    dataset: String,
    repository: String,
    revision: String,
    depth_calibration: Calibration,
    calibration_source: Value,
    frames: Vec<VisualFrame>,
    files: Vec<SourceFile>,
}
fn vjson(p: Vec3) -> Value {
    json!([p.x, p.y, p.z])
}
fn configuration() -> Value {
    let c = VisualOdometry3dConfig::default();
    json!({"max_matches":c.max_matches,"max_hypotheses":c.max_hypotheses,"max_refits":c.max_refits,"max_point_checks":c.max_point_checks,"inlier_distance_m":c.inlier_distance_m,"min_inliers":c.min_inliers,"min_inlier_ratio":c.min_inlier_ratio,"min_geometry_ratio":c.min_geometry_ratio,"max_translation_m":c.max_translation_m,"max_rotation_rad":c.max_rotation_rad,"ambiguity_support_ratio":c.ambiguity_support_ratio,"ambiguity_rms_ratio":c.ambiguity_rms_ratio,"ambiguity_translation_m":c.ambiguity_translation_m,"ambiguity_rotation_rad":c.ambiguity_rotation_rad,"ambiguity_noise_floor_m":c.ambiguity_noise_floor_m})
}
fn refinement_configuration() -> Value {
    let c = ReprojectionConfig3d::default();
    json!({"max_observations":c.max_observations,"max_iterations":c.max_iterations,"max_line_search_steps":c.max_line_search_steps,"max_point_checks":c.max_point_checks,"min_observations":c.min_observations,"huber_delta_px":c.huber_delta_px,"min_depth_m":c.min_depth_m,"max_depth_m":c.max_depth_m,"max_condition_number":c.max_condition_number,"max_translation_m":c.max_translation_m,"max_rotation_rad":c.max_rotation_rad,"translation_tolerance_m":c.translation_tolerance_m,"rotation_tolerance_rad":c.rotation_tolerance_rad})
}
fn refinement_observations(
    pairs: &[Correspondence3d],
    depth_rows: &[Value],
    features: &[ImageFeature],
    inlier_indices: &[usize],
) -> Result<(Vec<ReprojectionObservation3d>, Vec<Value>), String> {
    let mut observations = Vec::new();
    let mut reported = Vec::new();
    let mut seen = BTreeSet::new();
    for &index in inlier_indices {
        if !seen.insert(index) {
            return Err("duplicate coarse inlier correspondence".into());
        }
        let pair = pairs
            .get(index)
            .ok_or("coarse inlier index out of bounds")?;
        let matched = depth_rows
            .iter()
            .find(|row| row["accepted"] == true && row["correspondence_index"] == index)
            .ok_or("coarse inlier has no measured feature association")?;
        let feature_index = matched["current_index"]
            .as_u64()
            .ok_or("coarse inlier current feature index missing")?
            as usize;
        let feature = features
            .get(feature_index)
            .ok_or("coarse inlier current feature index out of bounds")?;
        let pixel = [feature.x, feature.y];
        observations.push(ReprojectionObservation3d {
            previous: pair.previous,
            current_pixel: pixel,
        });
        reported.push(json!({"correspondence_index":index,"previous_index":matched["previous_index"],"current_index":feature_index,"previous_xyz":vjson(pair.previous),"current_pixel_xy":pixel}));
    }
    Ok((observations, reported))
}
fn validate(m: &VisualManifest) -> Result<(), String> {
    let c = &m.depth_calibration;
    let camera = match m.dataset.as_str() {
        "tum-fr1-desk-visual" => (
            517.306408,
            516.469215,
            318.643040,
            255.313989,
            "settings/TUM1.yaml",
            1615,
            "5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf",
            5000.,
        ),
        "tum-fr3-office-visual" | "tum-fr3-sitting-visual" => (
            535.4,
            539.2,
            320.1,
            247.6,
            "settings/TUM3.yaml",
            1520,
            "251e345996befa8057f5c51642bd4fe93a92907dac2e0564f400cd87eaf5785d",
            5000.,
        ),
        "tum-fr2-desk-reprojection" => (
            520.90862,
            521.007327,
            325.141442,
            249.701764,
            "settings/TUM2.yaml",
            1584,
            "8ec3a8c3b64495e8a308a40e0b75eb77300119c5502184ad77628324e6bae0f0",
            5208.,
        ),
        _ => return Err("unsupported visual dataset selection".into()),
    };
    let (repository, revision) = match m.dataset.as_str() {
        "tum-fr1-desk-visual" => (
            "FaridRash/slam-track-fusion",
            "477a059d640540b7e23fd56ec95f6458167c7af2",
        ),
        "tum-fr3-office-visual" => (
            "shihaozhaosiue/SLAM-project_shihao",
            "1f3bb58bbcbad2ec405c36d6d5a511d2f1cd050b",
        ),
        "tum-fr3-sitting-visual" => (
            "yakki12345/DygeoSLAM",
            "eae444878fc663fbd41307b1e655e762f988c714",
        ),
        "tum-fr2-desk-reprojection" => (
            "GKoutilya/real-time-predictive-maintenance-dashboard",
            "2c1304c46bc3d47e9056a01127382c95240a902a",
        ),
        _ => return Err("unsupported paired source identity".into()),
    };
    let start = 100;
    if m.schema_version != 1
        || m.repository != repository
        || m.revision != revision
        || m.revision.len() != 40
        || !m.revision.bytes().all(|b| b.is_ascii_hexdigit())
        || c.width != WIDTH
        || c.height != HEIGHT
        || (c.fx, c.fy, c.cx, c.cy) != (camera.0, camera.1, camera.2, camera.3)
        || c.units_per_metre != camera.7
        || c.invalid_depth != 0
        || m.frames.len() != 36
        || m.calibration_source
            != json!({"repository":"luigifreda/pyslam","revision":"96019cfafcfc099ac9866884d7143a9ed1451a0d","source_path":camera.4,"file":"camera-calibration.yaml","bytes":camera.5,"sha256":camera.6,"role":"source_calibration_documentation_only"})
        || m.frames.iter().enumerate().any(|(i, f)| {
            f.source_index != start + i
                || !safe_filename(&f.depth_file)
                || !safe_filename(&f.rgb_file)
                || !f.depth_timestamp.is_finite()
                || !f.rgb_timestamp.is_finite()
                || !f.pair_gap_seconds.is_finite()
                || (f.rgb_timestamp - f.depth_timestamp).abs() > PAIR_GAP_S
                || ((f.rgb_timestamp - f.depth_timestamp).abs() - f.pair_gap_seconds).abs() > 1e-9
                || f.depth_file
                    .strip_prefix("depth-")
                    .and_then(|s| s.strip_suffix(".png"))
                    .and_then(|s| s.parse::<f64>().ok())
                    != Some(f.depth_timestamp)
                || f.rgb_file
                    .strip_prefix("rgb-")
                    .and_then(|s| s.strip_suffix(".png"))
                    .and_then(|s| s.parse::<f64>().ok())
                    != Some(f.rgb_timestamp)
                || f.split
                    != if i == 0 {
                        "initialization"
                    } else if matches!(
                        m.dataset.as_str(),
                        "tum-fr3-sitting-visual" | "tum-fr2-desk-reprojection"
                    ) {
                        "held_out"
                    } else {
                        "viewed_development"
                    }
        })
        || m.frames.windows(2).any(|f| {
            f[1].depth_timestamp <= f[0].depth_timestamp
                || f[1].rgb_timestamp < f[0].rgb_timestamp
                || f[1].rgb_source_index < f[0].rgb_source_index
        })
    {
        return Err("visual selection, calibration or RGB/depth clock association invalid".into());
    }
    let mut seen = BTreeSet::new();
    let mut total = 0usize;
    for f in &m.files {
        total = total
            .checked_add(f.bytes)
            .ok_or("visual input byte total overflow")?;
        if !safe_filename(&f.file)
            || !seen.insert(&f.file)
            || f.bytes == 0
            || f.bytes > 4 * 1024 * 1024
            || f.sha256.len() != 64
            || !f
                .sha256
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
        {
            return Err("invalid visual source inventory".into());
        }
    }
    let expected_files: BTreeSet<String> = m
        .frames
        .iter()
        .flat_map(|f| [f.depth_file.clone(), f.rgb_file.clone()])
        .chain([
            "depth.txt".into(),
            "rgb.txt".into(),
            "groundtruth.txt".into(),
        ])
        .collect();
    if seen.into_iter().cloned().collect::<BTreeSet<_>>() != expected_files {
        return Err("visual source inventory omits or adds raw input".into());
    }
    if total > 32 * 1024 * 1024
        || m.frames.iter().any(|f| {
            !m.files
                .iter()
                .any(|s| s.file == f.depth_file && s.role == "depth_frame")
                || !m
                    .files
                    .iter()
                    .any(|s| s.file == f.rgb_file && s.role == "rgb_frame")
        })
    {
        return Err("visual source bound or measured frame inventory invalid".into());
    }
    for (file, role) in [
        ("depth.txt", "depth_index"),
        ("rgb.txt", "rgb_index"),
        ("groundtruth.txt", "evaluation_only_mocap_ground_truth"),
    ] {
        if m.files
            .iter()
            .filter(|s| s.file == file && s.role == role)
            .count()
            != 1
        {
            return Err("visual source metadata inventory invalid".into());
        }
    }
    Ok(())
}
fn protocol(path: &Path, regression: bool) -> Result<Value, String> {
    let bytes = bounded_read(path, 256 * 1024)?;
    let metadata: Value = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    let m: VisualManifest = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    validate(&m)?;
    if !regression {
        return Err(
            "visual reprojection requires --regression for every previously viewed recording"
                .into(),
        );
    }
    if metadata["redistribute_raw"] != false {
        return Err("visual raw redistribution is not authorized".into());
    }
    let mut value = json!({"schema_version":1,"protocol_version":3,"algorithm":"bounded_visual_reprojection_refinement","dataset":m.dataset,"manifest_sha256":sha(&bytes),"kind":"calibration_regression","regression_requested":regression,
 "evaluation_label_policy":{"invalid_source":"retain sensor rows, mark all references and accuracy invalid, report evaluation_label_failure and exit2; never normalize, deduplicate or subset invalid labels"},
 "json_metadata_audit":{"field":"pair_gap_seconds","absolute_tolerance_s":1e-15,"scope":"source-manifest versus parsed freeze/report derived gap only; source SHA, acquisition timestamps and indices remain exact; no sensor association or fit gate change"},
 "depth_calibration":metadata["depth_calibration"],"calibration_source":m.calibration_source,
 "preprocessing":{"width":WIDTH,"height":HEIGHT,"luma":"(77*R+150*G+29*B)>>8; RGB/RGBA8; RGBA must be fully opaque","pixel_coordinates":"nearest integer feature coordinate, ties round away from zero","min_depth_m":MIN_DEPTH_M,"max_depth_m":MAX_DEPTH_M,"depth_units_per_metre":m.depth_calibration.units_per_metre,"patch_radius_pixels":1,"patch_validity":"all 9 depths valid and range-bounded","patch_max_spread_m":DEPTH_PATCH_SPREAD_M,"point":"centre depth at feature pixel; optical x-right y-down z-forward","maximum_pair_gap_s":PAIR_GAP_S},
 "feature_policy":{"max_features":400,"max_matches":256,"detector":"fixed original FAST-9 threshold20 radius3, deterministic NMS,32pixel tile max2","descriptor":"intensity-centroid oriented deterministic256bit BRIEF on5x5binomial blur","matching":"both directional strict 5*best<4*second, maximumHamming64, mutual nearest; ties rejected"},
 "registration_config":configuration(),"refinement_config":refinement_configuration(),"refinement_policy":{"observations":"only original robust3D inlier correspondence indices; previous measured depth point and matched current RGB feature pixel; fixed support throughout refinement","initial_pose":"original previous_from_current robust3D fit, never motion-capture labels","pose":"refined previous_from_current pose is composed into accepted root; coarse fit and pose are separate diagnostics","failure":"reject without current root output or accepted reference/clock renewal; no coarse-pose fallback","method":"bounded Huber pixel reprojection, maximum8 iterations; convergence and work reported explicitly"},"tracking_policy":{"max_unobserved_s":MAX_AGE_S,"reference":"last accepted measured RGB features and depth image; initial root identity after measured geometry validation","pose":"reference-root pose composed with measured refined previous_from_current pixel-reprojection pose","rejection":"no root output or accepted clock/reference update; repeated/stale RGB timestamp versus last observed image rejects, including previously rejected fits; observed image clock advances without permission renewal; accepted-pose expiry checked before RGB freshness; evaluation never resets"},
 "accuracy_gates":{"translation_m":TRANSLATION_GATE_M,"rotation_rad":ROTATION_GATE_RAD},"ground_truth_interpolation":{"method":"linear translation and shortest-arc quaternion SLERP","max_bracket_s":GT_MAX_BRACKET_S,"extrapolation":false,"max_source_rows":MAX_GT_ROWS,"max_source_bytes":MAX_GT_BYTES},"frames":metadata["frames"],
 "freeze_preparation":"metadata only; RGB/depth/mocap raw bytes are not read, features not extracted and registration not run",
 "evaluator_source_sha256":sha(include_bytes!("main.rs")),"visual_source_sha256":sha(include_bytes!("visual.rs")),"reprojection_source_sha256":sha(include_bytes!("reprojection.rs")),"reprojection_pose_source_sha256":sha(include_bytes!("../../../crates/localization/src/reprojection3d.rs")),"feature_source_sha256":sha(include_bytes!("../../../crates/perception/src/image_features.rs")),"visual_pose_source_sha256":sha(include_bytes!("../../../crates/localization/src/visual_odometry3d.rs")),"pose_source_sha256":sha(include_bytes!("../../../crates/localization/src/registration3d.rs")),"localization_lib_source_sha256":sha(include_bytes!("../../../crates/localization/src/lib.rs")),"perception_lib_source_sha256":sha(include_bytes!("../../../crates/perception/src/lib.rs")),"core_lib_source_sha256":sha(include_bytes!("../../../crates/core/src/lib.rs")),"cargo_lock_sha256":sha(include_bytes!("../Cargo.lock")),"cargo_manifest_sha256":sha(include_bytes!("../Cargo.toml")),"rust_toolchain_sha256":sha(include_bytes!("../../../rust-toolchain.toml")),"independent_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-reprojection.py")),"visual_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-visual.py")),"geometry_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-rgbd.py")),"keyframe_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-keyframes.py")),"checker_requirements_sha256":sha(include_bytes!("../../../scripts/requirements-visual.txt")),"acquisition_source_sha256":sha(include_bytes!("../../../scripts/fetch-visual-datasets.py")),"reprojection_acquisition_source_sha256":sha(include_bytes!("../../../scripts/fetch-reprojection-dataset.py"))});
    for field in [
        "depth_calibration",
        "calibration_source",
        "evaluation_label_policy",
        "preprocessing",
        "feature_policy",
        "registration_config",
        "refinement_config",
        "refinement_policy",
        "tracking_policy",
    ] {
        value[format!("{field}_sha256")] = json!(sha(
            &serde_json::to_vec(&value[field]).map_err(|e| e.to_string())?
        ));
    }
    Ok(value)
}
// Evaluation-only parser: the original numeric and timestamp checks are retained,
// with explicitly frozen source bounds suitable for full recorded GT metadata.
// This is invoked only after every operational sensor fit has completed.
fn evaluation_ground_truth(bytes: &[u8]) -> Result<Vec<GroundTruth>, String> {
    if bytes.len() > MAX_GT_BYTES {
        return Err("ground truth exceeds byte limit".into());
    }
    let text = std::str::from_utf8(bytes).map_err(|e| e.to_string())?;
    let mut result = Vec::new();
    for (line_no, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let values: Vec<f64> = line
            .split_whitespace()
            .map(|v| {
                v.parse::<f64>()
                    .map_err(|_| format!("invalid GT number on line {}", line_no + 1))
            })
            .collect::<Result<_, _>>()?;
        if values.len() != 8 || values.iter().any(|v| !v.is_finite()) {
            return Err(format!("invalid GT row on line {}", line_no + 1));
        }
        let q = Quaternion {
            w: values[7],
            x: values[4],
            y: values[5],
            z: values[6],
        };
        let qnorm = q.w.hypot(q.x).hypot(q.y).hypot(q.z);
        if (qnorm - 1.0).abs() > 0.001
            || result
                .last()
                .is_some_and(|last: &GroundTruth| values[0] <= last.timestamp)
        {
            return Err("invalid GT quaternion or nonmonotonic timestamps".into());
        }
        result.push(GroundTruth {
            timestamp: values[0],
            pose: Pose3 {
                translation: Vec3::new(values[1], values[2], values[3]),
                rotation: q.normalized()?,
            },
        });
        if result.len() > MAX_GT_ROWS {
            return Err("too many GT poses".into());
        }
    }
    if result.len() < 2 {
        return Err("insufficient ground truth".into());
    }
    Ok(result)
}
fn index_rows(bytes: &[u8]) -> Result<Vec<(f64, String)>, String> {
    std::str::from_utf8(bytes)
        .map_err(|e| e.to_string())?
        .lines()
        .map(str::trim)
        .filter(|s| !s.is_empty() && !s.starts_with('#'))
        .map(|s| {
            let fields: Vec<_> = s.split_whitespace().collect();
            if fields.len() != 2 {
                return Err("invalid visual source index".into());
            }
            let timestamp = fields[0].parse::<f64>().map_err(|e| e.to_string())?;
            if !timestamp.is_finite() {
                return Err("invalid visual source clock".into());
            }
            Ok((timestamp, fields[1].to_string()))
        })
        .collect()
}
fn decode(bytes: &[u8], depth: bool) -> Result<Vec<u8>, String> {
    if bytes.len() > 4 * 1024 * 1024 {
        return Err("visual PNG byte limit exceeded".into());
    }
    let mut decoder = png::Decoder::new(Cursor::new(bytes));
    decoder.set_limits(png::Limits {
        bytes: 8 * 1024 * 1024,
    });
    decoder.ignore_checksums(false);
    decoder.set_transformations(png::Transformations::IDENTITY);
    let mut reader = decoder
        .read_info()
        .map_err(|e| format!("visual PNG header: {e}"))?;
    let info = reader.info();
    let valid = if depth {
        info.color_type == png::ColorType::Grayscale && info.bit_depth == png::BitDepth::Sixteen
    } else {
        matches!(info.color_type, png::ColorType::Rgb | png::ColorType::Rgba)
            && info.bit_depth == png::BitDepth::Eight
    };
    if info.width != WIDTH || info.height != HEIGHT || info.animation_control.is_some() || !valid {
        return Err("expected static640x480 depth16 or RGB/RGBA8 PNG".into());
    }
    let channels = if depth {
        2
    } else if info.color_type == png::ColorType::Rgb {
        3
    } else {
        4
    };
    let expected = (WIDTH * HEIGHT) as usize * channels;
    let length = reader
        .output_buffer_size()
        .ok_or("visual PNG buffer overflow")?;
    if length != expected {
        return Err("invalid visual PNG layout".into());
    }
    let mut out = vec![0; length];
    let output = reader
        .next_frame(&mut out)
        .map_err(|e| format!("visual PNG body: {e}"))?;
    if output.buffer_size() != expected {
        return Err("incomplete visual PNG".into());
    }
    reader
        .finish()
        .map_err(|e| format!("visual PNG ending/CRC: {e}"))?;
    if depth {
        return Ok(out);
    }
    let mut gray = Vec::with_capacity((WIDTH * HEIGHT) as usize);
    for p in out.chunks_exact(channels) {
        if channels == 4 && p[3] != 255 {
            return Err("visual RGBA must be opaque".into());
        }
        gray.push(((77 * p[0] as u32 + 150 * p[1] as u32 + 29 * p[2] as u32) >> 8) as u8);
    }
    Ok(gray)
}
fn depth_point(depth: &[u8], feature: &ImageFeature, c: &Calibration) -> Result<Vec3, String> {
    let x = feature.x.round() as usize;
    let y = feature.y.round() as usize;
    if x < 1 || y < 1 || x + 1 >= WIDTH as usize || y + 1 >= HEIGHT as usize {
        return Err("depth patch exceeds image".into());
    }
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    let mut centre = 0.;
    for py in y - 1..=y + 1 {
        for px in x - 1..=x + 1 {
            let i = 2 * (py * WIDTH as usize + px);
            let raw = u16::from_be_bytes([depth[i], depth[i + 1]]);
            let z = raw as f64 / c.units_per_metre;
            if raw == c.invalid_depth || !(MIN_DEPTH_M..=MAX_DEPTH_M).contains(&z) {
                return Err("invalid or range-limited measured depth patch".into());
            }
            minimum = minimum.min(z);
            maximum = maximum.max(z);
            if px == x && py == y {
                centre = z;
            }
        }
    }
    if maximum - minimum > DEPTH_PATCH_SPREAD_M {
        return Err("measured depth discontinuity exceeds patch gate".into());
    }
    Ok(Vec3::new(
        (x as f64 - c.cx) * centre / c.fx,
        (y as f64 - c.cy) * centre / c.fy,
        centre,
    ))
}
fn feature_json(f: &ImageFeature) -> Value {
    json!({"x":f.x,"y":f.y,"score":f.score,"orientation":f.orientation,"descriptor":f.descriptor})
}
fn root_score(estimate: Option<Pose3>, truth: Result<Pose3, String>) -> Value {
    let mut row = json!({"valid":estimate.is_some(),"reference_valid":truth.is_ok(),"within_accuracy_gates":false});
    match truth {
        Ok(t) => {
            row["evaluation_only_truth"] = pose_json(t);
            if let Some(p) = estimate {
                let te = (p.translation.x - t.translation.x)
                    .hypot(p.translation.y - t.translation.y)
                    .hypot(p.translation.z - t.translation.z);
                let re = p.rotation.angular_distance(t.rotation);
                row["estimate"] = pose_json(p);
                row["translation_error_m"] = json!(te);
                row["rotation_error_rad"] = json!(re);
                row["within_accuracy_gates"] =
                    json!(te <= TRANSLATION_GATE_M && re <= ROTATION_GATE_RAD);
            }
        }
        Err(e) => {
            row["reference_rejection"] = json!(e);
            if let Some(p) = estimate {
                row["estimate"] = pose_json(p);
            }
        }
    }
    row
}
// Labels are evaluated after operational tracking. A malformed complete source
// invalidates every reference while leaving sensor decisions and poses intact.
fn score_evaluation_labels(
    rows: &mut [Value],
    frames: &[VisualFrame],
    labels: &[u8],
) -> Option<Value> {
    let gt = evaluation_ground_truth(labels);
    let failure = gt.as_ref().err().map(
        |reason| json!({"file":"groundtruth.txt","source_sha256":sha(labels),"reason":reason}),
    );
    let origin = gt
        .as_ref()
        .map_err(Clone::clone)
        .and_then(|gt| interpolate(gt, frames[0].depth_timestamp));
    for (frame, row) in frames.iter().zip(rows) {
        let truth = origin.clone().and_then(|o| {
            gt.as_ref()
                .map_err(Clone::clone)
                .and_then(|gt| interpolate(gt, frame.depth_timestamp))
                .map(|p| o.inverse().compose(p))
        });
        let estimate = if row["accepted"] == true {
            let t = row["root_estimate"]["translation_m"].as_array().unwrap();
            let q = row["root_estimate"]["quaternion_wxyz"].as_array().unwrap();
            Some(Pose3 {
                translation: Vec3::new(
                    t[0].as_f64().unwrap(),
                    t[1].as_f64().unwrap(),
                    t[2].as_f64().unwrap(),
                ),
                rotation: Quaternion {
                    w: q[0].as_f64().unwrap(),
                    x: q[1].as_f64().unwrap(),
                    y: q[2].as_f64().unwrap(),
                    z: q[3].as_f64().unwrap(),
                },
            })
        } else {
            None
        };
        row["root_accuracy"] = root_score(estimate, truth);
    }
    failure
}
fn write_report(output: &Path, report: &Value) -> Result<bool, String> {
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    fs::write(
        output,
        serde_json::to_vec_pretty(report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    println!("{}", report["summary"]);
    if let Some(failure) = report.get("evaluation_label_failure") {
        return Err(format!(
            "evaluation label source invalid: {}",
            failure["reason"]
        ));
    }
    Ok(report["summary"]["all_updates_passed"] == true)
}
fn evaluate(
    manifest_path: &Path,
    raw: &Path,
    freeze_path: &Path,
    regression: bool,
) -> Result<Value, String> {
    let freeze_bytes = bounded_read(freeze_path, 256 * 1024)?;
    let freeze: Value = serde_json::from_slice(&freeze_bytes).map_err(|e| e.to_string())?;
    if freeze != protocol(manifest_path, regression)? {
        return Err("visual external freeze differs before raw access".into());
    }
    let manifest_bytes = bounded_read(manifest_path, 256 * 1024)?;
    let m: VisualManifest = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    validate(&m)?;
    let calibration_bytes = bounded_read(
        &raw.join("camera-calibration.yaml"),
        m.calibration_source["bytes"].as_u64().unwrap() as usize,
    )?;
    if calibration_bytes.len() as u64 != m.calibration_source["bytes"].as_u64().unwrap()
        || sha(&calibration_bytes) != m.calibration_source["sha256"]
    {
        return Err("visual calibration source byte mismatch".into());
    }
    let mut bytes = BTreeMap::new();
    let mut files = Vec::new();
    for f in &m.files {
        let b = bounded_read(&raw.join(&f.file), 4 * 1024 * 1024)?;
        if b.len() != f.bytes || sha(&b) != f.sha256 {
            return Err(format!("visual source hash mismatch {}", f.file));
        }
        files.push(json!({"file":f.file,"bytes":b.len(),"sha256":sha(&b),"role":f.role}));
        bytes.insert(f.file.clone(), b);
    }
    let depth_index = index_rows(&bytes["depth.txt"])?;
    let rgb_index = index_rows(&bytes["rgb.txt"])?;
    for frame in &m.frames {
        for (index, position, timestamp, file, prefix) in [
            (
                &depth_index,
                frame.source_index,
                frame.depth_timestamp,
                &frame.depth_file,
                "depth",
            ),
            (
                &rgb_index,
                frame.rgb_source_index,
                frame.rgb_timestamp,
                &frame.rgb_file,
                "rgb",
            ),
        ] {
            let (t, path) = index.get(position).ok_or("visual source index missing")?;
            let original = file
                .strip_prefix(&format!("{prefix}-"))
                .ok_or("invalid paired filename")?;
            if *t != timestamp || path != &format!("{prefix}/{original}") {
                return Err("visual pair differs from original source index".into());
            }
        }
    }
    let mut accepted = AcceptedState::default();
    let mut last_observed_rgb: Option<f64> = None;
    let mut lost = false;
    let mut rows = Vec::new();
    // All feature processing, geometry checks and fits finish BEFORE GT parsing.
    for (i, frame) in m.frames.iter().enumerate() {
        let clock = Instant::now();
        let before = accepted.stamp;
        let observed_rgb_before = last_observed_rgb;
        let ref_before = accepted
            .reference
            .as_ref()
            .map(|r| m.frames[r.0].source_index);
        let lost_before = lost;
        let clock_error = observe_clock(
            frame.depth_timestamp,
            frame.rgb_timestamp,
            accepted.stamp,
            &mut last_observed_rgb,
            &mut lost,
        )
        .err();
        let mut depth = Vec::new();
        let mut features = Vec::new();
        if clock_error.is_none() {
            let gray = decode(&bytes[&frame.rgb_file], false)?;
            depth = decode(&bytes[&frame.depth_file], true)?;
            features = extract_features(
                GrayImage::new(WIDTH as usize, HEIGHT as usize, &gray)
                    .map_err(|e| format!("visual image: {e:?}"))?,
            );
        }
        let mut row = json!({"source_index":frame.source_index,"depth_file":frame.depth_file,"depth_timestamp":frame.depth_timestamp,"rgb_file":frame.rgb_file,"rgb_timestamp":frame.rgb_timestamp,"rgb_source_index":frame.rgb_source_index,"pair_gap_seconds":frame.pair_gap_seconds,"split":frame.split,"features_computed":clock_error.is_none(),"features":features.iter().map(feature_json).collect::<Vec<_>>(),"feature_count":features.len(),"accepted":false,"initialized":false,"last_accepted_stamp_before":before,"last_observed_rgb_stamp_before":observed_rgb_before,"last_observed_rgb_stamp_after":last_observed_rgb,"reference_frame_index_before":ref_before,"lost_before":lost_before,"matches":[],"depth_matches":[],"correspondences":[],"refinement_attempted":false,"refinement_observations":[]});
        let operational = (|| -> Result<Pose3, String> {
            if let Some(reason) = &clock_error {
                return Err(reason.clone());
            }
            if let Some((ri, previous, previous_depth, root_reference)) = &accepted.reference {
                row["reference_frame_index"] = json!(m.frames[*ri].source_index);
                row["reference_stamp"] = json!(m.frames[*ri].depth_timestamp);
                row["root_from_reference"] = pose_json(*root_reference);
                let matches = match_features(previous, &features)
                    .map_err(|e| format!("visual matches: {e:?}"))?;
                let mut pairs = Vec::new();
                let mut depth_rows = Vec::new();
                row["matches"]=json!(matches.iter().map(|m|json!({"previous_index":m.previous_index,"current_index":m.current_index,"hamming_distance":m.hamming_distance})).collect::<Vec<_>>());
                for matched in &matches {
                    let a = depth_point(
                        previous_depth,
                        &previous[matched.previous_index],
                        &m.depth_calibration,
                    );
                    let b = depth_point(
                        &depth,
                        &features[matched.current_index],
                        &m.depth_calibration,
                    );
                    let mut point = json!({"previous_index":matched.previous_index,"current_index":matched.current_index,"accepted":a.is_ok()&&b.is_ok()});
                    match (a, b) {
                        (Ok(previous), Ok(current)) => {
                            point["previous_xyz"] = vjson(previous);
                            point["current_xyz"] = vjson(current);
                            point["correspondence_index"] = json!(pairs.len());
                            pairs.push(Correspondence3d { previous, current });
                        }
                        (a, b) => {
                            point["previous_rejection"] = json!(a.err());
                            point["current_rejection"] = json!(b.err());
                        }
                    }
                    depth_rows.push(point);
                }
                row["depth_matches"] = json!(depth_rows);
                row["correspondences"] = json!(
                    pairs
                        .iter()
                        .map(|p| json!({"previous":vjson(p.previous),"current":vjson(p.current)}))
                        .collect::<Vec<_>>()
                );
                let fit = register_correspondences(&pairs, &VisualOdometry3dConfig::default())?;
                row["coarse_relative_estimate"] = pose_json(fit.pose);
                row["fit"] = json!({"rms_m":fit.rms_m,"inlier_indices":fit.inlier_indices,"inlier_count":fit.inlier_count,"inlier_ratio":fit.inlier_ratio,"hypotheses_evaluated":fit.hypotheses_evaluated,"point_checks":fit.point_checks,"refits":fit.refits,"geometry_ratio_current":fit.geometry_ratio_current,"geometry_ratio_previous":fit.geometry_ratio_previous,"candidate_models":fit.candidate_models,"competing_models":fit.competing_models});
                let (observations, reported) =
                    refinement_observations(&pairs, &depth_rows, &features, &fit.inlier_indices)?;
                row["refinement_observations"] = json!(reported);
                row["refinement_attempted"] = json!(true);
                let refined = refine_reprojection(
                    &observations,
                    &CameraIntrinsics {
                        fx: m.depth_calibration.fx,
                        fy: m.depth_calibration.fy,
                        cx: m.depth_calibration.cx,
                        cy: m.depth_calibration.cy,
                    },
                    fit.pose,
                    &ReprojectionConfig3d::default(),
                );
                let refined = match refined {
                    Ok(result) => result,
                    Err(reason) => {
                        row["refinement_rejection"] = json!(reason);
                        return Err(reason);
                    }
                };
                let root = root_reference.compose(refined.pose);
                row["relative_estimate"] = pose_json(refined.pose);
                row["refinement"] = json!({"initial_rms_px":refined.initial_rms_px,"final_rms_px":refined.final_rms_px,"initial_huber_cost":refined.initial_huber_cost,"final_huber_cost":refined.final_huber_cost,"iterations":refined.iterations,"accepted_steps":refined.accepted_steps,"point_checks":refined.point_checks,"valid_support":refined.valid_support,"converged":refined.converged,"max_normal_condition_number":refined.max_normal_condition_number,"trace":refined.trace.iter().map(|step|json!({"current_from_previous":pose_json(step.current_from_previous),"huber_cost_before":step.huber_cost_before,"huber_cost_after":step.huber_cost_after,"scale":step.scale,"increment":step.increment,"normal_condition_number":step.normal_condition_number,"line_search_trials":step.line_search_trials})).collect::<Vec<_>>()});
                Ok(root)
            } else {
                let mut points = Vec::new();
                let mut point_rows = Vec::new();
                for (index, f) in features.iter().enumerate() {
                    if points.len() >= 256 {
                        break;
                    }
                    match depth_point(&depth, f, &m.depth_calibration) {
                        Ok(p) => {
                            point_rows.push(
                                json!({"feature_index":index,"accepted":true,"point":vjson(p)}),
                            );
                            points.push(p);
                        }
                        Err(e) => point_rows
                            .push(json!({"feature_index":index,"accepted":false,"rejection":e})),
                    }
                }
                row["initialization_depth_features"] = json!(point_rows);
                let geometry = validate_geometry(&points, &VisualOdometry3dConfig::default())
                    .inspect_err(|_| {
                        lost = true;
                    })?;
                row["initialization_geometry_ratio"] = json!(geometry);
                row["initialized"] = json!(true);
                Ok(Pose3::identity())
            }
        })();
        accepted.finish(
            operational,
            &mut row,
            i,
            frame.depth_timestamp,
            features,
            depth,
        );
        row["last_observed_rgb_stamp_after"] = json!(last_observed_rgb);
        row["last_accepted_stamp_after"] = json!(accepted.stamp);
        row["reference_frame_index_after"] = json!(
            accepted
                .reference
                .as_ref()
                .map(|r| m.frames[r.0].source_index)
        );
        row["lost_after"] = json!(lost);
        row["cpu_wall_seconds"] = json!(clock.elapsed().as_secs_f64());
        rows.push(row);
    }
    let label_failure = score_evaluation_labels(&mut rows, &m.frames, &bytes["groundtruth.txt"]);
    let updates = &rows[1..];
    let accepted = updates.iter().filter(|r| r["accepted"] == true).count();
    let accurate = updates
        .iter()
        .filter(|r| r["root_accuracy"]["within_accuracy_gates"] == true)
        .count();
    let valid = updates
        .iter()
        .filter(|r| r["root_accuracy"]["reference_valid"] == true)
        .count();
    let initialized = rows.iter().filter(|r| r["initialized"] == true).count();
    let summary = json!({"frames":rows.len(),"updates":updates.len(),"initialized_frames":initialized,"accepted_updates":accepted,"rejected_updates":updates.len()-accepted,"accurate_root_updates":accurate,"reference_valid_updates":valid,"lost":lost,"all_updates_passed":initialized==1&&accepted==updates.len()&&accurate==updates.len()&&valid==updates.len()});
    let mut report = json!({"schema_version":1,"algorithm":"bounded_visual_reprojection_refinement","dataset":m.dataset,"repository":m.repository,"revision":m.revision,"manifest_sha256":sha(&manifest_bytes),"freeze":freeze,"freeze_sha256":sha(&freeze_bytes),"files":files,"calibration_source":m.calibration_source,"calibration_sha256_verified":true,"frames":rows,"summary":summary,"raw_redistributed":false,"ground_truth_operational":false,"limits":["Desk/office/sitting legacy intervals are viewed regression; only the separately frozen first FR2 desk recording is preregistered.","Bounded short indoor RGB-D odometry without scale-invariant descriptors, loop closure, global relocalization or driving fusion.","No calibrated covariance or accumulated root confidence; rejected and unscorable frames stay in the denominator.","Pinned published pinhole camera profiles, without physical infrared extrinsics, vehicle calibration or additional pixel undistortion."]});
    if let Some(failure) = label_failure {
        report["evaluation_label_failure"] = failure;
    }
    Ok(report)
}
pub(super) fn run() -> Result<bool, String> {
    let mut regression = false;
    let mut manifest = None;
    let mut raw = None;
    let mut freeze = None;
    let mut prepare = None;
    let mut output = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--visual-reprojection" {
            continue;
        }
        if a == "--regression" {
            if regression {
                return Err("duplicate --regression".into());
            }
            regression = true;
            continue;
        }
        if matches!(
            a.as_str(),
            "--visual" | "--submaps" | "--motion" | "--keyframes"
        ) {
            return Err("choose exactly one evaluation mode".into());
        }
        let v = args.next().ok_or("missing visual argument value")?;
        match a.as_str() {
            "--manifest" => manifest = Some(v),
            "--raw" => raw = Some(v),
            "--freeze" => freeze = Some(v),
            "--prepare-freeze" => prepare = Some(v),
            "--output" => output = Some(v),
            _ => return Err(format!("unknown visual argument {a}")),
        }
    }
    let manifest = manifest.ok_or("--manifest required")?;
    if let Some(path) = prepare {
        if raw.is_some() || freeze.is_some() || output.is_some() {
            return Err("visual freeze preparation reads metadata only".into());
        }
        let f = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(f, &protocol(Path::new(&manifest), regression)?)
            .map_err(|e| e.to_string())?;
        return Ok(true);
    }
    let report = evaluate(
        Path::new(&manifest),
        Path::new(&raw.ok_or("--raw required")?),
        Path::new(&freeze.ok_or("--freeze required")?),
        regression,
    )?;
    let output = output.ok_or("--output required")?;
    write_report(Path::new(&output), &report)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn feature(x: f64, y: f64) -> ImageFeature {
        ImageFeature {
            x,
            y,
            score: 40,
            orientation: 0.,
            descriptor: [0; 4],
        }
    }
    #[test]
    fn invalid_whole_label_source_retains_sensor_traces_and_writes_before_failure() {
        let frames: Vec<VisualFrame> = (0..2)
            .map(|index| VisualFrame {
                source_index: index,
                depth_file: String::new(),
                depth_timestamp: index as f64,
                rgb_file: String::new(),
                rgb_timestamp: index as f64,
                rgb_source_index: index,
                pair_gap_seconds: 0.,
                split: String::new(),
            })
            .collect();
        let root = pose_json(Pose3::identity());
        let mut rows = vec![
            json!({"accepted":true,"root_estimate":root,"refinement":{"converged":true},"last_accepted_stamp_after":0.}),
            json!({"accepted":true,"root_estimate":root,"refinement":{"converged":true},"last_accepted_stamp_after":1.}),
        ];
        let original = rows.clone();
        // The bad duplicate is outside the selected interval. Full-source
        // validation still rejects it instead of scoring only a valid subset.
        let labels = b"0 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n60 0 0 0 0 0 0 1\n60 1 0 0 0 0 0 1\n";
        let failure = score_evaluation_labels(&mut rows, &frames, labels).unwrap();
        assert_eq!(
            failure,
            json!({"file":"groundtruth.txt","source_sha256":sha(labels),"reason":"invalid GT quaternion or nonmonotonic timestamps"})
        );
        for (row, before) in rows.iter().zip(original) {
            let mut operational = row.clone();
            operational.as_object_mut().unwrap().remove("root_accuracy");
            assert_eq!(operational, before);
            assert_eq!(row["root_accuracy"]["valid"], true);
            assert_eq!(row["root_accuracy"]["reference_valid"], false);
            assert_eq!(row["root_accuracy"]["within_accuracy_gates"], false);
            assert!(row["root_accuracy"].get("translation_error_m").is_none());
            assert!(row["root_accuracy"].get("evaluation_only_truth").is_none());
        }
        let report = json!({"frames":rows,"evaluation_label_failure":failure,"summary":{"all_updates_passed":false}});
        let output = std::env::temp_dir().join(format!(
            "rustdriving-invalid-label-report-{}.json",
            std::process::id()
        ));
        assert!(write_report(&output, &report).is_err());
        let saved: Value = serde_json::from_slice(&fs::read(&output).unwrap()).unwrap();
        assert_eq!(saved, report);
        fs::remove_file(output).unwrap();
    }
    #[test]
    fn evaluation_labels_preserve_numeric_checks_and_explicit_source_bounds() {
        let short = b"0 1 2 3 0 0 0 1\n1 4 5 6 0 0 0 1\n";
        let legacy = ground_truth(short).unwrap();
        let current = evaluation_ground_truth(short).unwrap();
        assert_eq!(legacy.len(), current.len());
        for (a, b) in legacy.iter().zip(&current) {
            assert_eq!(a.timestamp, b.timestamp);
            assert_eq!(pose_json(a.pose), pose_json(b.pose));
        }
        let mut labels = String::from("#");
        labels.push_str(&" ".repeat(1024 * 1024));
        labels.push('\n');
        for i in 0..MAX_GT_ROWS {
            labels.push_str(&format!("{i} 0 0 0 0 0 0 1\n"));
        }
        assert!(ground_truth(labels.as_bytes()).is_err());
        assert_eq!(
            evaluation_ground_truth(labels.as_bytes()).unwrap().len(),
            MAX_GT_ROWS
        );
        labels.push_str("30000 0 0 0 0 0 0 1\n");
        assert!(evaluation_ground_truth(labels.as_bytes()).is_err());
        assert!(evaluation_ground_truth(&vec![b' '; MAX_GT_BYTES + 1]).is_err());
        for bad in [
            "0 NaN 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n",
            "0 0 0 0 0 0 0 2\n1 0 0 0 0 0 0 1\n",
            "1 0 0 0 0 0 0 1\n0 0 0 0 0 0 0 1\n",
        ] {
            assert!(evaluation_ground_truth(bad.as_bytes()).is_err());
        }
    }
    #[test]
    fn refinement_uses_only_coarse_inliers_previous_depth_and_exact_current_pixels() {
        let pairs: Vec<_> = (0..3)
            .map(|i| Correspondence3d {
                previous: Vec3::new(i as f64, 0., 2.),
                // Refinement deliberately does not use the current depth endpoint.
                current: Vec3::new(999., 999., -1.),
            })
            .collect();
        let features = vec![
            feature(300., 200.),
            feature(310., 220.),
            feature(330., 240.),
        ];
        let rows = vec![
            json!({"accepted":true,"correspondence_index":2,"previous_index":20,"current_index":0}),
            json!({"accepted":true,"correspondence_index":0,"previous_index":10,"current_index":2}),
            json!({"accepted":true,"correspondence_index":1,"previous_index":15,"current_index":1}),
        ];
        let (observations, report) =
            refinement_observations(&pairs, &rows, &features, &[2, 0]).unwrap();
        assert_eq!(observations.len(), 2);
        assert_eq!(observations[0].previous, pairs[2].previous);
        assert_eq!(observations[0].current_pixel, [300., 200.]);
        assert_eq!(observations[1].previous, pairs[0].previous);
        assert_eq!(observations[1].current_pixel, [330., 240.]);
        assert_eq!(report[0]["correspondence_index"], 2);
        assert_eq!(report[1]["correspondence_index"], 0);
        assert!(refinement_observations(&pairs, &rows, &features, &[3]).is_err());
        assert!(refinement_observations(&pairs, &rows, &features, &[0, 0]).is_err());
        assert!(refinement_observations(&pairs, &[], &features, &[0]).is_err());
    }
    #[test]
    fn actual_refinement_failure_preserves_reference_clock_and_removes_pose_permission() {
        let reference_pose = Pose3 {
            translation: Vec3::new(1., 2., 3.),
            ..Pose3::identity()
        };
        let mut state = AcceptedState {
            reference: Some((7, vec![feature(321., 239.)], vec![1, 2], reference_pose)),
            stamp: Some(10.),
        };
        let camera = CameraIntrinsics {
            fx: 535.4,
            fy: 539.2,
            cx: 320.1,
            cy: 247.6,
        };
        let failure = refine_reprojection(
            &[],
            &camera,
            Pose3::identity(),
            &ReprojectionConfig3d::default(),
        )
        .map(|result| result.pose);
        assert!(failure.is_err());
        let mut row = json!({"accepted":true,"root_estimate":pose_json(Pose3::identity()),"relative_estimate":pose_json(Pose3::identity()),"coarse_relative_estimate":pose_json(Pose3::identity())});
        state.finish(
            failure,
            &mut row,
            8,
            10.1,
            vec![feature(100., 100.)],
            vec![3, 4],
        );
        assert_eq!(state.stamp, Some(10.));
        let preserved = state.reference.unwrap();
        assert_eq!(preserved.0, 7);
        assert_eq!(preserved.1[0].x, 321.);
        assert_eq!(preserved.1[0].y, 239.);
        assert_eq!(preserved.2, vec![1, 2]);
        assert_eq!(pose_json(preserved.3), pose_json(reference_pose));
        assert_eq!(row["accepted"], false);
        assert!(row.get("root_estimate").is_none());
        assert!(row.get("relative_estimate").is_none());
        assert!(row.get("coarse_relative_estimate").is_some());
        assert!(row["rejection"].is_string());
    }
}
