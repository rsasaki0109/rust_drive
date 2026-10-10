//! Standalone depth-supported recorded RGB-D pixel refinement. Associations and
//! rigid fits finish before any evaluation-only motion-capture labels are parsed.
use super::*;
use rustdriving_localization::reprojection3d::{
    CameraIntrinsics, ReprojectionConfig3d, ReprojectionObservation3d, refine_reprojection,
};
use rustdriving_localization::visual_odometry3d::{
    Correspondence3d, VisualOdometry3dConfig, register_correspondences, validate_geometry,
};
use rustdriving_perception::image_features::{GrayImage, ImageFeature, extract_features};

#[path = "depth_supported_matches.rs"]
mod depth_supported_matches;
use depth_supported_matches::match_depth_supported;

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
    let camera = (
        517.306408,
        516.469215,
        318.643040,
        255.313989,
        "settings/TUM1.yaml",
        1615,
        "5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf",
        5000.,
    );
    let (repository, revision) = (
        "edrishakimi1/Indoor-SLAM-Floorplan-with-Gaussian-Splatting",
        "1d5b2c3e1ee186abc1709042b739e9cd8a3b41d1",
    );
    let start = 100;
    if m.schema_version != 1
        || m.dataset != "tum-fr1-room-temporal"
        || m.repository != repository
        || m.revision != revision
        || m.revision.len() != 40
        || !m.revision.bytes().all(|b| b.is_ascii_hexdigit())
        || c.width != WIDTH
        || c.height != HEIGHT
        || (c.fx, c.fy, c.cx, c.cy) != (camera.0, camera.1, camera.2, camera.3)
        || c.units_per_metre != camera.7
        || c.invalid_depth != 0
        || m.frames.len() != 180
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
                    } else if f.source_index <= 135 {
                        "viewed_prefix"
                    } else {
                        "unviewed_extension"
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
    if total > 128 * 1024 * 1024
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
const QUALIFICATION_TIME_UNIT: &str =
    "rounded microseconds from source timestamps; qualification gates use unrounded seconds";
fn metadata_descriptors(metadata: &Value) -> Result<Value, String> {
    let inventory = metadata["files"]
        .as_array()
        .ok_or("qualification manifest inventory missing")?;
    let mut result = serde_json::Map::new();
    for (name, size, digest, git) in [
        (
            "depth.txt",
            62636,
            "86757dd3c8606f940477a353915ede7143811817b2288647414fe1459ed22c14",
            "6a8c4f13bdeaa5ebca5fd6a78de7230eb02d1995",
        ),
        (
            "rgb.txt",
            60006,
            "fe2f438ebe1b3d13a061e4db3b154df1136a82d95a723e4a6457b0e516c2451b",
            "a8b9fe8ce86576c0353b1182546163bea569188c",
        ),
        (
            "groundtruth.txt",
            332920,
            "66b6770d887a5a3b69c94c3b186c3f76fe092ca25399772177f41aed984bd1b0",
            "0adacc2b8c371eefc7f82993eb8e30128283e888",
        ),
    ] {
        let item = inventory
            .iter()
            .find(|item| item["file"] == name)
            .ok_or("qualification metadata descriptor missing")?;
        let blob = item["git_blob_sha1"]
            .as_str()
            .ok_or("qualification metadata Git source hash missing")?;
        if blob.len() != 40
            || !blob
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
        {
            return Err("qualification metadata Git source hash invalid".into());
        }
        if item["bytes"] != size || item["sha256"] != digest || blob != git {
            return Err("qualification metadata differs from pinned original source".into());
        }
        result.insert(
            name.into(),
            json!({"bytes":item["bytes"],"sha256":item["sha256"],"git_blob_sha1":blob}),
        );
    }
    Ok(Value::Object(result))
}
fn qualification_header(
    metadata: &Value,
    manifest_bytes: &[u8],
    proof: &Value,
) -> Result<(), String> {
    for (name, expected) in [
        ("schema_version", json!(1)),
        ("dataset", metadata["dataset"].clone()),
        ("manifest_sha256", json!(sha(manifest_bytes))),
        ("source_commit", metadata["revision"].clone()),
        (
            "window",
            json!({"first_depth_index":100,"last_depth_index":279,"frames":180,"updates":179}),
        ),
        ("metadata_files", metadata_descriptors(metadata)?),
        ("passed", json!(true)),
        ("pixels_read", json!(false)),
        ("image_headers_read", json!(false)),
        ("ground_truth_pose_values_parsed", json!(false)),
        ("features_or_fits_run", json!(false)),
        (
            "helper_sha256",
            json!(sha(include_bytes!(
                "../../../scripts/qualify-rgbd-temporal.py"
            ))),
        ),
        (
            "acquisition_helper_sha256",
            json!(sha(include_bytes!(
                "../../../scripts/fetch-temporal-dataset.py"
            ))),
        ),
    ] {
        if proof[name] != expected {
            return Err(format!("qualification proof binding invalid: {name}"));
        }
    }
    for (name, expected) in [
        ("strict_depth_order", json!(true)),
        ("strict_rgb_order", json!(true)),
        ("strict_ground_truth_order", json!(true)),
        ("all_frame_brackets_valid", json!(true)),
        ("max_ground_truth_bracket_s", json!(GT_MAX_BRACKET_S)),
        ("max_pair_gap_s", json!(PAIR_GAP_S)),
        ("summary_time_unit", json!(QUALIFICATION_TIME_UNIT)),
    ] {
        if proof["timestamps"][name] != expected {
            return Err(format!("qualification timestamp policy invalid: {name}"));
        }
    }
    Ok(())
}
fn strict_metadata_index(bytes: &[u8]) -> Result<Vec<(f64, String)>, String> {
    let rows = index_rows(bytes)?;
    if rows.len() < 2 || rows.len() > 20_000 || rows.windows(2).any(|pair| pair[1].0 <= pair[0].0) {
        return Err("qualification source index timestamps are not strictly increasing".into());
    }
    Ok(rows)
}
fn ground_truth_timestamps(bytes: &[u8]) -> Result<Vec<f64>, String> {
    if bytes.len() > MAX_GT_BYTES {
        return Err("qualification GT byte bound".into());
    }
    let text = std::str::from_utf8(bytes).map_err(|e| e.to_string())?;
    let mut stamps = Vec::new();
    for line in text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
    {
        let fields: Vec<_> = line.split_whitespace().collect();
        if fields.len() != 8 {
            return Err("qualification GT row arity invalid".into());
        }
        // The seven position/quaternion tokens deliberately stay opaque here.
        let stamp = fields[0]
            .parse::<f64>()
            .map_err(|_| "qualification GT timestamp invalid")?;
        if !stamp.is_finite() || stamps.last().is_some_and(|previous| stamp <= *previous) {
            return Err("qualification GT timestamps are not strictly increasing".into());
        }
        stamps.push(stamp);
        if stamps.len() > MAX_GT_ROWS {
            return Err("qualification GT row bound".into());
        }
    }
    if stamps.len() < 2 {
        return Err("qualification GT timestamp source too short".into());
    }
    Ok(stamps)
}
fn metadata_statistics(
    frames: &[VisualFrame],
    metadata_bytes: &BTreeMap<String, Vec<u8>>,
) -> Result<Value, String> {
    let depth = strict_metadata_index(&metadata_bytes["depth.txt"])?;
    let rgb = strict_metadata_index(&metadata_bytes["rgb.txt"])?;
    let gt = ground_truth_timestamps(&metadata_bytes["groundtruth.txt"])?;
    let mut maximum_bracket = 0f64;
    let mut maximum_pair_gap = 0f64;
    let mut unique_rgb = BTreeSet::new();
    for frame in frames {
        for (index, ordinal, stamp, file, prefix) in [
            (
                &depth,
                frame.source_index,
                frame.depth_timestamp,
                &frame.depth_file,
                "depth",
            ),
            (
                &rgb,
                frame.rgb_source_index,
                frame.rgb_timestamp,
                &frame.rgb_file,
                "rgb",
            ),
        ] {
            let (observed, path) = index
                .get(ordinal)
                .ok_or("qualification selected source index absent")?;
            if *observed != stamp
                || path
                    != &format!(
                        "{prefix}/{}",
                        file.strip_prefix(&format!("{prefix}-"))
                            .ok_or("qualification local file prefix invalid")?
                    )
            {
                return Err("qualification selected source index does not match manifest".into());
            }
        }
        let upper = rgb.partition_point(|row| row.0 < frame.depth_timestamp);
        let mut nearest = upper.min(rgb.len() - 1);
        if upper > 0
            && (rgb[upper - 1].0 - frame.depth_timestamp).abs()
                <= (rgb[nearest].0 - frame.depth_timestamp).abs()
        {
            nearest = upper - 1;
        }
        if frame.rgb_source_index != nearest {
            return Err("qualification RGB association is not original nearest timestamp".into());
        }
        let gap = (frame.depth_timestamp - frame.rgb_timestamp).abs();
        if gap > PAIR_GAP_S {
            return Err("qualification RGB/depth association gate exceeded".into());
        }
        maximum_pair_gap = maximum_pair_gap.max(gap);
        unique_rgb.insert(frame.rgb_source_index);
        let right = gt.partition_point(|stamp| *stamp < frame.depth_timestamp);
        let width = if right < gt.len() && gt[right] == frame.depth_timestamp {
            0.
        } else {
            if right == 0 || right == gt.len() {
                return Err("qualification GT extrapolation forbidden".into());
            }
            let width = gt[right] - gt[right - 1];
            if width > GT_MAX_BRACKET_S {
                return Err("qualification GT bracket gate exceeded".into());
            }
            width
        };
        maximum_bracket = maximum_bracket.max(width);
    }
    Ok(
        json!({"depth_index_rows":depth.len(),"rgb_index_rows":rgb.len(),"ground_truth_rows":gt.len(),"strict_depth_order":true,"strict_rgb_order":true,"strict_ground_truth_order":true,"all_frame_brackets_valid":true,"max_ground_truth_bracket_s":GT_MAX_BRACKET_S,"max_pair_gap_s":PAIR_GAP_S,"maximum_observed_ground_truth_bracket_us":(maximum_bracket*1e6+0.5).floor() as u64,"maximum_pair_gap_us":(maximum_pair_gap*1e6+0.5).floor() as u64,"unique_rgb_acquisitions":unique_rgb.len(),"duplicate_rgb_associations":frames.len()-unique_rgb.len(),"summary_time_unit":QUALIFICATION_TIME_UNIT}),
    )
}
fn qualified_sensor<T>(
    computed: Result<Value, String>,
    proof: &Value,
    sensor: impl FnOnce() -> Result<T, String>,
) -> Result<T, String> {
    if computed? != *proof {
        return Err("qualification proof differs from independently recomputed metadata".into());
    }
    sensor()
}
fn verify_metadata_bytes(
    descriptors: &Value,
    metadata_bytes: &BTreeMap<String, Vec<u8>>,
) -> Result<(), String> {
    for name in ["depth.txt", "rgb.txt", "groundtruth.txt"] {
        let bytes = metadata_bytes
            .get(name)
            .ok_or("qualification metadata bytes missing")?;
        if descriptors[name]["bytes"] != json!(bytes.len())
            || descriptors[name]["sha256"] != sha(bytes)
        {
            return Err(format!(
                "qualification metadata source hash/size mismatch: {name}"
            ));
        }
    }
    Ok(())
}
fn qualification_proof(
    manifest_bytes: &[u8],
    m: &VisualManifest,
    metadata_bytes: &BTreeMap<String, Vec<u8>>,
) -> Result<Value, String> {
    let metadata: Value = serde_json::from_slice(manifest_bytes).map_err(|e| e.to_string())?;
    if metadata["source_prefix"] != "data/rgbd_dataset_freiburg1_room/" {
        return Err("qualification source prefix invalid".into());
    }
    for item in metadata["files"]
        .as_array()
        .ok_or("qualification source inventory missing")?
    {
        let name = item["file"]
            .as_str()
            .ok_or("qualification source filename missing")?;
        let relative = if let Some(name) = name.strip_prefix("depth-") {
            format!("depth/{name}")
        } else if let Some(name) = name.strip_prefix("rgb-") {
            format!("rgb/{name}")
        } else {
            name.into()
        };
        if item["source_path"] != format!("data/rgbd_dataset_freiburg1_room/{relative}") {
            return Err("qualification source path outside pinned sequence".into());
        }
    }
    let descriptors = metadata_descriptors(&metadata)?;
    verify_metadata_bytes(&descriptors, metadata_bytes)?;
    Ok(
        json!({"schema_version":1,"dataset":m.dataset,"manifest_sha256":sha(manifest_bytes),"source_commit":m.revision,
        "window":{"first_depth_index":100,"last_depth_index":279,"frames":180,"updates":179},"metadata_files":descriptors,
        "timestamps":metadata_statistics(&m.frames, metadata_bytes)?,
        "passed":true,"pixels_read":false,"image_headers_read":false,"ground_truth_pose_values_parsed":false,"features_or_fits_run":false,
        "helper_sha256":sha(include_bytes!("../../../scripts/qualify-rgbd-temporal.py")),"acquisition_helper_sha256":sha(include_bytes!("../../../scripts/fetch-temporal-dataset.py"))}),
    )
}
fn preflight_with_metadata<T>(
    manifest_bytes: &[u8],
    m: &VisualManifest,
    proof: &Value,
    metadata_bytes: &BTreeMap<String, Vec<u8>>,
    sensor: impl FnOnce() -> Result<T, String>,
) -> Result<T, String> {
    validate(m)?;
    let metadata: Value = serde_json::from_slice(manifest_bytes).map_err(|e| e.to_string())?;
    qualification_header(&metadata, manifest_bytes, proof)?;
    qualified_sensor(
        qualification_proof(manifest_bytes, m, metadata_bytes),
        proof,
        sensor,
    )
}
fn preflight_then<T>(
    manifest_bytes: &[u8],
    m: &VisualManifest,
    proof: &Value,
    raw: &Path,
    sensor: impl FnOnce() -> Result<T, String>,
) -> Result<T, String> {
    let mut metadata = BTreeMap::new();
    for name in ["depth.txt", "rgb.txt", "groundtruth.txt"] {
        metadata.insert(name.into(), bounded_read(&raw.join(name), MAX_GT_BYTES)?);
    }
    preflight_with_metadata(manifest_bytes, m, proof, &metadata, sensor)
}
fn protocol(path: &Path, qualification_path: &Path, regression: bool) -> Result<Value, String> {
    if !regression {
        return Err(
            "depth-supported evaluation requires --regression; all selected images are viewed"
                .into(),
        );
    }
    let bytes = bounded_read(path, 512 * 1024)?;
    let metadata: Value = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    let m: VisualManifest = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    validate(&m)?;
    let qualification_bytes = bounded_read(qualification_path, 512 * 1024)?;
    let qualification: Value =
        serde_json::from_slice(&qualification_bytes).map_err(|e| e.to_string())?;
    qualification_header(&metadata, &bytes, &qualification)?;
    if metadata["redistribute_raw"] != false {
        return Err("visual raw redistribution is not authorized".into());
    }
    let mut value = json!({"schema_version":1,"protocol_version":1,"algorithm":"bounded_visual_reprojection_depth_supported","dataset":m.dataset,"manifest_sha256":sha(&bytes),"kind":"calibration_regression","regression_requested":regression,
 "viewed_prefix":{"first_depth_index":100,"last_depth_index":135,"frames":36},
 "extension_window":{"first_depth_index":136,"last_depth_index":279,"frames":144},
 "continuous_state":{"initialization_frame_index":100,"maximum_initializations":1,"chunk_resets":false,"lost_recovery":false},
 "evidence_scope":"same previously evaluated180-frame room recording; depth-supported descriptor matching calibration variant; all images and reference labels viewed; no new-recording or fresh-generalization claim",
 "resources":{"max_raw_bytes":134217728,"max_report_bytes":67108864,"max_frames":180,"max_manifest_bytes":524288,"max_freeze_bytes":524288,"max_qualification_bytes":524288},
 "qualification":qualification,"qualification_sha256":sha(&qualification_bytes),
 "qualification_order":"source-bound timestamp and row-arity metadata before any image bytes, headers, features or fits; ground-truth pose values only after all fits",
 "evaluation_label_policy":{"invalid_source":"retain sensor rows, mark all references and accuracy invalid, report evaluation_label_failure and exit2; never normalize, deduplicate or subset invalid labels"},
 "json_metadata_audit":{"field":"pair_gap_seconds","absolute_tolerance_s":1e-15,"scope":"source-manifest versus parsed freeze/report derived gap only; source SHA, acquisition timestamps and indices remain exact; no sensor association or fit gate change"},
 "depth_calibration":metadata["depth_calibration"],"calibration_source":m.calibration_source,
 "preprocessing":{"width":WIDTH,"height":HEIGHT,"luma":"(77*R+150*G+29*B)>>8; RGB/RGBA8; RGBA must be fully opaque","pixel_coordinates":"nearest integer feature coordinate, ties round away from zero","min_depth_m":MIN_DEPTH_M,"max_depth_m":MAX_DEPTH_M,"depth_units_per_metre":m.depth_calibration.units_per_metre,"patch_radius_pixels":1,"patch_validity":"all 9 depths valid and range-bounded","patch_max_spread_m":DEPTH_PATCH_SPREAD_M,"point":"centre depth at feature pixel; optical x-right y-down z-forward","maximum_pair_gap_s":PAIR_GAP_S},
 "feature_policy":{"max_features":400,"max_matches":256,"detector":"fixed original FAST-9 threshold20 radius3, deterministic NMS,32pixel tile max2","descriptor":"intensity-centroid oriented deterministic256bit BRIEF on5x5binomial blur","matching":"both directional strict 5*best<4*second, maximumHamming64, mutual nearest; ties rejected"},
 "depth_supported_matching_policy":{"eligibility":"original depth_point: all nine 3x3 depths valid, within 0.3..5m, max spread0.05m; independently checked per feature before descriptor matching","domain":"both directional mutual unique strict ratio matching considers only eligible previous/current features; at least two eligible candidates in each direction","matching":"unchanged strict 5*best<4*second, maximumHamming64; ties rejected; cap256 sorted distance then original previous/current indices","indices":"eligible feature IDs and matches refer to unchanged original feature extraction order"},"registration_config":configuration(),"refinement_config":refinement_configuration(),"refinement_policy":{"observations":"only original robust3D inlier correspondence indices; previous measured depth point and matched current RGB feature pixel; fixed support throughout refinement","initial_pose":"original previous_from_current robust3D fit, never motion-capture labels","pose":"refined previous_from_current pose is composed into accepted root; coarse fit and pose are separate diagnostics","failure":"reject without current root output or accepted reference/clock renewal; no coarse-pose fallback","method":"bounded Huber pixel reprojection, maximum8 iterations; convergence and work reported explicitly"},"tracking_policy":{"max_unobserved_s":MAX_AGE_S,"reference":"last accepted measured RGB features and depth image; initial root identity after measured geometry validation","pose":"reference-root pose composed with measured refined previous_from_current pixel-reprojection pose","rejection":"no root output or accepted clock/reference update; repeated/stale RGB timestamp versus last observed image rejects, including previously rejected fits; observed image clock advances without permission renewal; accepted-pose expiry checked before RGB freshness; evaluation never resets"},
 "accuracy_gates":{"translation_m":TRANSLATION_GATE_M,"rotation_rad":ROTATION_GATE_RAD},"ground_truth_interpolation":{"method":"linear translation and shortest-arc quaternion SLERP","max_bracket_s":GT_MAX_BRACKET_S,"extrapolation":false,"max_source_rows":MAX_GT_ROWS,"max_source_bytes":MAX_GT_BYTES},"frames":metadata["frames"],
 "freeze_preparation":"metadata only; RGB/depth/mocap raw bytes are not read, features not extracted and registration not run"});
    let sources = json!({"evaluator_source_sha256":sha(include_bytes!("bin/rustdriving-rgbd-supported.rs")),"supported_source_sha256":sha(include_bytes!("supported.rs")),"depth_supported_matcher_source_sha256":sha(include_bytes!("depth_supported_matches.rs")),"temporal_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-temporal.py")),"reprojection_pose_source_sha256":sha(include_bytes!("../../../crates/localization/src/reprojection3d.rs")),"feature_source_sha256":sha(include_bytes!("../../../crates/perception/src/image_features.rs")),"visual_pose_source_sha256":sha(include_bytes!("../../../crates/localization/src/visual_odometry3d.rs")),"pose_source_sha256":sha(include_bytes!("../../../crates/localization/src/registration3d.rs")),"localization_lib_source_sha256":sha(include_bytes!("../../../crates/localization/src/lib.rs")),"perception_lib_source_sha256":sha(include_bytes!("../../../crates/perception/src/lib.rs")),"core_lib_source_sha256":sha(include_bytes!("../../../crates/core/src/lib.rs")),"cargo_lock_sha256":sha(include_bytes!("../Cargo.lock")),"cargo_manifest_sha256":sha(include_bytes!("../Cargo.toml")),"rust_toolchain_sha256":sha(include_bytes!("../../../rust-toolchain.toml")),"independent_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-supported.py")),"reprojection_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-reprojection.py")),"visual_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-visual.py")),"geometry_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-rgbd.py")),"keyframe_checker_sha256":sha(include_bytes!("../../../scripts/check-recorded-keyframes.py")),"checker_requirements_sha256":sha(include_bytes!("../../../scripts/requirements-visual.txt")),"acquisition_source_sha256":sha(include_bytes!("../../../scripts/fetch-temporal-dataset.py")),"qualification_source_sha256":sha(include_bytes!("../../../scripts/qualify-rgbd-temporal.py"))});
    value
        .as_object_mut()
        .unwrap()
        .extend(sources.as_object().unwrap().clone());
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
        "depth_supported_matching_policy",
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
    let serialized = serde_json::to_vec(report).map_err(|e| e.to_string())?;
    if serialized.len() > 64 * 1024 * 1024 {
        return Err("temporal report exceeds 64MiB byte bound".into());
    }
    fs::write(output, serialized).map_err(|e| e.to_string())?;
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
    qualification_path: &Path,
    regression: bool,
) -> Result<Value, String> {
    let freeze_bytes = bounded_read(freeze_path, 512 * 1024)?;
    let freeze: Value = serde_json::from_slice(&freeze_bytes).map_err(|e| e.to_string())?;
    if freeze != protocol(manifest_path, qualification_path, regression)? {
        return Err("visual external freeze differs before raw access".into());
    }
    let manifest_bytes = bounded_read(manifest_path, 512 * 1024)?;
    let m: VisualManifest = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    validate(&m)?;
    let qualification_bytes = bounded_read(qualification_path, 512 * 1024)?;
    let qualification: Value =
        serde_json::from_slice(&qualification_bytes).map_err(|e| e.to_string())?;
    preflight_then(&manifest_bytes, &m, &qualification, raw, || {
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
            let mut row = json!({"source_index":frame.source_index,"depth_file":frame.depth_file,"depth_timestamp":frame.depth_timestamp,"rgb_file":frame.rgb_file,"rgb_timestamp":frame.rgb_timestamp,"rgb_source_index":frame.rgb_source_index,"pair_gap_seconds":frame.pair_gap_seconds,"split":frame.split,"features_computed":clock_error.is_none(),"features":features.iter().map(feature_json).collect::<Vec<_>>(),"feature_count":features.len(),"accepted":false,"initialized":false,"last_accepted_stamp_before":before,"last_observed_rgb_stamp_before":observed_rgb_before,"last_observed_rgb_stamp_after":last_observed_rgb,"reference_frame_index_before":ref_before,"lost_before":lost_before,"previous_depth_eligible_feature_indices":[],"current_depth_eligible_feature_indices":[],"matches":[],"depth_matches":[],"correspondences":[],"refinement_attempted":false,"refinement_observations":[]});
            let operational = (|| -> Result<Pose3, String> {
                if let Some(reason) = &clock_error {
                    return Err(reason.clone());
                }
                if let Some((ri, previous, previous_depth, root_reference)) = &accepted.reference {
                    row["reference_frame_index"] = json!(m.frames[*ri].source_index);
                    row["reference_stamp"] = json!(m.frames[*ri].depth_timestamp);
                    row["root_from_reference"] = pose_json(*root_reference);
                    let previous_eligible: Vec<bool> = previous
                        .iter()
                        .map(|feature| {
                            depth_point(previous_depth, feature, &m.depth_calibration).is_ok()
                        })
                        .collect();
                    let current_eligible: Vec<bool> = features
                        .iter()
                        .map(|feature| depth_point(&depth, feature, &m.depth_calibration).is_ok())
                        .collect();
                    row["previous_depth_eligible_feature_indices"] = json!(
                        previous_eligible
                            .iter()
                            .enumerate()
                            .filter_map(|(i, &eligible)| eligible.then_some(i))
                            .collect::<Vec<_>>()
                    );
                    row["current_depth_eligible_feature_indices"] = json!(
                        current_eligible
                            .iter()
                            .enumerate()
                            .filter_map(|(i, &eligible)| eligible.then_some(i))
                            .collect::<Vec<_>>()
                    );
                    let matches = match_depth_supported(
                        previous,
                        &features,
                        &previous_eligible,
                        &current_eligible,
                    )?;
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
                    row["correspondences"] =
                        json!(
                    pairs
                        .iter()
                        .map(|p| json!({"previous":vjson(p.previous),"current":vjson(p.current)}))
                        .collect::<Vec<_>>()
                );
                    let fit = register_correspondences(&pairs, &VisualOdometry3dConfig::default())?;
                    row["coarse_relative_estimate"] = pose_json(fit.pose);
                    row["fit"] = json!({"rms_m":fit.rms_m,"inlier_indices":fit.inlier_indices,"inlier_count":fit.inlier_count,"inlier_ratio":fit.inlier_ratio,"hypotheses_evaluated":fit.hypotheses_evaluated,"point_checks":fit.point_checks,"refits":fit.refits,"geometry_ratio_current":fit.geometry_ratio_current,"geometry_ratio_previous":fit.geometry_ratio_previous,"candidate_models":fit.candidate_models,"competing_models":fit.competing_models});
                    let (observations, reported) = refinement_observations(
                        &pairs,
                        &depth_rows,
                        &features,
                        &fit.inlier_indices,
                    )?;
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
                            Err(e) => point_rows.push(
                                json!({"feature_index":index,"accepted":false,"rejection":e}),
                            ),
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
        let label_failure =
            score_evaluation_labels(&mut rows, &m.frames, &bytes["groundtruth.txt"]);
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
        let mut report = json!({"schema_version":1,"algorithm":"bounded_visual_reprojection_depth_supported","dataset":m.dataset,"repository":m.repository,"revision":m.revision,"manifest_sha256":sha(&manifest_bytes),"freeze":freeze,"freeze_sha256":sha(&freeze_bytes),"files":files,"calibration_source":m.calibration_source,"calibration_sha256_verified":true,"frames":rows,"summary":summary,"raw_redistributed":false,"ground_truth_operational":false,"limits":["All selected images are previously viewed; depth-supported matching is a viewed regression, not a fresh generalization trial.","Bounded short indoor RGB-D odometry without scale-invariant descriptors, loop closure, global relocalization or driving fusion.","No calibrated covariance or accumulated root confidence; rejected and unscorable frames stay in the denominator.","Pinned published pinhole camera profiles, without physical infrared extrinsics, vehicle calibration or additional pixel undistortion."]});
        if let Some(failure) = label_failure {
            report["evaluation_label_failure"] = failure;
        }
        report["qualification_verified"] = json!(true);
        report["qualification"] = qualification.clone();
        report["qualification_sha256"] = json!(sha(&qualification_bytes));
        Ok(report)
    })
}
pub(super) fn run() -> Result<bool, String> {
    let mut regression = false;
    let mut manifest = None;
    let mut qualification = None;
    let mut raw = None;
    let mut freeze = None;
    let mut prepare = None;
    let mut output = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--supported" {
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
            "--qualification" => qualification = Some(v),
            "--raw" => raw = Some(v),
            "--freeze" => freeze = Some(v),
            "--prepare-freeze" => prepare = Some(v),
            "--output" => output = Some(v),
            _ => return Err(format!("unknown visual argument {a}")),
        }
    }
    if !regression {
        return Err(
            "depth-supported evaluation requires --regression; all selected images are viewed"
                .into(),
        );
    }
    let manifest = manifest.ok_or("--manifest required")?;
    let qualification = qualification.ok_or("--qualification required")?;
    if let Some(path) = prepare {
        if raw.is_some() || freeze.is_some() || output.is_some() {
            return Err("visual freeze preparation reads metadata only".into());
        }
        let f = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(
            f,
            &protocol(Path::new(&manifest), Path::new(&qualification), regression)?,
        )
        .map_err(|e| e.to_string())?;
        return Ok(true);
    }
    let report = evaluate(
        Path::new(&manifest),
        Path::new(&raw.ok_or("--raw required")?),
        Path::new(&freeze.ok_or("--freeze required")?),
        Path::new(&qualification),
        regression,
    )?;
    let output = output.ok_or("--output required")?;
    write_report(Path::new(&output), &report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    fn synthetic_metadata() -> (Vec<VisualFrame>, BTreeMap<String, Vec<u8>>) {
        let frames = [(0.00390625, 0.), (0.03515625, 0.03125)]
            .into_iter()
            .enumerate()
            .map(|(i, (depth, rgb))| VisualFrame {
                source_index: i,
                depth_file: format!("depth-{depth}.png"),
                depth_timestamp: depth,
                rgb_file: format!("rgb-{rgb}.png"),
                rgb_timestamp: rgb,
                rgb_source_index: i,
                pair_gap_seconds: depth - rgb,
                split: if i == 0 {
                    "initialization".into()
                } else {
                    "held_out".into()
                },
            })
            .collect();
        let metadata = BTreeMap::from([
            ("depth.txt".into(),b"0.00390625 depth/0.00390625.png\n0.03515625 depth/0.03515625.png\n".to_vec()),
            ("rgb.txt".into(),b"0 rgb/0.png\n0.03125 rgb/0.03125.png\n".to_vec()),
            ("groundtruth.txt".into(),b"0 opaque pose values stay unparsed before fits\n0.015625 opaque pose values stay unparsed before fits\n0.03125 opaque pose values stay unparsed before fits\n0.046875 opaque pose values stay unparsed before fits\n".to_vec()),
        ]);
        (frames, metadata)
    }
    fn assert_sensor_not_called(computed: Result<Value, String>, proof: &Value) {
        let reader = Cell::new(0);
        let features = Cell::new(0);
        let result = qualified_sensor(computed, proof, || {
            reader.set(reader.get() + 1);
            let pixels = vec![0u8; 35 * 35];
            features.set(features.get() + 1);
            let _ = extract_features(GrayImage::new(35, 35, &pixels).unwrap());
            Ok(())
        });
        assert!(result.is_err());
        assert_eq!(reader.get(), 0);
        assert_eq!(features.get(), 0);
    }
    #[test]
    fn eligibility_uses_all_original_patch_pixels_before_descriptor_ratio() {
        let camera = Calibration {
            width: WIDTH,
            height: HEIGHT,
            fx: 517.306408,
            fy: 516.469215,
            cx: 318.643040,
            cy: 255.313989,
            units_per_metre: 5000.,
            invalid_depth: 0,
        };
        let make = |x, bits| ImageFeature {
            x,
            y: 20.,
            score: 40,
            orientation: 0.,
            descriptor: [bits; 4],
        };
        let previous = [make(20., 0), make(40., u64::MAX)];
        let current = [make(20., 0), make(40., 1), make(60., u64::MAX)];
        let previous_depth: Vec<u8> = (0..WIDTH * HEIGHT)
            .flat_map(|_| 5000u16.to_be_bytes())
            .collect();
        for raw in [0u16, 5256, 1500 - 1] {
            let mut current_depth = previous_depth.clone();
            // Off-centre patch member: testing only feature centre would be wrong.
            let index = 2 * (19 * WIDTH as usize + 19);
            current_depth[index..index + 2].copy_from_slice(&raw.to_be_bytes());
            let previous_mask: Vec<_> = previous
                .iter()
                .map(|f| depth_point(&previous_depth, f, &camera).is_ok())
                .collect();
            let current_mask: Vec<_> = current
                .iter()
                .map(|f| depth_point(&current_depth, f, &camera).is_ok())
                .collect();
            assert_eq!(previous_mask, vec![true, true]);
            assert_eq!(current_mask, vec![false, true, true]);
            let matches =
                match_depth_supported(&previous, &current, &previous_mask, &current_mask).unwrap();
            assert!(
                matches.iter().any(|m| m.previous_index == 0
                    && m.current_index == 1
                    && m.hamming_distance == 4)
            );
        }
    }
    #[test]
    fn preparation_requires_viewed_regression_before_opening_manifest() {
        let result = protocol(
            Path::new("/nonexistent-supported-manifest"),
            Path::new("/nonexistent-supported-proof"),
            false,
        );
        assert_eq!(
            result.unwrap_err(),
            "depth-supported evaluation requires --regression; all selected images are viewed"
        );
    }
    #[test]
    fn prefix_tail_boundary_preserves_reference_then_expiry_latches_through_all_180_rows() {
        let mut accepted = AcceptedState::default();
        let mut observed = None;
        let mut lost = false;
        let mut initialized = 0;
        let mut rows = Vec::new();
        for offset in 0..180 {
            let source_index = 100 + offset;
            let stamp = offset as f64 * 0.03125;
            let mut row = json!({"accepted":false,"source_index":source_index});
            let gate = observe_clock(stamp, stamp, accepted.stamp, &mut observed, &mut lost);
            if gate.is_ok() {
                if offset <= 35 {
                    if accepted.reference.is_none() {
                        initialized += 1;
                    }
                    accepted.finish(
                        Ok(Pose3::identity()),
                        &mut row,
                        offset,
                        stamp,
                        vec![],
                        vec![],
                    );
                } else {
                    // Actual measured-pair backend rejection at the prefix/tail
                    // boundary must not silently create a new root or clock.
                    let failure = register_correspondences(&[], &VisualOdometry3dConfig::default())
                        .map(|fit| fit.pose);
                    accepted.finish(failure, &mut row, offset, stamp, vec![], vec![]);
                }
            } else {
                row["rejection"] = json!(gate.unwrap_err());
            }
            if source_index == 136 {
                assert_eq!(accepted.reference.as_ref().unwrap().0, 35);
                assert_eq!(accepted.stamp, Some(35. * 0.03125));
                assert!(!lost);
                assert_eq!(row["accepted"], false);
                assert!(row.get("root_estimate").is_none());
            }
            if source_index >= 142 {
                assert!(lost);
                assert_eq!(row["accepted"], false);
                assert!(row.get("root_estimate").is_none());
            }
            rows.push(row);
        }
        assert_eq!(rows.len(), 180);
        assert_eq!(initialized, 1);
        assert_eq!(accepted.reference.unwrap().0, 35);
        assert_eq!(accepted.stamp, Some(35. * 0.03125));
        assert_eq!(observed, Some(41. * 0.03125));
        assert!(lost);
    }
    #[test]
    fn qualification_inspects_timestamps_and_arity_without_parsing_pose_columns() {
        let (frames, metadata) = synthetic_metadata();
        let statistics = metadata_statistics(&frames, &metadata).unwrap();
        assert_eq!(
            statistics["maximum_observed_ground_truth_bracket_us"],
            15625
        );
        assert_eq!(statistics["maximum_pair_gap_us"], 3906);
        assert_eq!(statistics["ground_truth_rows"], 4);
        assert!(evaluation_ground_truth(&metadata["groundtruth.txt"]).is_err());
        let called = Cell::new(0);
        qualified_sensor(Ok(statistics.clone()), &statistics, || {
            called.set(1);
            Ok(())
        })
        .unwrap();
        assert_eq!(called.get(), 1);
    }
    #[test]
    fn wrong_proof_gt_hash_and_duplicate_timestamp_stop_actual_sensor_callback() {
        let (frames, mut metadata) = synthetic_metadata();
        let proof = metadata_statistics(&frames, &metadata).unwrap();
        let mut altered = proof.clone();
        altered["ground_truth_rows"] = json!(999);
        assert_sensor_not_called(metadata_statistics(&frames, &metadata), &altered);
        metadata.get_mut("groundtruth.txt").unwrap().extend_from_slice(b"60 opaque pose values stay unparsed before fits\n60 opaque pose values stay unparsed before fits\n");
        let failure = metadata_statistics(&frames, &metadata);
        assert!(
            failure
                .as_ref()
                .unwrap_err()
                .contains("not strictly increasing")
        );
        assert_sensor_not_called(failure, &proof);
        let mut descriptors = serde_json::Map::new();
        for (name, bytes) in &metadata {
            descriptors.insert(
                name.clone(),
                json!({"bytes":bytes.len(),"sha256":sha(bytes)}),
            );
        }
        descriptors.get_mut("groundtruth.txt").unwrap()["sha256"] = json!("0".repeat(64));
        let failure =
            verify_metadata_bytes(&Value::Object(descriptors), &metadata).map(|()| proof.clone());
        assert!(
            failure
                .as_ref()
                .unwrap_err()
                .contains("hash/size mismatch: groundtruth.txt")
        );
        assert_sensor_not_called(failure, &proof);
    }
    #[test]
    fn rounded_display_statistics_never_admit_a_bracket_over_the_unrounded_gate() {
        let (frames, mut metadata) = synthetic_metadata();
        metadata.insert("groundtruth.txt".into(),b"0 opaque pose values stay unparsed before fits\n0.0200001 opaque pose values stay unparsed before fits\n0.03125 opaque pose values stay unparsed before fits\n0.046875 opaque pose values stay unparsed before fits\n".to_vec());
        let failure = metadata_statistics(&frames, &metadata);
        assert!(
            failure
                .as_ref()
                .unwrap_err()
                .contains("bracket gate exceeded")
        );
        assert_sensor_not_called(failure, &json!({}));
        assert!(ground_truth_timestamps(b"0 a b c d e f g\nNaN a b c d e f g\n").is_err());
        assert!(ground_truth_timestamps(b"0 a b c\n1 a b c\n").is_err());
        assert!(strict_metadata_index(b"0 rgb/a.png\n0 rgb/b.png\n").is_err());
    }
    #[test]
    fn nearest_rgb_association_and_source_hashes_cannot_be_rebound() {
        let (mut frames, mut metadata) = synthetic_metadata();
        metadata.insert(
            "rgb.txt".into(),
            b"0 rgb/0.png\n0.001953125 rgb/0.001953125.png\n0.03125 rgb/0.03125.png\n".to_vec(),
        );
        frames[1].rgb_source_index = 2;
        let failure = metadata_statistics(&frames, &metadata);
        assert!(
            failure
                .as_ref()
                .unwrap_err()
                .contains("not original nearest")
        );
        assert_sensor_not_called(failure, &json!({}));
        let mut manifest: Value = serde_json::from_slice(include_bytes!(
            "../../../data/tum-fr1-room-temporal/manifest.json"
        ))
        .unwrap();
        metadata_descriptors(&manifest).unwrap();
        let item = manifest["files"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .find(|x| x["file"] == "groundtruth.txt")
            .unwrap();
        item["sha256"] = json!("0".repeat(64));
        assert!(metadata_descriptors(&manifest).is_err());
    }
}
