//! Recorded RGB-D relative-motion evaluation. Motion capture enters only after
//! the matcher has computed a pose from two depth frames and identity prior.
use rustdriving_core::Vec3;
use rustdriving_localization::registration3d::{
    Pose3, Quaternion, Registration3dConfig, match_scan,
};
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs,
    io::{Cursor, Read},
    path::{Component, Path},
    time::Instant,
};

mod keyframes;
mod motion;
mod submaps;

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const PIXEL_STEP: usize = 8;
const VOXEL_M: f64 = 0.03;
const MIN_DEPTH_M: f64 = 0.3;
const MAX_DEPTH_M: f64 = 5.0;
const GT_MAX_BRACKET_S: f64 = 0.02;
const TRANSLATION_GATE_M: f64 = 0.1;
const ROTATION_GATE_RAD: f64 = 0.1;

#[derive(Deserialize)]
struct Calibration {
    width: u32,
    height: u32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    units_per_metre: f64,
    invalid_depth: u16,
}
#[derive(Deserialize)]
struct Frame {
    file: String,
    timestamp: f64,
    source_index: usize,
    split: String,
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
    schema_version: u32,
    dataset: String,
    repository: String,
    revision: String,
    depth_calibration: Calibration,
    frames: Vec<Frame>,
    files: Vec<SourceFile>,
}
#[derive(Clone, Copy)]
struct GroundTruth {
    timestamp: f64,
    pose: Pose3,
}

fn sha(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn bounded_read(path: &Path, limit: usize) -> Result<Vec<u8>, String> {
    let file = fs::File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut bytes = Vec::new();
    file.take(limit as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > limit {
        return Err(format!("{} exceeds {limit}-byte limit", path.display()));
    }
    Ok(bytes)
}
fn safe_filename(name: &str) -> bool {
    let mut parts = Path::new(name).components();
    !name.contains(['/', '\\'])
        && matches!(parts.next(), Some(Component::Normal(_)))
        && parts.next().is_none()
}
fn validate_manifest(m: &Manifest) -> Result<(), String> {
    let c = &m.depth_calibration;
    if m.schema_version != 1
        || !matches!(
            m.dataset.as_str(),
            "tum-fr1-xyz"
                | "tum-fr1-xyz-fast"
                | "tum-fr1-xyz-tight"
                | "tum-fr1-xyz-motion"
                | "tum-fr1-xyz-motion-v2"
                | "tum-fr1-xyz-keyframes"
        )
        || m.repository.is_empty()
        || m.revision.len() != 40
        || !m.revision.bytes().all(|b| b.is_ascii_hexdigit())
        || c.width != WIDTH
        || c.height != HEIGHT
        || c.fx != 525.0
        || c.fy != 525.0
        || c.cx != 319.5
        || c.cy != 239.5
        || c.units_per_metre != 5000.0
        || c.invalid_depth != 0
        || m.frames.len()
            != if m.dataset == "tum-fr1-xyz-keyframes" {
                36
            } else {
                12
            }
        || m.files.len() > 64
        || m.frames.iter().enumerate().any(|(i, f)| {
            !safe_filename(&f.file)
                || !f.timestamp.is_finite()
                || f.file
                    .strip_suffix(".png")
                    .and_then(|s| s.parse::<f64>().ok())
                    != Some(f.timestamp)
                || f.source_index
                    != if m.dataset == "tum-fr1-xyz-keyframes" {
                        340 + i
                    } else if m.dataset == "tum-fr1-xyz-motion-v2" {
                        260 + i
                    } else if m.dataset == "tum-fr1-xyz-motion" {
                        200 + i
                    } else if m.dataset == "tum-fr1-xyz-tight" {
                        140 + i
                    } else if m.dataset == "tum-fr1-xyz-fast" {
                        120 + i
                    } else {
                        i * 10
                    }
                || f.split
                    != if m.dataset == "tum-fr1-xyz-keyframes" {
                        if i == 0 { "initialization" } else { "held_out" }
                    } else if i < 3 {
                        "calibration"
                    } else {
                        "held_out"
                    }
        })
        || m.frames
            .windows(2)
            .any(|f| f[1].timestamp <= f[0].timestamp || f[1].timestamp - f[0].timestamp > 0.5)
    {
        return Err("manifest does not match preregistered TUM selection/calibration".into());
    }
    let mut seen = BTreeSet::new();
    for f in &m.files {
        if !safe_filename(&f.file)
            || !seen.insert(&f.file)
            || f.bytes > 4 * 1024 * 1024
            || f.sha256.len() != 64
            || !f
                .sha256
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
        {
            return Err("invalid or duplicate manifest source file".into());
        }
    }
    if m.frames.iter().any(|f| {
        !m.files
            .iter()
            .any(|s| s.file == f.file && s.role == "depth_frame")
    }) || m
        .files
        .iter()
        .filter(|s| s.role == "evaluation_only_mocap_ground_truth")
        .count()
        != 1
    {
        return Err("missing depth frames or unique evaluation-only ground truth".into());
    }
    Ok(())
}
fn validate_depth_index(bytes: &[u8], frames: &[Frame]) -> Result<(), String> {
    let text = std::str::from_utf8(bytes).map_err(|e| e.to_string())?;
    let rows: Vec<_> = text
        .lines()
        .map(str::trim)
        .filter(|s| !s.is_empty() && !s.starts_with('#'))
        .collect();
    for frame in frames {
        let row = rows
            .get(frame.source_index)
            .ok_or("selected depth index is missing")?;
        let fields: Vec<_> = row.split_whitespace().collect();
        if fields.len() != 2
            || fields[0].parse::<f64>().ok() != Some(frame.timestamp)
            || fields[1] != format!("depth/{}", frame.file)
        {
            return Err("frame selection differs from SHA-verified original depth index".into());
        }
    }
    Ok(())
}
fn depth_cloud(bytes: &[u8]) -> Result<Vec<Vec3>, String> {
    if bytes.len() > 4 * 1024 * 1024 {
        return Err("depth PNG exceeds byte limit".into());
    }
    let mut decoder = png::Decoder::new(Cursor::new(bytes));
    decoder.set_limits(png::Limits {
        bytes: 8 * 1024 * 1024,
    });
    decoder.ignore_checksums(false);
    decoder.set_transformations(png::Transformations::IDENTITY);
    let mut reader = decoder
        .read_info()
        .map_err(|e| format!("depth PNG header: {e}"))?;
    let info = reader.info();
    if info.width != WIDTH
        || info.height != HEIGHT
        || info.color_type != png::ColorType::Grayscale
        || info.bit_depth != png::BitDepth::Sixteen
        || info.animation_control.is_some()
    {
        return Err("expected static 640x480 grayscale uint16 depth PNG".into());
    }
    let len = reader.output_buffer_size().ok_or("depth buffer overflow")?;
    if len != (WIDTH * HEIGHT * 2) as usize {
        return Err("unexpected depth buffer layout".into());
    }
    let mut buffer = vec![0; len];
    let output = reader
        .next_frame(&mut buffer)
        .map_err(|e| format!("depth PNG body: {e}"))?;
    if output.buffer_size() != len {
        return Err("incomplete depth frame".into());
    }
    reader
        .finish()
        .map_err(|e| format!("depth PNG ending/CRC: {e}"))?;
    let mut voxels = BTreeMap::new();
    for y in (0..HEIGHT as usize).step_by(PIXEL_STEP) {
        for x in (0..WIDTH as usize).step_by(PIXEL_STEP) {
            let at = 2 * (y * WIDTH as usize + x);
            let raw = u16::from_be_bytes([buffer[at], buffer[at + 1]]);
            let z = raw as f64 / 5000.0;
            if raw == 0 || !(MIN_DEPTH_M..=MAX_DEPTH_M).contains(&z) {
                continue;
            }
            let p = Vec3::new(
                (x as f64 - 319.5) * z / 525.0,
                (y as f64 - 239.5) * z / 525.0,
                z,
            );
            let key = (
                (p.x / VOXEL_M).floor() as i64,
                (p.y / VOXEL_M).floor() as i64,
                (p.z / VOXEL_M).floor() as i64,
            );
            voxels.entry(key).or_insert(p);
        }
    }
    // BTree ordering makes point and nearest-neighbor tie ordering repeatable.
    Ok(voxels.into_values().collect())
}
fn ground_truth(bytes: &[u8]) -> Result<Vec<GroundTruth>, String> {
    if bytes.len() > 1024 * 1024 {
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
        if result.len() > 20_000 {
            return Err("too many GT poses".into());
        }
    }
    if result.len() < 2 {
        return Err("insufficient ground truth".into());
    }
    Ok(result)
}
fn interpolate(gt: &[GroundTruth], stamp: f64) -> Result<Pose3, String> {
    if !stamp.is_finite() {
        return Err("invalid evaluation timestamp".into());
    }
    let right = gt.partition_point(|g| g.timestamp < stamp);
    if right < gt.len() && gt[right].timestamp == stamp {
        return Ok(gt[right].pose);
    }
    if right == 0 || right == gt.len() {
        return Err("GT cannot bracket timestamp; extrapolation forbidden".into());
    }
    let a = gt[right - 1];
    let b = gt[right];
    let gap = b.timestamp - a.timestamp;
    if gap > GT_MAX_BRACKET_S || gap <= 0.0 {
        return Err("GT interpolation gap exceeds 0.02 seconds".into());
    }
    let t = (stamp - a.timestamp) / gap;
    let qa = a.pose.rotation;
    let mut qb = b.pose.rotation;
    let mut dot = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
    if dot < 0.0 {
        dot = -dot;
        qb = Quaternion {
            w: -qb.w,
            x: -qb.x,
            y: -qb.y,
            z: -qb.z,
        };
    }
    let (sa, sb) = if dot > 0.9995 {
        (1.0 - t, t)
    } else {
        let angle = dot.clamp(-1.0, 1.0).acos();
        (
            ((1.0 - t) * angle).sin() / angle.sin(),
            (t * angle).sin() / angle.sin(),
        )
    };
    let rotation = Quaternion {
        w: sa * qa.w + sb * qb.w,
        x: sa * qa.x + sb * qb.x,
        y: sa * qa.y + sb * qb.y,
        z: sa * qa.z + sb * qb.z,
    }
    .normalized()?;
    let pa = a.pose.translation;
    let pb = b.pose.translation;
    Ok(Pose3 {
        translation: Vec3::new(
            pa.x + (pb.x - pa.x) * t,
            pa.y + (pb.y - pa.y) * t,
            pa.z + (pb.z - pa.z) * t,
        ),
        rotation,
    })
}
fn pose_json(p: Pose3) -> Value {
    json!({"translation_m":[p.translation.x,p.translation.y,p.translation.z],"quaternion_wxyz":[p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z]})
}
fn config_json(c: &Registration3dConfig) -> Value {
    json!({
        "max_scan_points":c.max_scan_points,"max_map_points":c.max_map_points,"max_iterations":c.max_iterations,"max_neighbor_checks":c.max_neighbor_checks,
        "max_correspondence_m":c.max_correspondence_m,"trim_fraction":c.trim_fraction,"min_pairs":c.min_pairs,"min_overlap":c.min_overlap,
        "max_translation_jump_m":c.max_translation_jump_m,"max_rotation_jump_rad":c.max_rotation_jump_rad,"max_rms_m":c.max_rms_m,
        "min_geometry_ratio":c.min_geometry_ratio,"max_condition_number":c.max_condition_number,"max_position_variance_m2":c.max_position_variance_m2,
        "max_rotation_variance_rad2":c.max_rotation_variance_rad2,"translation_tolerance_m":c.translation_tolerance_m,"rotation_tolerance_rad":c.rotation_tolerance_rad,
        "ambiguity_translation_probe_m":c.ambiguity_translation_probe_m,"ambiguity_rotation_probe_rad":c.ambiguity_rotation_probe_rad,"ambiguity_rms_ratio":c.ambiguity_rms_ratio
    })
}
fn preprocessing_json() -> Value {
    json!({"width":WIDTH,"height":HEIGHT,"fx":525.0,"fy":525.0,"cx":319.5,"cy":239.5,"units_per_metre":5000,"pixel_step":PIXEL_STEP,"sample_origin_pixel":[0,0],"min_depth_m":MIN_DEPTH_M,"max_depth_m":MAX_DEPTH_M,"voxel_m":VOXEL_M,"voxel_representative":"first row-major sampled valid pixel","point_order":"lexicographic voxel key","maximum_sampled_pixels":4800})
}
fn protocol(manifest_path: &Path) -> Result<Value, String> {
    let bytes = bounded_read(manifest_path, 128 * 1024)?;
    let m: Manifest = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    validate_manifest(&m)?;
    let registration_config = config_json(&Registration3dConfig::default());
    let preprocessing = preprocessing_json();
    let config_hash = sha(&serde_json::to_vec(&registration_config).map_err(|e| e.to_string())?);
    let preprocessing_hash = sha(&serde_json::to_vec(&preprocessing).map_err(|e| e.to_string())?);
    let kind = if m.dataset == "tum-fr1-xyz-tight" {
        "preregistered_temporal"
    } else if m.dataset == "tum-fr1-xyz-fast" {
        "calibration_regression"
    } else {
        "baseline_retrospective"
    };
    Ok(
        json!({"schema_version":1,"protocol_version":2,"dataset":m.dataset,"manifest_sha256":sha(&bytes),
        "matcher_source_sha256":sha(include_bytes!("../../../crates/localization/src/registration3d.rs")),
        "evaluator_source_sha256":sha(include_bytes!("main.rs")),"cargo_lock_sha256":sha(include_bytes!("../Cargo.lock")),
            "kind":kind,"preprocessing":preprocessing,"registration_config":registration_config,
            "registration_config_sha256":config_hash,"preprocessing_sha256":preprocessing_hash,
        "accuracy_gates":{"translation_m":TRANSLATION_GATE_M,"rotation_rad":ROTATION_GATE_RAD},
        "ground_truth_interpolation":{"method":"linear translation and shortest-arc quaternion SLERP","max_bracket_s":GT_MAX_BRACKET_S,"extrapolation":false},
        "frames":m.frames.iter().map(|f|json!({"file":f.file,"timestamp":f.timestamp,"source_index":f.source_index,"split":f.split})).collect::<Vec<_>>(),
        "freeze_preparation":"metadata only; raw depth and mocap are not read and registration is not invoked"}),
    )
}
fn verify_freeze(manifest_path: &Path, freeze_path: &Path) -> Result<(Value, String), String> {
    let bytes = bounded_read(freeze_path, 128 * 1024)?;
    let value: Value = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    if value != protocol(manifest_path)? {
        return Err("external freeze differs from compiled sources, manifest, configuration or preprocessing; evaluation forbidden".into());
    }
    Ok((value, sha(&bytes)))
}
fn evaluate(manifest_path: &Path, raw: &Path, freeze_path: &Path) -> Result<Value, String> {
    // An externally saved protocol must match before any raw sensor/GT access.
    let (freeze, freeze_hash) = verify_freeze(manifest_path, freeze_path)?;
    let manifest_bytes = bounded_read(manifest_path, 128 * 1024)?;
    let manifest: Manifest = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    validate_manifest(&manifest)?;
    if manifest.dataset == "tum-fr1-xyz-keyframes" {
        return Err("keyframe selection requires --keyframes mode".into());
    }
    let mut source_bytes = BTreeMap::new();
    let mut provenance = Vec::new();
    for file in &manifest.files {
        let bytes = bounded_read(&raw.join(&file.file), 4 * 1024 * 1024)?;
        let digest = sha(&bytes);
        if bytes.len() != file.bytes || digest != file.sha256 {
            return Err(format!("source bytes/SHA-256 mismatch: {}", file.file));
        }
        provenance
            .push(json!({"file":file.file,"bytes":bytes.len(),"sha256":digest,"role":file.role}));
        source_bytes.insert(file.file.clone(), bytes);
    }
    let cfg = Registration3dConfig::default();
    let index = manifest
        .files
        .iter()
        .find(|f| f.role == "depth_index")
        .ok_or("missing original depth index")?;
    validate_depth_index(&source_bytes[&index.file], &manifest.frames)?;
    let mut clouds = Vec::new();
    for frame in &manifest.frames {
        clouds.push(depth_cloud(&source_bytes[&frame.file])?);
    }
    // Compute all sensor-only matches first. Motion capture is parsed afterward;
    // neither pose labels nor evaluation outcomes can influence correspondence,
    // initial guesses, point sampling, pair selection or configuration.
    let mut fits = Vec::new();
    for points in clouds.windows(2) {
        let start = Instant::now();
        let result = match_scan(&points[1], &points[0], Pose3::identity(), &cfg);
        fits.push((result, start.elapsed().as_secs_f64()));
    }
    let gt_file = manifest
        .files
        .iter()
        .find(|f| f.role == "evaluation_only_mocap_ground_truth")
        .unwrap();
    let gt = ground_truth(&source_bytes[&gt_file.file])?;
    let mut pairs = Vec::new();
    let mut passed = 0;
    let mut accepted = 0;
    let mut heldout_passed = 0;
    let mut heldout_pairs = 0;
    for (i, (fit, seconds)) in fits.into_iter().enumerate() {
        let previous = &manifest.frames[i];
        let current = &manifest.frames[i + 1];
        let truth = interpolate(&gt, previous.timestamp)?
            .inverse()
            .compose(interpolate(&gt, current.timestamp)?);
        let heldout = current.split == "held_out";
        if heldout {
            heldout_pairs += 1;
        }
        let mut row = json!({"previous_file":previous.file,"current_file":current.file,"previous_timestamp":previous.timestamp,"current_timestamp":current.timestamp,"interval_s":current.timestamp-previous.timestamp,"split":current.split,"map_points":clouds[i].len(),"scan_points":clouds[i+1].len(),"initial_pose":"identity; no current motion-capture pose supplied","evaluation_only_relative_truth":pose_json(truth),"cpu_wall_seconds":seconds});
        match fit {
            Ok(r) => {
                accepted += 1;
                let delta = Vec3::new(
                    r.pose.translation.x - truth.translation.x,
                    r.pose.translation.y - truth.translation.y,
                    r.pose.translation.z - truth.translation.z,
                );
                let translation_error = delta.x.hypot(delta.y).hypot(delta.z);
                let rotation_error = r.pose.rotation.angular_distance(truth.rotation);
                let ok =
                    translation_error <= TRANSLATION_GATE_M && rotation_error <= ROTATION_GATE_RAD;
                if ok {
                    passed += 1;
                    if heldout {
                        heldout_passed += 1;
                    }
                }
                row["accepted"] = json!(true);
                row["within_accuracy_gates"] = json!(ok);
                row["estimate"] = pose_json(r.pose);
                row["translation_error_m"] = json!(translation_error);
                row["rotation_error_rad"] = json!(rotation_error);
                row["rms_m"] = json!(r.rms_m);
                row["inlier_fraction"] = json!(r.inlier_fraction);
                row["inlier_count"] = json!(r.inlier_count);
                row["iterations"] = json!(r.iterations);
                row["converged"] = json!(r.converged);
                row["conditional_covariance_xyz_rotation"] = json!(r.covariance);
                row["geometry_ratio"] = json!(r.conditioning.geometry_ratio);
                row["condition_number"] = json!(r.conditioning.condition_number);
                row["neighbor_checks"] = json!(r.conditioning.neighbor_checks);
                row["ambiguity_probes"] = json!(r.conditioning.ambiguity_probes);
            }
            Err(e) => {
                row["accepted"] = json!(false);
                row["within_accuracy_gates"] = json!(false);
                row["rejection"] = json!(e);
            }
        }
        pairs.push(row);
    }
    Ok(
        json!({"schema_version":1,"dataset":manifest.dataset,"repository":manifest.repository,"revision":manifest.revision,"manifest_sha256":sha(&manifest_bytes),
        "matcher_source_sha256":sha(include_bytes!("../../../crates/localization/src/registration3d.rs")),"evaluator_source_sha256":sha(include_bytes!("main.rs")),"cargo_lock_sha256":sha(include_bytes!("../Cargo.lock")),
        "freeze_sha256":freeze_hash,"evaluation_role":freeze["kind"],"freeze_json":freeze,"preprocessing":preprocessing_json(),
        "registration_config":config_json(&cfg),"accuracy_gates":{"translation_m":TRANSLATION_GATE_M,"rotation_rad":ROTATION_GATE_RAD},"ground_truth_interpolation":{"method":"linear translation and shortest-arc quaternion SLERP","max_bracket_s":GT_MAX_BRACKET_S,"extrapolation":false},
        "selection":"all 11 consecutive pairs from the 12 preregistered depth frames; no pair omission; current frame determines split","files":provenance,"pairs":pairs,"summary":{"pairs":11,"accepted":accepted,"passed":passed,"rejected":11-accepted,"heldout_pairs":heldout_pairs,"heldout_passed":heldout_passed,"all_pairs_passed":passed==11},
        "limitations":["Temporal held-out frames share one room and acquisition; no independent-scene generalization.","Calibration is the upstream registered-depth teaching reader model, not measured vehicle sensor calibration.","Local point-to-point pair registration only; no camera AI, global localization, loop closure, SLAM, driving or covariance calibration.","Conditional covariance excludes map error, depth correlation and association uncertainty.","Recorded mocap is evaluation-only; failed pairs remain in the report."]}),
    )
}
fn run() -> Result<bool, String> {
    let mut manifest = "data/tum-fr1-xyz/manifest.json".to_string();
    let mut raw = "data/tum-fr1-xyz/raw".to_string();
    let mut output = "artifacts/tum-rgbd/results.json".to_string();
    let mut freeze = None;
    let mut prepare = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| format!("missing value for {arg}"))?;
        match arg.as_str() {
            "--manifest" => manifest = value,
            "--raw" => raw = value,
            "--output" => output = value,
            "--freeze" => freeze = Some(value),
            "--prepare-freeze" => prepare = Some(value),
            _ => return Err(format!("unknown argument {arg}")),
        }
    }
    if let Some(path) = prepare {
        if freeze.is_some() {
            return Err("choose either freeze preparation or evaluation".into());
        }
        let protocol = protocol(Path::new(&manifest))?;
        let file = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|e| format!("freeze {}: {e}", path))?;
        serde_json::to_writer_pretty(file, &protocol).map_err(|e| e.to_string())?;
        println!(
            "Saved metadata-only protocol freeze: {path}. No depth decoding, motion-capture parsing or registration performed."
        );
        return Ok(true);
    }
    let freeze=freeze.ok_or("--freeze FILE is mandatory; first create an external protocol with --prepare-freeze FILE before evaluating raw data")?;
    let report = evaluate(Path::new(&manifest), Path::new(&raw), Path::new(&freeze))?;
    let passed = report["summary"]["all_pairs_passed"].as_bool().unwrap();
    if let Some(parent) = Path::new(&output).parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    fs::write(
        &output,
        serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    println!("{}; report: {output}", report["summary"]);
    Ok(passed)
}
fn main() {
    match if std::env::args().any(|a| a == "--submaps") {
        submaps::run()
    } else if std::env::args().any(|a| a == "--keyframes") {
        keyframes::run()
    } else if std::env::args().any(|a| a == "--motion") {
        motion::run()
    } else {
        run()
    } {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(e) => {
            eprintln!("rgbd evaluation: {e}");
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn png_bytes(depth: png::BitDepth, width: u32, height: u32) -> Vec<u8> {
        let mut data = Vec::new();
        {
            let mut encoder = png::Encoder::new(&mut data, width, height);
            encoder.set_color(png::ColorType::Grayscale);
            encoder.set_depth(depth);
            let mut writer = encoder.write_header().unwrap();
            let bytes = if depth == png::BitDepth::Sixteen {
                [0x27, 0x10].repeat((width * height) as usize)
            } else {
                vec![1; (width * height) as usize]
            };
            writer.write_image_data(&bytes).unwrap();
        }
        data
    }
    #[test]
    fn native_uint16_depth_preserves_big_endian_metres_and_bounded_sampling() {
        let bytes = png_bytes(png::BitDepth::Sixteen, WIDTH, HEIGHT);
        let cloud = depth_cloud(&bytes).unwrap();
        assert!(!cloud.is_empty() && cloud.len() <= 4800);
        assert!(cloud.iter().all(|p| p.z == 2.0));
        assert!(
            cloud
                .iter()
                .any(|p| (p.x - (-319.5 * 2.0 / 525.0)).abs() < 1e-12)
        );
    }
    #[test]
    fn corrupt_crc_wrong_depth_size_and_truncation_are_rejected() {
        let bytes = png_bytes(png::BitDepth::Sixteen, WIDTH, HEIGHT);
        let mut crc = bytes.clone();
        crc[29] ^= 1;
        assert!(depth_cloud(&crc).is_err());
        assert!(depth_cloud(&bytes[..bytes.len() - 8]).is_err());
        assert!(depth_cloud(&png_bytes(png::BitDepth::Eight, WIDTH, HEIGHT)).is_err());
        assert!(depth_cloud(&png_bytes(png::BitDepth::Sixteen, WIDTH - 1, HEIGHT)).is_err());
    }
    #[test]
    fn quaternion_timestamp_and_extrapolation_fail_closed() {
        assert!(ground_truth(b"1 0 0 0 0 0 0 0\n2 0 0 0 0 0 0 1\n").is_err());
        assert!(ground_truth(b"1 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n").is_err());
        let gt = ground_truth(b"1 0 0 0 0 0 0 1\n1.01 1 0 0 0 0 0 -1\n").unwrap();
        let p = interpolate(&gt, 1.005).unwrap();
        assert!((p.translation.x - 0.5).abs() < 1e-12);
        assert!(p.rotation.angular_distance(Quaternion::identity()) < 1e-12);
        assert!(interpolate(&gt, 0.99).is_err());
        assert!(interpolate(&gt, 1.02).is_err());
        let bad = ground_truth(b"1 0 0 0 0 0 0 1\n1.1 1 0 0 0 0 0 1\n").unwrap();
        assert!(interpolate(&bad, 1.05).unwrap_err().contains("gap"));
    }
    #[test]
    fn mocap_slerp_and_relative_transform_have_scan_to_previous_orientation() {
        let a = GroundTruth {
            timestamp: 1.0,
            pose: Pose3::identity(),
        };
        let b = GroundTruth {
            timestamp: 1.01,
            pose: Pose3 {
                translation: Vec3::new(0.2, 0.1, -0.1),
                rotation: Quaternion::from_axis_angle(
                    Vec3::new(0.0, 0.0, 1.0),
                    std::f64::consts::FRAC_PI_2,
                )
                .unwrap(),
            },
        };
        let half = interpolate(&[a, b], 1.005).unwrap();
        let expected =
            Quaternion::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f64::consts::FRAC_PI_4)
                .unwrap();
        assert!(half.rotation.angular_distance(expected) < 1e-12);
        let p = Vec3::new(1.0, 2.0, 3.0);
        let relative = half.inverse().compose(b.pose);
        let direct = half.inverse().transform(b.pose.transform(p));
        let composed = relative.transform(p);
        assert!((composed.x - direct.x).abs() < 1e-12);
        assert!((composed.y - direct.y).abs() < 1e-12);
        assert!((composed.z - direct.z).abs() < 1e-12);
        assert!(relative.rotation.angular_distance(expected) < 1e-12);
    }
    #[test]
    fn original_depth_index_prevents_relabeling_or_reordering_frames() {
        let frame = Frame {
            file: "1.000000.png".into(),
            timestamp: 1.0,
            source_index: 0,
            split: "calibration".into(),
        };
        assert!(validate_depth_index(b"# source\n1 depth/1.000000.png\n", &[frame]).is_ok());
        let changed = Frame {
            file: "1.000000.png".into(),
            timestamp: 1.01,
            source_index: 0,
            split: "calibration".into(),
        };
        assert!(validate_depth_index(b"1 depth/1.000000.png\n", &[changed]).is_err());
        assert!(!safe_filename("../groundtruth.txt"));
        assert!(!safe_filename("..\\groundtruth.txt"));
        assert!(!safe_filename("/groundtruth.txt"));
    }
    #[test]
    fn keyframe_selection_is_exact_and_chronological_without_depth_access() {
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/tum-fr1-xyz-fast/manifest.json");
        let mut metadata: Value = serde_json::from_slice(&fs::read(path).unwrap()).unwrap();
        metadata["dataset"] = json!("tum-fr1-xyz-keyframes");
        let prototype = metadata["files"]
            .as_array()
            .unwrap()
            .iter()
            .find(|f| f["role"] == "depth_frame")
            .unwrap()
            .clone();
        let mut frames = Vec::new();
        let mut files: Vec<_> = metadata["files"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|f| f["role"] != "depth_frame")
            .cloned()
            .collect();
        for i in 0..36 {
            let timestamp = 100.0 + i as f64 * 0.033;
            let file = format!("{timestamp:.6}.png");
            let timestamp = file.strip_suffix(".png").unwrap().parse::<f64>().unwrap();
            frames.push(json!({"file":file,"timestamp":timestamp,
                "source_index":340+i,"split":if i==0 {"initialization"} else {"held_out"}}));
            let mut source = prototype.clone();
            source["file"] = json!(file);
            files.push(source);
        }
        metadata["frames"] = json!(frames);
        metadata["files"] = json!(files);
        let validate = |value: &Value| {
            validate_manifest(&serde_json::from_value::<Manifest>(value.clone()).unwrap())
        };
        assert!(validate(&metadata).is_ok());
        let mut changed = metadata.clone();
        changed["frames"][1] = changed["frames"][0].clone();
        assert!(validate(&changed).is_err());
        let mut changed = metadata.clone();
        changed["frames"].as_array_mut().unwrap().swap(1, 2);
        assert!(validate(&changed).is_err());
        let mut changed = metadata.clone();
        changed["frames"].as_array_mut().unwrap().pop();
        assert!(validate(&changed).is_err());
        let mut changed = metadata.clone();
        changed["frames"][1]["split"] = json!("calibration");
        assert!(validate(&changed).is_err());
    }
    #[test]
    fn external_freeze_changes_are_rejected_before_sensor_access() {
        let manifest =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/tum-fr1-xyz/manifest.json");
        let mut value = protocol(&manifest).unwrap();
        let tmp = std::env::temp_dir().join(format!(
            "rustdriving-rgbd-freeze-test-{}.json",
            std::process::id()
        ));
        fs::write(&tmp, serde_json::to_vec(&value).unwrap()).unwrap();
        assert!(verify_freeze(&manifest, &tmp).is_ok());
        value["registration_config"]["min_overlap"] = json!(0.1);
        fs::write(&tmp, serde_json::to_vec(&value).unwrap()).unwrap();
        let error = evaluate(
            &manifest,
            Path::new("/this/sensor/directory/does/not/exist"),
            &tmp,
        )
        .unwrap_err();
        assert!(error.contains("external freeze differs"));
        fs::remove_file(&tmp).unwrap();
    }
}
