//! Recorded-depth localization against a bounded fused measured map. Every
//! operational observation finishes before any evaluation-only mocap parsing.
use super::*;
use rustdriving_localization::submap3d::{SubmapConfig3d, SubmapLocalizer3d};

const FRESH_DATASET: &str = "tum-fr3-office-submaps";
const VIEWED_DESK_DATASET: &str = "tum-fr1-desk-submaps";
fn calibrated_dataset(dataset: &str) -> bool {
    dataset == FRESH_DATASET || dataset == VIEWED_DESK_DATASET
}
fn validate_calibration_source(metadata: &Value) -> Result<(), String> {
    let (source_path, bytes, digest) = match metadata["dataset"].as_str() {
        Some(VIEWED_DESK_DATASET) => (
            "settings/TUM1.yaml",
            1615,
            "5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf",
        ),
        Some(FRESH_DATASET) => (
            "settings/TUM3.yaml",
            1520,
            "251e345996befa8057f5c51642bd4fe93a92907dac2e0564f400cd87eaf5785d",
        ),
        _ => return Err("unsupported precise calibration profile".into()),
    };
    if metadata["calibration_source"]
        != json!({"repository":"luigifreda/pyslam","revision":"96019cfafcfc099ac9866884d7143a9ed1451a0d","source_path":source_path,"file":"camera-calibration.yaml","bytes":bytes,"sha256":digest,"role":"source_calibration_documentation_only"})
        || metadata["redistribute_raw"] != false
    {
        return Err("camera calibration provenance differs from pinned source".into());
    }
    Ok(())
}
fn validate_submap_manifest(m: &Manifest) -> Result<(), String> {
    if !calibrated_dataset(&m.dataset) {
        return validate_manifest(m);
    }
    let (repository, revision, fx, fy, cx, cy) = if m.dataset == FRESH_DATASET {
        (
            "shihaozhaosiue/SLAM-project_shihao",
            "1f3bb58bbcbad2ec405c36d6d5a511d2f1cd050b",
            535.4,
            539.2,
            320.1,
            247.6,
        )
    } else {
        (
            "FaridRash/slam-track-fusion",
            "477a059d640540b7e23fd56ec95f6458167c7af2",
            517.306408,
            516.469215,
            318.643040,
            255.313989,
        )
    };
    let c = &m.depth_calibration;
    if m.schema_version != 1
        || m.repository != repository
        || m.revision != revision
        || c.width != WIDTH
        || c.height != HEIGHT
        || c.fx != fx
        || c.fy != fy
        || c.cx != cx
        || c.cy != cy
        || c.units_per_metre != 5000.0
        || c.invalid_depth != 0
        || m.frames.len() != 36
        || m.files.len() != 38
        || m.frames.iter().enumerate().any(|(i, f)| {
            !safe_filename(&f.file)
                || !f.timestamp.is_finite()
                || f.file
                    .strip_suffix(".png")
                    .and_then(|s| s.parse::<f64>().ok())
                    != Some(f.timestamp)
                || f.source_index != 100 + i
                || f.split != if i == 0 { "initialization" } else { "held_out" }
        })
        || m.frames
            .windows(2)
            .any(|f| f[1].timestamp <= f[0].timestamp || f[1].timestamp - f[0].timestamp > 0.5)
    {
        return Err("submap manifest differs from pinned recorded selection/calibration".into());
    }
    let mut seen = BTreeSet::new();
    for f in &m.files {
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
            return Err("invalid or duplicate submap source file".into());
        }
    }
    if m.frames.iter().any(|f| {
        !m.files
            .iter()
            .any(|s| s.file == f.file && s.role == "depth_frame")
    }) || m
        .files
        .iter()
        .filter(|s| s.file == "groundtruth.txt" && s.role == "evaluation_only_mocap_ground_truth")
        .count()
        != 1
        || m.files
            .iter()
            .filter(|s| s.file == "depth.txt" && s.role == "depth_index")
            .count()
            != 1
    {
        return Err("submap source inventory lacks unique depth index or evaluation labels".into());
    }
    Ok(())
}
fn measured_cloud(bytes: &[u8], m: &Manifest) -> Result<Vec<Vec3>, String> {
    if !calibrated_dataset(&m.dataset) {
        return motion::cloud(bytes);
    }
    let mut voxels = BTreeMap::new();
    for p in calibrated_depth_cloud(bytes, &m.depth_calibration)? {
        let key = (
            (p.x / 0.06).floor() as i64,
            (p.y / 0.06).floor() as i64,
            (p.z / 0.06).floor() as i64,
        );
        voxels.entry(key).or_insert(p);
    }
    Ok(voxels.into_values().collect())
}

fn calibrated_depth_cloud(bytes: &[u8], calibration: &Calibration) -> Result<Vec<Vec3>, String> {
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
            let z = raw as f64 / calibration.units_per_metre;
            if raw == calibration.invalid_depth || !(MIN_DEPTH_M..=MAX_DEPTH_M).contains(&z) {
                continue;
            }
            let p = Vec3::new(
                (x as f64 - calibration.cx) * z / calibration.fx,
                (y as f64 - calibration.cy) * z / calibration.fy,
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

fn config() -> SubmapConfig3d {
    SubmapConfig3d {
        registration: Registration3dConfig {
            max_map_points: 5000,
            ..motion::config()
        },
        voxel_m: 0.06,
        map_update_interval_s: 0.10,
        max_unobserved_s: 0.20,
        max_points_per_voxel: 1000,
    }
}
fn protocol(path: &Path, regression: bool) -> Result<Value, String> {
    let manifest_bytes = bounded_read(path, 128 * 1024)?;
    let m: Manifest = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    validate_submap_manifest(&m)?;
    let metadata: Value = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    if calibrated_dataset(&m.dataset) {
        validate_calibration_source(&metadata)?;
    }
    let mut value = if calibrated_dataset(&m.dataset) {
        let c = &m.depth_calibration;
        let mut preprocessing = preprocessing_json();
        preprocessing["fx"] = json!(c.fx);
        preprocessing["fy"] = json!(c.fy);
        preprocessing["cx"] = json!(c.cx);
        preprocessing["cy"] = json!(c.cy);
        let preprocessing_hash =
            sha(&serde_json::to_vec(&preprocessing).map_err(|e| e.to_string())?);
        json!({"schema_version":1,"dataset":m.dataset,"manifest_sha256":sha(&manifest_bytes),
            "matcher_source_sha256":sha(include_bytes!("../../../crates/localization/src/registration3d.rs")),
            "evaluator_source_sha256":sha(include_bytes!("main.rs")),"cargo_lock_sha256":sha(include_bytes!("../Cargo.lock")),
            "preprocessing":preprocessing,"preprocessing_sha256":preprocessing_hash,
            "accuracy_gates":{"translation_m":TRANSLATION_GATE_M,"rotation_rad":ROTATION_GATE_RAD},
            "ground_truth_interpolation":{"method":"linear translation and shortest-arc quaternion SLERP","max_bracket_s":GT_MAX_BRACKET_S,"extrapolation":false},
            "frames":m.frames.iter().map(|f|json!({"file":f.file,"timestamp":f.timestamp,"source_index":f.source_index,"split":f.split})).collect::<Vec<_>>(),
            "freeze_preparation":"metadata only; raw depth and mocap are not read and registration is not invoked"})
    } else {
        super::protocol(path)?
    };
    let cfg = config();
    value["protocol_version"] = json!(8);
    value["regression_requested"] = json!(regression);
    value["kind"] = json!(if m.dataset == FRESH_DATASET && !regression {
        "preregistered_sequence"
    } else {
        "calibration_regression"
    });
    if calibrated_dataset(&m.dataset) {
        for field in ["depth_calibration", "calibration_source"] {
            value[field] = metadata[field].clone();
            value[format!("{field}_sha256")] = json!(sha(
                &serde_json::to_vec(&value[field]).map_err(|e| e.to_string())?
            ));
        }
    }
    value["submap_source_sha256"] = json!(sha(include_bytes!("submaps.rs")));
    value["motion_source_sha256"] = json!(sha(include_bytes!("motion.rs")));
    value["submap_core_source_sha256"] = json!(sha(include_bytes!(
        "../../../crates/localization/src/submap3d.rs"
    )));
    value["localization_lib_source_sha256"] = json!(sha(include_bytes!(
        "../../../crates/localization/src/lib.rs"
    )));
    value["core_lib_source_sha256"] = json!(sha(include_bytes!("../../../crates/core/src/lib.rs")));
    value["workspace_cargo_lock_sha256"] = json!(sha(include_bytes!("../../../Cargo.lock")));
    value["rust_toolchain_sha256"] = json!(sha(include_bytes!("../../../rust-toolchain.toml")));
    value["cargo_manifest_sha256"] = json!(sha(include_bytes!("../Cargo.toml")));
    value["independent_checker_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/check-recorded-submaps.py"
    )));
    value["acquisition_source_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/fetch-submap-datasets.py"
    )));
    value["geometry_checker_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/check-recorded-rgbd.py"
    )));
    value["keyframe_checker_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/check-recorded-keyframes.py"
    )));
    value["registration_config"] = config_json(&cfg.registration);
    for field in [
        "registration_config",
        "submap_policy",
        "motion_preprocessing",
    ] {
        if field == "submap_policy" {
            value[field] = json!({
                "voxel_m":cfg.voxel_m,
                "map_update_interval_s":cfg.map_update_interval_s,
                "max_unobserved_s":cfg.max_unobserved_s,
                "max_points_per_voxel":cfg.max_points_per_voxel,
                "maximum_map_points":cfg.registration.max_map_points,
                "initialization":"first measured cloud self-registers at identity with all numerical guards; then atomic identity fusion",
                "frame":"fixed initial measured depth camera; all map representatives and accepted poses remain in this root",
                "prior":"last accepted root pose, initially identity; no velocity or mocap prior",
                "fusion":"transform measured scan into root; floor xyz/voxel size; lexicographic voxel keys; sequential online mean and raw-observation count in input point order",
                "update":"atomic proposed copy; only accepted fits at minimum interval; any voxel, coordinate, count or capacity violation preserves entire old map; valid pose may survive skipped fusion",
                "rejection":"no pose, map update or accepted-clock renewal; elapsed accepted-pose age latches lost until explicit new origin; evaluator never resets",
                "digest":"map_sha256: concatenated xyz IEEE754 f64 little endian in voxel-key order; map_statistics_sha256: each key xyz i64 LE, count u64 LE then mean xyz f64 LE"
            });
        } else if field == "motion_preprocessing" {
            value[field] = json!({"additional_voxel_m":0.06,"representative":"first lexicographic original voxel point","point_order":"lexicographic coarse voxel key"});
        }
        value[format!("{field}_sha256")] = json!(sha(
            &serde_json::to_vec(&value[field]).map_err(|e| e.to_string())?
        ));
    }
    value["uncertainty"] = json!({"position_std_floor_m":0.02,"rotation_std_floor_rad":0.03,"model":"per-fit conditional covariance plus previously frozen diagonal engineering allowance only; excludes correlated fused-map errors and is not calibrated root confidence","calibration_data":"all previously viewed Freiburg 1 XYZ intervals; no registration or allowance threshold changes"});
    Ok(value)
}
fn state(localizer: &SubmapLocalizer3d) -> Value {
    let voxels = localizer.map_voxels();
    let mut point_bytes = Vec::with_capacity(voxels.len() * 24);
    let mut statistics = Vec::with_capacity(voxels.len() * 56);
    for voxel in &voxels {
        for key in [voxel.key.0, voxel.key.1, voxel.key.2] {
            statistics.extend_from_slice(&key.to_le_bytes());
        }
        statistics.extend_from_slice(&(voxel.count as u64).to_le_bytes());
        for coordinate in [voxel.mean.x, voxel.mean.y, voxel.mean.z] {
            let bytes = coordinate.to_le_bytes();
            point_bytes.extend_from_slice(&bytes);
            statistics.extend_from_slice(&bytes);
        }
    }
    json!({
        "root_origin_frame_index":localizer.root_origin_frame_index(),
        "map_generation":localizer.map_generation(),
        "map_points":voxels.len(),
        "map_sha256":sha(&point_bytes),
        "map_statistics_sha256":sha(&statistics),
        "last_accepted_stamp":localizer.last_accepted_stamp(),
        "last_map_update_stamp":localizer.last_map_update_stamp(),
        "last_accepted_pose":localizer.last_pose().map(pose_json),
        "lost":localizer.is_lost()
    })
}
fn root_accuracy(estimated: Option<Pose3>, truth: Result<Pose3, String>) -> Value {
    let mut row = json!({"valid":estimated.is_some(),"reference_valid":truth.is_ok(),"within_accuracy_gates":false});
    match truth {
        Ok(t) => {
            row["evaluation_only_truth"] = pose_json(t);
            if let Some(p) = estimated {
                let a = p.translation;
                let b = t.translation;
                let position = (a.x - b.x).hypot(a.y - b.y).hypot(a.z - b.z);
                let angle = p.rotation.angular_distance(t.rotation);
                row["estimate"] = pose_json(p);
                row["translation_error_m"] = json!(position);
                row["rotation_error_rad"] = json!(angle);
                row["within_accuracy_gates"] =
                    json!(position <= TRANSLATION_GATE_M && angle <= ROTATION_GATE_RAD);
            }
        }
        Err(reason) => {
            row["reference_rejection"] = json!(reason);
            if let Some(p) = estimated {
                row["estimate"] = pose_json(p);
            }
        }
    }
    row
}
fn evaluate(
    manifest_path: &Path,
    raw: &Path,
    freeze_path: &Path,
    regression: bool,
) -> Result<Value, String> {
    let freeze_bytes = bounded_read(freeze_path, 128 * 1024)?;
    let freeze: Value = serde_json::from_slice(&freeze_bytes).map_err(|e| e.to_string())?;
    if freeze != protocol(manifest_path, regression)? {
        return Err("submap external freeze differs before raw access".into());
    }
    let manifest_bytes = bounded_read(manifest_path, 128 * 1024)?;
    let m: Manifest = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    validate_submap_manifest(&m)?;
    let metadata: Value = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    if calibrated_dataset(&m.dataset) {
        validate_calibration_source(&metadata)?;
        let source = &metadata["calibration_source"];
        let expected_bytes = source["bytes"]
            .as_u64()
            .ok_or("invalid calibration byte bound")? as usize;
        let calibration_bytes = bounded_read(&raw.join("camera-calibration.yaml"), expected_bytes)?;
        if calibration_bytes.len() != expected_bytes || sha(&calibration_bytes) != source["sha256"]
        {
            return Err("pinned camera calibration source bytes differ".into());
        }
    }
    let mut bytes = BTreeMap::new();
    let mut provenance = Vec::new();
    for file in &m.files {
        let data = bounded_read(&raw.join(&file.file), 4 * 1024 * 1024)?;
        if data.len() != file.bytes || sha(&data) != file.sha256 {
            return Err(format!("source hash mismatch: {}", file.file));
        }
        provenance.push(
            json!({"file":file.file,"bytes":data.len(),"sha256":sha(&data),"role":file.role}),
        );
        bytes.insert(file.file.clone(), data);
    }
    let index = m
        .files
        .iter()
        .find(|f| f.role == "depth_index")
        .ok_or("missing depth index")?;
    validate_depth_index(&bytes[&index.file], &m.frames)?;
    let clouds = m
        .frames
        .iter()
        .map(|f| measured_cloud(&bytes[&f.file], &m))
        .collect::<Result<Vec<_>, _>>()?;
    let mut localizer = SubmapLocalizer3d::new(config())?;
    let mut fits = Vec::new();
    // Raw mocap bytes are pinned for provenance, but never parsed or exposed to
    // the operational localizer. Every fit and map update finishes first.
    for (frame, scan) in m.frames.iter().zip(&clouds) {
        let before = state(&localizer);
        let clock = Instant::now();
        let result = localizer.observe(frame.source_index, frame.timestamp, scan);
        let seconds = clock.elapsed().as_secs_f64();
        let after = state(&localizer);
        fits.push((result, seconds, before, after));
    }
    let gt_file = m
        .files
        .iter()
        .find(|f| f.role == "evaluation_only_mocap_ground_truth")
        .unwrap();
    let gt = ground_truth(&bytes[&gt_file.file])?;
    let origin = interpolate(&gt, m.frames[0].timestamp);
    let mut rows = Vec::new();
    for (i, (result, seconds, before, after)) in fits.into_iter().enumerate() {
        let frame = &m.frames[i];
        let root_truth = origin
            .clone()
            .and_then(|o| interpolate(&gt, frame.timestamp).map(|p| o.inverse().compose(p)));
        let mut row = json!({"file":frame.file,"timestamp":frame.timestamp,"source_index":frame.source_index,"split":frame.split,"scan_points":clouds[i].len(),"cpu_wall_seconds":seconds,"accepted":result.is_ok(),"state_before":before,"state_after":after,"initialized":false,"map_updated":false});
        for field in [
            "map_generation",
            "map_points",
            "map_sha256",
            "map_statistics_sha256",
            "last_accepted_stamp",
            "last_map_update_stamp",
            "lost",
        ] {
            row[format!("{field}_before")] = row["state_before"][field].clone();
            row[format!("{field}_after")] = row["state_after"][field].clone();
        }
        match result {
            Ok(r) => {
                if r.root_origin_frame_index != m.frames[0].source_index {
                    return Err(
                        "submap changed the declared origin without an explicit reset".into(),
                    );
                }
                row["root_origin_frame_index"] = json!(r.root_origin_frame_index);
                row["initial_pose"] = pose_json(r.initial_pose);
                row["root_estimate"] = pose_json(r.root_from_scan);
                row["initialized"] = json!(r.initialized);
                row["map_updated"] = json!(r.map_updated);
                row["map_update_rejection"] = json!(r.map_update_rejection);
                row["registration"] =
                    motion::scored(&Ok(r.registration), root_truth.clone(), seconds);
                row["root_accuracy"] = root_accuracy(Some(r.root_from_scan), root_truth);
            }
            Err(reason) => {
                row["rejection"] = json!(reason);
                row["root_accuracy"] = root_accuracy(None, root_truth);
            }
        }
        rows.push(row);
    }
    let updates = &rows[1..];
    let count = |field: &str| {
        updates
            .iter()
            .filter(|r| r["root_accuracy"][field] == true)
            .count()
    };
    let initialized = rows.iter().filter(|r| r["initialized"] == true).count();
    let accepted = updates.iter().filter(|r| r["accepted"] == true).count();
    let accurate = count("within_accuracy_gates");
    let valid = count("reference_valid");
    let all_passed = initialized == 1
        && accepted == updates.len()
        && accurate == updates.len()
        && valid == updates.len();
    let summary = json!({"frames":rows.len(),"updates":updates.len(),"initialized_frames":initialized,"accepted_updates":accepted,"rejected_updates":updates.len()-accepted,"accurate_root_updates":accurate,"reference_valid_updates":valid,"map_updates":rows.iter().filter(|r|r["map_updated"]==true).count(),"map_update_rejections":rows.iter().filter(|r|r["map_update_rejection"].is_string()).count(),"final_map_points":localizer.map_points().len(),"final_map_generation":localizer.map_generation(),"lost":localizer.is_lost(),"all_updates_passed":all_passed});
    let mut report = json!({"schema_version":1,"dataset":m.dataset,"repository":m.repository,"revision":m.revision,"manifest_sha256":sha(&manifest_bytes),"freeze":freeze,"freeze_sha256":sha(&freeze_bytes),"files":provenance,"frames":rows,"summary":summary,"raw_redistributed":false,"ground_truth_operational":false,"limits":["Bounded local measured depth map, without loop closure or global relocalization; no automotive or operational driving validation.","ICP covariance excludes correlated fused-map errors and is not calibrated root-frame uncertainty.","Rejected or unscorable updates stay in the denominator; no ground-truth resets or interpolated operational poses."]});
    if calibrated_dataset(&m.dataset) {
        report["calibration_source"] = metadata["calibration_source"].clone();
        report["calibration_sha256_verified"] = json!(true);
        report["limits"].as_array_mut().unwrap().push(json!("Explicit registered-depth pinhole camera profiles; source distortion coefficients are provenance only and additional pixel undistortion is not applied. Distinct desk/office recordings and camera profiles do not establish driving validity or broad room diversity."));
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
    while let Some(argument) = args.next() {
        if argument == "--submaps" {
            continue;
        }
        if argument == "--regression" {
            if regression {
                return Err("duplicate --regression flag".into());
            }
            regression = true;
            continue;
        }
        if argument == "--motion" || argument == "--keyframes" {
            return Err("choose exactly one evaluation mode".into());
        }
        let value = args.next().ok_or("missing argument value")?;
        match argument.as_str() {
            "--manifest" => manifest = Some(value),
            "--raw" => raw = Some(value),
            "--freeze" => freeze = Some(value),
            "--prepare-freeze" => prepare = Some(value),
            "--output" => output = Some(value),
            _ => return Err(format!("unknown submap argument {argument}")),
        }
    }
    let manifest = manifest.ok_or("--manifest required")?;
    if let Some(path) = prepare {
        if freeze.is_some() || raw.is_some() || output.is_some() {
            return Err("freeze preparation does not access raw data or produce a report".into());
        }
        let file = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(file, &protocol(Path::new(&manifest), regression)?)
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
    if let Some(parent) = Path::new(&output).parent() {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    fs::write(
        output,
        serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    println!("{}", report["summary"]);
    Ok(report["summary"]["all_updates_passed"] == true)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn desk_metadata_freeze_uses_precise_pins_and_legacy_modes_refuse_it() {
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/tum-fr1-desk-submaps/manifest.json");
        let metadata: Value = serde_json::from_slice(&fs::read(&manifest).unwrap()).unwrap();
        let parsed: Manifest = serde_json::from_value(metadata.clone()).unwrap();
        assert!(validate_submap_manifest(&parsed).is_ok());
        assert!(validate_manifest(&parsed).is_err());
        let freeze = protocol(&manifest, false).unwrap();
        assert_eq!(freeze["kind"], "calibration_regression");
        assert_eq!(freeze["preprocessing"]["fx"], 517.306408);
        assert_eq!(freeze["depth_calibration"], metadata["depth_calibration"]);
        assert_eq!(freeze["calibration_source"], metadata["calibration_source"]);
        let office = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/tum-fr3-office-submaps/manifest.json");
        let office_freeze = protocol(&office, false).unwrap();
        assert_eq!(office_freeze["kind"], "preregistered_sequence");
        assert_eq!(office_freeze["protocol_version"], 8);
        assert_eq!(office_freeze["preprocessing"]["fx"], 535.4);
        assert_eq!(office_freeze["preprocessing"]["fy"], 539.2);
        let regression_freeze = protocol(&office, true).unwrap();
        assert_eq!(regression_freeze["kind"], "calibration_regression");
        assert_eq!(regression_freeze["regression_requested"], true);
        assert_eq!(
            regression_freeze["registration_config"],
            office_freeze["registration_config"]
        );
        assert_eq!(
            regression_freeze["submap_policy"],
            office_freeze["submap_policy"]
        );
        let mut changed = metadata.clone();
        changed["depth_calibration"]["fx"] = json!(525.0);
        assert!(validate_submap_manifest(&serde_json::from_value(changed).unwrap()).is_err());
        let mut changed = metadata;
        changed["calibration_source"]["sha256"] = json!("0".repeat(64));
        assert!(validate_calibration_source(&changed).is_err());
    }
    #[test]
    fn precise_freiburg_projection_preserves_recorded_big_endian_metres() {
        let calibration = Calibration {
            width: WIDTH,
            height: HEIGHT,
            fx: 517.306408,
            fy: 516.469215,
            cx: 318.643040,
            cy: 255.313989,
            units_per_metre: 5000.0,
            invalid_depth: 0,
        };
        let mut bytes = Vec::new();
        {
            let mut encoder = png::Encoder::new(&mut bytes, WIDTH, HEIGHT);
            encoder.set_color(png::ColorType::Grayscale);
            encoder.set_depth(png::BitDepth::Sixteen);
            let mut writer = encoder.write_header().unwrap();
            writer
                .write_image_data(&[0x27, 0x10].repeat((WIDTH * HEIGHT) as usize))
                .unwrap();
        }
        let cloud = calibrated_depth_cloud(&bytes, &calibration).unwrap();
        let expected = Vec3::new(
            -calibration.cx * 2.0 / calibration.fx,
            -calibration.cy * 2.0 / calibration.fy,
            2.0,
        );
        assert!(cloud.contains(&expected));
        assert!(cloud.iter().all(|p| p.z == 2.0));
        assert!(cloud.len() <= 4800);
        // The camera-specific geometry is distinct from the historical
        // default registered-depth projection, which remains unchanged.
        let legacy = depth_cloud(&bytes).unwrap();
        assert_ne!(cloud, legacy);
    }
    #[test]
    fn altered_submap_policy_is_rejected_before_any_raw_access() {
        let manifest =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/tum-fr1-xyz-fast/manifest.json");
        let mut freeze = protocol(&manifest, false).unwrap();
        freeze["submap_policy"]["maximum_map_points"] = json!(10000);
        let path = std::env::temp_dir().join(format!(
            "submap-policy-negative-{}.json",
            std::process::id()
        ));
        fs::write(&path, serde_json::to_vec(&freeze).unwrap()).unwrap();
        let error =
            evaluate(&manifest, Path::new("nonexistent-submap-raw"), &path, false).unwrap_err();
        fs::remove_file(path).unwrap();
        assert_eq!(error, "submap external freeze differs before raw access");
    }
    #[test]
    fn rejected_or_unscorable_root_never_passes_accuracy() {
        let missing = root_accuracy(Some(Pose3::identity()), Err("missing bracket".into()));
        assert_eq!(missing["valid"], true);
        assert_eq!(missing["within_accuracy_gates"], false);
        assert_eq!(
            root_accuracy(None, Ok(Pose3::identity()))["within_accuracy_gates"],
            false
        );
    }
}
