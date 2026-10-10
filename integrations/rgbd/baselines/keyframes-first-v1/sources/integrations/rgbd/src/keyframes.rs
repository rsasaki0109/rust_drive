//! Recorded-depth keyframe localization. Root-frame poses chain accepted local
//! matches; evaluation-only mocap is parsed after every operational observation.
use super::*;
use rustdriving_localization::keyframes3d::{
    KeyframeConfig3d, KeyframeLocalizer3d, KeyframeRegistration3d,
};

fn config() -> KeyframeConfig3d {
    KeyframeConfig3d {
        registration: motion::config(),
        keyframe_interval_s: 0.10,
        max_unobserved_s: 0.20,
    }
}
fn protocol(path: &Path) -> Result<Value, String> {
    let mut value = super::protocol(path)?;
    let cfg = config();
    value["protocol_version"] = json!(5);
    value["kind"] = json!(if value["dataset"] == "tum-fr1-xyz-keyframes" {
        "preregistered_temporal"
    } else {
        "calibration_regression"
    });
    value["keyframe_source_sha256"] = json!(sha(include_bytes!("keyframes.rs")));
    value["motion_source_sha256"] = json!(sha(include_bytes!("motion.rs")));
    value["keyframe_core_source_sha256"] = json!(sha(include_bytes!(
        "../../../crates/localization/src/keyframes3d.rs"
    )));
    value["independent_checker_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/check-recorded-keyframes.py"
    )));
    value["acquisition_source_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/fetch-keyframe-dataset.py"
    )));
    value["geometry_checker_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/check-recorded-rgbd.py"
    )));
    value["registration_config"] = config_json(&cfg.registration);
    value["registration_config_sha256"] = json!(sha(&serde_json::to_vec(
        &value["registration_config"]
    )
    .map_err(|e| e.to_string())?));
    value["motion_preprocessing"] = json!({"additional_voxel_m":0.06,"representative":"first lexicographic original voxel point","point_order":"lexicographic coarse voxel key"});
    value["keyframe_policy"] = json!({"keyframe_interval_s":cfg.keyframe_interval_s,"max_unobserved_s":cfg.max_unobserved_s,"initialization":"first measured cloud self-registers at identity with all numerical guards","prior":"last accepted sensor-only relative pose; identity after reference replacement","replacement":"copy accepted measured scan after minimum interval; no map union","root_chain":"root_from_reference composed with reference_from_scan","rejection":"no pose output, no map update; excessive accepted-observation gap latches lost until explicit reset; evaluation never resets","frame":"initial accepted depth camera is the root; each reference is a recorded depth camera frame"});
    value["uncertainty"] = json!({"position_std_floor_m":0.02,"rotation_std_floor_rad":0.03,"model":"per-fit conditional covariance plus previously frozen diagonal engineering allowance only; no accumulated root-frame covariance or calibrated confidence","calibration_data":"already viewed original, fast120..131, tight140..151 and motion-v2 260..271; all are calibration/regression here"});
    value["keyframe_policy_sha256"] = json!(sha(
        &serde_json::to_vec(&value["keyframe_policy"]).map_err(|e| e.to_string())?
    ));
    value["motion_preprocessing_sha256"] = json!(sha(&serde_json::to_vec(
        &value["motion_preprocessing"]
    )
    .map_err(|e| e.to_string())?));
    Ok(value)
}
fn root_accuracy(estimated: Option<Pose3>, truth: Result<Pose3, String>) -> Value {
    let mut row = json!({"valid":estimated.is_some(),"reference_valid":truth.is_ok(),"within_accuracy_gates":false});
    match truth {
        Ok(t) => {
            row["evaluation_only_truth"] = pose_json(t);
            if let Some(p) = estimated {
                let d = p.translation;
                let q = t.translation;
                let position = (d.x - q.x).hypot(d.y - q.y).hypot(d.z - q.z);
                let angle = p.rotation.angular_distance(t.rotation);
                row["estimate"] = pose_json(p);
                row["translation_error_m"] = json!(position);
                row["rotation_error_rad"] = json!(angle);
                row["within_accuracy_gates"] =
                    json!(position <= TRANSLATION_GATE_M && angle <= ROTATION_GATE_RAD);
            }
        }
        Err(e) => {
            row["reference_rejection"] = json!(e);
            if let Some(p) = estimated {
                row["estimate"] = pose_json(p);
            }
        }
    }
    row
}
fn evaluate(manifest_path: &Path, raw: &Path, freeze_path: &Path) -> Result<Value, String> {
    let freeze_bytes = bounded_read(freeze_path, 128 * 1024)?;
    let freeze: Value = serde_json::from_slice(&freeze_bytes).map_err(|e| e.to_string())?;
    if freeze != protocol(manifest_path)? {
        return Err("keyframe external freeze differs before raw access".into());
    }
    let manifest_bytes = bounded_read(manifest_path, 128 * 1024)?;
    let m: Manifest = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    validate_manifest(&m)?;
    let mut bytes = BTreeMap::new();
    let mut provenance = Vec::new();
    for file in &m.files {
        let b = bounded_read(&raw.join(&file.file), 4 * 1024 * 1024)?;
        if b.len() != file.bytes || sha(&b) != file.sha256 {
            return Err(format!("source hash mismatch: {}", file.file));
        }
        provenance
            .push(json!({"file":file.file,"bytes":b.len(),"sha256":sha(&b),"role":file.role}));
        bytes.insert(file.file.clone(), b);
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
        .map(|f| motion::cloud(&bytes[&f.file]))
        .collect::<Result<Vec<_>, _>>()?;
    let mut localizer = KeyframeLocalizer3d::new(config())?;
    let mut fits: Vec<(Result<KeyframeRegistration3d, String>, f64)> = Vec::new();
    // No mocap parsing, scoring or retry/reset occurs in this operational phase.
    for (frame, scan) in m.frames.iter().zip(&clouds) {
        let clock = Instant::now();
        let fit = localizer.observe(frame.source_index, frame.timestamp, scan);
        fits.push((fit, clock.elapsed().as_secs_f64()));
    }
    let gt_file = m
        .files
        .iter()
        .find(|f| f.role == "evaluation_only_mocap_ground_truth")
        .unwrap();
    let gt = ground_truth(&bytes[&gt_file.file])?;
    let origin = interpolate(&gt, m.frames[0].timestamp);
    let mut rows = Vec::new();
    for (i, (fit, seconds)) in fits.into_iter().enumerate() {
        let frame = &m.frames[i];
        let current = interpolate(&gt, frame.timestamp);
        let root_truth = origin
            .clone()
            .and_then(|o| current.clone().map(|p| o.inverse().compose(p)));
        let mut row = json!({"file":frame.file,"timestamp":frame.timestamp,"source_index":frame.source_index,"split":frame.split,"scan_points":clouds[i].len(),"cpu_wall_seconds":seconds,"accepted":fit.is_ok()});
        match fit {
            Ok(r) => {
                let reference = m
                    .frames
                    .iter()
                    .position(|f| {
                        f.source_index == r.reference_frame_index
                            && f.timestamp == r.reference_stamp
                    })
                    .ok_or("keyframe reference is not a manifest observation")?;
                let reference_truth = interpolate(&gt, r.reference_stamp)
                    .and_then(|p| current.map(|c| p.inverse().compose(c)));
                if r.root_origin_frame_index != m.frames[0].source_index {
                    return Err(
                        "keyframe changed the declared root origin without an explicit reset"
                            .into(),
                    );
                }
                row["root_origin_frame_index"] = json!(r.root_origin_frame_index);
                row["reference_frame_index"] = json!(r.reference_frame_index);
                row["reference_stamp"] = json!(r.reference_stamp);
                row["reference_file"] = json!(m.frames[reference].file);
                row["reference_points"] = json!(clouds[reference].len());
                row["root_estimate"] = pose_json(r.root_from_scan);
                row["root_from_reference"] = pose_json(r.root_from_reference);
                row["initial_pose"] = pose_json(r.initial_pose);
                row["initialized"] = json!(r.initialized);
                row["keyframe_replaced"] = json!(r.keyframe_replaced);
                row["registration"] = motion::scored(&Ok(r.registration), reference_truth, seconds);
                row["root_accuracy"] = root_accuracy(Some(r.root_from_scan), root_truth);
            }
            Err(e) => {
                row["rejection"] = json!(e);
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
    let valid_references = count("reference_valid");
    let all_passed = initialized == 1
        && accepted == updates.len()
        && accurate == updates.len()
        && valid_references == updates.len();
    let summary = json!({"frames":rows.len(),"updates":updates.len(),"initialized_frames":initialized,"accepted_updates":accepted,"rejected_updates":updates.len()-accepted,"accurate_root_updates":accurate,"reference_valid_updates":valid_references,"keyframe_replacements":rows.iter().filter(|r|r["keyframe_replaced"]==true).count(),"all_updates_passed":all_passed});
    Ok(
        json!({"schema_version":1,"dataset":m.dataset,"repository":m.repository,"revision":m.revision,"freeze":freeze,"freeze_sha256":sha(&freeze_bytes),"files":provenance,"frames":rows,"summary":summary,"raw_redistributed":false,"ground_truth_operational":false,"limits":["Short temporal interval in one indoor Kinect room; no independent environment or automotive validation.","Accepted measured keyframes replace one reference cloud; no map union, loop closure, SLAM or global relocalization.","Root poses compose accepted local fits; per-fit covariance is not accumulated root confidence.","Missing references and rejected observations remain in the denominator; no truth resets or interpolated operational poses."]}),
    )
}
pub(super) fn run() -> Result<bool, String> {
    let mut manifest = None;
    let mut raw = None;
    let mut freeze = None;
    let mut prepare = None;
    let mut output = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        if a == "--keyframes" {
            continue;
        }
        if a == "--motion" {
            return Err("choose exactly one evaluation mode".into());
        }
        let v = args.next().ok_or("missing argument value")?;
        match a.as_str() {
            "--manifest" => manifest = Some(v),
            "--raw" => raw = Some(v),
            "--freeze" => freeze = Some(v),
            "--prepare-freeze" => prepare = Some(v),
            "--output" => output = Some(v),
            _ => return Err(format!("unknown keyframe argument {a}")),
        }
    }
    let manifest = manifest.ok_or("--manifest required")?;
    if let Some(p) = prepare {
        if freeze.is_some() || raw.is_some() {
            return Err("freeze preparation does not access raw data".into());
        }
        let f = fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(p)
            .map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(f, &protocol(Path::new(&manifest))?)
            .map_err(|e| e.to_string())?;
        return Ok(true);
    }
    let report = evaluate(
        Path::new(&manifest),
        Path::new(&raw.ok_or("--raw required")?),
        Path::new(&freeze.ok_or("--freeze required")?),
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
    fn changed_keyframe_policy_rejects_before_raw_access() {
        let manifest =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/tum-fr1-xyz-fast/manifest.json");
        let mut freeze = protocol(&manifest).unwrap();
        freeze["keyframe_policy"]["max_unobserved_s"] = json!(100.0);
        let path = std::env::temp_dir().join(format!(
            "keyframe-freeze-negative-{}.json",
            std::process::id()
        ));
        fs::write(&path, serde_json::to_vec(&freeze).unwrap()).unwrap();
        let e = evaluate(&manifest, Path::new("nonexistent-keyframe-depth"), &path).unwrap_err();
        fs::remove_file(path).unwrap();
        assert_eq!(e, "keyframe external freeze differs before raw access");
    }
    #[test]
    fn missing_reference_never_counts_as_an_accurate_root_pose() {
        let row = root_accuracy(Some(Pose3::identity()), Err("missing bracket".into()));
        assert_eq!(row["valid"], true);
        assert_eq!(row["reference_valid"], false);
        assert_eq!(row["within_accuracy_gates"], false);
        assert!(row.get("translation_error_m").is_none());
        assert_eq!(
            root_accuracy(None, Ok(Pose3::identity()))["within_accuracy_gates"],
            false
        );
    }
}
