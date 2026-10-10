//! Bounded recorded-depth odometry and fixed-first-cloud map localization.
//! Motion-capture labels are inaccessible to the registration phase.
use super::*;
const MOTION_VOXEL_M: f64 = 0.06;
const POSITION_STD_M: f64 = 0.02;
const ROTATION_STD_RAD: f64 = 0.03;
fn config() -> Registration3dConfig {
    Registration3dConfig {
        max_correspondence_m: 0.15,
        ..Default::default()
    }
}
fn cloud(bytes: &[u8]) -> Result<Vec<Vec3>, String> {
    let mut voxels = BTreeMap::new();
    for p in depth_cloud(bytes)? {
        let key = (
            (p.x / MOTION_VOXEL_M).floor() as i64,
            (p.y / MOTION_VOXEL_M).floor() as i64,
            (p.z / MOTION_VOXEL_M).floor() as i64,
        );
        voxels.entry(key).or_insert(p);
    }
    Ok(voxels.into_values().collect())
}
fn covariance(c: [[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut result = c;
    for (i, row) in result.iter_mut().enumerate() {
        row[i] += if i < 3 {
            POSITION_STD_M.powi(2)
        } else {
            ROTATION_STD_RAD.powi(2)
        };
    }
    result
}
fn protocol(path: &Path) -> Result<Value, String> {
    let mut value = super::protocol(path)?;
    value["protocol_version"] = json!(3);
    value["kind"] = json!(if value["dataset"] == "tum-fr1-xyz-motion" {
        "preregistered_temporal"
    } else {
        "calibration_regression"
    });
    value["motion_source_sha256"] = json!(sha(include_bytes!("motion.rs")));
    value["independent_checker_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/check-recorded-motion.py"
    )));
    value["geometry_checker_sha256"] = json!(sha(include_bytes!(
        "../../../scripts/check-recorded-rgbd.py"
    )));
    value["registration_config"] = config_json(&config());
    value["registration_config_sha256"] = json!(sha(&serde_json::to_vec(
        &value["registration_config"]
    )
    .unwrap()));
    value["motion_preprocessing"] = json!({"additional_voxel_m":MOTION_VOXEL_M,"representative":"first lexicographic original voxel point","point_order":"lexicographic coarse voxel key"});
    value["uncertainty"] = json!({"position_std_floor_m":POSITION_STD_M,"rotation_std_floor_rad":ROTATION_STD_RAD,"model":"conditional covariance plus diagonal engineering systematic-error allowance; provisional, not a statistical confidence guarantee","calibration_data":"already viewed original0..110/10, fast120..131 and tight140..151 only"});
    value["map_policy"] = json!({"map":"first measured depth cloud; never mocap geometry","initialization":"last accepted sensor-only fixed-map estimate, initially identity","rejection":"retain all failures; no map update","odometry":"compose only contiguous accepted pair transforms; first rejection invalidates continuous trajectory"});
    Ok(value)
}
fn error(estimated: Pose3, truth: Pose3) -> (f64, f64) {
    let a = estimated.translation;
    let b = truth.translation;
    (
        (a.x - b.x).hypot(a.y - b.y).hypot(a.z - b.z),
        estimated.rotation.angular_distance(truth.rotation),
    )
}
fn scored(
    fit: &Result<rustdriving_localization::registration3d::Registration3dResult, String>,
    truth: Pose3,
    seconds: f64,
) -> Value {
    let mut row = json!({"evaluation_only_truth":pose_json(truth),"cpu_wall_seconds":seconds});
    match fit {
        Ok(r) => {
            let (t, a) = error(r.pose, truth);
            row["accepted"] = json!(true);
            row["estimate"] = pose_json(r.pose);
            row["translation_error_m"] = json!(t);
            row["rotation_error_rad"] = json!(a);
            row["within_accuracy_gates"] = json!(t <= TRANSLATION_GATE_M && a <= ROTATION_GATE_RAD);
            row["conditional_covariance_xyz_rotation"] = json!(r.covariance);
            row["provisional_covariance_xyz_rotation"] = json!(covariance(r.covariance));
            row["rms_m"] = json!(r.rms_m);
            row["inlier_fraction"] = json!(r.inlier_fraction);
            row["neighbor_checks"] = json!(r.conditioning.neighbor_checks);
            row["ambiguity_probes"] = json!(r.conditioning.ambiguity_probes);
        }
        Err(e) => {
            row["accepted"] = json!(false);
            row["within_accuracy_gates"] = json!(false);
            row["rejection"] = json!(e);
        }
    }
    row
}
fn evaluate(manifest_path: &Path, raw: &Path, freeze_path: &Path) -> Result<Value, String> {
    let freeze_bytes = bounded_read(freeze_path, 128 * 1024)?;
    let freeze: Value = serde_json::from_slice(&freeze_bytes).map_err(|e| e.to_string())?;
    if freeze != protocol(manifest_path)? {
        return Err("motion external freeze differs before raw access".into());
    }
    let m: Manifest = serde_json::from_slice(&bounded_read(manifest_path, 128 * 1024)?)
        .map_err(|e| e.to_string())?;
    let mut bytes = BTreeMap::new();
    for f in &m.files {
        let b = bounded_read(&raw.join(&f.file), 4 * 1024 * 1024)?;
        if b.len() != f.bytes || sha(&b) != f.sha256 {
            return Err(format!("source hash mismatch: {}", f.file));
        }
        bytes.insert(f.file.clone(), b);
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
        .map(|f| cloud(&bytes[&f.file]))
        .collect::<Result<Vec<_>, _>>()?;
    let cfg = config();
    let mut pair_fits = Vec::new();
    let mut map_fits = Vec::new();
    let mut map_prior = Pose3::identity();
    let mut map_priors = Vec::new();
    let mut trajectory = Some(Pose3::identity());
    let mut trajectories = Vec::new();
    // Every operational estimate finishes before pose labels are parsed.
    for i in 1..clouds.len() {
        let clock = Instant::now();
        let pair = match_scan(&clouds[i], &clouds[i - 1], Pose3::identity(), &cfg);
        let pair_seconds = clock.elapsed().as_secs_f64();
        trajectory = match (&pair, trajectory) {
            (Ok(r), Some(p)) => Some(p.compose(r.pose)),
            _ => None,
        };
        trajectories.push(trajectory);
        let clock = Instant::now();
        map_priors.push(map_prior);
        let map = match_scan(&clouds[i], &clouds[0], map_prior, &cfg);
        let map_seconds = clock.elapsed().as_secs_f64();
        if let Ok(r) = &map {
            map_prior = r.pose;
        }
        pair_fits.push((pair, pair_seconds));
        map_fits.push((map, map_seconds));
    }
    let gt_file = m
        .files
        .iter()
        .find(|f| f.role == "evaluation_only_mocap_ground_truth")
        .unwrap();
    let gt = ground_truth(&bytes[&gt_file.file])?;
    let origin = interpolate(&gt, m.frames[0].timestamp)?;
    let mut rows = Vec::new();
    for i in 1..clouds.len() {
        let previous = interpolate(&gt, m.frames[i - 1].timestamp)?;
        let current = interpolate(&gt, m.frames[i].timestamp)?;
        let truth = origin.inverse().compose(current);
        let mut row = json!({"previous_file":m.frames[i-1].file,"current_file":m.frames[i].file,"previous_timestamp":m.frames[i-1].timestamp,"current_timestamp":m.frames[i].timestamp,"source_index":m.frames[i].source_index,"pair_initial_pose":pose_json(Pose3::identity()),"fixed_map_initial_pose":pose_json(map_priors[i-1]),"scan_points":clouds[i].len(),"pair_map_points":clouds[i-1].len(),"fixed_map_points":clouds[0].len(),"pair":scored(&pair_fits[i-1].0,previous.inverse().compose(current),pair_fits[i-1].1),"fixed_map":scored(&map_fits[i-1].0,truth,map_fits[i-1].1)});
        row["continuous_odometry"] = match trajectories[i - 1] {
            Some(p) => {
                let (t, a) = error(p, truth);
                json!({"valid":true,"estimate":pose_json(p),"translation_error_m":t,"rotation_error_rad":a})
            }
            None => {
                json!({"valid":false,"reason":"an earlier pair rejected; no invented bridge or truth reset"})
            }
        };
        rows.push(row);
    }
    let count = |name: &str, field: &str| rows.iter().filter(|r| r[name][field] == true).count();
    let summary = json!({"pairs":11,"pair_accepted":count("pair","accepted"),"pair_accurate":count("pair","within_accuracy_gates"),"map_accepted":count("fixed_map","accepted"),"map_accurate":count("fixed_map","within_accuracy_gates"),"continuous_odometry_frames":rows.iter().filter(|r|r["continuous_odometry"]["valid"]==true).count()});
    Ok(
        json!({"schema_version":1,"freeze":freeze,"freeze_sha256":sha(&freeze_bytes),"frames":rows,"summary":summary,"raw_redistributed":false,"ground_truth_operational":false,"limits":["One indoor Kinect room and short temporal interval; no driving validation or independent environment.","First cloud is a measured fixed local map, not SLAM, global localization or loop closure.","Provisional covariance allowance is not independently established statistical calibration; all coverage reported."]}),
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
        if a == "--motion" {
            continue;
        }
        let v = args.next().ok_or("missing argument value")?;
        match a.as_str() {
            "--manifest" => manifest = Some(v),
            "--raw" => raw = Some(v),
            "--freeze" => freeze = Some(v),
            "--prepare-freeze" => prepare = Some(v),
            "--output" => output = Some(v),
            _ => return Err(format!("unknown motion argument {a}")),
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
    fs::write(
        output,
        serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    println!("{}", report["summary"]);
    Ok(report["summary"]["pair_accurate"] == 11 && report["summary"]["map_accurate"] == 11)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn systematic_allowance_preserves_correlations_and_positive_floor() {
        let mut c = [[0.; 6]; 6];
        c[0][1] = 1e-8;
        c[1][0] = 1e-8;
        let r = covariance(c);
        assert_eq!(r[0][1], 1e-8);
        assert_eq!(r[0][0], 0.0004);
        assert_eq!(r[5][5], 0.0009);
    }
    #[test]
    fn motion_limits_are_stricter_than_existing_baseline() {
        let c = config();
        c.validate().unwrap();
        assert!(c.max_correspondence_m < Registration3dConfig::default().max_correspondence_m);
        assert_eq!(c.max_neighbor_checks, 20_000_000);
        assert_eq!(c.min_overlap, 0.4);
        assert_eq!(c.ambiguity_rms_ratio, 1.05);
    }
    #[test]
    fn changed_motion_freeze_rejects_before_raw_access() {
        let manifest =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data/tum-fr1-xyz-fast/manifest.json");
        let mut frozen = protocol(&manifest).unwrap();
        frozen["uncertainty"]["position_std_floor_m"] = json!(0.00001);
        let path = std::env::temp_dir().join(format!(
            "motion-freeze-negative-{}.json",
            std::process::id()
        ));
        fs::write(&path, serde_json::to_vec(&frozen).unwrap()).unwrap();
        let error = evaluate(&manifest, Path::new("nonexistent-raw-depth"), &path).unwrap_err();
        fs::remove_file(path).unwrap();
        assert_eq!(error, "motion external freeze differs before raw access");
    }
    #[test]
    fn unobservable_planar_recorded_geometry_rejects_instead_of_issuing_a_pose() {
        let points = (0..20)
            .flat_map(|x| (0..20).map(move |y| Vec3::new(x as f64 * 0.06, y as f64 * 0.06, 2.)))
            .collect::<Vec<_>>();
        assert!(match_scan(&points, &points, Pose3::identity(), &config()).is_err());
    }
}
