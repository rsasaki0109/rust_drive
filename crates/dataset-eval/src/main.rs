//! Offline measured geometry evaluation; reference labels never enter algorithms.
use rustdrive_core::{Pose, Vec2, Vec3, wrap_angle};
use rustdrive_dataset_eval::{parse_pcd, parse_vtk, read_bounded};
use rustdrive_localization::registration::{RegistrationConfig, RegistrationResult, match_scan};
use rustdrive_perception::ground3d::{TerrainConfig, classify_ground};
use rustdrive_perception::objects3d::{ObjectClusterConfig, cluster_objects};
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Instant;

fn canonical_sha(value: &Value, python: &str) -> Result<String, String> {
    let mut child=Command::new(python).args(["-c","import sys,json,hashlib; print(hashlib.sha256(json.dumps(json.load(sys.stdin),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest())"]).stdin(Stdio::piped()).stdout(Stdio::piped()).spawn().map_err(|e|e.to_string())?;
    child
        .stdin
        .take()
        .ok_or("missing hash helper stdin")?
        .write_all(&serde_json::to_vec(value).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    let output = child.wait_with_output().map_err(|e| e.to_string())?;
    if !output.status.success() {
        return Err("canonical parameter hash helper failed".into());
    }
    let digest = String::from_utf8(output.stdout).map_err(|e| e.to_string())?;
    if digest.trim().len() != 64 {
        return Err("invalid canonical parameter digest".into());
    }
    Ok(digest.trim().into())
}
fn verify(repo: &Path, data: &Path, python: &str) -> Result<String, String> {
    let output = Command::new(python)
        .arg(repo.join("scripts/fetch-datasets.py"))
        .args(["--verify-only", "--output"])
        .arg(data)
        .output()
        .map_err(|e| e.to_string())?;
    if !output.status.success() {
        return Err(format!(
            "dataset SHA verification failed: {}",
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    Ok(String::from_utf8_lossy(&output.stdout).trim().into())
}
fn manifest(repo: &Path, name: &str) -> Result<Value, String> {
    serde_json::from_slice(&read_bounded(
        &repo.join("data").join(name).join("manifest.json"),
    )?)
    .map_err(|e| e.to_string())
}
fn entry<'a>(manifest: &'a Value, sample: &str, role: &str) -> Result<&'a Value, String> {
    manifest["files"]
        .as_array()
        .ok_or("manifest files missing")?
        .iter()
        .find(|e| e["sample"] == sample && e["role"] == role)
        .ok_or_else(|| format!("missing {sample}/{role} manifest entry"))
}
fn file_path(data: &Path, dataset: &str, entry: &Value) -> Result<PathBuf, String> {
    let filename = entry["file"].as_str().ok_or("manifest filename missing")?;
    if Path::new(filename).components().count() != 1 || filename == ".." {
        return Err("manifest filename is not a simple local filename".into());
    }
    Ok(data.join(dataset).join("raw").join(filename))
}
#[derive(Default, Clone, Copy)]
struct Confusion {
    tp: u64,
    fp: u64,
    tn: u64,
    fn_: u64,
}
impl Confusion {
    fn add(self, other: Self) -> Self {
        Self {
            tp: self.tp + other.tp,
            fp: self.fp + other.fp,
            tn: self.tn + other.tn,
            fn_: self.fn_ + other.fn_,
        }
    }
    fn value(self) -> Value {
        json!({"tp":self.tp,"fp":self.fp,"tn":self.tn,"fn":self.fn_})
    }
    fn metrics(self) -> Value {
        let ratio = |a: u64, b: u64| {
            if b == 0 {
                None
            } else {
                Some(a as f64 / b as f64)
            }
        };
        json!({"precision":ratio(self.tp,self.tp+self.fp),"recall":ratio(self.tp,self.tp+self.fn_),"f1":ratio(2*self.tp,2*self.tp+self.fp+self.fn_),"accuracy":ratio(self.tp+self.tn,self.tp+self.fp+self.tn+self.fn_)})
    }
}
fn key(p: Vec3) -> [u32; 3] {
    [
        (p.x as f32).to_bits(),
        (p.y as f32).to_bits(),
        (p.z as f32).to_bits(),
    ]
}
fn terrain_config(c: &TerrainConfig) -> Value {
    json!({"cell_size_m":c.cell_size_m,"initial_height_m":c.initial_height_m,"max_height_m":c.max_height_m,"max_slope":c.max_slope,"window_radii_cells":c.window_radii_cells,"min_support_neighbors":c.min_support_neighbors,"min_supported_cells":c.min_supported_cells,"min_supported_fraction":c.min_supported_fraction,"max_points":c.max_points,"max_cells":c.max_cells,"max_candidate_work":c.max_candidate_work,"coordinate_bound_m":c.coordinate_bound_m})
}
fn object_config(c: &ObjectClusterConfig) -> Value {
    json!({"tolerance_m":c.tolerance_m,"voxel_size_m":c.voxel_size_m,"min_points":c.min_points,"max_points":c.max_points,"max_cluster_points":c.max_cluster_points,"max_clusters":c.max_clusters,"max_candidate_work":c.max_candidate_work,"coordinate_bound_m":c.coordinate_bound_m})
}
fn registration_config(c: &RegistrationConfig) -> Value {
    json!({"max_scan_points":c.max_scan_points,"max_map_points":c.max_map_points,"max_iterations":c.max_iterations,"max_neighbor_checks":c.max_neighbor_checks,"max_correspondence_m":c.max_correspondence_m,"trim_fraction":c.trim_fraction,"min_pairs":c.min_pairs,"min_overlap":c.min_overlap,"max_translation_jump_m":c.max_translation_jump_m,"max_yaw_jump_rad":c.max_yaw_jump_rad,"max_rms_m":c.max_rms_m,"min_geometry_ratio":c.min_geometry_ratio,"max_condition_number":c.max_condition_number,"max_position_variance_m2":c.max_position_variance_m2,"max_yaw_variance_rad2":c.max_yaw_variance_rad2,"translation_tolerance_m":c.translation_tolerance_m,"yaw_tolerance_rad":c.yaw_tolerance_rad,"ambiguity_translation_probe_m":c.ambiguity_translation_probe_m,"ambiguity_yaw_probe_rad":c.ambiguity_yaw_probe_rad,"ambiguity_rms_ratio":c.ambiguity_rms_ratio})
}
fn terrain(
    data: &Path,
    manifest: &Value,
    config: &TerrainConfig,
    object_config: &ObjectClusterConfig,
) -> Result<(Vec<Value>, Value), String> {
    let mut rows = vec![];
    let mut aggregates = BTreeMap::<String, Confusion>::new();
    for (list, split) in [
        ("calibration_samples", "calibration"),
        ("held_out_samples", "held_out"),
    ] {
        for sample in manifest[list]
            .as_array()
            .ok_or("missing terrain split list")?
        {
            let sample = sample.as_str().ok_or("invalid terrain sample identifier")?;
            let input = entry(manifest, sample, "input_cloud")?;
            let reference = entry(manifest, sample, "ground_reference")?;
            if input["split"] != split || reference["split"] != split {
                return Err("sample split disagrees with manifest roles".into());
            }
            let raw = parse_pcd(&read_bounded(&file_path(data, "isprs-terrain", input)?)?)?;
            if raw.len() as u64
                != input["points"]
                    .as_u64()
                    .ok_or("manifest point count missing")?
            {
                return Err("input cloud point count differs from manifest".into());
            }
            let origin = *raw.first().ok_or("empty terrain input")?;
            let points: Vec<_> = raw
                .iter()
                .map(|p| Vec3::new(p.x - origin.x, p.y - origin.y, p.z - origin.z))
                .collect();
            let start = Instant::now();
            let result = classify_ground(&points, config)?;
            let classification_ms = start.elapsed().as_secs_f64() * 1000.0;
            // Ground-reference data are first read AFTER classification. They are
            // evaluation labels, never fit parameters, point selection or thresholds.
            let truth = parse_pcd(&read_bounded(&file_path(
                data,
                "isprs-terrain",
                reference,
            )?)?)?;
            if truth.len() as u64
                != reference["points"]
                    .as_u64()
                    .ok_or("reference point count missing")?
            {
                return Err("reference point count differs from manifest".into());
            }
            let keys: BTreeSet<_> = raw.iter().copied().map(key).collect();
            let labels: BTreeSet<_> = truth.iter().copied().map(key).collect();
            if !labels.is_subset(&keys) {
                return Err(format!(
                    "{sample}: ground reference is not an exact float32 XYZ subset"
                ));
            }
            let predicted: BTreeSet<_> = result.ground_indices.iter().copied().collect();
            let mut confusion = Confusion::default();
            for (index, p) in raw.iter().enumerate() {
                match (predicted.contains(&index), labels.contains(&key(*p))) {
                    (true, true) => confusion.tp += 1,
                    (true, false) => confusion.fp += 1,
                    (false, true) => confusion.fn_ += 1,
                    (false, false) => confusion.tn += 1,
                }
            }
            aggregates
                .entry(split.into())
                .and_modify(|c| *c = c.add(confusion))
                .or_insert(confusion);
            let start = Instant::now();
            let objects = match cluster_objects(&points, &result.non_ground_indices, object_config)
            {
                Ok(result) => {
                    json!({"status":"evaluated","cluster_count":result.objects.len(),"noise_indices":result.noise_indices,"candidate_work":result.candidate_work,"occupied_voxels":result.occupied_voxels,"aabbs":result.objects.iter().map(|o|json!({"min":o.min,"max":o.max,"center":o.center,"point_count":o.point_count,"original_point_indices":o.indices})).collect::<Vec<_>>()})
                }
                Err(error) => json!({"status":"rejected","reason":error}),
            };
            let cluster_ms = start.elapsed().as_secs_f64() * 1000.0;
            let d = &result.diagnostics;
            rows.push(json!({"sample":sample,"split":split,"points":raw.len(),"reference_points":truth.len(),"reference_unique_points":labels.len(),"raw_unique_points":keys.len(),"raw_duplicate_points":raw.len()-keys.len(),"reference_duplicate_points":truth.len()-labels.len(),"origin_xyz_m":origin,"input_sha256":input["sha256"],"ground_reference_sha256":reference["sha256"],"confusion":confusion.value(),"metrics":confusion.metrics(),"ground_indices":result.ground_indices,"non_ground_indices":result.non_ground_indices,"classification_diagnostics":{"occupied_cells":d.occupied_cells,"candidate_ground_cells":d.candidate_ground_cells,"supported_cells":d.supported_cells,"rejected_cells":d.rejected_cells,"candidate_work":d.candidate_work,"supported_fraction":d.supported_fraction,"confident":d.confident},"classification_ms":classification_ms,"objects":objects,"cluster_ms":cluster_ms}));
            eprintln!(
                "terrain {sample} {split}: {} points, ground F1 {:?}",
                raw.len(),
                confusion.metrics()["f1"]
            );
        }
    }
    let aggregate = Value::Object(
        aggregates
            .into_iter()
            .map(|(split, c)| (split, json!({"confusion":c.value(),"metrics":c.metrics()})))
            .collect(),
    );
    Ok((rows, aggregate))
}
fn project(points: &[Vec3]) -> Vec<(usize, Vec2)> {
    let mut cells = BTreeMap::new();
    for (index, p) in points.iter().enumerate() {
        if (0.5..=2.0).contains(&p.z) {
            cells
                .entry(((p.x / 0.2).floor() as i64, (p.y / 0.2).floor() as i64))
                .or_insert((index, Vec2::new(p.x, p.y)));
        }
    }
    cells.into_values().collect()
}
fn nees(error: [f64; 3], matrix: [[f64; 3]; 3]) -> Option<f64> {
    let mut a = [[0.0; 4]; 3];
    for i in 0..3 {
        a[i][..3].copy_from_slice(&matrix[i]);
        a[i][3] = error[i];
    }
    for column in 0..3 {
        let pivot =
            (column..3).max_by(|i, j| a[*i][column].abs().total_cmp(&a[*j][column].abs()))?;
        if a[pivot][column].abs() < 1e-20 {
            return None;
        }
        a.swap(column, pivot);
        let d = a[column][column];
        for v in &mut a[column][column..] {
            *v /= d;
        }
        let row = a[column];
        for (i, values) in a.iter_mut().enumerate() {
            if i != column {
                let factor = values[column];
                for j in column..4 {
                    values[j] -= factor * row[j];
                }
            }
        }
    }
    let value = (0..3).map(|i| error[i] * a[i][3]).sum::<f64>();
    (value.is_finite() && value >= 0.0).then_some(value)
}
fn result_json(
    result: Result<RegistrationResult, String>,
    truth: Option<Pose>,
    initial: Pose,
    duration_ms: f64,
) -> Value {
    match result {
        Err(reason) => {
            json!({"status":"rejected","reason":reason,"initial_pose":initial,"duration_ms":duration_ms})
        }
        Ok(r) => {
            let mut value = json!({"status":"accepted","initial_pose":initial,"recovered_pose":r.pose,"rms_m":r.rms_m,"inlier_fraction":r.inlier_fraction,"inlier_count":r.inlier_count,"iterations":r.iterations,"converged":r.converged,"covariance":r.covariance,"conditioning":{"geometry_ratio":r.conditioning.geometry_ratio,"condition_number":r.conditioning.condition_number,"neighbor_checks":r.conditioning.neighbor_checks,"ambiguity_probes":r.conditioning.ambiguity_probes},"duration_ms":duration_ms});
            if let Some(truth) = truth {
                let error = [
                    r.pose.position.x - truth.position.x,
                    r.pose.position.y - truth.position.y,
                    wrap_angle(r.pose.yaw - truth.yaw),
                ];
                let statistic = nees(error, r.covariance);
                value["pose_error"] = json!({"translation_m":error[0].hypot(error[1]),"yaw_rad":error[2].abs(),"components":error});
                value["nees"] = json!(statistic);
                value["standardized_component_errors"] = json!(
                    (0..3)
                        .map(|i| error[i] / r.covariance[i][i].sqrt())
                        .collect::<Vec<_>>()
                );
                value["within_nominal_95_percent_ellipsoid"] =
                    json!(statistic.map(|v| v <= 7.814727903251179));
            }
            value
        }
    }
}
fn registration(
    data: &Path,
    manifest: &Value,
    config: &RegistrationConfig,
) -> Result<Value, String> {
    let mut clouds = vec![];
    let mut preprocessing = vec![];
    let mut synthetic = vec![];
    for (file, split) in [("cloud_0.vtk", "calibration"), ("cloud_1.vtk", "held_out")] {
        let entry = manifest["files"]
            .as_array()
            .ok_or("missing apartment files")?
            .iter()
            .find(|e| e["file"] == file)
            .ok_or("missing measured apartment cloud")?;
        let raw = parse_vtk(&read_bounded(&file_path(data, "libpointmatcher", entry)?)?)?;
        if raw.len() as u64
            != entry["points"]
                .as_u64()
                .ok_or("apartment point count absent")?
        {
            return Err("VTK point count differs from manifest".into());
        }
        let selected = project(&raw);
        let map: Vec<_> = selected.iter().map(|(_, p)| *p).collect();
        preprocessing.push(json!({"cloud":file,"split":split,"input_sha256":entry["sha256"],"raw_points":raw.len(),"selected_point_indices":selected.iter().map(|(i,_)|*i).collect::<Vec<_>>(),"selected_points":map.len()}));
        for (case, (x, y, yaw)) in [(0.4, -0.25, 0.06), (-0.5, 0.35, -0.08)]
            .into_iter()
            .enumerate()
        {
            let truth = Pose {
                position: Vec2::new(x, y),
                yaw,
            };
            let initial = Pose {
                position: truth.position.plus(Vec2::new(0.04, -0.03)),
                yaw: truth.yaw + 0.015,
            };
            let scan: Vec<_> = map
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    p.minus(truth.position).rotated(-truth.yaw).plus(Vec2::new(
                        (i as f64 * 1.73).sin() * 0.001,
                        (i as f64 * 0.77).cos() * 0.001,
                    ))
                })
                .collect();
            let start = Instant::now();
            let matched = match_scan(&scan, &map, initial, config);
            let mut row = result_json(
                matched,
                Some(truth),
                initial,
                start.elapsed().as_secs_f64() * 1000.0,
            );
            row["case_id"] = json!(format!("{split}-transform-{case}"));
            row["split"] = json!(split);
            row["cloud"] = json!(file);
            row["input_sha256"] = entry["sha256"].clone();
            row["known_pose"] = json!(truth);
            row["points"] = json!(scan.len());
            row["scope"] = json!(
                "SEMI-SYNTHETIC: measured apartment geometry with an imposed SE2 pose and deterministic noise; not natural-pair pose ground truth"
            );
            synthetic.push(row);
        }
        clouds.push(map);
    }
    let initial = Pose::default();
    let start = Instant::now();
    let matched = match_scan(&clouds[1], &clouds[0], initial, config);
    let mut natural = result_json(
        matched,
        None,
        initial,
        start.elapsed().as_secs_f64() * 1000.0,
    );
    natural["physical_pose_ground_truth"] = Value::Null;
    natural["scope"] = json!(
        "Two measured views of one apartment; local XY projected residual/overlap only, no pose accuracy, independent-environment, or 6DOF claim"
    );
    natural["map_cloud"] = json!("cloud_0.vtk");
    natural["scan_cloud"] = json!("cloud_1.vtk");
    Ok(
        json!({"cloud_preprocessing":preprocessing,"semi_synthetic":synthetic,"natural_pair":natural,"covariance_scope":"Local independent-point ICP least-squares covariance; four correlated semi-synthetic cases do not establish calibration or statistical 95% coverage"}),
    )
}
fn run() -> Result<(), String> {
    let mut repository = PathBuf::from(".");
    let mut python = std::env::var("DATASET_EVAL_PYTHON").unwrap_or_else(|_| "python3".into());
    let mut data = None;
    let mut output = PathBuf::from("artifacts/dataset-evaluation.json");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--python" => python = args.next().ok_or("--python needs executable name")?,
            "--repository" => {
                repository = PathBuf::from(args.next().ok_or("--repository needs path")?)
            }
            "--data-root" => {
                data = Some(PathBuf::from(args.next().ok_or("--data-root needs path")?))
            }
            "--output" => output = PathBuf::from(args.next().ok_or("--output needs file")?),
            "--help" | "-h" => {
                println!(
                    "rustdrive-dataset-eval [--python PYTHON] [--repository REPO] [--data-root RAW_BASE] [--output REPORT.json]\nRequires downloaded SHA-pinned ISPRS and apartment data and python3 for the existing verification helper. No download occurs."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown option {arg}")),
        }
    }
    let data = data.unwrap_or_else(|| repository.join("data"));
    // Replace a preceding successful report before any fallible evaluation.
    // A failed invocation must never leave evaluation_complete=true behind.
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    std::fs::write(
        &output,
        b"{\"schema_version\":1,\"evaluation_complete\":false}\n",
    )
    .map_err(|e| e.to_string())?;
    let before = verify(&repository, &data, &python)?;
    let terrain_manifest = manifest(&repository, "isprs-terrain")?;
    let apartment_manifest = manifest(&repository, "libpointmatcher")?;
    let tc = TerrainConfig::default();
    let oc = ObjectClusterConfig::default();
    let rc = RegistrationConfig::default();
    let parameters = json!({"terrain_config":terrain_config(&tc),"object_config":object_config(&oc),"registration_config":registration_config(&rc),"registration_preprocessing":{"z_min_m":0.5,"z_max_m":2.,"xy_voxel_m":0.2,"selection":"first input point per XY voxel, voxels sorted lexicographically","transforms":[[0.4,-0.25,0.06],[-0.5,0.35,-0.08]],"initial_error":[0.04,-0.03,0.015],"noise":"sensor XY: 0.001*sin(index*1.73), 0.001*cos(index*0.77) meters"}});
    let parameter_hash = canonical_sha(&parameters, &python)?;
    let (samples, aggregate) = terrain(&data, &terrain_manifest, &tc, &oc)?;
    let registration = registration(&data, &apartment_manifest, &rc)?;
    let after = verify(&repository, &data, &python)?;
    if terrain_manifest != manifest(&repository, "isprs-terrain")?
        || apartment_manifest != manifest(&repository, "libpointmatcher")?
    {
        return Err("dataset manifests changed during evaluation".into());
    }
    let rustc = Command::new("rustc")
        .arg("--version")
        .output()
        .ok()
        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned());
    let report = json!({"schema_version":1,"scope":"Offline real measured geometry evaluation; ISPRS is airborne terrain, apartment registration is planar projection; no vehicle-driving or real-time acceptance","manifests":{"isprs_terrain":terrain_manifest,"libpointmatcher":apartment_manifest},"hash_verification":{"before":before,"after":after,"raw_sha256_verified":true,"verifier":"scripts/fetch-datasets.py --verify-only","python_executable":python},"frozen_parameters":parameters,"frozen_parameters_sha256":parameter_hash,"terrain_samples":samples,"terrain_aggregate":aggregate,"terrain_label_policy":"Exact original float32 XYZ membership in separate ground-reference subset; labels are read only after algorithm classification; duplicate coordinates share a label","registration":registration,"timing_context":{"profile":if cfg!(debug_assertions){"debug"}else{"release"},"os":std::env::consts::OS,"architecture":std::env::consts::ARCH,"rustc":rustc,"measurement":"single synchronous elapsed wall-clock per operation; excludes raw file parsing and reference scoring; not a latency or real-time guarantee"},"evaluation_complete":true,"metric_acceptance_claim":false});
    let bytes = serde_json::to_vec(&report).map_err(|e| e.to_string())?;
    if bytes.len() > 15_000_000 {
        return Err("evaluation report exceeds 15 MB bound".into());
    }
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    std::fs::write(&output, bytes).map_err(|e| e.to_string())?;
    println!(
        "{}: 15 measured terrain samples and 4 SEMI-SYNTHETIC registration cases evaluated; natural pair has no physical pose truth",
        output.display()
    );
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("dataset evaluation: {error}");
        std::process::exit(2);
    }
}
