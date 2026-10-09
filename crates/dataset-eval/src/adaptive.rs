//! Separate opt-in evaluation of the adaptive classifier; the original report
//! and parameter defaults remain reproducible through the original command.
use super::{Confusion, canonical_sha, entry, file_path, key, object_config};
use rustdrive_core::Vec3;
use rustdrive_dataset_eval::{parse_las_labels, parse_las_points, parse_pcd, read_bounded};
use rustdrive_perception::objects3d::{ObjectClusterConfig, cluster_objects};
use rustdrive_perception::terrain_adaptive::{AdaptiveTerrainConfig, classify_ground_adaptive};
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Instant;

fn hash_file(path: &Path, python: &str) -> Result<String, String> {
    let output = Command::new(python)
        .args([
            "-c",
            "import hashlib,sys; print(hashlib.sha256(open(sys.argv[1],'rb').read()).hexdigest())",
        ])
        .arg(path)
        .output()
        .map_err(|e| e.to_string())?;
    if !output.status.success() {
        return Err("file SHA256 helper failed".into());
    }
    let digest = String::from_utf8(output.stdout).map_err(|e| e.to_string())?;
    if digest.trim().len() != 64 {
        return Err("invalid file SHA256 digest".into());
    }
    Ok(digest.trim().to_owned())
}
fn verify_file(path: &Path, e: &Value, python: &str) -> Result<(), String> {
    if std::fs::metadata(path).map_err(|e| e.to_string())?.len()
        != e["bytes"].as_u64().ok_or("missing manifest bytes")?
        || hash_file(path, python)? != e["sha256"].as_str().ok_or("missing manifest SHA256")?
    {
        return Err(format!("{}: source hash/size mismatch", path.display()));
    }
    Ok(())
}
fn parameters(c: &AdaptiveTerrainConfig) -> Value {
    json!({"cell_size_m":c.cell_size_m,"support_radius_m":c.support_radius_m,"max_slope":c.max_slope,"max_residual_m":c.max_residual_m,"min_support_neighbors":c.min_support_neighbors,"max_support_neighbors":c.max_support_neighbors,"min_supported_cells":c.min_supported_cells,"min_supported_fraction":c.min_supported_fraction,"max_points":c.max_points,"max_cells":c.max_cells,"max_candidate_work":c.max_candidate_work,"coordinate_bound_m":c.coordinate_bound_m})
}
fn apply_config(c: &mut AdaptiveTerrainConfig, value: &Value) -> Result<(), String> {
    let fields = value
        .as_object()
        .ok_or("configuration must be a JSON object")?;
    for (name, value) in fields {
        macro_rules! float {
            ($field:ident) => {
                c.$field = value
                    .as_f64()
                    .ok_or(concat!(stringify!($field), " must be numeric"))?
            };
        }
        macro_rules! count {
            ($field:ident) => {
                c.$field = usize::try_from(
                    value
                        .as_u64()
                        .ok_or(concat!(stringify!($field), " must be nonnegative integer"))?,
                )
                .map_err(|_| "configuration integer overflow")?
            };
        }
        match name.as_str() {
            "cell_size_m" => float!(cell_size_m),
            "support_radius_m" => float!(support_radius_m),
            "max_slope" => float!(max_slope),
            "max_residual_m" => float!(max_residual_m),
            "min_support_neighbors" => count!(min_support_neighbors),
            "max_support_neighbors" => count!(max_support_neighbors),
            "min_supported_cells" => count!(min_supported_cells),
            "min_supported_fraction" => float!(min_supported_fraction),
            "max_points" => count!(max_points),
            "max_cells" => count!(max_cells),
            "max_candidate_work" => count!(max_candidate_work),
            "coordinate_bound_m" => float!(coordinate_bound_m),
            _ => return Err(format!("unknown configuration field {name}")),
        }
    }
    Ok(())
}
fn check_partition(
    ground: &[usize],
    other: &[usize],
    count: usize,
) -> Result<BTreeSet<usize>, String> {
    let mut all = BTreeSet::new();
    for &i in ground.iter().chain(other) {
        if i >= count || !all.insert(i) {
            return Err("classifier returned duplicate/out-of-range original indices".into());
        }
    }
    if all.len() != count {
        return Err("classifier indices do not partition the input".into());
    }
    Ok(ground.iter().copied().collect())
}
pub(super) fn run() -> Result<(), String> {
    let mut repository = PathBuf::from(".");
    let mut data = None;
    let mut output = PathBuf::from("artifacts/adaptive-ground-evaluation.json");
    let mut python = std::env::var("DATASET_EVAL_PYTHON").unwrap_or_else(|_| "python3".into());
    let mut split = "calibration_original".to_owned();
    let mut dataset = "isprs-terrain".to_owned();
    let mut config_file = None;
    let mut freeze_file = None;
    let mut args = std::env::args().skip(2);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--repository" => repository = PathBuf::from(args.next().ok_or("missing repository")?),
            "--data-root" => data = Some(PathBuf::from(args.next().ok_or("missing data root")?)),
            "--output" => output = PathBuf::from(args.next().ok_or("missing output")?),
            "--python" => python = args.next().ok_or("missing python")?,
            "--split" => split = args.next().ok_or("missing split")?,
            "--dataset" => dataset = args.next().ok_or("missing dataset")?,
            "--config" => config_file = Some(PathBuf::from(args.next().ok_or("missing config")?)),
            "--freeze" => {
                freeze_file = Some(PathBuf::from(args.next().ok_or("missing freeze file")?))
            }
            "--help" | "-h" => {
                println!(
                    "rustdrive-dataset-eval adaptive-ground [--repository REPO] [--data-root RAW_BASE] [--dataset isprs-terrain|PDAL_DATASET] [--split calibration_original|regression|fresh_heldout] [--config PARAMS.json] [--freeze FREEZE.json] [--output REPORT.json] [--python PYTHON]\nNo downloads; sources must match pinned manifest hashes. Fresh held-out evaluation requires an externally recorded source/configuration freeze before invocation."
                );
                return Ok(());
            }
            _ => return Err(format!("unknown adaptive option {arg}")),
        }
    }
    if Path::new(&dataset).components().count() != 1 || dataset == ".." {
        return Err("dataset must be a simple directory name".into());
    }
    let list=match split.as_str(){"calibration_original" if dataset=="isprs-terrain"=>"calibration_samples","regression" if dataset=="isprs-terrain"=>"held_out_samples","fresh_heldout" if dataset!="isprs-terrain"=>"held_out_samples",_=>return Err("original ISPRS sites are calibration/regression only; fresh-heldout needs a different dataset".into())};
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    std::fs::write(
        &output,
        b"{\"schema\":\"rustdrive-adaptive-ground-v1\",\"evaluation_complete\":false}\n",
    )
    .map_err(|e| e.to_string())?;
    let data = data.unwrap_or_else(|| repository.join("data"));
    let manifest_path = repository.join("data").join(&dataset).join("manifest.json");
    let manifest_bytes = read_bounded(&manifest_path)?;
    let manifest: Value = serde_json::from_slice(&manifest_bytes).map_err(|e| e.to_string())?;
    let mut c = AdaptiveTerrainConfig::default();
    if let Some(path) = config_file {
        apply_config(
            &mut c,
            &serde_json::from_slice(&read_bounded(&path)?).map_err(|e| e.to_string())?,
        )?;
    }
    c.validate()?;
    let oc = ObjectClusterConfig::default();
    let frozen = json!({"algorithm":"rustdrive_perception::terrain_adaptive::classify_ground_adaptive","terrain_config":parameters(&c),"object_config":object_config(&oc)});
    let parameter_hash = canonical_sha(&frozen, &python)?;
    let algorithm_source_path = repository.join("crates/perception/src/terrain_adaptive.rs");
    let algorithm_source_sha256 = hash_file(&algorithm_source_path, &python)?;
    let manifest_sha256 = hash_file(&manifest_path, &python)?;
    let freeze = if let Some(path) = freeze_file {
        let value: Value =
            serde_json::from_slice(&read_bounded(&path)?).map_err(|e| e.to_string())?;
        if value["algorithm_source_sha256"] != algorithm_source_sha256
            || value["frozen_parameters_sha256"] != parameter_hash
            || value["manifest_sha256"] != manifest_sha256
        {
            return Err(
                "freeze source/configuration/manifest hashes disagree with current evaluation"
                    .into(),
            );
        }
        Some(value)
    } else if split == "fresh_heldout" {
        return Err("fresh_heldout requires --freeze FILE created before evaluation".into());
    } else {
        None
    };
    let mut rows = Vec::new();
    let mut total = Confusion::default();
    let mut sites = BTreeMap::<String, Confusion>::new();
    let samples = manifest[list]
        .as_array()
        .ok_or("manifest split list missing")?;
    for sample in samples {
        let sample = sample.as_str().ok_or("sample name must be string")?;
        let input = entry(&manifest, sample, "input_cloud")?;
        let path = file_path(&data, &dataset, input)?;
        verify_file(&path, input, &python)?;
        let raw_bytes = read_bounded(&path)?;
        let is_las = input["encoding"]
            .as_str()
            .is_some_and(|e| e.to_ascii_lowercase().contains("las"));
        let (raw, input_geometry) = if is_las {
            let geometry = parse_las_points(&raw_bytes)?;
            (
                geometry.points,
                json!({"format":"uncompressed LAS", "version_minor":geometry.version_minor,"point_format":geometry.point_format,"scales":geometry.scales,"offsets":geometry.offsets,"axis_units":"source manifest units; integer XYZ multiplied by LAS scale then offset"}),
            )
        } else {
            (
                parse_pcd(&raw_bytes)?,
                json!({"format":"PCD binary_compressed XYZ float32","axis_units":"meters per source manifest"}),
            )
        };
        if raw.len() as u64
            != input["points"]
                .as_u64()
                .ok_or("input point count missing")?
        {
            return Err("input point count differs from manifest".into());
        }
        let unit_to_m = if is_las {
            let factor = input["unit_to_m"]
                .as_f64()
                .or_else(|| manifest["unit_to_m"].as_f64())
                .ok_or("LAS manifest must declare unit_to_m")?;
            let units = input["coordinate_units"]
                .as_str()
                .or_else(|| manifest["coordinate_units"].as_str())
                .ok_or("LAS manifest must declare coordinate_units")?;
            let expected = match units {
                "foot" | "feet" | "international_foot" => 0.3048,
                "us_survey_foot" => 1200. / 3937.,
                "meter" | "metre" | "m" => 1.,
                _ => return Err("unsupported LAS manifest coordinate units".into()),
            };
            if !factor.is_finite() || (factor - expected).abs() > 1e-12 {
                return Err("LAS unit_to_m conflicts with declared coordinate units".into());
            }
            factor
        } else {
            1.
        };
        let raw_origin = *raw.first().ok_or("empty input cloud")?;
        let origin = Vec3::new(
            raw_origin.x * unit_to_m,
            raw_origin.y * unit_to_m,
            raw_origin.z * unit_to_m,
        );
        let points: Vec<_> = raw
            .iter()
            .map(|p| {
                Vec3::new(
                    (p.x - raw_origin.x) * unit_to_m,
                    (p.y - raw_origin.y) * unit_to_m,
                    (p.z - raw_origin.z) * unit_to_m,
                )
            })
            .collect();
        let start = Instant::now();
        let classified = classify_ground_adaptive(&points, &c)?;
        let classification_ms = start.elapsed().as_secs_f64() * 1000.;
        let predicted = check_partition(
            &classified.ground_indices,
            &classified.non_ground_indices,
            raw.len(),
        )?;
        // Coordinates alone enter the algorithm. Labels are decoded/read only now.
        let (labels, label_metadata) = if is_las {
            let labels = parse_las_labels(&raw_bytes)?;
            let info = json!({"policy":"LAS class 2 is ground; withheld/class 0, 1, 7, 8, 12 and reserved classes excluded from scoring; all coordinates still enter classification","source_sha256":input["sha256"]});
            (labels, info)
        } else {
            let reference = entry(&manifest, sample, "ground_reference")?;
            let reference_path = file_path(&data, &dataset, reference)?;
            verify_file(&reference_path, reference, &python)?;
            let truth = parse_pcd(&read_bounded(&reference_path)?)?;
            if truth.len() as u64
                != reference["points"]
                    .as_u64()
                    .ok_or("reference point count missing")?
            {
                return Err("reference point count differs from manifest".into());
            }
            let keys: BTreeSet<_> = raw.iter().copied().map(key).collect();
            let labelled: BTreeSet<_> = truth.iter().copied().map(key).collect();
            if !labelled.is_subset(&keys) {
                return Err("ground-reference is not exact float32 XYZ subset".into());
            }
            (
                raw.iter()
                    .map(|p| Some(labelled.contains(&key(*p))))
                    .collect(),
                json!({"policy":"original float32 XYZ membership in separate ground-reference cloud; duplicates share label","reference_sha256":reference["sha256"],"reference_points":truth.len(),"reference_unique_points":labelled.len(),"raw_unique_points":keys.len()}),
            )
        };
        let mut counts = Confusion::default();
        let mut excluded = Vec::new();
        for (i, label) in labels.iter().enumerate() {
            match (predicted.contains(&i), label) {
                (_, None) => excluded.push(i),
                (true, Some(true)) => counts.tp += 1,
                (true, Some(false)) => counts.fp += 1,
                (false, Some(true)) => counts.fn_ += 1,
                (false, Some(false)) => counts.tn += 1,
            }
        }
        total = total.add(counts);
        let site = input["site"]
            .as_str()
            .map(str::to_owned)
            .unwrap_or_else(|| {
                if dataset == "isprs-terrain" && sample.starts_with("samp") {
                    format!("site{}", sample.chars().nth(4).unwrap_or('?'))
                } else {
                    sample.to_owned()
                }
            });
        sites
            .entry(site.clone())
            .and_modify(|v| *v = v.add(counts))
            .or_insert(counts);
        let start = Instant::now();
        let objects = match cluster_objects(&points, &classified.non_ground_indices, &oc) {
            Ok(o) => {
                json!({"status":"evaluated","cluster_count":o.objects.len(),"noise_indices":o.noise_indices,"candidate_work":o.candidate_work,"occupied_voxels":o.occupied_voxels,"aabbs":o.objects.iter().map(|v|json!({"min":v.min,"max":v.max,"center":v.center,"point_count":v.point_count,"original_point_indices":v.indices})).collect::<Vec<_>>()})
            }
            Err(e) => json!({"status":"rejected","reason":e}),
        };
        let cluster_ms = start.elapsed().as_secs_f64() * 1000.;
        let d = &classified.diagnostics;
        rows.push(json!({"sample":sample,"site":site,"split":split,"points":raw.len(),"origin_xyz_m":origin,"raw_origin_xyz_source_units":raw_origin,"unit_to_m":unit_to_m,"input_sha256":input["sha256"],"input_geometry":input_geometry,"label_metadata":label_metadata,"excluded_scoring_indices":excluded,"confusion":counts.value(),"metrics":counts.metrics(),"ground_indices":classified.ground_indices,"non_ground_indices":classified.non_ground_indices,"classification_diagnostics":{"occupied_cells":d.occupied_cells,"supported_cells":d.supported_cells,"rejected_low_cells":d.rejected_low_cells,"rejected_elevated_cells":d.rejected_elevated_cells,"candidate_work":d.candidate_work,"supported_fraction":d.supported_fraction,"confident":d.confident},"classification_ms":classification_ms,"objects":objects,"cluster_ms":cluster_ms}));
        verify_file(&path, input, &python)?;
        if !is_las {
            verify_file(
                &file_path(
                    &data,
                    &dataset,
                    entry(&manifest, sample, "ground_reference")?,
                )?,
                entry(&manifest, sample, "ground_reference")?,
                &python,
            )?;
        }
        eprintln!(
            "adaptive {sample} {split}: ground F1 {}",
            counts.metrics()["f1"]
        );
    }
    if samples.is_empty()
        || read_bounded(&manifest_path)? != manifest_bytes
        || hash_file(&algorithm_source_path, &python)? != algorithm_source_sha256
    {
        return Err("empty sample split or changed manifest".into());
    }
    let per_site: Value = sites
        .into_iter()
        .map(|(site, c)| (site, json!({"confusion":c.value(),"metrics":c.metrics()})))
        .collect();
    let report = json!({"schema":"rustdrive-adaptive-ground-v1","evaluation_complete":true,"scope":"Offline measured airborne geometry; original ISPRS calibration/regression and independently sourced fresh-heldout are distinct; no driving or realtime validation claim","dataset":dataset,"split":split,"manifest":manifest,"manifest_sha256":manifest_sha256,"algorithm_source_sha256":algorithm_source_sha256,"freeze":freeze,"frozen_parameters":frozen,"frozen_parameters_sha256":parameter_hash,"terrain_samples":rows,"terrain_aggregate":{"confusion":total.value(),"metrics":total.metrics()},"terrain_per_site":per_site,"hash_verification":{"raw_sha256_verified":true,"before_and_after":true},"label_policy":"Algorithm sees measured XYZ only; reference labels are decoded after classification, scoring exclusions never change algorithm input","timing_context":{"profile":if cfg!(debug_assertions){"debug"}else{"release"},"measurement":"single synchronous wall-clock; excludes parsing/scoring; not realtime guarantee"},"metric_acceptance_claim":false});
    let bytes = serde_json::to_vec(&report).map_err(|e| e.to_string())?;
    if bytes.len() > 30_000_000 {
        return Err("adaptive report exceeds 30 MB bound".into());
    }
    std::fs::write(&output, bytes).map_err(|e| e.to_string())?;
    println!(
        "{}: {} measured samples, split {}",
        output.display(),
        samples.len(),
        split
    );
    Ok(())
}
