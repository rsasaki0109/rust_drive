//! Viewed-data PAIR diagnostics, not a sequential odometry/permission evaluator.
//! Every sensor fit finishes before numeric motion-capture labels are opened.
use rustdriving_core::Vec3;
use rustdriving_localization::{
    registration3d::{Pose3, Quaternion},
    reprojection3d::{
        CameraIntrinsics, ReprojectionConfig3d, ReprojectionObservation3d, refine_reprojection,
    },
    visual_odometry3d::{Correspondence3d, VisualOdometry3dConfig, register_correspondences},
};
use serde::Deserialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs,
    io::{Cursor, Read, Write},
    path::{Component, Path},
};
mod image_features {
    pub use rustdriving_perception::image_features::*;
}
#[path = "../../../../crates/perception/src/multiscale_features.rs"]
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
fn sources() -> Value {
    json!({
        "pair_binary":sha(include_bytes!("rustdriving-rgbd-multiscale-pairs.rs")),
        "multiscale_features":sha(include_bytes!("../../../../crates/perception/src/multiscale_features.rs")),
        "image_features":sha(include_bytes!("../../../../crates/perception/src/image_features.rs")),
        "registration3d":sha(include_bytes!("../../../../crates/localization/src/registration3d.rs")),
        "visual_odometry3d":sha(include_bytes!("../../../../crates/localization/src/visual_odometry3d.rs")),
        "reprojection3d":sha(include_bytes!("../../../../crates/localization/src/reprojection3d.rs")),
        "localization_lib":sha(include_bytes!("../../../../crates/localization/src/lib.rs")),
        "perception_lib":sha(include_bytes!("../../../../crates/perception/src/lib.rs")),
        "core_lib":sha(include_bytes!("../../../../crates/core/src/lib.rs")),
        "cargo_lock":sha(include_bytes!("../../Cargo.lock")),
        "cargo_manifest":sha(include_bytes!("../../Cargo.toml")),
        "rust_toolchain":sha(include_bytes!("../../../../rust-toolchain.toml"))
    })
}
fn protocol(bytes: &[u8]) -> Value {
    json!({"schema_version":1,"algorithm":"bounded_multiscale_pair_prototype",
        "kind":"viewed_pair_regression","manifest_sha256":sha(bytes),"sources":sources(),
        "level_budgets":[200,120,80],"max_features":400,"max_matches":256,
        "max_level_candidates":1200,"levels":[1,2,4],"downsample":"2x2 integer box: (sum+2)/4, floor dimensions; repeated at quarter",
        "original_pixel":"level_pixel*scale+(scale-1)/2", "depth_ray":"fractional original feature coordinate; nearest rounded original depth sample with all-nine stencil gate",
        "depth_range_m":[0.3,5.0],"depth_patch_spread_m":0.05,"pair_gap_s":0.02,
        "hamming_max":64,"ratio_strict":0.8,"mutual":true,
        "registration":"unchanged VisualOdometry3dConfig::default()", "refinement":"unchanged ReprojectionConfig3d::default()",
        "evaluation_translation_gate_m":0.1,"evaluation_rotation_gate_rad":0.1,"gt_bracket_max_s":0.02,
        "max_frames":180,"max_raw_png_bytes":4194304,"max_total_sensor_bytes":134217728,"max_report_bytes":67108864,
        "no_sequence_pose_permission":true,"no_truth_operational":true,"no_raw_redistribution":true})
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
fn sensor(c: &Calibration, gray: &[u8], depth: Vec<u8>) -> Result<Sensor, String> {
    let a = native(c.width, c.height, gray)?;
    let b = extract_multiscale(c.width, c.height, gray).map_err(|e| format!("{e:?}"))?;
    let mut level_hashes =
        vec![json!({"level":0,"width":c.width,"height":c.height,"gray_sha256":sha(gray)})];
    let (mut w, mut h, mut p) = (c.width, c.height, gray.to_vec());
    for level in 1..3 {
        if w <= 34 || h <= 34 {
            break;
        }
        (w, h, p) = downsample_half(w, h, &p).map_err(|e| format!("{e:?}"))?;
        level_hashes.push(json!({"level":level,"width":w,"height":h,"gray_sha256":sha(&p)}));
    }
    let witness = json!({"levels":level_hashes,"native_features":a.iter().map(feature_json).collect::<Vec<_>>(),
        "multiscale_features":b.iter().map(feature_json).collect::<Vec<_>>()});
    Ok(Sensor {
        native: a,
        multi: b,
        depth,
        witness,
    })
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
fn evaluate(manifest_bytes: &[u8], raw: &Path) -> Result<Value, String> {
    let m: Manifest = serde_json::from_slice(manifest_bytes).map_err(|e| e.to_string())?;
    let c = &m.depth_calibration;
    if m.frames.len() < 2
        || m.frames.len() > 180
        || c.width != 640
        || c.height != 480
        || [c.fx, c.fy, c.cx, c.cy, c.units_per_metre]
            .iter()
            .any(|x| !x.is_finite())
        || c.fx <= 0.
        || c.fy <= 0.
        || c.units_per_metre <= 0.
    {
        return Err("unsupported bounded source calibration".into());
    }
    for pair in m.frames.windows(2) {
        if pair[1].source_index <= pair[0].source_index
            || !pair[1].depth_timestamp.is_finite()
            || pair[1].depth_timestamp <= pair[0].depth_timestamp
            || !pair[1].rgb_timestamp.is_finite()
            || pair[1].rgb_timestamp < pair[0].rgb_timestamp
        {
            return Err("invalid frame chronology".into());
        }
    }
    if !m.frames[0].depth_timestamp.is_finite() || !m.frames[0].rgb_timestamp.is_finite() {
        return Err("invalid initial stamp".into());
    }
    let mut inventory = BTreeMap::new();
    for pin in &m.files {
        if !safe(&pin.file) || inventory.insert(pin.file.as_str(), pin).is_some() {
            return Err("invalid inventory".into());
        }
    }
    let mut selected_bytes = 0usize;
    for frame in &m.frames {
        for (name, role) in [
            (&frame.rgb_file, "rgb_frame"),
            (&frame.depth_file, "depth_frame"),
        ] {
            let pin = inventory.get(name.as_str()).ok_or("missing sensor pin")?;
            if pin.role != role || pin.bytes > 4 * 1024 * 1024 {
                return Err("sensor pin role or byte bound".into());
            }
            selected_bytes = selected_bytes
                .checked_add(pin.bytes)
                .ok_or("sensor byte overflow")?;
        }
    }
    if selected_bytes > 128 * 1024 * 1024 {
        return Err("total sensor byte bound".into());
    }
    let mut sensors = Vec::new();
    let mut witnesses = Vec::new();
    for (i, f) in m.frames.iter().enumerate() {
        let rgb = verified(raw, &f.rgb_file, &inventory)?;
        let dep = verified(raw, &f.depth_file, &inventory)?;
        let gray = decode(&rgb, false, c)?;
        let depth = decode(&dep, true, c)?;
        let s = sensor(c, &gray, depth)?;
        witnesses.push(json!({"index":i,"source_index":f.source_index,"rgb_file":f.rgb_file,"depth_file":f.depth_file,
            "rgb_timestamp":f.rgb_timestamp,"depth_timestamp":f.depth_timestamp,"rgb_sha256":sha(&rgb),"depth_sha256":sha(&dep),"frontend":s.witness}));
        sensors.push(s);
    }
    let mut rows = Vec::new();
    let mut estimates = Vec::new();
    for i in 1..sensors.len() {
        let (a, b) = (&m.frames[i - 1], &m.frames[i]);
        let clock = if b.rgb_timestamp <= a.rgb_timestamp {
            Err("duplicate or stale RGB acquisition".into())
        } else if (a.rgb_timestamp - a.depth_timestamp).abs() > 0.02
            || (b.rgb_timestamp - b.depth_timestamp).abs() > 0.02
        {
            Err("sensor association gap exceeds .02s".into())
        } else {
            Ok(())
        };
        let (pn, rn) = pair(&sensors[i - 1], &sensors[i], c, false, clock.clone());
        let (pm, rm) = pair(&sensors[i - 1], &sensors[i], c, true, clock);
        rows.push(json!({"previous_frame":i-1,"current_frame":i,"native":rn,"multiscale":rm}));
        estimates.push((pn, pm));
    }
    // All extraction, association, measured registration and refinement ABOVE.
    let labels = (|| {
        let pins: Vec<_> = m
            .files
            .iter()
            .filter(|p| p.role == "evaluation_only_mocap_ground_truth")
            .collect();
        if pins.len() != 1 {
            return Err("expected exactly one evaluation-only label source".into());
        }
        truth(&verified(raw, &pins[0].file, &inventory)?)
    })();
    let mut counts = [[0usize; 3]; 2];
    for (i, (pn, pm)) in estimates.into_iter().enumerate() {
        let reference = match &labels {
            Ok(gt) => interpolate(gt, m.frames[i].depth_timestamp).and_then(|a| {
                interpolate(gt, m.frames[i + 1].depth_timestamp).map(|b| a.inverse().compose(b))
            }),
            Err(e) => Err(e.clone()),
        };
        for (k, (name, pose)) in [("native", pn), ("multiscale", pm)].into_iter().enumerate() {
            let s = score(pose, reference.clone());
            counts[k][0] += usize::from(pose.is_some());
            counts[k][1] += usize::from(s["within_accuracy_gates"] == true);
            counts[k][2] += usize::from(s["reference_valid"] == true);
            rows[i][name]["evaluation"] = s;
        }
    }
    Ok(
        json!({"schema_version":1,"dataset":m.dataset,"protocol":protocol(manifest_bytes),"ground_truth_operational":false,"frames":witnesses,"pairs":rows,
        "summary":{"total_pairs":m.frames.len()-1,"native":{"fitted":counts[0][0],"accurate":counts[0][1],"scorable":counts[0][2]},"multiscale":{"fitted":counts[1][0],"accurate":counts[1][1],"scorable":counts[1][2]}},"evaluation_label_failure":labels.as_ref().err(),
        "limits":["Previously viewed indoor regression only; no held-out generalization claim.","Each adjacent pair independently fitted; no composed trajectory, accepted-reference renewal, expiry, restart or permission inference.","Original default registration and refinement gates and strict nine-depth stencil remain unchanged.","Fixed multi-level quotas reduce native allocation; either improvement or regression is possible.","Fractional feature ray uses nearest original measured depth; no calibrated covariance or real-vehicle claim."]}),
    )
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
fn frontend_controls() -> Result<Value, String> {
    let mut cases = Vec::new();
    for kind in ["repeat_twofold", "clockwise_quarter_turn"] {
        let (w, h) = if kind == "repeat_twofold" {
            (160, 120)
        } else {
            (320, 240)
        };
        let previous = texture(w, h);
        let (cw, ch, current) = if kind == "repeat_twofold" {
            (
                2 * w,
                2 * h,
                (0..4 * w * h)
                    .map(|i| previous[(i / (2 * w) / 2) * w + (i % (2 * w)) / 2])
                    .collect::<Vec<_>>(),
            )
        } else {
            let mut pixels = vec![0; w * h];
            for y in 0..h {
                for x in 0..w {
                    pixels[x * h + h - 1 - y] = previous[y * w + x];
                }
            }
            (h, w, pixels)
        };
        let mut c = Calibration {
            width: w,
            height: h,
            fx: 240.,
            fy: 240.,
            cx: 0.,
            cy: 0.,
            units_per_metre: 5000.,
            invalid_depth: 0,
        };
        // Feature-only controls carry no depth/3D or pose claim.
        let a = sensor(&c, &previous, Vec::new())?;
        c.width = cw;
        c.height = ch;
        let b = sensor(&c, &current, Vec::new())?;
        let mut row = json!({"kind":kind,"previous_dimensions":[w,h],"current_dimensions":[cw,ch],
            "previous":a.witness,"current":b.witness,"geometric_claim":"analytic pixel correspondence only; no 3D pose acceptance"});
        for (name, fa, fb) in [
            ("native", &a.native, &b.native),
            ("multiscale", &a.multi, &b.multi),
        ] {
            let matches = match_multiscale(fa, fb).map_err(|e| format!("{e:?}"))?;
            row[name]=json!(matches.iter().map(|m|json!({"previous_index":m.previous_index,"current_index":m.current_index,"hamming_distance":m.hamming_distance})).collect::<Vec<_>>());
        }
        cases.push(row);
    }
    Ok(json!(cases))
}
fn control() -> Result<Value, String> {
    let c = Calibration {
        width: 320,
        height: 240,
        fx: 240.,
        fy: 240.,
        cx: 159.5,
        cy: 119.5,
        units_per_metre: 5000.,
        invalid_depth: 0,
    };
    // A rendered textured fronto-parallel surface at measured 1.5m. Camera
    // translation projects every point +8,+4 pixels; both depth maps remain1.5m.
    let gray = texture(c.width, c.height);
    let mut moved = vec![0; c.width * c.height];
    for y in 4..c.height {
        for x in 8..c.width {
            moved[y * c.width + x] = gray[(y - 4) * c.width + x - 8];
        }
    }
    let depth: Vec<u8> = (0..c.width * c.height)
        .flat_map(|_| 7500u16.to_be_bytes())
        .collect();
    let a = sensor(&c, &gray, depth.clone())?;
    let b = sensor(&c, &moved, depth)?;
    let (pn, rn) = pair(&a, &b, &c, false, Ok(()));
    let (pm, rm) = pair(&a, &b, &c, true, Ok(()));
    // Expected pose is consulted only after both actual sensor fits finish.
    let expected = Pose3 {
        translation: Vec3::new(-8. * 1.5 / c.fx, -4. * 1.5 / c.fy, 0.),
        rotation: Quaternion::identity(),
    };
    Ok(
        json!({"schema_version":1,"kind":"analytic_rendered_measured_pair","sources":sources(),"calibration":{"width":320,"height":240,"fx":240.,"fy":240.,"cx":159.5,"cy":119.5,"units_per_metre":5000.,"invalid_depth":0},
        "render":{"depth_m":1.5,"shift_pixels":[8,4],"texture":"aperiodic checker: base40/170 plus (x*97+y*193+x*y*17)%61"},
        "frames":[a.witness,b.witness],"frontend_controls":frontend_controls()?,"native":rn,"multiscale":rm,"native_evaluation":score(pn,Ok(expected)),"multiscale_evaluation":score(pm,Ok(expected))}),
    )
}
fn run() -> Result<(), String> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let mut options = BTreeMap::new();
    let mut mode = "evaluate";
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--control" => {
                mode = "control";
                i += 1;
            }
            "--prepare-freeze" => {
                mode = "freeze";
                i += 1;
            }
            key @ ("--manifest" | "--raw" | "--freeze" | "--output") => {
                let v = args.get(i + 1).ok_or("missing argument")?;
                if options.insert(key, v.as_str()).is_some() {
                    return Err("duplicate argument".into());
                }
                i += 2;
            }
            _ => return Err("unknown argument".into()),
        }
    }
    let output = Path::new(*options.get("--output").ok_or("--output required")?);
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    }
    // Never overwrite a prior result, including a failed or first-trial file.
    let mut destination = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)
        .map_err(|e| e.to_string())?;
    let report = if mode == "control" {
        control()?
    } else {
        let bytes = bounded(
            Path::new(*options.get("--manifest").ok_or("--manifest required")?),
            512 * 1024,
        )?;
        if mode == "freeze" {
            protocol(&bytes)
        } else {
            let frozen: Value = serde_json::from_slice(&bounded(
                Path::new(*options.get("--freeze").ok_or("--freeze required")?),
                512 * 1024,
            )?)
            .map_err(|e| e.to_string())?;
            if frozen != protocol(&bytes) {
                return Err("source/protocol freeze mismatch".into());
            }
            evaluate(
                &bytes,
                Path::new(*options.get("--raw").ok_or("--raw required")?),
            )?
        }
    };
    let bytes = serde_json::to_vec(&report).map_err(|e| e.to_string())?;
    if bytes.len() > 64 * 1024 * 1024 {
        return Err("report byte bound".into());
    }
    destination.write_all(&bytes).map_err(|e| e.to_string())?;
    if report["evaluation_label_failure"].is_string() {
        return Err("invalid evaluation labels; unscorable sensor rows preserved".into());
    }
    println!(
        "{}",
        report.get("summary").unwrap_or(&json!({"output":output}))
    );
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("multiscale pair diagnostic: {e}");
        std::process::exit(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn real_rendered_depth_correspondences_register_and_refine() {
        let r = control().unwrap();
        for mode in ["native", "multiscale"] {
            assert_eq!(r[mode]["accepted"], true, "{}", r[mode]);
            let e = &r[format!("{mode}_evaluation")];
            assert!(e["translation_error_m"].as_f64().unwrap() < 1e-6);
            assert!(e["rotation_error_rad"].as_f64().unwrap() < 1e-6);
            assert!(r[mode]["refinement"]["final_rms_px"].as_f64().unwrap() < 1e-6);
        }
    }
    #[test]
    fn measured_depth_stencil_failures_remain_visible() {
        let c = Calibration {
            width: 40,
            height: 40,
            fx: 100.,
            fy: 100.,
            cx: 20.,
            cy: 20.,
            units_per_metre: 5000.,
            invalid_depth: 0,
        };
        let feature = MultiscaleFeature {
            feature: image_features::ImageFeature {
                x: 20.5,
                y: 20.5,
                score: 10,
                orientation: 0.,
                descriptor: [0; 4],
            },
            level: 1,
            level_x: 10.,
            level_y: 10.,
        };
        let mut depth: Vec<u8> = (0..1600).flat_map(|_| 7500u16.to_be_bytes()).collect();
        let (p, w) = depth_point(&depth, &feature, &c);
        assert_eq!(w["sample_x"], 21);
        assert_eq!(p.unwrap().x, 0.0075);
        depth[2 * (20 * 40 + 20)] = 0;
        depth[2 * (20 * 40 + 20) + 1] = 0;
        let (p, w) = depth_point(&depth, &feature, &c);
        assert!(p.is_err());
        assert_eq!(w["raw_stencil"].as_array().unwrap().len(), 9);
        let a = sensor(&c, &vec![127; 1600], depth.clone()).unwrap();
        let b = sensor(&c, &vec![127; 1600], depth).unwrap();
        let (p, r) = pair(
            &a,
            &b,
            &c,
            true,
            Err("duplicate or stale RGB acquisition".into()),
        );
        assert!(p.is_none());
        assert_eq!(r["accepted"], false);
        assert!(r.get("relative_estimate").is_none());
    }
}
