//! Recorded XYZ diagnostic: frozen PMF ground selection, dense XY control and XYZ components.
use rustdriving_core::{Detection, Vec2, Vec3};
use rustdriving_perception::{
    ObjectClusterConfig, TerrainConfig, classify_ground, cluster_objects,
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    collections::HashSet,
    fs::File,
    io::{Read, Write},
    path::{Component, Path},
};
const MAX_INPUT: usize = 4 * 1024 * 1024;
const MAX_SCAN: usize = 8 * 1024 * 1024;
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Profile {
    z_min: f64,
    z_max: f64,
    min_range: f64,
    max_range: f64,
    bins: usize,
}
impl Profile {
    fn validate(self) -> Result<(), String> {
        if ![self.z_min, self.z_max, self.min_range, self.max_range]
            .into_iter()
            .all(f64::is_finite)
            || self.z_min < -5.0
            || self.z_max > 5.0
            || self.z_min >= self.z_max
            || self.min_range < 0.2
            || self.min_range >= self.max_range
            || self.max_range > 200.0
            || !(4..=4096).contains(&self.bins)
        {
            return Err("invalid bounded projection profile".into());
        }
        Ok(())
    }
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Frame {
    id: String,
    path: String,
    stamp: f64,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    profile: Profile,
    frames: Vec<Frame>,
}
impl Input {
    fn validate(&self) -> Result<(), String> {
        self.profile.validate()?;
        if self.frames.is_empty() || self.frames.len() > 128 {
            return Err("one to 128 recorded frames required".into());
        }
        let mut ids = HashSet::new();
        for f in &self.frames {
            if f.id.is_empty()
                || f.id.len() > 160
                || !ids.insert(&f.id)
                || !f.stamp.is_finite()
                || f.stamp < 0.0
                || f.path.is_empty()
                || Path::new(&f.path)
                    .components()
                    .any(|c| !matches!(c, Component::Normal(_)))
            {
                return Err("invalid frame identity, timestamp or relative point path".into());
            }
        }
        Ok(())
    }
}
fn read_bounded(path: &Path, cap: usize) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let file = File::open(path)?;
    if !file.metadata()?.is_file() {
        return Err("regular input file required".into());
    }
    let mut bytes = Vec::new();
    file.take(cap as u64 + 1).read_to_end(&mut bytes)?;
    if bytes.len() > cap {
        return Err("recorded input exceeds byte budget".into());
    }
    Ok(bytes)
}

const Z_MIN: f64 = -3.0;
const Z_MAX: f64 = 3.0;
struct Cropped {
    points: Vec<Vec3>,
    selected_rows: Vec<usize>,
    total_rows: usize,
    invalid_xyz_rows: usize,
}
fn crop(bytes: &[u8], stamp: f64) -> Result<Cropped, String> {
    if bytes.is_empty()
        || bytes.len() > MAX_SCAN
        || !bytes.len().is_multiple_of(16)
        || !stamp.is_finite()
        || stamp < 0.0
    {
        return Err(
            "finite timestamp and bounded complete little-endian XYZI rows required".into(),
        );
    }
    let mut points = Vec::new();
    let mut selected_rows = Vec::new();
    let mut invalid_xyz_rows = 0;
    for (index, row) in bytes.chunks_exact(16).enumerate() {
        let value = |offset: usize| {
            f64::from(f32::from_le_bytes(
                row[offset..offset + 4].try_into().expect("four bytes"),
            ))
        };
        let point = Vec3::new(value(0), value(4), value(8));
        if !point.finite() {
            invalid_xyz_rows += 1;
            continue;
        }
        if !(2.0..=45.0).contains(&point.x.hypot(point.y)) || !(Z_MIN..=Z_MAX).contains(&point.z) {
            continue;
        }
        points.push(point);
        selected_rows.push(index);
    }
    Ok(Cropped {
        points,
        selected_rows,
        total_rows: bytes.len() / 16,
        invalid_xyz_rows,
    })
}
fn settings() -> Value {
    let g = TerrainConfig::default();
    let c = ObjectClusterConfig::default();
    json!({
        "volume":{"z_min":Z_MIN,"z_max":Z_MAX,"min_range":2.0,"max_range":45.0},
        "terrain":{
            "cell_size_m":g.cell_size_m,"initial_height_m":g.initial_height_m,
            "max_height_m":g.max_height_m,"max_slope":g.max_slope,
            "window_radii_cells":g.window_radii_cells,"min_support_neighbors":g.min_support_neighbors,
            "min_supported_cells":g.min_supported_cells,"min_supported_fraction":g.min_supported_fraction,
            "max_points":g.max_points,"max_cells":g.max_cells,"max_candidate_work":g.max_candidate_work,
            "coordinate_bound_m":g.coordinate_bound_m
        },
        "clustering":{
            "tolerance_m":c.tolerance_m,"voxel_size_m":c.voxel_size_m,"min_points":c.min_points,
            "max_points":c.max_points,"max_cluster_points":c.max_cluster_points,"max_clusters":c.max_clusters,
            "max_candidate_work":c.max_candidate_work,"coordinate_bound_m":c.coordinate_bound_m
        },
        "proxy":{"padding_m":0.65,"min_radius_m":0.85,"max_radius_m":3.0}
    })
}
#[derive(Serialize)]
struct ObjectResult {
    min: Vec3,
    max: Vec3,
    point_count: usize,
    indices: Vec<usize>,
    proxy: Detection,
}
#[derive(Serialize)]
struct FrameResult {
    id: String,
    stamp: f64,
    total_rows: usize,
    invalid_xyz_rows: usize,
    selected_rows: Vec<usize>,
    ground_indices: Vec<usize>,
    retained_indices: Vec<usize>,
    ground: Value,
    dense_xy: Vec<Detection>,
    xy_objects: Vec<ObjectResult>,
    xy_noise_indices: Vec<usize>,
    xy_object_work: usize,
    xy_occupied_voxels: usize,
    objects: Vec<ObjectResult>,
    noise_indices: Vec<usize>,
    object_work: usize,
    occupied_voxels: usize,
    sweep_present: bool,
}
fn proxy_objects(objects: Vec<rustdriving_perception::ObjectAabb>) -> Vec<ObjectResult> {
    objects
        .into_iter()
        .map(|o| {
            let radius =
                (((o.max.x - o.min.x).hypot(o.max.y - o.min.y)) * 0.5 + 0.65).clamp(0.85, 3.0);
            ObjectResult {
                min: o.min,
                max: o.max,
                point_count: o.point_count,
                indices: o.indices,
                proxy: Detection {
                    center: Vec2::new(o.center.x, o.center.y),
                    radius,
                },
            }
        })
        .collect()
}
fn evaluate(bytes: &[u8], id: String, stamp: f64) -> Result<FrameResult, String> {
    let crop = crop(bytes, stamp)?;
    let ground = classify_ground(&crop.points, &TerrainConfig::default())?;
    // Support is geometric, not semantic truth. Insufficient support never removes points.
    let retained_indices: Vec<usize> = if ground.diagnostics.confident {
        ground.non_ground_indices.clone()
    } else {
        (0..crop.points.len()).collect()
    };
    let xy_points: Vec<_> = crop
        .points
        .iter()
        .map(|p| Vec3::new(p.x, p.y, 0.0))
        .collect();
    let xy = cluster_objects(
        &xy_points,
        &retained_indices,
        &ObjectClusterConfig::default(),
    )?;
    let xy_objects = proxy_objects(xy.objects);
    let dense_xy = xy_objects
        .iter()
        .map(|o| Detection {
            center: o.proxy.center,
            radius: o.proxy.radius,
        })
        .collect();
    let components = cluster_objects(
        &crop.points,
        &retained_indices,
        &ObjectClusterConfig::default(),
    )?;
    let objects = proxy_objects(components.objects);
    let d = &ground.diagnostics;
    let diagnostics = json!({
        "occupied_cells":d.occupied_cells,"candidate_ground_cells":d.candidate_ground_cells,
        "supported_cells":d.supported_cells,"rejected_cells":d.rejected_cells,
        "candidate_work":d.candidate_work,"supported_fraction":d.supported_fraction,"confident":d.confident
    });
    Ok(FrameResult {
        id,
        stamp,
        total_rows: crop.total_rows,
        invalid_xyz_rows: crop.invalid_xyz_rows,
        selected_rows: crop.selected_rows,
        ground_indices: ground.ground_indices,
        retained_indices,
        ground: diagnostics,
        dense_xy,
        xy_objects,
        xy_noise_indices: xy.noise_indices,
        xy_object_work: xy.candidate_work,
        xy_occupied_voxels: xy.occupied_voxels,
        objects,
        noise_indices: components.noise_indices,
        object_work: components.candidate_work,
        occupied_voxels: components.occupied_voxels,
        sweep_present: false,
    })
}
#[derive(Serialize)]
struct Report {
    schema_version: u32,
    scope: &'static str,
    settings: Value,
    frames: Vec<FrameResult>,
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 2 {
        return Err("usage: cadc_volume_study INPUT.json".into());
    }
    let path = Path::new(&args[1]);
    let input: Input = serde_json::from_slice(&read_bounded(path, MAX_INPUT)?)?;
    input.validate()?;
    if input.profile.z_min != -0.9
        || input.profile.z_max != -0.3
        || input.profile.min_range != 2.0
        || input.profile.max_range != 45.0
        || input.profile.bins != 360
    {
        return Err("recorded baseline sensor profile differs from the frozen comparison".into());
    }
    let base = path.parent().ok_or("input parent missing")?;
    let mut frames = Vec::new();
    for frame in input.frames {
        frames.push(evaluate(
            &read_bounded(&base.join(frame.path), MAX_SCAN)?,
            frame.id,
            frame.stamp,
        )?);
    }
    let report = Report {
        schema_version: 1,
        scope: "Recorded LiDAR-local XYZ, fixed crop, existing PMF and XYZ components; the XY control zeroes Z before the same connectivity/proxy rules. Unknown ground support keeps every cropped point. No calibrated sweep, semantic AP, inferred hidden shape, weather calibration or driving qualification.",
        settings: settings(),
        frames,
    };
    let mut out = std::io::BufWriter::new(std::io::stdout().lock());
    serde_json::to_writer(&mut out, &report)?;
    out.write_all(b"\n")?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    fn bytes(rows: &[[f32; 4]]) -> Vec<u8> {
        rows.iter()
            .flatten()
            .flat_map(|v| v.to_le_bytes())
            .collect()
    }
    #[test]
    fn xyz_crop_preserves_dense_measured_returns_and_ignores_reflectance() {
        let c = crop(
            &bytes(&[
                [3.0, 0.0, 1.0, f32::NAN],
                [3.1, 0.0, 2.0, 1.0],
                [1.0, 0.0, 0.0, 1.0],
                [4.0, 0.0, 4.0, 1.0],
                [f32::NAN, 0.0, 0.0, 1.0],
            ]),
            0.0,
        )
        .unwrap();
        assert_eq!(c.selected_rows, vec![0, 1]);
        assert_eq!(c.invalid_xyz_rows, 1);
        assert_eq!(c.points[1].z, 2.0);
    }
    #[test]
    fn malformed_binary_timestamp_and_scan_budget_are_rejected() {
        for raw in [vec![], vec![0; 17], vec![0; MAX_SCAN + 16]] {
            assert!(crop(&raw, 0.0).is_err());
        }
        assert!(crop(&[0; 16], f64::NAN).is_err());
        assert!(crop(&[0; 16], -1.0).is_err());
    }
    #[test]
    fn insufficient_ground_support_keeps_measured_points() {
        let f = evaluate(&bytes(&[[5.0, 0.0, -1.5, 1.0]]), "sparse".into(), 0.0).unwrap();
        assert_eq!(f.ground["confident"], false);
        assert_eq!(f.retained_indices, vec![0]);
        assert_eq!(f.noise_indices, vec![0]);
        assert!(!f.sweep_present);
    }
    #[test]
    fn supported_ground_does_not_remove_elevated_measured_objects() {
        let mut rows: Vec<_> = (10..31)
            .flat_map(|x| (-10..11).map(move |y| [x as f32, y as f32, -1.5, 1.0]))
            .collect();
        let count = rows.len();
        rows.extend([
            [15.0, 0.0, 0.5, 1.0],
            [15.0, 0.0, 1.0, 1.0],
            [15.0, 0.0, 1.5, 1.0],
        ]);
        let f = evaluate(&bytes(&rows), "ground".into(), 0.0).unwrap();
        assert_eq!(f.ground["confident"], true);
        assert!(f.ground_indices.len() > 350);
        assert!((count..count + 3).all(|i| f.retained_indices.contains(&i)));
        assert!(!f.objects.is_empty());
        assert!(!f.sweep_present);
    }
    #[test]
    fn xyz_components_keep_vertical_gaps_that_dense_xy_merges() {
        let f = evaluate(
            &bytes(&[
                [5.0, 0.0, 0.5, 1.0],
                [5.1, 0.0, 0.5, 1.0],
                [5.2, 0.0, 0.5, 1.0],
                [5.0, 0.0, 2.5, 1.0],
                [5.1, 0.0, 2.5, 1.0],
                [5.2, 0.0, 2.5, 1.0],
            ]),
            "gap".into(),
            0.0,
        )
        .unwrap();
        assert_eq!(f.dense_xy.len(), 1);
        assert_eq!(f.objects.len(), 2);
        assert_eq!(f.objects[0].indices, vec![0, 1, 2]);
        assert_eq!(f.objects[1].indices, vec![3, 4, 5]);
        assert!(f.noise_indices.is_empty());
    }
    #[test]
    fn labels_and_parent_paths_are_rejected_before_file_reads() {
        assert!(serde_json::from_str::<Input>(r#"{"profile":{"z_min":-0.9,"z_max":-0.3,"min_range":2,"max_range":45,"bins":360},"frames":[],"cuboids":[]}"#).is_err());
        let p = Profile {
            z_min: -0.9,
            z_max: -0.3,
            min_range: 2.0,
            max_range: 45.0,
            bins: 360,
        };
        let i = Input {
            profile: p,
            frames: vec![Frame {
                id: "x".into(),
                path: "../labels.json".into(),
                stamp: 0.0,
            }],
        };
        assert!(i.validate().is_err());
    }
}
