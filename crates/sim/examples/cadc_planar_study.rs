//! Bounded, label-free CADC XYZI adapter for an offline planar diagnostic.
use rustdriving_core::{Detection, LidarScan, Perception, Pose, Vec2};
use rustdriving_perception::LidarClusters;
use serde::{Deserialize, Serialize};
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
        if ![self.z_min, self.z_max, self.min_range, self.max_range].into_iter().all(f64::is_finite)
            || self.z_min < -5.0 || self.z_max > 5.0 || self.z_min >= self.z_max
            || self.min_range < 0.2 || self.min_range >= self.max_range || self.max_range > 200.0
            || !(4..=4096).contains(&self.bins) {
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
            if f.id.is_empty() || f.id.len() > 160 || !ids.insert(&f.id)
                || !f.stamp.is_finite() || f.stamp < 0.0 || f.path.is_empty()
                || Path::new(&f.path).components().any(|c| !matches!(c, Component::Normal(_))) {
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
struct Projection {
    scan: LidarScan,
    total_rows: usize,
    invalid_xyz_rows: usize,
    slab_rows: usize,
    selected_rows: Vec<usize>,
}
fn project(bytes: &[u8], p: Profile, stamp: f64) -> Result<Projection, String> {
    p.validate()?;
    if bytes.is_empty() || bytes.len() > MAX_SCAN || !bytes.len().is_multiple_of(16)
        || !stamp.is_finite() || stamp < 0.0 {
        return Err("finite timestamp and bounded complete little-endian XYZI rows required".into());
    }
    let mut bins: Vec<Option<(f64, usize, Vec2)>> = vec![None; p.bins];
    let (mut invalid, mut slab) = (0, 0);
    for (index, row) in bytes.chunks_exact(16).enumerate() {
        let value = |offset: usize| f64::from(f32::from_le_bytes(
            row[offset..offset + 4].try_into().expect("four bytes")));
        let (x, y, z) = (value(0), value(4), value(8));
        // Reflectance is not used by this geometric detector.
        if ![x, y, z].into_iter().all(f64::is_finite) { invalid += 1; continue; }
        let range = x.hypot(y);
        if z < p.z_min || z > p.z_max || range < p.min_range || range > p.max_range { continue; }
        slab += 1;
        let angle = y.atan2(x).rem_euclid(std::f64::consts::TAU);
        let bin = ((angle / std::f64::consts::TAU * p.bins as f64).floor() as usize).min(p.bins - 1);
        if bins[bin].as_ref().is_none_or(|old| range < old.0) {
            bins[bin] = Some((range, index, Vec2::new(x, y)));
        }
    }
    let selected: Vec<_> = bins.into_iter().flatten().collect();
    Ok(Projection {
        scan: LidarScan { stamp, points: selected.iter().map(|v| v.2).collect(), sweep: None },
        total_rows: bytes.len() / 16, invalid_xyz_rows: invalid, slab_rows: slab,
        selected_rows: selected.into_iter().map(|v| v.1).collect(),
    })
}
#[derive(Serialize)]
struct FrameResult {
    id: String,
    stamp: f64,
    total_rows: usize,
    invalid_xyz_rows: usize,
    slab_rows: usize,
    selected_rows: Vec<usize>,
    sweep_present: bool,
    detections: Vec<Detection>,
}
#[derive(Serialize)]
struct Report {
    schema_version: u32,
    scope: &'static str,
    profile: Profile,
    frames: Vec<FrameResult>,
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 2 { return Err("usage: cadc_planar_study INPUT.json".into()); }
    let path = Path::new(&args[1]);
    let input: Input = serde_json::from_slice(&read_bounded(path, MAX_INPUT)?)?;
    input.validate()?;
    let base = path.parent().ok_or("input parent missing")?;
    let mut frames = Vec::new();
    for frame in input.frames {
        let projected = project(&read_bounded(&base.join(frame.path), MAX_SCAN)?, input.profile, frame.stamp)?;
        let detections = LidarClusters.detect(&projected.scan, Pose { position: Vec2::default(), yaw: 0.0 });
        frames.push(FrameResult { id: frame.id, stamp: frame.stamp, total_rows: projected.total_rows,
            invalid_xyz_rows: projected.invalid_xyz_rows, slab_rows: projected.slab_rows,
            selected_rows: projected.selected_rows, sweep_present: projected.scan.sweep.is_some(), detections });
    }
    let report = Report { schema_version: 1,
        scope: "Recorded CADC LiDAR-local XYZI, fixed height slab and angular nearest-return thinning; unchanged LidarClusters. No calibrated sweep, semantic 3D AP, weather coefficient fit, localization or driving qualification.",
        profile: input.profile, frames };
    let mut out = std::io::BufWriter::new(std::io::stdout().lock());
    serde_json::to_writer(&mut out, &report)?; out.write_all(b"\n")?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    fn profile() -> Profile { Profile { z_min: -0.9, z_max: -0.3, min_range: 2.0, max_range: 45.0, bins: 360 } }
    fn bytes(rows: &[[f32; 4]]) -> Vec<u8> { rows.iter().flatten().flat_map(|v| v.to_le_bytes()).collect() }
    #[test]
    fn nearest_measured_return_preserves_original_xy_and_tie_order() {
        let raw = bytes(&[[4.0,0.0,-0.6,1.0],[3.0,0.0,-0.6,1.0],[3.0,0.0,-0.6,2.0],[0.0,4.0,-0.6,1.0]]);
        let p = project(&raw, profile(), 1.0).unwrap();
        assert_eq!(p.selected_rows, vec![1,3]);
        assert_eq!(p.scan.points, vec![Vec2::new(3.0,0.0),Vec2::new(0.0,4.0)]);
        assert!(p.scan.sweep.is_none());
    }
    #[test]
    fn slab_and_range_filter_do_not_use_reflectance_or_annotations() {
        let raw = bytes(&[[3.0,0.02,-0.6,f32::NAN],[4.0,0.0,0.0,1.0],[1.0,0.0,-0.6,1.0],[f32::NAN,0.0,-0.6,1.0]]);
        let p = project(&raw, profile(), 2.0).unwrap();
        assert_eq!(p.total_rows,4); assert_eq!(p.invalid_xyz_rows,1); assert_eq!(p.slab_rows,1);
        assert_eq!(p.selected_rows,vec![0]);
        assert_eq!(p.scan.points[0].y, f64::from(0.02_f32));
    }
    #[test]
    fn malformed_binary_and_invalid_timestamps_are_rejected() {
        assert!(project(&[],profile(),0.0).is_err());
        assert!(project(&[0;17],profile(),0.0).is_err());
        assert!(project(&[0;16],profile(),f64::NAN).is_err());
        assert!(project(&vec![0;MAX_SCAN+16],profile(),0.0).is_err());
    }
    #[test]
    fn empty_slab_stays_uncertified() {
        let p = project(&bytes(&[[3.0,0.0,2.0,1.0]]),profile(),0.0).unwrap();
        assert!(p.scan.points.is_empty()); assert!(p.scan.sweep.is_none());
        assert!(LidarClusters.detect(&p.scan,Pose { position:Vec2::default(),yaw:0.0 }).is_empty());
    }
    #[test]
    fn labels_and_extra_profile_fields_are_not_detector_inputs() {
        assert!(serde_json::from_str::<Input>(r#"{"profile":{"z_min":-0.9,"z_max":-0.3,"min_range":2,"max_range":45,"bins":360},"frames":[],"cuboids":[]}"#).is_err());
        assert!(serde_json::from_str::<Profile>(r#"{"z_min":-0.9,"z_max":-0.3,"min_range":2,"max_range":45,"bins":360,"label":"Car"}"#).is_err());
    }
    #[test]
    fn identities_paths_and_projection_bounds_are_checked_before_reads() {
        let mut p = profile(); p.bins = 0; assert!(p.validate().is_err());
        p = profile(); p.z_max = p.z_min; assert!(p.validate().is_err());
        let input = Input { profile:profile(),frames:vec![Frame { id:"f".into(),path:"../labels.json".into(),stamp:0.0 }] };
        assert!(input.validate().is_err());
        let input = Input { profile:profile(),frames:vec![Frame { id:"f".into(),path:"a.bin".into(),stamp:-1.0 }] };
        assert!(input.validate().is_err());
    }
}
