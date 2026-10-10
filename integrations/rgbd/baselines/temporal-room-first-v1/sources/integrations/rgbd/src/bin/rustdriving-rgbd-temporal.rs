//! Standalone continuous temporal-extension recorded RGB-D reprojection evaluation.
use rustdriving_core::Vec3;
use rustdriving_localization::registration3d::{Pose3, Quaternion};
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
#[path = "../temporal.rs"]
mod temporal;
const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
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
struct SourceFile {
    file: String,
    bytes: usize,
    sha256: String,
    role: String,
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
fn main() {
    match temporal::run() {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(error) => {
            eprintln!("temporal RGB-D evaluation: {error}");
            std::process::exit(2);
        }
    }
}
