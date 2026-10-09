//! Simulator-only upright cuboids, native scan evidence and conservative capsule guards.
use rne_math::Vec3;
use rustdrive_core::{Pose, Vec2};
use serde_json::{Value, json};
use std::collections::BTreeSet;

/// An upright yaw-rotated box in ENU meters; never an operational obstacle label.
#[derive(Clone, Debug)]
pub struct StaticCuboid {
    pub(crate) id: String,
    pub(crate) center_m: [f64; 3],
    pub(crate) half_extents_m: [f64; 3],
    pub(crate) yaw_rad: f64,
}
#[derive(Clone, Debug)]
pub struct Scene {
    name: String,
    pub(crate) static_cuboids: Vec<StaticCuboid>,
}
fn fields(value: &Value, expected: &[&str]) -> Result<(), String> {
    let object = value.as_object().ok_or("scene field must be an object")?;
    if object.len() != expected.len() || expected.iter().any(|key| !object.contains_key(*key)) {
        return Err(format!("scene fields must be exactly {expected:?}"));
    }
    Ok(())
}
fn label(value: &Value) -> Result<String, String> {
    let text = value.as_str().ok_or("scene label must be a string")?;
    if text.trim().is_empty() || text.len() > 128 {
        return Err("scene labels must contain 1..128 bytes".into());
    }
    Ok(text.into())
}
fn triple(value: &Value, positive: bool) -> Result<[f64; 3], String> {
    let array = value.as_array().ok_or("scene vector must be an array")?;
    if array.len() != 3 {
        return Err("scene vectors require exactly three ENU coordinates".into());
    }
    let mut result = [0.0; 3];
    for (target, input) in result.iter_mut().zip(array) {
        *target = input.as_f64().ok_or("scene coordinates must be numeric")?;
        if !target.is_finite() || target.abs() > 2000.0 || (positive && *target <= 0.0) {
            return Err(
                "scene coordinates must be finite within 2000 m; half extents positive".into(),
            );
        }
    }
    Ok(result)
}
impl Scene {
    /// Parse a strict, bounded schema without adding dependency or lock-file changes.
    pub fn from_json(input: &str) -> Result<Self, String> {
        let value: Value = serde_json::from_str(input).map_err(|error| error.to_string())?;
        fields(&value, &["schema_version", "name", "static_cuboids"])?;
        if value["schema_version"].as_u64() != Some(1) {
            return Err("scene schema_version must be 1".into());
        }
        let name = label(&value["name"])?;
        let boxes = value["static_cuboids"]
            .as_array()
            .ok_or("static_cuboids must be an array")?;
        if boxes.is_empty() || boxes.len() > 128 {
            return Err("scene requires 1..128 static cuboids".into());
        }
        let mut ids = BTreeSet::new();
        let mut static_cuboids = Vec::with_capacity(boxes.len());
        for object in boxes {
            fields(object, &["id", "center_m", "half_extents_m", "yaw_rad"])?;
            let id = label(&object["id"])?;
            if !ids.insert(id.clone()) {
                return Err("scene cuboid ids must be unique".into());
            }
            let yaw_rad = object["yaw_rad"]
                .as_f64()
                .ok_or("yaw_rad must be numeric")?;
            if !yaw_rad.is_finite() || yaw_rad.abs() > std::f64::consts::PI {
                return Err("yaw_rad must be finite within [-pi, pi]".into());
            }
            static_cuboids.push(StaticCuboid {
                id,
                center_m: triple(&object["center_m"], false)?,
                half_extents_m: triple(&object["half_extents_m"], true)?,
                yaw_rad,
            });
        }
        Ok(Self {
            name,
            static_cuboids,
        })
    }
    pub fn to_json(&self) -> Value {
        json!({"schema_version": 1, "name": self.name, "static_cuboids": self.static_cuboids.iter().map(|b| json!({"id": b.id,"center_m": b.center_m,"half_extents_m": b.half_extents_m,"yaw_rad": b.yaw_rad})).collect::<Vec<_>>()})
    }
}

#[derive(Clone, Copy)]
pub(crate) struct MotionSample {
    pub time: f64,
    pub position: Vec2,
}
pub(crate) struct SceneCapture {
    pub scene: Scene,
    pub motion_samples: Vec<MotionSample>,
    pub observations: Vec<Value>,
    pub acquisitions: Vec<Value>,
    pub clock: f64,
}
impl SceneCapture {
    pub fn new(scene: Scene, position: Vec2) -> Self {
        Self {
            scene,
            motion_samples: vec![MotionSample {
                time: 0.0,
                position,
            }],
            observations: vec![],
            acquisitions: vec![],
            clock: 0.0,
        }
    }
    pub fn evidence(&self, radius: f64) -> Value {
        let mut minimum = f64::INFINITY;
        let mut overlaps = 0;
        let mut checks = 0;
        // The adapter validates each native substep's translation / dt <=12 m/s.
        // The pinned integrator translates by a fixed midpoint velocity per step;
        // its modeled translation is the recorded center segment. An additional
        // 12*dt/2 horizontal inflation is conservative for this discrete model,
        // without claiming a bound on continuous physical tire or chassis motion.
        for pair in self.motion_samples.windows(2) {
            let inflation = 12.0 * (pair[1].time - pair[0].time) / 2.0;
            let mut overlap = false;
            for cuboid in &self.scene.static_cuboids {
                let clearance = capsule_clearance(
                    pair[0].position,
                    pair[1].position,
                    cuboid,
                    radius,
                    inflation,
                );
                minimum = minimum.min(clearance);
                overlap |= clearance <= 0.0;
                checks += 1;
            }
            overlaps += usize::from(overlap);
        }
        // Retain endpoint checks even for an immediately finished run.
        for point in &self.motion_samples {
            for cuboid in &self.scene.static_cuboids {
                minimum = minimum.min(capsule_clearance(
                    point.position,
                    point.position,
                    cuboid,
                    radius,
                    0.0,
                ));
                checks += 1;
            }
        }
        let mut failures = Vec::new();
        if overlaps > 0 {
            failures.push(format!(
                "{overlaps} native scene conservative capsule guard overlaps"
            ));
        }
        if minimum < 1.0 {
            failures.push(format!(
                "native scene capsule clearance {minimum:.6} m is below fixed 1 m floor"
            ));
        }
        json!({"schema_version":1,"scene":self.scene.to_json(),"ego_capsule":{"axis_bottom_m":0.1,"axis_top_m":1.1,"radius_m":radius,"speed_bound_m_s":12.0,"clearance_floor_m":1.0},"motion_samples":self.motion_samples.iter().map(|s|json!({"time":s.time,"position":[s.position.x,s.position.y]})).collect::<Vec<_>>(),"observations":self.observations,"acquisitions":self.acquisitions,"summary":{"passed":failures.is_empty(),"min_clearance_m":minimum,"guard_overlap_intervals":overlaps,"checks":checks,"failures":failures}})
    }
}
/// Preserve native firing ordinals; no-return rays remain null.
pub(crate) fn scan_channel(
    points: &[Vec3],
    indices: &[u32],
    mount: Vec3,
    height: f64,
) -> Result<Value, String> {
    if points.len() != indices.len() {
        return Err("native scene cloud ray indices are not aligned".into());
    }
    let mut ranges: Vec<Option<f64>> = vec![None; 720];
    for (&point, &index) in points.iter().zip(indices) {
        let slot = ranges
            .get_mut(index as usize)
            .ok_or("native scene ray index exceeds scan width")?;
        let distance = (point - mount).length();
        if !distance.is_finite() || slot.is_some() {
            return Err("native scene cloud has invalid or duplicate return".into());
        }
        *slot = Some(distance);
    }
    Ok(json!({"height_m":height,"ranges_m":ranges}))
}
pub(crate) fn pose_json(pose: Pose) -> Value {
    json!({"position":{"x":pose.position.x,"y":pose.position.y},"yaw":pose.yaw})
}

fn point_segment(point: Vec2, a: Vec2, b: Vec2) -> f64 {
    let direction = b.minus(a);
    let length = direction.x * direction.x + direction.y * direction.y;
    if length == 0.0 {
        return point.distance(a);
    }
    let t = ((point.x - a.x) * direction.x + (point.y - a.y) * direction.y) / length;
    point.distance(a.plus(direction.scaled(t.clamp(0.0, 1.0))))
}
/// Exact minimum distance of a planar segment to an axis-aligned rectangle.
fn segment_rectangle(a: Vec2, b: Vec2, hx: f64, hy: f64) -> f64 {
    let direction = b.minus(a);
    let mut low = 0.0_f64;
    let mut high = 1.0_f64;
    for (origin, delta, extent) in [(a.x, direction.x, hx), (a.y, direction.y, hy)] {
        if delta.abs() < 1e-15 {
            if origin.abs() > extent {
                low = 2.0;
                break;
            }
        } else {
            let t0 = (-extent - origin) / delta;
            let t1 = (extent - origin) / delta;
            low = low.max(t0.min(t1));
            high = high.min(t0.max(t1));
        }
    }
    if low <= high {
        return 0.0;
    }
    let endpoint = |p: Vec2| (p.x.abs() - hx).max(0.0).hypot((p.y.abs() - hy).max(0.0));
    let mut minimum = endpoint(a).min(endpoint(b));
    for x in [-hx, hx] {
        for y in [-hy, hy] {
            minimum = minimum.min(point_segment(Vec2::new(x, y), a, b));
        }
    }
    minimum
}
/// Surface clearance of the actual vertical capsule, with conservative chord inflation.
fn capsule_clearance(a: Vec2, b: Vec2, cuboid: &StaticCuboid, radius: f64, inflation: f64) -> f64 {
    let center = Vec2::new(cuboid.center_m[0], cuboid.center_m[1]);
    let a = a.minus(center).rotated(-cuboid.yaw_rad);
    let b = b.minus(center).rotated(-cuboid.yaw_rad);
    let horizontal = (segment_rectangle(a, b, cuboid.half_extents_m[0], cuboid.half_extents_m[1])
        - inflation)
        .max(0.0);
    let bottom = cuboid.center_m[2] - cuboid.half_extents_m[2];
    let top = cuboid.center_m[2] + cuboid.half_extents_m[2];
    let vertical = (bottom - 1.1).max(0.1 - top).max(0.0);
    (horizontal.hypot(vertical) - radius).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn box_at(center: [f64; 3], half: [f64; 3], yaw: f64) -> StaticCuboid {
        StaticCuboid {
            id: "box".into(),
            center_m: center,
            half_extents_m: half,
            yaw_rad: yaw,
        }
    }
    #[test]
    fn rotated_rectangle_segment_and_capsule_height_use_actual_geometry() {
        let box_ = box_at(
            [5.0, 0.0, 0.5],
            [1.0, 2.0, 0.5],
            std::f64::consts::FRAC_PI_2,
        );
        assert_eq!(
            capsule_clearance(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), &box_, 1.0, 0.0),
            0.0
        );
        assert!(
            (capsule_clearance(Vec2::new(0.0, 4.0), Vec2::new(10.0, 4.0), &box_, 1.0, 0.0) - 2.0)
                .abs()
                < 1e-12
        );
        let high = box_at([5.0, 0.0, 4.5], [1.0, 2.0, 1.0], 0.0);
        assert!(
            (capsule_clearance(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), &high, 1.0, 0.03) - 1.4)
                .abs()
                < 1e-12
        );
        let low = box_at([5.0, 0.0, 0.1], [1.0, 2.0, 0.1], 0.0);
        assert_eq!(
            capsule_clearance(Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), &low, 1.0, 0.03),
            0.0
        );
    }
    #[test]
    fn strict_scene_rejects_unknown_fields_duplicate_ids_and_bad_extents() {
        let valid = json!({"schema_version":1,"name":"fixture","static_cuboids":[{"id":"a","center_m":[1,2,3],"half_extents_m":[1,1,1],"yaw_rad":0}]});
        assert!(Scene::from_json(&valid.to_string()).is_ok());
        let mut unknown = valid.clone();
        unknown["truth"] = json!(true);
        assert!(Scene::from_json(&unknown.to_string()).is_err());
        let mut duplicate = valid.clone();
        duplicate["static_cuboids"]
            .as_array_mut()
            .unwrap()
            .push(valid["static_cuboids"][0].clone());
        assert!(Scene::from_json(&duplicate.to_string()).is_err());
        let mut invalid = valid;
        invalid["static_cuboids"][0]["half_extents_m"][0] = json!(0);
        assert!(Scene::from_json(&invalid.to_string()).is_err());
    }
}
