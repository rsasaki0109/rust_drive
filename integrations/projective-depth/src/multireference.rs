//! Bounded raw-keyframe registration, optionally using phase-qualified intensity.
//! Optical poses only; no reference labels or driving permission.
use crate::{Camera, Equation, H, Level, Pose, cross, dot, norm, pyramid, solve_normal, sub};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    Geometry,
    PhaseGatedRgbd,
}
pub struct ImageObservation<'a> {
    pub timestamp: f64,
    pub gray: &'a [u8],
}
#[derive(Clone)]
pub(super) struct Gray {
    pub(super) width: usize,
    pub(super) height: usize,
    pub(super) pixels: Vec<f64>,
}
impl Gray {
    pub(super) fn sample(&self, x: f64, y: f64) -> Option<(f64, f64, f64)> {
        // Bilinear derivatives have different one-sided limits at pixel knots.
        // Canonicalize roundoff from inverse/forward pose composition so the
        // right/down cell is selected consistently at an integer coordinate.
        let canonical = |v: f64| {
            let nearest = v.round_ties_even();
            if (v - nearest).abs() <= 1e-10 {
                nearest
            } else {
                v
            }
        };
        let (x, y) = (canonical(x), canonical(y));
        if !x.is_finite()
            || !y.is_finite()
            || x < 0.
            || y < 0.
            || x >= (self.width - 1) as f64
            || y >= (self.height - 1) as f64
        {
            return None;
        }
        let u = x.floor() as usize;
        let v = y.floor() as usize;
        let dx = x - u as f64;
        let dy = y - v as f64;
        let i = v * self.width + u;
        let (a, b, c, d) = (
            self.pixels[i],
            self.pixels[i + 1],
            self.pixels[i + self.width],
            self.pixels[i + self.width + 1],
        );
        Some((
            (1. - dx) * (1. - dy) * a + dx * (1. - dy) * b + (1. - dx) * dy * c + dx * dy * d,
            (1. - dy) * (b - a) + dy * (d - c),
            (1. - dx) * (c - a) + dx * (d - b),
        ))
    }
}
pub(super) fn gray_pyramid(camera: Camera, pixels: &[u8]) -> Vec<Gray> {
    let mut layers = vec![Gray {
        width: camera.width,
        height: camera.height,
        pixels: pixels.iter().map(|&v| f64::from(v)).collect(),
    }];
    for _ in 0..2 {
        let old = layers.last().expect("initial image");
        let w = old.width / 2;
        let h = old.height / 2;
        let data = (0..w * h)
            .map(|i| {
                let k = i / w * 2 * old.width + i % w * 2;
                (old.pixels[k]
                    + old.pixels[k + 1]
                    + old.pixels[k + old.width]
                    + old.pixels[k + old.width + 1])
                    / 4.
            })
            .collect();
        layers.push(Gray {
            width: w,
            height: h,
            pixels: data,
        });
    }
    layers.reverse();
    layers
}
struct Reference {
    timestamp: f64,
    root: Pose,
    surface: Vec<Level>,
    gray: Option<Vec<Gray>>,
    phase: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct JointEquation {
    pub geometry: Equation,
    pub photometric_count: usize,
    pub photometric_rms: Option<f64>,
    pub h: H,
    pub g: [f64; 6],
    pub reference_support: Vec<usize>,
    pub reference_photo_support: Vec<usize>,
}
fn zero_equation() -> Equation {
    Equation {
        count: 0,
        rms_m: 0.,
        h: [[0.; 6]; 6],
        g: [0.; 6],
        direction_counts: None,
    }
}
fn accumulate(e: &mut Equation, j: [f64; 6], r: f64, weight: f64) {
    for a in 0..6 {
        e.g[a] += j[a] * r * weight;
        for b in 0..6 {
            e.h[a][b] += j[a] * j[b] * weight;
        }
    }
    e.count += 1;
    e.rms_m += r * r;
}
fn equation(
    references: &[&Reference],
    source: &Reference,
    pose: Pose,
    level: usize,
    stride: usize,
    photo_enabled: bool,
) -> JointEquation {
    let source_level = &source.surface[level];
    let scale = [4, 2, 1][level];
    let mut out = JointEquation {
        geometry: zero_equation(),
        photometric_count: 0,
        photometric_rms: None,
        h: [[0.; 6]; 6],
        g: [0.; 6],
        reference_support: Vec::new(),
        reference_photo_support: Vec::new(),
    };
    let mut photo_squared = 0.;
    for target in references {
        let inverse = target.root.inverse();
        let target_level = &target.surface[level];
        let mut geometric = zero_equation();
        let mut photometric = zero_equation();
        let photo_pair = photo_enabled && (source.phase - target.phase).abs() <= 0.005;
        for y in (0..source_level.camera.height).step_by(stride) {
            for x in (0..source_level.camera.width).step_by(stride) {
                let i = y * source_level.camera.width + x;
                let sn = source_level.normals[i];
                if dot(sn, sn) <= 0.25 {
                    continue;
                }
                let root_point = pose.transform(source_level.vertices[i]);
                let point = inverse.transform(root_point);
                let Some(j) = target_level.camera.project(point, true) else {
                    continue;
                };
                let n = target_level.normals[j];
                let root_normal = target.root.rotate(n);
                if dot(n, n) <= 0.25
                    || norm(sub(point, target_level.vertices[j]))
                        >= if scale == 4 { 0.12 } else { 0.08 }
                    || dot(pose.rotate(sn), root_normal) <= 0.8
                {
                    continue;
                }
                let r = dot(n, sub(point, target_level.vertices[j]));
                let rot = cross(root_point, root_normal);
                accumulate(
                    &mut geometric,
                    [
                        root_normal[0],
                        root_normal[1],
                        root_normal[2],
                        rot[0],
                        rot[1],
                        rot[2],
                    ],
                    r,
                    (0.01 / r.abs().max(1e-15)).min(1.),
                );
                if photo_pair
                    && let (Some(target_gray), Some(source_gray)) = (&target.gray, &source.gray)
                {
                    let c = target_level.camera;
                    let uf = c.fx * point[0] / point[2] + c.cx;
                    let vf = c.fy * point[1] / point[2] + c.cy;
                    if let Some((value, gx, gy)) = target_gray[level].sample(uf, vf) {
                        let residual = value - source_gray[level].pixels[i];
                        if residual.abs() < 30. && gx.hypot(gy) >= 1. {
                            let z = point[2];
                            let grad = target.root.rotate([
                                gx * c.fx / z,
                                gy * c.fy / z,
                                -(gx * c.fx * point[0] + gy * c.fy * point[1]) / (z * z),
                            ]);
                            let rot = cross(root_point, grad);
                            let weight = (1. - (residual / 30.).powi(2)).powi(2);
                            accumulate(
                                &mut photometric,
                                [grad[0], grad[1], grad[2], rot[0], rot[1], rot[2]],
                                residual,
                                weight,
                            );
                        }
                    }
                }
            }
        }
        out.reference_support.push(geometric.count);
        out.reference_photo_support
            .push(if photometric.count >= 200 {
                photometric.count
            } else {
                0
            });
        out.geometry.count += geometric.count;
        out.geometry.rms_m += geometric.rms_m;
        let used = photometric.count >= 200;
        if used {
            out.photometric_count += photometric.count;
            photo_squared += photometric.rms_m;
        }
        for a in 0..6 {
            out.geometry.g[a] += geometric.g[a];
            out.g[a] += geometric.g[a] / 0.005_f64.powi(2);
            if used {
                out.g[a] += photometric.g[a] / 100.;
            }
            for b in 0..6 {
                out.geometry.h[a][b] += geometric.h[a][b];
                out.h[a][b] += geometric.h[a][b] / 0.005_f64.powi(2);
                if used {
                    out.h[a][b] += photometric.h[a][b] / 100.;
                }
            }
        }
    }
    if out.geometry.count > 0 {
        out.geometry.rms_m = (out.geometry.rms_m / out.geometry.count as f64).sqrt();
    }
    if out.photometric_count > 0 {
        out.photometric_rms = Some((photo_squared / out.photometric_count as f64).sqrt());
    }
    out
}
#[derive(Clone, Debug, Serialize)]
pub struct JointIteration {
    pub scale: usize,
    pub before: Pose,
    pub equation: JointEquation,
    pub delta: [f64; 6],
    pub scaled_condition: f64,
    pub after: Pose,
}
fn fit(
    references: &[&Reference],
    source: &Reference,
    initial: Pose,
    photo: bool,
    trace: &mut Vec<JointIteration>,
) -> Result<(Pose, JointEquation), String> {
    let mut pose = initial;
    for (level, (scale, cap, stride)) in [(4, 10, 1), (2, 7, 2), (1, 5, 4)].into_iter().enumerate()
    {
        for _ in 0..cap {
            let e = equation(references, source, pose, level, stride, photo);
            if e.geometry.count < 200 {
                return Err("insufficient multi-reference support".into());
            }
            let (d, condition) = solve_normal(e.h, e.g, 1e6)?;
            if norm([d[0], d[1], d[2]]) > 0.1 || norm([d[3], d[4], d[5]]) > 0.15 {
                return Err("oversized multi-reference increment".into());
            }
            let next = pose.increment(d);
            if !next.finite_rigid() {
                return Err("nonrigid multi-reference pose".into());
            }
            trace.push(JointIteration {
                scale,
                before: pose,
                equation: e,
                delta: d,
                scaled_condition: condition,
                after: next,
            });
            pose = next;
            if norm([d[0], d[1], d[2]]) < 1e-5 && norm([d[3], d[4], d[5]]) < 1e-5 {
                break;
            }
        }
    }
    let final_e = equation(references, source, pose, 2, 4, photo);
    if final_e.geometry.count < 200 || final_e.geometry.rms_m > 0.03 {
        return Err("multi-reference final support/residual".into());
    }
    Ok((pose, final_e))
}
#[derive(Debug, Serialize)]
pub struct MultiUpdate {
    pub timestamp: f64,
    pub accepted: bool,
    pub lost: bool,
    pub rejection: Option<String>,
    pub first_from_current: Option<Pose>,
    pub last_accepted_timestamp: Option<f64>,
    pub reference_timestamps: Vec<f64>,
    pub keyframe_replaced: bool,
    pub photo_enabled: bool,
    pub iterations: Vec<JointIteration>,
    pub final_equation: Option<JointEquation>,
}
pub struct MultiReferenceTracker {
    camera: Camera,
    mode: Mode,
    keys: Vec<Arc<Reference>>,
    latest: Option<Arc<Reference>>,
    last_observed: Option<f64>,
    last_rgb: Option<f64>,
    lost: bool,
}
impl MultiReferenceTracker {
    pub fn new(camera: Camera, mode: Mode) -> Result<Self, String> {
        camera.validate()?;
        Ok(Self {
            camera,
            mode,
            keys: Vec::new(),
            latest: None,
            last_observed: None,
            last_rgb: None,
            lost: false,
        })
    }
    pub fn step(
        &mut self,
        timestamp: f64,
        depth: &[f64],
        image: Option<ImageObservation<'_>>,
    ) -> MultiUpdate {
        let last = self.latest.as_ref().map(|r| r.timestamp);
        let mut out = MultiUpdate {
            timestamp,
            accepted: false,
            lost: self.lost,
            rejection: None,
            first_from_current: None,
            last_accepted_timestamp: last,
            reference_timestamps: Vec::new(),
            keyframe_replaced: false,
            photo_enabled: false,
            iterations: Vec::new(),
            final_equation: None,
        };
        let result = (|| -> Result<(), String> {
            if !timestamp.is_finite()
                || !(0.0..=1e12).contains(&timestamp)
                || self.last_observed.is_some_and(|t| timestamp <= t)
            {
                return Err("invalid/duplicate depth clock".into());
            }
            self.last_observed = Some(timestamp);
            if last.is_some_and(|t| timestamp - t > 0.2) {
                self.lost = true;
            }
            if self.lost {
                return Err("multi-reference accepted-pose expiry latched".into());
            }
            let (gray, offset, fresh) = match (self.mode, image) {
                (Mode::Geometry, None) => (None, 0., false),
                (Mode::Geometry, Some(_)) => return Err("geometry mode expects depth only".into()),
                (Mode::PhaseGatedRgbd, None) => {
                    return Err("RGB-D mode requires an associated image".into());
                }
                (Mode::PhaseGatedRgbd, Some(image)) => {
                    if !image.timestamp.is_finite()
                        || image.timestamp < 0.
                        || (image.timestamp - timestamp).abs() > 0.02
                        || self.last_rgb.is_some_and(|t| image.timestamp < t)
                    {
                        return Err("invalid RGB association clock".into());
                    }
                    let fresh = self.last_rgb.is_none_or(|t| image.timestamp > t);
                    self.last_rgb = Some(image.timestamp);
                    if image.gray.len() != self.camera.width * self.camera.height {
                        return Err("invalid grayscale image size".into());
                    }
                    (
                        Some(gray_pyramid(self.camera, image.gray)),
                        image.timestamp - timestamp,
                        fresh,
                    )
                }
            };
            if depth.len() != self.camera.width * self.camera.height
                || depth
                    .iter()
                    .any(|z| !z.is_finite() || !(0.0..=100.).contains(z))
            {
                return Err("invalid bounded multi-reference depth".into());
            }
            let mut current = Reference {
                timestamp,
                root: Pose::identity(),
                surface: pyramid(self.camera, depth),
                gray,
                phase: offset,
            };
            if self.latest.is_none() {
                if current.surface[0]
                    .normals
                    .iter()
                    .filter(|n| dot(**n, **n) > 0.25)
                    .count()
                    < 200
                {
                    return Err("insufficient initialization surface".into());
                }
                let first = Arc::new(current);
                self.keys.push(first.clone());
                self.latest = Some(first);
                out.first_from_current = Some(Pose::identity());
            } else {
                let latest = self.latest.as_ref().expect("initialized");
                let mut references: Vec<&Reference> = self.keys.iter().map(AsRef::as_ref).collect();
                if !references.iter().any(|r| r.timestamp == latest.timestamp) {
                    references.push(latest);
                }
                out.reference_timestamps = references.iter().map(|r| r.timestamp).collect();
                out.photo_enabled = fresh;
                let (pose, e) = fit(
                    &references,
                    &current,
                    latest.root,
                    fresh,
                    &mut out.iterations,
                )?;
                if norm(latest.root.inverse().compose(pose).translation) > 0.5 {
                    return Err("multi-reference displacement gate".into());
                }
                let delta = self
                    .keys
                    .last()
                    .expect("first retained")
                    .root
                    .inverse()
                    .compose(pose);
                let replace = norm(delta.translation) > 0.1 || delta.angle() > 0.1;
                current.root = pose;
                let current = Arc::new(current);
                if replace {
                    if self.keys.len() == 4 {
                        self.keys.remove(1);
                    }
                    self.keys.push(current.clone());
                }
                self.latest = Some(current);
                out.first_from_current = Some(pose);
                out.final_equation = Some(e);
                out.keyframe_replaced = replace;
            }
            out.accepted = true;
            Ok(())
        })();
        out.rejection = result.err();
        out.lost = self.lost;
        out.last_accepted_timestamp = self.latest.as_ref().map(|r| r.timestamp);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn camera() -> Camera {
        Camera {
            width: 160,
            height: 120,
            fx: 130.,
            fy: 130.,
            cx: 79.5,
            cy: 59.5,
        }
    }
    fn texture(tx: f64, ty: f64) -> Vec<u8> {
        let c = camera();
        (0..c.width * c.height)
            .map(|i| {
                let x = ((i % c.width) as f64 - c.cx) * 2. / c.fx + tx;
                let y = ((i / c.width) as f64 - c.cy) * 2. / c.fy + ty;
                (127. + 40. * (8. * x).sin() + 30. * (6. * y).cos() + 20. * (3. * x + 2. * y).sin())
                    .round()
                    .clamp(0., 255.) as u8
            })
            .collect()
    }
    fn surface() -> Vec<f64> {
        (0..160 * 120)
            .map(|i| {
                1.5 + 0.12 * ((i % 160) as f64 / 9.).sin() + 0.08 * ((i / 160) as f64 / 11.).cos()
            })
            .collect()
    }
    #[test]
    fn bilinear_derivatives_match_hand_calculated_linear_image() {
        let g = Gray {
            width: 8,
            height: 8,
            pixels: (0..64)
                .map(|i| (3 * (i % 8) + 5 * (i / 8) + 7) as f64)
                .collect(),
        };
        let (value, gx, gy) = g.sample(2.25, 3.5).unwrap();
        assert!((value - 31.25).abs() < 1e-12);
        assert_eq!((gx, gy), (3., 5.));
        assert!(g.sample(-1., 1.).is_none());
        assert!(g.sample(7., 1.).is_none());
    }
    #[test]
    fn bilinear_pixel_knots_have_canonical_right_down_derivatives() {
        let g = Gray {
            width: 8,
            height: 8,
            pixels: (0_usize..64)
                .map(|i| ((i % 8).pow(2) + 3 * (i / 8).pow(2)) as f64)
                .collect(),
        };
        for offset in [-1e-12, 0., 1e-12] {
            let (value, dx, dy) = g.sample(2. + offset, 3. - offset).unwrap();
            assert_eq!((value, dx, dy), (31., 5., 21.));
            assert!(g.sample(7. + offset, 2.).is_none());
        }
        assert_eq!(g.sample(2. - 1e-8, 3.).unwrap().1, 3.);
    }
    #[test]
    fn image_texture_recovers_translation_unobservable_from_a_single_plane() {
        let d = vec![2.; 160 * 120];
        let first = texture(0., 0.);
        let moved = texture(0.02, -0.01);
        let mut geometry = MultiReferenceTracker::new(camera(), Mode::Geometry).unwrap();
        assert!(geometry.step(1., &d, None).accepted);
        assert!(!geometry.step(1.033, &d, None).accepted);
        let mut joint = MultiReferenceTracker::new(camera(), Mode::PhaseGatedRgbd).unwrap();
        assert!(
            joint
                .step(
                    1.,
                    &d,
                    Some(ImageObservation {
                        timestamp: 1.001,
                        gray: &first
                    })
                )
                .accepted
        );
        let result = joint.step(
            1.033,
            &d,
            Some(ImageObservation {
                timestamp: 1.034,
                gray: &moved,
            }),
        );
        assert!(result.accepted, "{:?}", result.rejection);
        let p = result.first_from_current.unwrap();
        assert!(
            (p.translation[0] - 0.02).abs() < 0.004,
            "{:?}",
            p.translation
        );
        assert!(
            (p.translation[1] + 0.01).abs() < 0.004,
            "{:?}",
            p.translation
        );
        assert!(p.angle() < 0.004);
        assert!(result.final_equation.unwrap().photometric_count >= 200);
    }
    #[test]
    fn repeated_or_incompatible_phase_images_cannot_fill_geometric_degeneracy() {
        let d = vec![2.; 160 * 120];
        let gray = texture(0., 0.);
        let mut repeated = MultiReferenceTracker::new(camera(), Mode::PhaseGatedRgbd).unwrap();
        repeated.step(
            1.,
            &d,
            Some(ImageObservation {
                timestamp: 1.015,
                gray: &gray,
            }),
        );
        let result = repeated.step(
            1.033,
            &d,
            Some(ImageObservation {
                timestamp: 1.015,
                gray: &gray,
            }),
        );
        assert!(!result.accepted && !result.photo_enabled);
        assert_eq!(result.last_accepted_timestamp, Some(1.));
        let mut shifted = MultiReferenceTracker::new(camera(), Mode::PhaseGatedRgbd).unwrap();
        shifted.step(
            1.,
            &d,
            Some(ImageObservation {
                timestamp: 1.001,
                gray: &gray,
            }),
        );
        let result = shifted.step(
            1.033,
            &d,
            Some(ImageObservation {
                timestamp: 1.048,
                gray: &gray,
            }),
        );
        assert!(!result.accepted);
        assert_eq!(shifted.keys.len(), 1);
    }
    #[test]
    fn rejected_frames_preserve_reference_and_age_until_latched_loss() {
        let d = surface();
        let mut tracker = MultiReferenceTracker::new(camera(), Mode::Geometry).unwrap();
        assert!(tracker.step(1., &d, None).accepted);
        assert!(!tracker.step(1., &d, None).accepted);
        let bad = tracker.step(1.03, &[], None);
        assert!(!bad.accepted);
        assert_eq!(bad.last_accepted_timestamp, Some(1.));
        assert_eq!(tracker.keys.len(), 1);
        assert!(tracker.step(1.06, &d, None).accepted);
        let lost = tracker.step(1.27, &d, None);
        assert!(lost.lost && !lost.accepted);
        let retry = tracker.step(1.28, &d, None);
        assert!(retry.lost && !retry.accepted);
        assert_eq!(retry.last_accepted_timestamp, Some(1.06));
    }
    #[test]
    fn malformed_image_does_not_initialize_or_refresh_an_estimate() {
        let d = surface();
        let mut tracker = MultiReferenceTracker::new(camera(), Mode::PhaseGatedRgbd).unwrap();
        assert!(
            !tracker
                .step(
                    1.,
                    &d,
                    Some(ImageObservation {
                        timestamp: 1.001,
                        gray: &[]
                    })
                )
                .accepted
        );
        assert!(tracker.latest.is_none() && tracker.keys.is_empty());
        assert!(
            !tracker
                .step(
                    1.033,
                    &d,
                    Some(ImageObservation {
                        timestamp: 2.,
                        gray: &[]
                    })
                )
                .accepted
        );
        let gray = texture(0., 0.);
        assert!(
            tracker
                .step(
                    1.066,
                    &d,
                    Some(ImageObservation {
                        timestamp: 1.067,
                        gray: &gray
                    })
                )
                .accepted
        );
    }
}
