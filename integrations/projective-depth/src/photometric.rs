//! Measured keyframe intensity complements the fused depth surface. No labels.
use crate::multireference::Gray;
use crate::{Equation, H, Level, Pose, cross, dot, norm, sub};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Photometric {
    #[default]
    Disabled,
    Keyframe,
}

#[derive(Clone, Debug, Serialize)]
pub struct PhotoEquation {
    pub count: usize,
    pub rms_gray: Option<f64>,
    pub used: bool,
    pub scale_m2_per_gray2: f64,
    pub h: H,
    pub g: [f64; 6],
    pub illumination: Option<Illumination>,
}
#[derive(Clone, Debug, Serialize)]
pub struct Illumination {
    /// Reference intensity = gain * source intensity + offset_gray.
    pub gain: f64,
    pub offset_gray: f64,
    pub candidate_count: usize,
    pub source_variance_gray2: f64,
}
struct Sample {
    target: f64,
    source: f64,
    jacobian: [f64; 6],
}

fn tukey(residual: f64) -> f64 {
    if residual.abs() < 30. {
        (1. - (residual / 30.).powi(2)).powi(2)
    } else {
        0.
    }
}

// Centering avoids cancellation for bright, nearly constant patches. Exposure
// is a nuisance parameter, never a source of pose information or persistence.
fn exposure(samples: &[Sample], weights: &[f64]) -> Option<(f64, f64)> {
    let total: f64 = weights.iter().sum();
    if total <= 0. {
        return None;
    }
    let mean = |source: bool| {
        samples
            .iter()
            .zip(weights)
            .map(|(s, w)| w * if source { s.source } else { s.target })
            .sum::<f64>()
            / total
    };
    let (x, y) = (mean(true), mean(false));
    let (mut variance, mut covariance) = (0., 0.);
    for (s, w) in samples.iter().zip(weights) {
        variance += w * (s.source - x).powi(2);
        covariance += w * (s.source - x) * (s.target - y);
    }
    if variance / total < 25. {
        return None;
    }
    let gain = covariance / variance;
    Some((gain, y - gain * x))
}

fn affine_equation(samples: &[Sample], out: &mut PhotoEquation) {
    if samples.len() < 200 {
        return;
    }
    let mut weights = vec![1.; samples.len()];
    let Some((mut gain, mut offset)) = exposure(samples, &weights) else {
        return;
    };
    for _ in 0..3 {
        for (s, w) in samples.iter().zip(&mut weights) {
            *w = tukey(s.target - gain * s.source - offset);
        }
        let Some(next) = exposure(samples, &weights) else {
            return;
        };
        (gain, offset) = next;
    }
    if !(0.5..=2.).contains(&gain) || offset.abs() > 80. {
        return;
    }
    let (mut total, mut x_sum, mut squared, mut count) = (0., 0., 0., 0);
    for (s, w) in samples.iter().zip(&mut weights) {
        let r = s.target - gain * s.source - offset;
        *w = tukey(r);
        if *w > 0. {
            count += 1;
            squared += r * r;
        }
        total += *w;
        x_sum += *w * s.source;
    }
    if count < 200 || total <= 0. {
        return;
    }
    let center = x_sum / total;
    let mut variance = 0.;
    let mut cross_terms = [[0.; 2]; 6];
    let mut nuisance_g = [0.; 2];
    for (s, w) in samples.iter().zip(weights) {
        let x = s.source - center;
        let r = s.target - gain * s.source - offset;
        variance += w * x * x;
        // The minus signs of both nuisance columns cancel in the Schur term.
        nuisance_g[0] += w * x * r;
        nuisance_g[1] += w * r;
        for (a, terms) in cross_terms.iter_mut().enumerate() {
            out.g[a] += w * s.jacobian[a] * r;
            terms[0] += w * s.jacobian[a] * x;
            terms[1] += w * s.jacobian[a];
            for b in 0..6 {
                out.h[a][b] += w * (s.jacobian[a] * s.jacobian[b]);
            }
        }
    }
    if variance / total < 25. {
        out.h = [[0.; 6]; 6];
        out.g = [0.; 6];
        return;
    }
    // Centered nuisance columns have diagonal normal matrix (variance,total).
    // Project out both directions rather than fixing the estimated exposure
    // while differentiating pose, which would invent pose information.
    for a in 0..6 {
        out.g[a] -= cross_terms[a][0] * nuisance_g[0] / variance
            + cross_terms[a][1] * nuisance_g[1] / total;
        for b in 0..6 {
            out.h[a][b] -= cross_terms[a][0] * cross_terms[b][0] / variance
                + cross_terms[a][1] * cross_terms[b][1] / total;
        }
    }
    out.count = count;
    out.rms_gray = Some((squared / count as f64).sqrt());
    out.used = true;
    out.illumination = Some(Illumination {
        gain,
        offset_gray: offset,
        candidate_count: samples.len(),
        source_variance_gray2: variance / total,
    });
}
#[derive(Debug, Serialize)]
pub struct PhotoState {
    pub source_timestamp: f64,
    pub reference_timestamp: Option<f64>,
    pub phase_qualified: bool,
    pub fresh: bool,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timing: Option<TimingState>,
}
#[derive(Debug, Serialize)]
pub struct TimingState {
    pub source: crate::motion::TimeWarp,
    pub reference: Option<crate::motion::TimeWarp>,
}
pub(super) struct PhotoPair<'a> {
    pub target: &'a [Gray],
    pub source: &'a [Gray],
    pub enabled: bool,
    pub warps: Option<(Pose, Pose)>,
}
impl PhotoPair<'_> {
    pub(super) fn equation(
        &self,
        target: &Level,
        source: &Level,
        pose: Pose,
        scale: usize,
    ) -> PhotoEquation {
        let mut out = PhotoEquation {
            count: 0,
            rms_gray: None,
            used: false,
            scale_m2_per_gray2: 0.005_f64.powi(2) / 100.,
            h: [[0.; 6]; 6],
            g: [0.; 6],
            illumination: None,
        };
        if !self.enabled {
            return out;
        }
        let level = match scale {
            4 => 0,
            2 => 1,
            _ => 2,
        };
        let stride = if scale == 1 { 2 } else { 1 };
        let mut samples = Vec::new();
        for y in (0..source.camera.height).step_by(stride) {
            for x in (0..source.camera.width).step_by(stride) {
                let i = y * source.camera.width + x;
                if dot(source.normals[i], source.normals[i]) <= 0.25 {
                    continue;
                }
                let q = pose.transform(source.vertices[i]);
                let Some(j) = target.camera.project(q, true) else {
                    continue;
                };
                let n = target.normals[j];
                if dot(n, n) <= 0.25
                    || norm(sub(q, target.vertices[j])) >= if scale == 4 { 0.12 } else { 0.08 }
                    || dot(pose.rotate(source.normals[i]), n) <= 0.8
                {
                    continue;
                }
                let c = target.camera;
                let (reference_warp, source_warp) =
                    self.warps.unwrap_or((Pose::identity(), Pose::identity()));
                let image_q = if self.warps.is_some() {
                    reference_warp.transform(q)
                } else {
                    q
                };
                let z = image_q[2];
                if z <= 0.3 {
                    continue;
                }
                let Some((value, gx, gy)) = self.target[level]
                    .sample(c.fx * image_q[0] / z + c.cx, c.fy * image_q[1] / z + c.cy)
                else {
                    continue;
                };
                if gx.hypot(gy) < 1. {
                    continue;
                }
                let source_value = if self.warps.is_some() {
                    let p = source_warp.transform(source.vertices[i]);
                    if p[2] <= 0.3 {
                        continue;
                    }
                    let c = source.camera;
                    let Some((value, _, _)) = self.source[level]
                        .sample(c.fx * p[0] / p[2] + c.cx, c.fy * p[1] / p[2] + c.cy)
                    else {
                        continue;
                    };
                    value
                } else {
                    self.source[level].pixels[i]
                };
                samples.push(Sample {
                    target: value,
                    source: source_value,
                    jacobian: if self.warps.is_some() {
                        warped_jacobian(c, q, reference_warp, gx, gy)
                    } else {
                        image_jacobian(c, q, gx, gy)
                    },
                });
            }
        }
        affine_equation(&samples, &mut out);
        out
    }
}
fn warped_jacobian(c: crate::Camera, depth_q: [f64; 3], warp: Pose, gx: f64, gy: f64) -> [f64; 6] {
    let q = warp.transform(depth_q);
    let image = image_jacobian(c, q, gx, gy);
    let g = warp.inverse().rotate([image[0], image[1], image[2]]);
    let r = cross(depth_q, g);
    [g[0], g[1], g[2], r[0], r[1], r[2]]
}
fn image_jacobian(c: crate::Camera, q: [f64; 3], gx: f64, gy: f64) -> [f64; 6] {
    let z = q[2];
    let g = [
        gx * c.fx / z,
        gy * c.fy / z,
        -(gx * c.fx * q[0] + gy * c.fy * q[1]) / (z * z),
    ];
    let r = cross(q, g);
    [g[0], g[1], g[2], r[0], r[1], r[2]]
}

pub(super) fn combine(geometry: &Equation, photo: Option<&PhotoEquation>) -> (H, [f64; 6]) {
    let (mut h, mut g) = (geometry.h, geometry.g);
    if let Some(p) = photo
        && p.used
    {
        for a in 0..6 {
            g[a] += p.scale_m2_per_gray2 * p.g[a];
            for b in 0..6 {
                h[a][b] += p.scale_m2_per_gray2 * p.h[a][b];
            }
        }
    }
    (h, g)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Camera, ImageObservation, Tracker, TrackerConfig};
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
    fn texture(c: Camera, transform: Pose) -> Vec<u8> {
        (0..c.width * c.height)
            .map(|i| {
                let p = transform.transform(c.point(i, 2.5));
                (128.
                    + 30. * (7. * p[0]).sin()
                    + 25. * (9. * p[1]).cos()
                    + 20. * (4. * p[0] + 5. * p[1]).sin())
                .round()
                .clamp(0., 255.) as u8
            })
            .collect()
    }
    #[test]
    fn measured_image_jacobian_matches_all_six_pose_directions() {
        let c = camera();
        let q = [0.17, -0.12, 2.3];
        let expected = image_jacobian(c, q, 2., 3.);
        let value = |delta: [f64; 6]| {
            let p = Pose::identity().increment(delta).transform(q);
            2. * (c.fx * p[0] / p[2] + c.cx) + 3. * (c.fy * p[1] / p[2] + c.cy)
        };
        for i in 0..6 {
            let mut a = [0.; 6];
            let mut b = [0.; 6];
            a[i] = 1e-7;
            b[i] = -1e-7;
            assert!(((value(a) - value(b)) / 2e-7 - expected[i]).abs() < 2e-6);
        }
    }
    #[test]
    fn exposure_warp_jacobian_matches_all_six_depth_pose_directions() {
        let c = camera();
        let q = [0.17, -0.12, 2.3];
        let warp = Pose::identity().increment([0.012, -0.008, 0.003, 0.008, -0.01, 0.015]);
        let expected = warped_jacobian(c, q, warp, 2., 3.);
        let value = |delta: [f64; 6]| {
            let p = warp.transform(Pose::identity().increment(delta).transform(q));
            2. * (c.fx * p[0] / p[2] + c.cx) + 3. * (c.fy * p[1] / p[2] + c.cy)
        };
        for i in 0..6 {
            let mut a = [0.; 6];
            let mut b = [0.; 6];
            a[i] = 1e-7;
            b[i] = -1e-7;
            assert!(((value(a) - value(b)) / 2e-7 - expected[i]).abs() < 2e-6);
        }
    }
    #[test]
    fn textured_plane_recovers_motion_unobservable_from_depth() {
        let c = camera();
        let depth = vec![2.5; c.width * c.height];
        let transform = Pose::identity().increment([0.015, -0.01, 0., 0., 0., 0.01]);
        let first = texture(c, Pose::identity());
        let current = texture(c, transform);
        let mut geometry = Tracker::new(c).unwrap();
        assert!(geometry.step(1., &depth).accepted);
        assert!(!geometry.step(1.03, &depth).accepted);
        let mut tracker = Tracker::with_config(
            c,
            TrackerConfig {
                photometric: Photometric::Keyframe,
                ..TrackerConfig::default()
            },
        )
        .unwrap();
        assert!(
            tracker
                .step_with_image(
                    1.,
                    &depth,
                    Some(ImageObservation {
                        timestamp: 1.001,
                        gray: &first
                    })
                )
                .accepted
        );
        let result = tracker.step_with_image(
            1.03,
            &depth,
            Some(ImageObservation {
                timestamp: 1.031,
                gray: &current,
            }),
        );
        assert!(result.accepted, "{:?}", result.rejection);
        assert!(result.final_photometric.unwrap().used);
        let error = transform
            .inverse()
            .compose(result.first_from_current.unwrap());
        assert!(norm(error.translation) < 0.004 && error.angle() < 0.004);
    }
    #[test]
    fn exposure_change_does_not_become_plane_motion() {
        let c = camera();
        let depth = vec![2.5; c.width * c.height];
        for transform in [
            Pose::identity(),
            Pose::identity().increment([0.015, -0.01, 0., 0., 0., 0.01]),
        ] {
            let first = texture(c, Pose::identity());
            let current: Vec<_> = texture(c, transform)
                .iter()
                .map(|&v| (0.75 * f64::from(v) + 25.).round() as u8)
                .collect();
            let mut tracker = Tracker::with_config(
                c,
                TrackerConfig {
                    photometric: Photometric::Keyframe,
                    ..TrackerConfig::default()
                },
            )
            .unwrap();
            tracker.step_with_image(
                1.,
                &depth,
                Some(ImageObservation {
                    timestamp: 1.001,
                    gray: &first,
                }),
            );
            let result = tracker.step_with_image(
                1.03,
                &depth,
                Some(ImageObservation {
                    timestamp: 1.031,
                    gray: &current,
                }),
            );
            assert!(result.accepted, "{:?}", result.rejection);
            let exposure = result.final_photometric.unwrap().illumination.unwrap();
            assert!((exposure.gain - 4. / 3.).abs() < 0.03);
            assert!((exposure.offset_gray + 100. / 3.).abs() < 3.);
            let error = transform
                .inverse()
                .compose(result.first_from_current.unwrap());
            assert!(norm(error.translation) < 0.004 && error.angle() < 0.004);
        }
    }
    #[test]
    fn affine_brightness_directions_supply_no_pose_information() {
        let samples: Vec<_> = (0..1000)
            .map(|i| {
                let source = (i % 200) as f64 + 20.;
                Sample {
                    source,
                    target: 1.1 * source + 7.,
                    jacobian: [source, 1., (i as f64).sin(), 0., 0., 0.],
                }
            })
            .collect();
        let mut out = PhotoEquation {
            count: 0,
            rms_gray: None,
            used: false,
            scale_m2_per_gray2: 1.,
            h: [[0.; 6]; 6],
            g: [0.; 6],
            illumination: None,
        };
        affine_equation(&samples, &mut out);
        assert!(out.used);
        assert!(out.h[0][0].abs() < 1e-5 && out.h[1][1].abs() < 1e-8);
        assert!(out.h[2][2] > 400.);
        assert!(out.g.iter().all(|v| v.abs() < 1e-8));
        // Bright but textureless data cannot determine exposure gain.
        let flat: Vec<_> = samples
            .into_iter()
            .map(|s| Sample { source: 100., ..s })
            .collect();
        out.count = 0;
        out.used = false;
        out.illumination = None;
        affine_equation(&flat, &mut out);
        assert!(!out.used && out.illumination.is_none());
    }
    #[test]
    fn repeated_rgb_adds_no_constraint_but_new_depth_can_update() {
        let c = camera();
        let depth: Vec<_> = (0..c.width * c.height)
            .map(|i| {
                2. + 0.2 * ((i % c.width) as f64 * 0.07).sin() * ((i / c.width) as f64 * 0.09).sin()
            })
            .collect();
        let gray = texture(c, Pose::identity());
        let mut tracker = Tracker::with_config(
            c,
            TrackerConfig {
                photometric: Photometric::Keyframe,
                ..TrackerConfig::default()
            },
        )
        .unwrap();
        assert!(
            tracker
                .step_with_image(
                    1.,
                    &depth,
                    Some(ImageObservation {
                        timestamp: 1.001,
                        gray: &gray
                    })
                )
                .accepted
        );
        let repeated = tracker.step_with_image(
            1.004,
            &depth,
            Some(ImageObservation {
                timestamp: 1.001,
                gray: &gray,
            }),
        );
        assert!(repeated.accepted);
        assert!(!repeated.photometric.unwrap().fresh);
        let e = repeated.final_photometric.unwrap();
        assert_eq!(e.count, 0);
        assert!(!e.used);
        assert!(
            !tracker
                .step_with_image(
                    1.004,
                    &depth,
                    Some(ImageObservation {
                        timestamp: 1.001,
                        gray: &gray
                    })
                )
                .accepted
        );
        assert!(!tracker.step_with_image(1.01, &depth, None).accepted);
        assert_eq!(tracker.last_accepted, Some(1.004));
        assert!(
            !tracker
                .step_with_image(
                    1.02,
                    &depth,
                    Some(ImageObservation {
                        timestamp: 1.,
                        gray: &gray
                    })
                )
                .accepted
        );
        assert_eq!(tracker.last_accepted, Some(1.004));
        assert!(
            tracker
                .step_with_image(
                    1.03,
                    &depth,
                    Some(ImageObservation {
                        timestamp: 1.031,
                        gray: &gray
                    })
                )
                .accepted
        );
    }
}
