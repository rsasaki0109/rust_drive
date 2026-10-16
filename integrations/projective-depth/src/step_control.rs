//! Bounded nonlinear checks on fixed measurements and fixed IRLS weights.
use crate::multireference::Gray;
use crate::photometric::{PhotoEquation, PhotoPair};
use crate::{Camera, Level, Pose, V, dot, norm, sub};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum StepPolicy {
    #[default]
    Disabled,
    FixedSupport,
}

#[derive(Clone, Debug, Serialize)]
pub struct StepControl {
    pub geometric_count: usize,
    pub photometric_count: usize,
    pub before_cost_m2: f64,
    pub trial_costs_m2: Vec<Option<f64>>,
    /// Zero keeps the current pose and ends this pyramid level.
    pub applied_scale: f64,
}

struct Plane {
    query: V,
    target: V,
    normal: V,
    weight: f64,
}
struct Pixel {
    query: V,
    source: f64,
    weight: f64,
}
struct Image<'a> {
    gray: &'a Gray,
    camera: Camera,
    pixels: Vec<Pixel>,
    mean_source: f64,
    total: f64,
    variance: f64,
    scale: f64,
}
pub(super) struct Witnesses<'a> {
    planes: Vec<Plane>,
    image: Option<Image<'a>>,
}
impl<'a> Witnesses<'a> {
    pub(super) fn new(
        target: &Level,
        source: &Level,
        pose: Pose,
        scale: usize,
        photo: Option<&'a PhotoPair<'a>>,
        equation: Option<&PhotoEquation>,
    ) -> Self {
        let level = match scale {
            4 => 0,
            2 => 1,
            _ => 2,
        };
        let pair = photo.zip(equation).filter(|(_, e)| e.used);
        let mut planes = Vec::new();
        let mut pixels = Vec::new();
        let stride = if scale == 1 { 2 } else { 1 };
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
                let (p, n) = (target.vertices[j], target.normals[j]);
                if dot(n, n) <= 0.25
                    || norm(sub(q, p)) >= if scale == 4 { 0.12 } else { 0.08 }
                    || dot(pose.rotate(source.normals[i]), n) <= 0.8
                {
                    continue;
                }
                let r = dot(n, sub(q, p));
                planes.push(Plane {
                    query: q,
                    target: p,
                    normal: n,
                    weight: (0.01 / r.abs().max(1e-15)).min(1.),
                });
                if let Some((pair, e)) = pair {
                    let c = target.camera;
                    let Some((value, gx, gy)) = pair.target[level]
                        .sample(c.fx * q[0] / q[2] + c.cx, c.fy * q[1] / q[2] + c.cy)
                    else {
                        continue;
                    };
                    if gx.hypot(gy) < 1. {
                        continue;
                    }
                    let illumination = e.illumination.as_ref().expect("used exposure fit");
                    let color = pair.source[level].pixels[i];
                    let residual = value - illumination.gain * color - illumination.offset_gray;
                    if residual.abs() < 30. {
                        pixels.push(Pixel {
                            query: q,
                            source: color,
                            weight: (1. - (residual / 30.).powi(2)).powi(2),
                        });
                    }
                }
            }
        }
        let image = pair.map(|(pair, e)| {
            let total = pixels.iter().map(|p| p.weight).sum::<f64>();
            let mean_source = pixels.iter().map(|p| p.weight * p.source).sum::<f64>() / total;
            let variance = pixels
                .iter()
                .map(|p| p.weight * (p.source - mean_source).powi(2))
                .sum();
            Image {
                gray: &pair.target[level],
                camera: target.camera,
                pixels,
                mean_source,
                total,
                variance,
                scale: e.scale_m2_per_gray2,
            }
        });
        Self { planes, image }
    }
    pub(super) fn counts(&self) -> (usize, usize) {
        (
            self.planes.len(),
            self.image.as_ref().map_or(0, |i| i.pixels.len()),
        )
    }
    fn cost(&self, increment: Pose) -> Option<f64> {
        let mut cost = 0.;
        for p in &self.planes {
            let q = increment.transform(p.query);
            if q[2] <= 0.3 {
                return None;
            }
            cost += 0.5 * p.weight * dot(p.normal, sub(q, p.target)).powi(2);
        }
        if let Some(image) = &self.image {
            let mut values = Vec::with_capacity(image.pixels.len());
            let (mut sum, mut cross) = (0., 0.);
            for p in &image.pixels {
                let q = increment.transform(p.query);
                if q[2] <= 0.3 {
                    return None;
                }
                let c = image.camera;
                // Every retained pixel must survive. No support re-selection,
                // residual cutoff or gradient gate can reduce this trial cost.
                let (v, _, _) = image
                    .gray
                    .sample(c.fx * q[0] / q[2] + c.cx, c.fy * q[1] / q[2] + c.cy)?;
                values.push(v);
                sum += p.weight * v;
                cross += p.weight * (p.source - image.mean_source) * v;
            }
            let gain = cross / image.variance;
            let offset = sum / image.total - gain * image.mean_source;
            for (p, v) in image.pixels.iter().zip(values) {
                cost += 0.5 * image.scale * p.weight * (v - gain * p.source - offset).powi(2);
            }
        }
        cost.is_finite().then_some(cost)
    }
    pub(super) fn select(&self, delta: [f64; 6]) -> Result<StepControl, String> {
        let before = self
            .cost(Pose::identity())
            .ok_or("invalid initial fixed-support cost")?;
        let mut report = StepControl {
            geometric_count: self.counts().0,
            photometric_count: self.counts().1,
            before_cost_m2: before,
            trial_costs_m2: Vec::new(),
            applied_scale: 0.,
        };
        for scale in [1., 0.5, 0.25, 0.125, 0.0625] {
            let cost = self.cost(Pose::identity().increment(delta.map(|v| v * scale)));
            report.trial_costs_m2.push(cost);
            if cost.is_some_and(|v| v <= before + 1e-12 * (1. + before)) {
                report.applied_scale = scale;
                break;
            }
        }
        Ok(report)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn overshoot_is_halved_and_uphill_steps_end_the_level() {
        let witnesses = Witnesses {
            planes: vec![Plane {
                query: [0., 0., 1.],
                target: [0., 0., 1.01],
                normal: [0., 0., 1.],
                weight: 1.,
            }],
            image: None,
        };
        let report = witnesses.select([0., 0., 0.03, 0., 0., 0.]).unwrap();
        assert_eq!(report.applied_scale, 0.5);
        assert!(report.trial_costs_m2[0].unwrap() > report.before_cost_m2);
        assert!(report.trial_costs_m2[1].unwrap() < report.before_cost_m2);
        let uphill = witnesses.select([0., 0., -0.01, 0., 0., 0.]).unwrap();
        assert_eq!(uphill.applied_scale, 0.);
        assert_eq!(uphill.trial_costs_m2.len(), 5);
    }
    #[test]
    fn one_pixel_leaving_the_image_cannot_reduce_the_cost() {
        let gray = Gray {
            width: 4,
            height: 4,
            pixels: (0..16).map(|i| i as f64 * 10.).collect(),
        };
        let witnesses = Witnesses {
            planes: Vec::new(),
            image: Some(Image {
                gray: &gray,
                camera: Camera {
                    width: 4,
                    height: 4,
                    fx: 1.,
                    fy: 1.,
                    cx: 0.,
                    cy: 0.,
                },
                pixels: vec![
                    Pixel {
                        query: [2.9, 1., 1.],
                        source: 50.,
                        weight: 1.,
                    },
                    Pixel {
                        query: [1., 1., 1.],
                        source: 100.,
                        weight: 1.,
                    },
                ],
                mean_source: 75.,
                total: 2.,
                variance: 1250.,
                scale: 0.005_f64.powi(2) / 100.,
            }),
        };
        let report = witnesses.select([0.2, 0., 0., 0., 0., 0.]).unwrap();
        assert_eq!(report.photometric_count, 2);
        assert_eq!(report.applied_scale, 0.25);
        assert_eq!(&report.trial_costs_m2[..2], &[None, None]);
        assert!(report.trial_costs_m2[2].unwrap() < 1e-12);
    }
}
