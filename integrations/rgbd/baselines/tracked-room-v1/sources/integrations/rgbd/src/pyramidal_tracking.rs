//! Bounded, bidirectional pyramidal Lucas–Kanade point tracking.
//!
//! Original translation-only inverse-compositional implementation. No image
//! IO, geometric pose estimation, motion priors, or external dependencies.
//! Coordinates are original image pixels (x right, y down). Patch means are
//! removed to tolerate a small additive brightness change; scale/rotation,
//! exposure gain, uniqueness on repeated textures, and occlusion recovery are
//! not claimed. Rejected tracks must not be used as measurements.

pub const WIDTH: usize = 640;
pub const HEIGHT: usize = 480;
pub const MAX_POINTS: usize = 400;
pub const PYRAMID_LEVELS: usize = 3;
pub const PATCH_RADIUS: isize = 4;
pub const PATCH_PIXELS: usize = 81;
pub const MAX_ITERATIONS: usize = 10;
pub const MIN_EIGENVALUE: f64 = 4.0;
pub const MAX_CONDITION_NUMBER: f64 = 100.0;
pub const MAX_STEP: f64 = 2.0;
pub const CONVERGENCE_STEP: f64 = 0.01;
pub const MAX_PHOTOMETRIC_RMS: f64 = 15.0;
pub const MAX_FORWARD_BACKWARD_ERROR: f64 = 1.0;
/// Template: five bilinear samples/pixel. Each of ten iterations and the
/// final photometric check: one sample/pixel. Both directions, three levels.
pub const MAX_BILINEAR_SAMPLES: usize =
    MAX_POINTS * 2 * PYRAMID_LEVELS * PATCH_PIXELS * (5 + MAX_ITERATIONS + 1);
pub const MAX_TOTAL_ITERATIONS: usize = MAX_POINTS * 2 * PYRAMID_LEVELS * MAX_ITERATIONS;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingInputError {
    InvalidDimensions,
    InvalidPixelCount,
    InvalidMask,
    TooManyPoints,
    InvalidPoint,
}

/// Tightly packed grayscale pixels and optional binary validity mask.
/// Only 640×480 is supported; 0 means invalid and 1 valid in the mask.
#[derive(Clone, Copy, Debug)]
pub struct GrayFrame<'a> {
    pixels: &'a [u8],
    validity_mask: Option<&'a [u8]>,
}

impl<'a> GrayFrame<'a> {
    pub fn new(width: usize, height: usize, pixels: &'a [u8]) -> Result<Self, TrackingInputError> {
        if width != WIDTH || height != HEIGHT {
            return Err(TrackingInputError::InvalidDimensions);
        }
        if pixels.len() != WIDTH * HEIGHT {
            return Err(TrackingInputError::InvalidPixelCount);
        }
        Ok(Self {
            pixels,
            validity_mask: None,
        })
    }

    pub fn with_mask(mut self, mask: &'a [u8]) -> Result<Self, TrackingInputError> {
        if mask.len() != WIDTH * HEIGHT || mask.iter().any(|&value| value > 1) {
            return Err(TrackingInputError::InvalidMask);
        }
        self.validity_mask = Some(mask);
        Ok(self)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingRejection {
    Border,
    Masked,
    LowTexture,
    IllConditioned,
    NonConvergence,
    Photometric,
    ForwardBackward,
}

/// Only level summaries are retained, not iteration trajectories.
/// Zero eigenvalue/condition are sentinels before a valid template is built;
/// the level's explicit rejection identifies that case.
#[derive(Clone, Debug, PartialEq)]
pub struct LevelTrace {
    pub level: u8,
    pub start: [f64; 2],
    pub endpoint: [f64; 2],
    pub iterations: u8,
    pub bilinear_samples: usize,
    pub min_eigenvalue: f64,
    pub condition_number: f64,
    pub photometric_rms: Option<f64>,
    pub converged: bool,
    pub rejection: Option<TrackingRejection>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct PointTrack {
    pub point_index: usize,
    pub start: [f64; 2],
    /// Populated only if ALL forward levels pass, even when backward rejects.
    pub forward_endpoint: Option<[f64; 2]>,
    /// Populated only if ALL backward levels pass.
    pub backward_endpoint: Option<[f64; 2]>,
    pub forward_backward_error: Option<f64>,
    pub accepted: bool,
    pub rejection: Option<TrackingRejection>,
    pub forward_levels: Vec<LevelTrace>,
    pub backward_levels: Vec<LevelTrace>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct TrackingReport {
    pub points: Vec<PointTrack>,
    pub bilinear_samples: usize,
    pub iterations: usize,
    /// Both pyramids produce 320×240 and 160×120 pixels, unless input is empty.
    pub pyramid_output_pixels: usize,
}

struct Level {
    width: usize,
    height: usize,
    pixels: Vec<u8>,
    mask: Vec<u8>,
}

fn pyramid(frame: GrayFrame<'_>) -> [Level; PYRAMID_LEVELS] {
    let initial = Level {
        width: WIDTH,
        height: HEIGHT,
        pixels: frame.pixels.to_vec(),
        mask: frame
            .validity_mask
            .map_or_else(|| vec![1; WIDTH * HEIGHT], <[u8]>::to_vec),
    };
    let middle = downsample(&initial);
    let coarse = downsample(&middle);
    [initial, middle, coarse]
}

fn downsample(input: &Level) -> Level {
    let width = input.width / 2;
    let height = input.height / 2;
    let mut pixels = vec![0_u8; width * height];
    let mut mask = vec![1_u8; width * height];
    let weights = [1_u32, 4, 6, 4, 1];
    for y in 0..height {
        for x in 0..width {
            let mut sum = 0_u32;
            let mut valid = 1_u8;
            for (dy, &wy) in weights.iter().enumerate() {
                let row = (2 * y + dy).saturating_sub(2).min(input.height - 1);
                for (dx, &wx) in weights.iter().enumerate() {
                    let column = (2 * x + dx).saturating_sub(2).min(input.width - 1);
                    let index = row * input.width + column;
                    sum += wy * wx * u32::from(input.pixels[index]);
                    valid &= input.mask[index];
                }
            }
            pixels[y * width + x] = ((sum + 128) / 256) as u8;
            mask[y * width + x] = valid;
        }
    }
    Level {
        width,
        height,
        pixels,
        mask,
    }
}

fn sample(image: &Level, x: f64, y: f64, samples: &mut usize) -> Result<f64, TrackingRejection> {
    *samples += 1;
    if !x.is_finite()
        || !y.is_finite()
        || x < 0.0
        || y < 0.0
        || x >= (image.width - 1) as f64
        || y >= (image.height - 1) as f64
    {
        return Err(TrackingRejection::Border);
    }
    let column = x.floor() as usize;
    let row = y.floor() as usize;
    let indices = [
        row * image.width + column,
        row * image.width + column + 1,
        (row + 1) * image.width + column,
        (row + 1) * image.width + column + 1,
    ];
    if indices.iter().any(|&index| image.mask[index] == 0) {
        return Err(TrackingRejection::Masked);
    }
    let dx = x - column as f64;
    let dy = y - row as f64;
    let top =
        f64::from(image.pixels[indices[0]]) * (1.0 - dx) + f64::from(image.pixels[indices[1]]) * dx;
    let bottom =
        f64::from(image.pixels[indices[2]]) * (1.0 - dx) + f64::from(image.pixels[indices[3]]) * dx;
    Ok(top * (1.0 - dy) + bottom * dy)
}

fn patch(
    image: &Level,
    center: [f64; 2],
    samples: &mut usize,
) -> Result<[f64; PATCH_PIXELS], TrackingRejection> {
    let mut values = [0.0; PATCH_PIXELS];
    let mut index = 0;
    for dy in -PATCH_RADIUS..=PATCH_RADIUS {
        for dx in -PATCH_RADIUS..=PATCH_RADIUS {
            values[index] = sample(image, center[0] + dx as f64, center[1] + dy as f64, samples)?;
            index += 1;
        }
    }
    let mean = values.iter().sum::<f64>() / PATCH_PIXELS as f64;
    for value in &mut values {
        *value -= mean;
    }
    Ok(values)
}

struct Template {
    values: [f64; PATCH_PIXELS],
    gradients: [[f64; 2]; PATCH_PIXELS],
    hessian: [f64; 3],
    min_eigenvalue: f64,
    condition_number: f64,
}

fn template(
    image: &Level,
    center: [f64; 2],
    samples: &mut usize,
) -> Result<Template, TrackingRejection> {
    let mut values = [0.0; PATCH_PIXELS];
    let mut gradients = [[0.0; 2]; PATCH_PIXELS];
    let mut index = 0;
    for dy in -PATCH_RADIUS..=PATCH_RADIUS {
        for dx in -PATCH_RADIUS..=PATCH_RADIUS {
            let x = center[0] + dx as f64;
            let y = center[1] + dy as f64;
            values[index] = sample(image, x, y, samples)?;
            let gx =
                (sample(image, x + 1.0, y, samples)? - sample(image, x - 1.0, y, samples)?) * 0.5;
            let gy =
                (sample(image, x, y + 1.0, samples)? - sample(image, x, y - 1.0, samples)?) * 0.5;
            gradients[index] = [gx, gy];
            index += 1;
        }
    }
    let mean = values.iter().sum::<f64>() / PATCH_PIXELS as f64;
    let mean_x = gradients.iter().map(|gradient| gradient[0]).sum::<f64>() / PATCH_PIXELS as f64;
    let mean_y = gradients.iter().map(|gradient| gradient[1]).sum::<f64>() / PATCH_PIXELS as f64;
    let mut hessian = [0.0; 3];
    for (value, gradient) in values.iter_mut().zip(&mut gradients) {
        *value -= mean;
        gradient[0] -= mean_x;
        gradient[1] -= mean_y;
        hessian[0] += gradient[0] * gradient[0];
        hessian[1] += gradient[0] * gradient[1];
        hessian[2] += gradient[1] * gradient[1];
    }
    let [a, b, c] = hessian.map(|value| value / PATCH_PIXELS as f64);
    let discriminant = ((a - c) * (a - c) + 4.0 * b * b).sqrt();
    let minimum = ((a + c - discriminant) * 0.5).max(0.0);
    let maximum = (a + c + discriminant) * 0.5;
    // Finite sentinels keep a diagnostic JSON writer from receiving infinity.
    let condition = if minimum > 0.0 {
        maximum / minimum
    } else {
        f64::MAX
    };
    Ok(Template {
        values,
        gradients,
        hessian,
        min_eigenvalue: minimum,
        condition_number: condition,
    })
}

fn fit_level(
    previous: &Level,
    current: &Level,
    point: [f64; 2],
    initial: [f64; 2],
    level: u8,
) -> LevelTrace {
    let mut trace = LevelTrace {
        level,
        start: initial,
        endpoint: initial,
        iterations: 0,
        bilinear_samples: 0,
        min_eigenvalue: 0.0,
        condition_number: 0.0,
        photometric_rms: None,
        converged: false,
        rejection: None,
    };
    let result = (|| {
        let template = template(previous, point, &mut trace.bilinear_samples)?;
        trace.min_eigenvalue = template.min_eigenvalue;
        trace.condition_number = template.condition_number;
        if template.min_eigenvalue < MIN_EIGENVALUE {
            return Err(TrackingRejection::LowTexture);
        }
        if template.condition_number > MAX_CONDITION_NUMBER {
            return Err(TrackingRejection::IllConditioned);
        }
        let [a, b, c] = template.hessian;
        let determinant = a * c - b * b;
        if !determinant.is_finite() || determinant <= 0.0 {
            return Err(TrackingRejection::IllConditioned);
        }
        for _ in 0..MAX_ITERATIONS {
            trace.iterations += 1;
            let values = patch(current, trace.endpoint, &mut trace.bilinear_samples)?;
            let mut rhs = [0.0; 2];
            for ((value, reference), gradient) in
                values.iter().zip(template.values).zip(template.gradients)
            {
                let residual = value - reference;
                rhs[0] += gradient[0] * residual;
                rhs[1] += gradient[1] * residual;
            }
            let mut step = [
                (c * rhs[0] - b * rhs[1]) / determinant,
                (a * rhs[1] - b * rhs[0]) / determinant,
            ];
            let length = step[0].hypot(step[1]);
            if !length.is_finite() {
                return Err(TrackingRejection::NonConvergence);
            }
            if length > MAX_STEP {
                let scale = MAX_STEP / length;
                step[0] *= scale;
                step[1] *= scale;
            }
            trace.endpoint[0] -= step[0];
            trace.endpoint[1] -= step[1];
            if length <= CONVERGENCE_STEP {
                trace.converged = true;
                break;
            }
        }
        if !trace.converged {
            return Err(TrackingRejection::NonConvergence);
        }
        let final_values = patch(current, trace.endpoint, &mut trace.bilinear_samples)?;
        let rms = (final_values
            .iter()
            .zip(template.values)
            .map(|(value, reference)| (value - reference).powi(2))
            .sum::<f64>()
            / PATCH_PIXELS as f64)
            .sqrt();
        trace.photometric_rms = Some(rms);
        if rms > MAX_PHOTOMETRIC_RMS {
            return Err(TrackingRejection::Photometric);
        }
        Ok(())
    })();
    trace.rejection = result.err();
    trace
}

fn direction(
    previous: &[Level; PYRAMID_LEVELS],
    current: &[Level; PYRAMID_LEVELS],
    point: [f64; 2],
) -> (Option<[f64; 2]>, Vec<LevelTrace>, Option<TrackingRejection>) {
    let mut endpoint = [point[0] / 4.0, point[1] / 4.0];
    let mut levels = Vec::with_capacity(PYRAMID_LEVELS);
    for index in (0..PYRAMID_LEVELS).rev() {
        let scale = (1_usize << index) as f64;
        if index != PYRAMID_LEVELS - 1 {
            endpoint[0] *= 2.0;
            endpoint[1] *= 2.0;
        }
        let trace = fit_level(
            &previous[index],
            &current[index],
            [point[0] / scale, point[1] / scale],
            endpoint,
            index as u8,
        );
        endpoint = trace.endpoint;
        let rejection = trace.rejection;
        levels.push(trace);
        if rejection.is_some() {
            return (None, levels, rejection);
        }
    }
    (Some(endpoint), levels, None)
}

/// Track original feature points without descriptor matches or depth input.
/// All inputs are validated before any pyramid allocation or point processing.
/// Invalid frames/points reject the whole call. A valid point can still reject
/// at a border, invalid mask, insufficient texture, or failed fit; no rejected
/// level or direction supplies a usable endpoint. Each reverse track starts
/// independently with zero motion at the coarsest level. Work counters count
/// attempted bilinear samples and iteration patch evaluations, including errors.
pub fn track_points(
    previous: GrayFrame<'_>,
    current: GrayFrame<'_>,
    points: &[[f64; 2]],
) -> Result<TrackingReport, TrackingInputError> {
    if points.len() > MAX_POINTS {
        return Err(TrackingInputError::TooManyPoints);
    }
    if points.iter().any(|point| {
        !point[0].is_finite()
            || !point[1].is_finite()
            || point[0] < 0.0
            || point[1] < 0.0
            || point[0] >= WIDTH as f64
            || point[1] >= HEIGHT as f64
    }) {
        return Err(TrackingInputError::InvalidPoint);
    }
    if points.is_empty() {
        return Ok(TrackingReport {
            points: Vec::new(),
            bilinear_samples: 0,
            iterations: 0,
            pyramid_output_pixels: 0,
        });
    }
    let previous = pyramid(previous);
    let current = pyramid(current);
    let mut report = TrackingReport {
        points: Vec::with_capacity(points.len()),
        bilinear_samples: 0,
        iterations: 0,
        pyramid_output_pixels: 2 * (320 * 240 + 160 * 120),
    };
    for (point_index, &start) in points.iter().enumerate() {
        let (forward_endpoint, forward_levels, mut rejection) =
            direction(&previous, &current, start);
        let mut backward_endpoint = None;
        let mut backward_levels = Vec::new();
        let mut forward_backward_error = None;
        if let Some(endpoint) = forward_endpoint {
            let (backward, levels, error) = direction(&current, &previous, endpoint);
            backward_endpoint = backward;
            backward_levels = levels;
            rejection = error;
            if let Some(endpoint) = backward_endpoint {
                let distance = (endpoint[0] - start[0]).hypot(endpoint[1] - start[1]);
                forward_backward_error = Some(distance);
                if distance > MAX_FORWARD_BACKWARD_ERROR {
                    rejection = Some(TrackingRejection::ForwardBackward);
                }
            }
        }
        for level in forward_levels.iter().chain(&backward_levels) {
            report.bilinear_samples += level.bilinear_samples;
            report.iterations += usize::from(level.iterations);
        }
        report.points.push(PointTrack {
            point_index,
            start,
            forward_endpoint,
            backward_endpoint,
            forward_backward_error,
            accepted: rejection.is_none(),
            rejection,
            forward_levels,
            backward_levels,
        });
    }
    debug_assert!(report.bilinear_samples <= MAX_BILINEAR_SAMPLES);
    debug_assert!(report.iterations <= MAX_TOTAL_ITERATIONS);
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn analytic(x: f64, y: f64) -> f64 {
        128.0
            + 40.0 * (0.20 * x + 0.10 * y).sin()
            + 35.0 * (0.13 * x - 0.24 * y).cos()
            + 30.0 * (0.39 * x + 0.31 * y).sin()
    }

    fn image(dx: f64, dy: f64, brightness: f64) -> Vec<u8> {
        (0..WIDTH * HEIGHT)
            .map(|index| {
                let x = (index % WIDTH) as f64 - dx;
                let y = (index / WIDTH) as f64 - dy;
                (analytic(x, y) + brightness).round().clamp(0.0, 255.0) as u8
            })
            .collect()
    }

    fn frame(pixels: &[u8]) -> GrayFrame<'_> {
        GrayFrame::new(WIDTH, HEIGHT, pixels).unwrap()
    }

    fn points() -> Vec<[f64; 2]> {
        vec![
            [100.0, 100.0],
            [240.5, 180.25],
            [360.0, 320.0],
            [520.3, 380.7],
        ]
    }

    fn assert_translation(dx: f64, dy: f64, brightness: f64, tolerance: f64) {
        let previous = image(0.0, 0.0, 0.0);
        let current = image(dx, dy, brightness);
        let report = track_points(frame(&previous), frame(&current), &points()).unwrap();
        for track in &report.points {
            assert!(track.accepted, "{track:?}");
            let endpoint = track.forward_endpoint.unwrap();
            let error =
                (endpoint[0] - track.start[0] - dx).hypot(endpoint[1] - track.start[1] - dy);
            assert!(error <= tolerance, "error={error}: {track:?}");
            assert!(track.forward_backward_error.unwrap() <= 0.2);
            assert_eq!(track.forward_levels.len(), PYRAMID_LEVELS);
            assert_eq!(track.backward_levels.len(), PYRAMID_LEVELS);
        }
        assert!(report.bilinear_samples <= MAX_BILINEAR_SAMPLES);
        assert!(report.iterations <= MAX_TOTAL_ITERATIONS);
    }

    #[test]
    fn analytic_subpixel_translation_is_measured() {
        assert_translation(0.4, -0.35, 0.0, 0.15);
    }

    #[test]
    fn pyramid_resolves_several_pixel_translation() {
        assert_translation(6.25, -4.5, 0.0, 0.15);
    }

    #[test]
    fn patch_normalization_tolerates_small_brightness_offset() {
        assert_translation(1.3, -0.6, 8.0, 0.15);
    }

    #[test]
    fn blank_and_aperture_images_are_rejected() {
        let blank = vec![100_u8; WIDTH * HEIGHT];
        let edge: Vec<_> = (0..WIDTH * HEIGHT)
            .map(|index| if index % WIDTH < 320 { 30 } else { 220 })
            .collect();
        for pixels in [&blank, &edge] {
            let report = track_points(frame(pixels), frame(pixels), &[[320.0, 240.0]]).unwrap();
            let track = &report.points[0];
            assert!(!track.accepted);
            assert_eq!(track.rejection, Some(TrackingRejection::LowTexture));
            assert!(track.forward_endpoint.is_none());
            assert!(track.backward_levels.is_empty());
        }
    }

    #[test]
    fn exhausted_iterations_reject_without_final_residual_or_fallback() {
        let previous = image(0.0, 0.0, 0.0);
        // A ramp cannot reproduce the curved template. Its normalized patch
        // changes little under translation, so iteration does not solve it.
        let ramp: Vec<_> = (0..WIDTH * HEIGHT)
            .map(|index| ((index % WIDTH) / 3) as u8)
            .collect();
        let previous = pyramid(frame(&previous));
        let current = pyramid(frame(&ramp));
        let trace = fit_level(&previous[0], &current[0], [100.0, 100.0], [100.0, 100.0], 0);
        assert_eq!(trace.rejection, Some(TrackingRejection::NonConvergence));
        assert_eq!(usize::from(trace.iterations), MAX_ITERATIONS);
        assert_eq!(trace.bilinear_samples, PATCH_PIXELS * (5 + MAX_ITERATIONS));
        assert!(!trace.converged);
        assert!(trace.photometric_rms.is_none());
    }

    #[test]
    fn occlusion_and_masked_support_are_not_measurements() {
        let previous = image(0.0, 0.0, 0.0);
        let mut current = previous.clone();
        for y in 70..131 {
            for x in 70..131 {
                current[y * WIDTH + x] = 0;
            }
        }
        let report = track_points(frame(&previous), frame(&current), &[[100.0, 100.0]]).unwrap();
        assert!(!report.points[0].accepted);
        let mut mask = vec![1_u8; WIDTH * HEIGHT];
        mask[100 * WIDTH + 100] = 0;
        let report = track_points(
            frame(&previous).with_mask(&mask).unwrap(),
            frame(&previous),
            &[[100.0, 100.0]],
        )
        .unwrap();
        assert_eq!(report.points[0].rejection, Some(TrackingRejection::Masked));
        assert!(!report.points[0].accepted);
    }

    #[test]
    fn bounds_fail_before_processing_and_border_has_no_fallback() {
        let pixels = image(0.0, 0.0, 0.0);
        assert!(matches!(
            GrayFrame::new(usize::MAX, HEIGHT, &pixels),
            Err(TrackingInputError::InvalidDimensions)
        ));
        assert!(matches!(
            GrayFrame::new(WIDTH, HEIGHT, &pixels[..10]),
            Err(TrackingInputError::InvalidPixelCount)
        ));
        assert!(matches!(
            frame(&pixels).with_mask(&[1; 5]),
            Err(TrackingInputError::InvalidMask)
        ));
        let mut mask = vec![1_u8; WIDTH * HEIGHT];
        mask[0] = 2;
        assert!(matches!(
            frame(&pixels).with_mask(&mask),
            Err(TrackingInputError::InvalidMask)
        ));
        for invalid in [
            [f64::NAN, 100.0],
            [100.0, f64::INFINITY],
            [-1.0, 50.0],
            [640.0, 100.0],
        ] {
            assert_eq!(
                track_points(frame(&pixels), frame(&pixels), &[invalid]),
                Err(TrackingInputError::InvalidPoint)
            );
        }
        assert_eq!(
            track_points(
                frame(&pixels),
                frame(&pixels),
                &vec![[100.0, 100.0]; MAX_POINTS + 1]
            ),
            Err(TrackingInputError::TooManyPoints)
        );
        let report = track_points(frame(&pixels), frame(&pixels), &[[2.0, 2.0]]).unwrap();
        assert_eq!(report.points[0].rejection, Some(TrackingRejection::Border));
        assert_eq!(report.points[0].forward_levels.len(), 1);
        assert!(report.points[0].forward_endpoint.is_none());
        let empty = track_points(frame(&pixels), frame(&pixels), &[]).unwrap();
        assert_eq!(empty.bilinear_samples, 0);
        assert_eq!(empty.pyramid_output_pixels, 0);
    }

    #[test]
    fn repeated_calls_are_deterministic_and_work_is_bounded() {
        let previous = image(0.0, 0.0, 0.0);
        let current = image(0.4, -0.35, 0.0);
        let points = vec![[100.0, 100.0]; MAX_POINTS];
        let first = track_points(frame(&previous), frame(&current), &points).unwrap();
        let second = track_points(frame(&previous), frame(&current), &points).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.points.len(), MAX_POINTS);
        assert!(first.points.iter().all(|point| point.accepted));
        assert!(first.bilinear_samples <= MAX_BILINEAR_SAMPLES);
        assert!(first.iterations <= MAX_TOTAL_ITERATIONS);
        assert_eq!(first.pyramid_output_pixels, 192_000);
    }
}
