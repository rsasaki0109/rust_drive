//! Standalone synthetic diagnostics for the frozen tracking helper.
//! Build/run: rustc --edition 2024 -O scripts/exercise-pyramidal-tracking.rs
//! -o /tmp/exercise-pyramidal-tracking && /tmp/exercise-pyramidal-tracking
//! No recordings, feature fitting, dependencies, or helper modifications.
#[allow(dead_code)]
#[path = "../integrations/rgbd/src/pyramidal_tracking.rs"]
mod tracker;

use tracker::{GrayFrame, HEIGHT, TrackingReport, WIDTH, track_points};

fn texture(x: f64, y: f64) -> f64 {
    128.0
        + 40.0 * (0.20 * x + 0.10 * y).sin()
        + 35.0 * (0.13 * x - 0.24 * y).cos()
        + 30.0 * (0.39 * x + 0.31 * y).sin()
}

fn render(function: impl Fn(f64, f64) -> f64) -> Vec<u8> {
    (0..WIDTH * HEIGHT)
        .map(|index| {
            function((index % WIDTH) as f64, (index / WIDTH) as f64)
                .round()
                .clamp(0.0, 255.0) as u8
        })
        .collect()
}

fn grid() -> Vec<[f64; 2]> {
    let mut points = Vec::new();
    for y in [70.0, 130.0, 190.0, 250.0, 310.0, 370.0, 430.0] {
        for x in [70.0, 130.0, 190.0, 250.0, 310.0, 370.0, 430.0, 490.0, 550.0] {
            points.push([x, y]);
        }
    }
    points
}

fn fit(previous: &[u8], current: &[u8], points: &[[f64; 2]]) -> TrackingReport {
    track_points(
        GrayFrame::new(WIDTH, HEIGHT, previous).unwrap(),
        GrayFrame::new(WIDTH, HEIGHT, current).unwrap(),
        points,
    )
    .unwrap()
}

fn optional_number(value: Option<f64>) -> String {
    value.map_or_else(|| "null".to_owned(), |value| format!("{value:.12}"))
}

fn optional_point(point: Option<[f64; 2]>) -> String {
    point.map_or_else(|| "null".to_owned(), |[x, y]| format!("[{x:.12},{y:.12}]"))
}

fn level_json(level: &tracker::LevelTrace) -> String {
    let rejection = level
        .rejection
        .map_or_else(|| "null".to_owned(), |value| format!("\"{value:?}\""));
    format!(
        "{{\"level\":{},\"start\":{},\"endpoint\":{},\"iterations\":{},\"bilinear_samples\":{},\"min_eigenvalue\":{},\"condition_number\":{},\"photometric_rms\":{},\"converged\":{},\"rejection\":{rejection}}}",
        level.level,
        optional_point(Some(level.start)),
        optional_point(Some(level.endpoint)),
        level.iterations,
        level.bilinear_samples,
        level.min_eigenvalue,
        level.condition_number,
        optional_number(level.photometric_rms),
        level.converged,
    )
}

fn levels_json(levels: &[tracker::LevelTrace]) -> String {
    format!(
        "[{}]",
        levels.iter().map(level_json).collect::<Vec<_>>().join(",")
    )
}

fn emit(
    name: &str,
    report: &TrackingReport,
    expected: Option<&dyn Fn([f64; 2]) -> [f64; 2]>,
    last: bool,
) {
    let accepted = report.points.iter().filter(|track| track.accepted).count();
    let mut errors = Vec::new();
    let mut rejections = std::collections::BTreeMap::new();
    for track in &report.points {
        if let Some(rejection) = track.rejection {
            *rejections.entry(format!("{rejection:?}")).or_insert(0) += 1;
        }
        if track.accepted
            && let (Some(endpoint), Some(expected)) = (track.forward_endpoint, expected)
        {
            let truth = expected(track.start);
            errors.push((endpoint[0] - truth[0]).hypot(endpoint[1] - truth[1]));
        }
    }
    let error_mean = (!errors.is_empty()).then(|| errors.iter().sum::<f64>() / errors.len() as f64);
    let error_max = errors.into_iter().reduce(f64::max);
    let rejection_json: Vec<_> = rejections
        .iter()
        .map(|(name, count)| format!("\"{name}\":{count}"))
        .collect();
    println!(
        "{{\"case\":\"{name}\",\"input_points\":{},\"accepted\":{accepted},\"accepted_error_mean\":{},\"accepted_error_max\":{},\"rejections\":{{{}}},\"iterations\":{},\"bilinear_samples\":{},\"points\":[",
        report.points.len(),
        optional_number(error_mean),
        optional_number(error_max),
        rejection_json.join(","),
        report.iterations,
        report.bilinear_samples,
    );
    for (index, track) in report.points.iter().enumerate() {
        let truth = expected.map(|expected| expected(track.start));
        let error = track
            .forward_endpoint
            .zip(truth)
            .map(|(endpoint, truth)| (endpoint[0] - truth[0]).hypot(endpoint[1] - truth[1]));
        let rejection = track
            .rejection
            .map_or_else(|| "null".to_owned(), |value| format!("\"{value:?}\""));
        println!(
            "{{\"point_index\":{},\"start\":[{},{}],\"accepted\":{},\"rejection\":{rejection},\"forward\":{},\"backward\":{},\"forward_backward_error\":{},\"expected\":{},\"forward_error\":{},\"forward_levels\":{},\"backward_levels\":{}}}{}",
            track.point_index,
            track.start[0],
            track.start[1],
            track.accepted,
            optional_point(track.forward_endpoint),
            optional_point(track.backward_endpoint),
            optional_number(track.forward_backward_error),
            optional_point(truth),
            optional_number(error),
            levels_json(&track.forward_levels),
            levels_json(&track.backward_levels),
            if index + 1 == report.points.len() {
                ""
            } else {
                ","
            },
        );
    }
    println!("]}}{}", if last { "" } else { "," });
}

fn main() {
    let points = grid();
    let previous = render(texture);
    println!(
        "{{\"schema_version\":1,\"evaluation_kind\":\"synthetic_diagnostic\",\"helper_sha256\":\"2118ecdd95e7788dc1937a0937209dab4f4f9f7d9b5fc0c28aaa3f2134b3ce8e\",\"cases\":["
    );
    for (gain, angle, dx, dy, name) in [
        (1.0, 0.0, 0.4, -0.35, "translation_control"),
        (1.15, 0.0, 0.4, -0.35, "exposure_gain_1_15"),
        (1.35, 0.0, 0.4, -0.35, "exposure_gain_1_35"),
        (1.8, 0.0, 0.4, -0.35, "exposure_gain_1_8"),
        (1.0, 0.02_f64, 0.0, 0.0, "rotation_0_02_rad"),
        (1.0, 0.10, 0.0, 0.0, "rotation_0_10_rad"),
        (1.0, 0.30, 0.0, 0.0, "rotation_0_30_rad"),
    ] {
        let (sin, cos) = angle.sin_cos();
        let current = render(|x, y| {
            let x = x - 320.0 - dx;
            let y = y - 240.0 - dy;
            128.0 + gain * (texture(320.0 + cos * x + sin * y, 240.0 - sin * x + cos * y) - 128.0)
        });
        let expected = |[x, y]: [f64; 2]| {
            let x = x - 320.0;
            let y = y - 240.0;
            [
                320.0 + cos * x - sin * y + dx,
                240.0 + sin * x + cos * y + dy,
            ]
        };
        let report = fit(&previous, &current, &points);
        if name == "translation_control" {
            assert!(report.points.iter().all(|track| {
                let endpoint = track.forward_endpoint.unwrap();
                let truth = expected(track.start);
                track.accepted && (endpoint[0] - truth[0]).hypot(endpoint[1] - truth[1]) < 0.25
            }));
        }
        if name == "rotation_0_10_rad" {
            assert!(report.points.iter().any(|track| {
                track.rejection == Some(tracker::TrackingRejection::ForwardBackward)
                    && track
                        .forward_backward_error
                        .is_some_and(|error| error > 1.0)
                    && track.forward_levels.len() == 3
                    && track.backward_levels.len() == 3
                    && track
                        .forward_levels
                        .iter()
                        .chain(&track.backward_levels)
                        .all(|level| level.converged && level.rejection.is_none())
            }));
        }
        emit(name, &report, Some(&expected), false);
    }
    // An exact two-period translation is invisible to this repeated pattern.
    // These physically distinct endpoints must remain in the diagnostic even
    // when bidirectional and photometric checks both accept zero motion.
    let periodic = |x: f64, y: f64| {
        let frequency = std::f64::consts::TAU / 16.0;
        128.0 + 45.0 * (frequency * x).sin() + 45.0 * (frequency * y).cos()
    };
    let repeated_previous = render(periodic);
    let repeated_current = render(|x, y| periodic(x - 32.0, y));
    let expected = |[x, y]: [f64; 2]| [x + 32.0, y];
    let repeated = fit(&repeated_previous, &repeated_current, &points);
    assert!(repeated.points.iter().all(|track| {
        let endpoint = track.forward_endpoint.unwrap();
        let truth = expected(track.start);
        track.accepted && (endpoint[0] - truth[0]).hypot(endpoint[1] - truth[1]) > 31.99
    }));
    emit(
        "repeated_texture_exact_32_pixel_translation",
        &repeated,
        Some(&expected),
        false,
    );
    // These nonrigid warps are synthetic stress controls, not calibrated motion
    // cases. Their purpose is to report the actual gate distribution, including
    // specific forward/backward failures, without inventing a ground-truth pose.
    for amplitude in [3.0, 6.0, 9.0, 12.0] {
        let current = render(|x, y| {
            texture(
                x - amplitude * (y / 31.0).sin(),
                y - amplitude * (x / 27.0).sin(),
            )
        });
        emit(
            &format!("nonrigid_warp_amplitude_{amplitude}"),
            &fit(&previous, &current, &points),
            None,
            amplitude == 12.0,
        );
    }
    println!("]}}");
}
