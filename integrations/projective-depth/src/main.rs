//! Bounded synchronous JSON-lines evaluation transport; no external services.
use rustdriving_projective_depth::{
    Camera, Correspondence, FusionSampling, ModelGeometry, MotionModel, NormalEstimation,
    Reprojection, Residual, StepPolicy, Timing, Tracker, TrackerConfig, Visibility,
};
use rustdriving_projective_depth::{ImageObservation, photometric::Photometric};
use serde::Deserialize;
use std::io::{self, BufRead, Read, Write};

#[derive(Deserialize)]
#[serde(tag = "kind", deny_unknown_fields)]
enum Request {
    Configure {
        camera: Camera,
        #[serde(default)]
        visibility: Visibility,
        #[serde(default)]
        fusion_sampling: FusionSampling,
        #[serde(default)]
        reprojection: Reprojection,
        #[serde(default)]
        correspondence: Correspondence,
        #[serde(default)]
        residual: Residual,
        #[serde(default)]
        normals: NormalEstimation,
        #[serde(default)]
        geometry: ModelGeometry,
        #[serde(default)]
        photometric: Photometric,
        #[serde(default)]
        step_control: StepPolicy,
        #[serde(default)]
        motion_model: MotionModel,
        #[serde(default)]
        timing: Timing,
    },
    Frame {
        timestamp: f64,
        depth_m: Vec<f64>,
        #[serde(default)]
        image: Option<Image>,
    },
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Image {
    timestamp: f64,
    gray: Vec<u8>,
}
fn run() -> Result<(), String> {
    let input = io::stdin();
    let mut input = input.lock();
    let output = io::stdout();
    let mut output = output.lock();
    let mut tracker = None;
    let mut count = 0;
    loop {
        let mut body = Vec::new();
        let bytes = input
            .by_ref()
            .take(8 * 1024 * 1024 + 1)
            .read_until(b'\n', &mut body)
            .map_err(|e| e.to_string())?;
        if bytes == 0 {
            break;
        }
        if body.len() > 8 * 1024 * 1024 || !body.ends_with(b"\n") {
            return Err("bounded newline-delimited request required".into());
        }
        let request: Request = serde_json::from_slice(&body).map_err(|e| e.to_string())?;
        match request {
            Request::Configure {
                camera,
                visibility,
                fusion_sampling,
                reprojection,
                correspondence,
                residual,
                normals,
                geometry,
                photometric,
                step_control,
                motion_model,
                timing,
            } => {
                if tracker.is_some() {
                    return Err("one configuration per process".into());
                }
                tracker = Some(Tracker::with_config(
                    camera,
                    TrackerConfig {
                        visibility,
                        fusion_sampling,
                        reprojection,
                        correspondence,
                        residual,
                        normals,
                        geometry,
                        photometric,
                        step_control,
                        motion_model,
                        timing,
                    },
                )?);
                writeln!(output, "{{\"configured\":true,\"schema_version\":1}}")
                    .map_err(|e| e.to_string())?;
            }
            Request::Frame {
                timestamp,
                depth_m,
                image,
            } => {
                count += 1;
                if count > 200 {
                    return Err("at most 200 frames per evaluation process".into());
                }
                let t = tracker.as_mut().ok_or("configure camera before frames")?;
                let image = image.as_ref().map(|v| ImageObservation {
                    timestamp: v.timestamp,
                    gray: &v.gray,
                });
                serde_json::to_writer(&mut output, &t.step_with_image(timestamp, &depth_m, image))
                    .map_err(|e| e.to_string())?;
                writeln!(output).map_err(|e| e.to_string())?;
            }
        }
        output.flush().map_err(|e| e.to_string())?;
    }
    if tracker.is_none() || count == 0 {
        return Err("configuration and at least one frame required".into());
    }
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
