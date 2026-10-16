//! Bounded JSON-lines transport for immutable measured-depth-map localization.
use rustdriving_projective_depth::{Camera, fixed_map::FixedMapTracker};
use serde::Deserialize;
use std::io::{self, BufRead, Read, Write};

#[derive(Deserialize)]
#[serde(tag = "kind", deny_unknown_fields)]
enum Request {
    Configure { camera: Camera },
    Frame { timestamp: f64, depth_m: Vec<f64> },
}

fn run() -> Result<(), String> {
    let mut input = io::stdin().lock();
    let mut output = io::stdout().lock();
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
        match serde_json::from_slice::<Request>(&body).map_err(|e| e.to_string())? {
            Request::Configure { camera } => {
                if tracker.is_some() {
                    return Err("one configuration per process".into());
                }
                tracker = Some(FixedMapTracker::new(camera)?);
                writeln!(output, "{{\"configured\":true,\"schema_version\":1}}")
                    .map_err(|e| e.to_string())?;
            }
            Request::Frame { timestamp, depth_m } => {
                count += 1;
                if count > 200 {
                    return Err("at most200 frames per process".into());
                }
                let state = tracker.as_mut().ok_or("configure camera before frames")?;
                serde_json::to_writer(&mut output, &state.step(timestamp, &depth_m))
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
