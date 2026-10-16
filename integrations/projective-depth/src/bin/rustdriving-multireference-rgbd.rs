use rustdriving_projective_depth::{
    Camera,
    multireference::{ImageObservation, Mode, MultiReferenceTracker},
};
use serde::Deserialize;
use std::io::{self, BufRead, Read, Write};
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Image {
    timestamp: f64,
    gray: Vec<u8>,
}
#[derive(Deserialize)]
#[serde(tag = "kind", deny_unknown_fields)]
enum Request {
    Configure {
        camera: Camera,
        mode: Mode,
    },
    Frame {
        timestamp: f64,
        depth_m: Vec<f64>,
        image: Option<Image>,
    },
}
fn run() -> Result<(), String> {
    let stdin = io::stdin();
    let mut input = stdin.lock();
    let stdout = io::stdout();
    let mut output = stdout.lock();
    let mut tracker = None;
    let mut count = 0;
    loop {
        let mut line = Vec::new();
        let bytes = input
            .by_ref()
            .take(8 * 1024 * 1024 + 1)
            .read_until(b'\n', &mut line)
            .map_err(|e| e.to_string())?;
        if bytes == 0 {
            break;
        }
        if line.len() > 8 * 1024 * 1024 || !line.ends_with(b"\n") {
            return Err("bounded JSON line required".into());
        }
        match serde_json::from_slice::<Request>(&line).map_err(|e| e.to_string())? {
            Request::Configure { camera, mode } => {
                if tracker.is_some() {
                    return Err("one configuration".into());
                }
                tracker = Some(MultiReferenceTracker::new(camera, mode)?);
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
                    return Err("at most 200 frames".into());
                }
                let tracker = tracker.as_mut().ok_or("configure before frames")?;
                let image = image.as_ref().map(|v| ImageObservation {
                    timestamp: v.timestamp,
                    gray: &v.gray,
                });
                serde_json::to_writer(&mut output, &tracker.step(timestamp, &depth_m, image))
                    .map_err(|e| e.to_string())?;
                writeln!(output).map_err(|e| e.to_string())?;
            }
        }
        output.flush().map_err(|e| e.to_string())?;
    }
    if tracker.is_none() || count == 0 {
        return Err("configuration and frames required".into());
    }
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
