//! Bounded tick/depth evaluation transport for explicit-clock supervision.
use rustdriving_projective_depth::{
    Camera,
    supervision::{Observation, Supervisor},
};
use serde::Deserialize;
use std::io::{self, BufRead, Read, Write};

#[derive(Deserialize)]
#[serde(tag = "kind", deny_unknown_fields)]
enum Request {
    Configure { camera: Camera },
    Tick { now: f64, event: Event },
}
#[derive(Deserialize)]
#[serde(tag = "kind", deny_unknown_fields)]
enum Event {
    Idle {},
    Failed {},
    Depth { timestamp: f64, depth_m: Vec<f64> },
}
fn run() -> Result<(), String> {
    let input = io::stdin();
    let mut input = input.lock();
    let output = io::stdout();
    let mut output = output.lock();
    let mut supervisor = None;
    let mut ticks = 0;
    let mut frames = 0;
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
                if supervisor.is_some() {
                    return Err("one configuration per process".into());
                }
                supervisor = Some(Supervisor::new(camera)?);
                writeln!(output, "{{\"configured\":true,\"schema_version\":1}}")
                    .map_err(|e| e.to_string())?;
            }
            Request::Tick { now, event } => {
                ticks += 1;
                frames += usize::from(matches!(event, Event::Depth { .. }));
                if ticks > 1000 || frames > 200 {
                    return Err("at most 1000 ticks and 200 depth frames".into());
                }
                let event = match &event {
                    Event::Idle {} => Observation::Idle,
                    Event::Failed {} => Observation::Failed,
                    Event::Depth { timestamp, depth_m } => Observation::Depth {
                        timestamp: *timestamp,
                        depth_m,
                    },
                };
                let result = supervisor
                    .as_mut()
                    .ok_or("configure before ticks")?
                    .tick(now, event)?;
                serde_json::to_writer(&mut output, &result).map_err(|e| e.to_string())?;
                writeln!(output).map_err(|e| e.to_string())?;
            }
        }
        output.flush().map_err(|e| e.to_string())?;
    }
    if supervisor.is_none() || ticks == 0 {
        return Err("configuration and at least one tick required".into());
    }
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
