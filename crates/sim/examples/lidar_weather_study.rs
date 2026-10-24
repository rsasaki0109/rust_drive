//! Frozen-scene comparison; stdout is the complete deterministic raw report.
use rustdriving_core::{LidarScan, Perception, Pose, Vec2};
use rustdriving_perception::LidarClusters;
use rustdriving_sim::{WorldObject, lidar_weather::{WeatherConfig, WeatherScan, apply, reference_scan}};
use serde::Serialize;

#[derive(Serialize)]
struct Condition { name: &'static str, config: WeatherConfig }
#[derive(Serialize)]
struct Case {
    condition: &'static str,
    weather: WeatherScan,
    detections: Vec<rustdriving_core::Detection>,
}
#[derive(Serialize)]
struct Trial {
    fixture: &'static str,
    seed: u64,
    weather_seed: u64,
    objects: Vec<WorldObject>,
    baseline: LidarScan,
    cases: Vec<Case>,
}
#[derive(Serialize)]
struct Report {
    schema_version: u32,
    scope: &'static str,
    conditions: Vec<Condition>,
    trials: Vec<Trial>,
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    if std::env::args().len() != 1 { return Err("this deterministic study accepts no arguments; redirect stdout to save its report".into()); }
    let conditions = vec![
        Condition { name: "clear", config: WeatherConfig::CLEAR },
        Condition { name: "rain_light", config: WeatherConfig { extinction_m_inv: 0.002, backscatter_rate_m_inv: 0.0004, range_jitter_m: 0.025 } },
        Condition { name: "rain_heavy", config: WeatherConfig { extinction_m_inv: 0.018, backscatter_rate_m_inv: 0.0025, range_jitter_m: 0.12 } },
        Condition { name: "snow_light", config: WeatherConfig { extinction_m_inv: 0.004, backscatter_rate_m_inv: 0.002, range_jitter_m: 0.06 } },
        Condition { name: "snow_heavy", config: WeatherConfig { extinction_m_inv: 0.01, backscatter_rate_m_inv: 0.015, range_jitter_m: 0.3 } },
    ];
    let pose = Pose { position: Vec2::default(), yaw: 0.0 };
    let mut trials = Vec::new();
    for fixture in ["targets", "empty"] {
        let objects = if fixture == "empty" { vec![] } else { vec![
            WorldObject { id: 1, position: Vec2::new(8.0, 0.0), radius: 0.8 },
            WorldObject { id: 2, position: Vec2::new(22.0, 6.0), radius: 1.1 },
            WorldObject { id: 3, position: Vec2::new(38.0, -8.0), radius: 1.5 },
        ] };
        for seed in 1..=16 {
            let baseline = reference_scan(pose, &objects, 1.0, seed)?;
            let weather_seed = seed ^ 0xa341316c9e3779b9;
            let mut cases = Vec::new();
            for condition in &conditions {
                let weather = apply(&baseline, condition.config, weather_seed)?;
                let detections = LidarClusters.detect(&weather.scan, pose);
                cases.push(Case { condition: condition.name, weather, detections });
            }
            trials.push(Trial { fixture, seed, weather_seed, objects: objects.clone(), baseline, cases });
        }
    }
    let report = Report {
        schema_version: 1,
        scope: "Frozen planar first-return scenes, synthetic uncalibrated weather coefficients and unchanged LidarClusters; no driving or real-weather qualification",
        conditions,
        trials,
    };
    serde_json::to_writer(std::io::stdout().lock(), &report)?;
    Ok(())
}
