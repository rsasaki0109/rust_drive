//! CLI bridge from a validated lane itinerary to an existing resolved-route simulation.
use rustdriving_core::VehicleConfig;
use rustdriving_pipeline::lane_change::LaneChangeConfig;
use rustdriving_routing::lanes::{LaneChangeRequest, LaneMapSpec, LaneNetwork, LaneSide};
use rustdriving_sim::Scenario;
use std::{
    error::Error,
    fs,
    io::{Read, Write},
    path::PathBuf,
};

pub(crate) fn run(args: &[String]) -> Result<bool, Box<dyn Error>> {
    let mut input = None;
    let mut start = None;
    let mut goal = None;
    let mut closed = Vec::new();
    let mut scenario_output = None;
    let mut cruise = None;
    let mut duration = None;
    let mut change_side = None;
    let mut change_start = None;
    let mut change_length = None;
    let mut wait_for_gap = false;
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--wait-for-gap" {
            wait_for_gap = true;
            i += 1;
            continue;
        }
        let value = args
            .get(i + 1)
            .ok_or("lane-route option requires a value")?;
        match args[i].as_str() {
            "--map" => input = Some(PathBuf::from(value)),
            "--start" => start = Some(value.clone()),
            "--goal" => goal = Some(value.clone()),
            "--closed-lane" => closed.push(value.clone()),
            "--scenario-output" => scenario_output = Some(PathBuf::from(value)),
            "--cruise-speed" => cruise = Some(value.parse::<f64>()?),
            "--duration" => duration = Some(value.parse::<f64>()?),
            "--change-side" => {
                change_side = Some(match value.as_str() {
                    "left" => LaneSide::Left,
                    "right" => LaneSide::Right,
                    _ => return Err("--change-side must be left or right".into()),
                })
            }
            "--change-start" => change_start = Some(value.parse::<f64>()?),
            "--change-length" => change_length = Some(value.parse::<f64>()?),
            _ => return Err(format!("invalid lane-route option {}", args[i]).into()),
        }
        i += 2;
    }
    if scenario_output.is_none() && (cruise.is_some() || duration.is_some()) {
        return Err("--cruise-speed and --duration require --scenario-output".into());
    }
    let change_request = match (change_side, change_start, change_length) {
        (None, None, None) => None,
        (Some(side), Some(start_m), Some(length_m)) => Some(LaneChangeRequest {
            side,
            start_m,
            length_m,
        }),
        _ => {
            return Err(
                "--change-side, --change-start and --change-length must be supplied together"
                    .into(),
            );
        }
    };
    if duration.is_some_and(|seconds| !seconds.is_finite() || !(1.0..=300.0).contains(&seconds)) {
        return Err("scenario duration must be finite and in 1..300 seconds".into());
    }
    let input = input.ok_or("--map is required")?;
    let mut bytes = Vec::new();
    fs::File::open(&input)?
        .take(16 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 16 * 1024 * 1024 {
        return Err("lane map input exceeds 16 MiB".into());
    }
    let network = LaneNetwork::new(serde_json::from_slice::<LaneMapSpec>(&bytes)?)?;
    let start = start.ok_or("--start lane is required")?;
    let goal = goal.ok_or("--goal lane is required")?;
    let (plan, maneuver) = if let Some(request) = change_request {
        let change =
            network.lane_change_route(&start, &goal, &closed, request, VehicleConfig::default())?;
        (change.plan, Some(change.maneuver))
    } else {
        (network.following_route(&start, &goal, &closed)?, None)
    };
    let gap_config = if wait_for_gap {
        let maneuver = maneuver
            .as_ref()
            .ok_or("--wait-for-gap requires a lane change")?;
        let target = network.lane(&maneuver.to_lane).unwrap();
        let start = target.centerline[0];
        let end = *target.centerline.last().unwrap();
        let finish =
            start.plus(end.minus(start).scaled(
                (maneuver.request.start_m + maneuver.request.length_m) / start.distance(end),
            ));
        let config = LaneChangeConfig {
            entry_s_m: maneuver.request.start_m,
            exit_s_m: plan.route.project(finish).0,
            target_start: start,
            target_end: end,
            target_width_m: target.width_m,
        };
        config.validate(&plan.route, VehicleConfig::default())?;
        Some(config)
    } else {
        None
    };
    let scenario = if scenario_output.is_some() {
        let requested_cruise = cruise.unwrap_or(2.0);
        if !requested_cruise.is_finite() || !(0.1..=8.0).contains(&requested_cruise) {
            return Err("requested cruise speed must be finite and in 0.1..8 m/s".into());
        }
        let scenario: Scenario = serde_json::from_value(serde_json::json!({
            "name":"Authored lane route", "duration":duration.unwrap_or(120.0),
            "road_length":plan.distance_m, "half_width":plan.route.half_width,
            "local_route_geometry":true,
            "lane_change":gap_config,
            "expected":"goal", "cruise_speed":requested_cruise.min(plan.speed_limit_m_s),
            "motion_limits":{"max_acceleration_m_s2":1.0,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.0},
            "navigation":{"network":plan.network,"start":plan.start_node,"goal":plan.goal_node,"closed_edges":[]},
            "objects":[]
        }))?;
        scenario.validate()?;
        Some(scenario)
    } else {
        None
    };
    // All map, route and scenario validation precedes output creation. Never clobber an input or link.
    if let (Some(path), Some(scenario)) = (scenario_output, scenario) {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
        let bytes = serde_json::to_vec_pretty(&scenario)?;
        fs::File::create_new(path)?.write_all(&bytes)?;
    }
    let mut report = serde_json::json!({
        "schema_version":1, "start_lane":start, "goal_lane":goal, "closed_lanes":closed,
        "plan":plan
    });
    if let Some(maneuver) = maneuver {
        report["lane_change"] = serde_json::to_value(maneuver)?;
    }
    if let Some(config) = gap_config {
        report["lane_change_gate"] = serde_json::to_value(config)?;
    }
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(true)
}
