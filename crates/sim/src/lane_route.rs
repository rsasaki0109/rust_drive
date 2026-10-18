//! CLI bridge from a validated lane itinerary to an existing resolved-route simulation.
use rustdriving_core::VehicleConfig;
use rustdriving_pipeline::lane_change::{
    LaneChangeConfig, LaneChangeResumeConfig, LaneChangeReturnConfig,
};
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
    let mut return_on_conflict = false;
    let mut return_distance = None;
    let mut resume_after_return = false;
    let mut resume_distance = None;
    let mut local_speed_limits = false;
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--local-speed-limits" {
            local_speed_limits = true;
            i += 1;
            continue;
        }
        if args[i] == "--resume-after-return" {
            resume_after_return = true;
            i += 1;
            continue;
        }
        if args[i] == "--wait-for-gap" {
            wait_for_gap = true;
            i += 1;
            continue;
        }
        if args[i] == "--return-on-conflict" {
            return_on_conflict = true;
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
            "--return-distance" => return_distance = Some(value.parse::<f64>()?),
            "--resume-distance" => resume_distance = Some(value.parse::<f64>()?),
            _ => return Err(format!("invalid lane-route option {}", args[i]).into()),
        }
        i += 2;
    }
    if scenario_output.is_none() && (cruise.is_some() || duration.is_some()) {
        return Err("--cruise-speed and --duration require --scenario-output".into());
    }
    if return_on_conflict && !wait_for_gap {
        return Err("--return-on-conflict requires --wait-for-gap".into());
    }
    if return_distance.is_some() && !return_on_conflict {
        return Err("--return-distance requires --return-on-conflict".into());
    }
    if resume_after_return && !return_on_conflict {
        return Err("--resume-after-return requires --return-on-conflict".into());
    }
    if resume_distance.is_some() && !resume_after_return {
        return Err("--resume-distance requires --resume-after-return".into());
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
    if local_speed_limits && change_request.is_some() {
        return Err(
            "--local-speed-limits currently requires a following route without lane changes".into(),
        );
    }
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
    let (plan, maneuver, speed_zones) = if let Some(request) = change_request {
        let change =
            network.lane_change_route(&start, &goal, &closed, request, VehicleConfig::default())?;
        (change.plan, Some(change.maneuver), vec![])
    } else if local_speed_limits {
        let compiled = network.following_route_with_speed_zones(&start, &goal, &closed)?;
        (compiled.plan, None, compiled.speed_zones)
    } else {
        (
            network.following_route(&start, &goal, &closed)?,
            None,
            vec![],
        )
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
        let return_config = if return_on_conflict {
            let source = network.lane(&maneuver.from_lane).unwrap();
            let reverse = match maneuver.request.side {
                LaneSide::Left => LaneSide::Right,
                LaneSide::Right => LaneSide::Left,
            };
            if network
                .lane_change_target(&target.id, reverse, &closed)?
                .is_none_or(|lane| lane.id != source.id)
            {
                return Err("map does not permit the return crossing".into());
            }
            Some(LaneChangeReturnConfig {
                source_start: source.centerline[0],
                source_end: *source.centerline.last().unwrap(),
                source_width_m: source.width_m,
                max_return_distance_m: return_distance.unwrap_or(30.0),
                resume_after_return: resume_after_return.then_some(LaneChangeResumeConfig {
                    max_change_distance_m: resume_distance.unwrap_or(30.0),
                }),
            })
        } else {
            None
        };
        let config = LaneChangeConfig {
            entry_s_m: maneuver.request.start_m,
            exit_s_m: plan.route.project(finish).0,
            target_start: start,
            target_end: end,
            target_width_m: target.width_m,
            return_on_conflict: return_config,
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
        let mut value = serde_json::json!({
            "name":"Authored lane route", "duration":duration.unwrap_or(120.0),
            "road_length":plan.distance_m, "half_width":plan.route.half_width,
            "local_route_geometry":true,
            "lane_change":gap_config,
            "expected":"goal", "cruise_speed":if local_speed_limits { requested_cruise } else { requested_cruise.min(plan.speed_limit_m_s) },
            "motion_limits":{"max_acceleration_m_s2":1.0,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.0},
            "navigation":{"network":plan.network,"start":plan.start_node,"goal":plan.goal_node,"closed_edges":[]},
            "objects":[]
        });
        if local_speed_limits {
            value["speed_zones"] = serde_json::to_value(&speed_zones)?;
        }
        let scenario: Scenario = serde_json::from_value(value)?;
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
    if local_speed_limits {
        report["speed_zones"] = serde_json::to_value(speed_zones)?;
    }
    if let Some(config) = gap_config {
        report["lane_change_gate"] = serde_json::to_value(config)?;
    }
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(true)
}
