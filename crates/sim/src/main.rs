mod lane_route;
mod osm_input;
mod osm_output;
mod replay_output;
mod road_route;
use rustdriving_routing::{
    RoadNetwork,
    osm::{self, ImportOptions},
};
use rustdriving_sim::{Scenario, run_output::RunOutput, simulate};
use std::{env, error::Error, fs, path::PathBuf, process};

fn import_osm(args: &[String]) -> Result<bool, Box<dyn Error>> {
    let mut input = None;
    let mut output = None;
    let mut origin_lat = None;
    let mut origin_lon = None;
    let mut half_width = 3.0;
    let mut scenario_output = None;
    let mut start = None;
    let mut goal = None;
    let mut closed = Vec::new();
    let mut cruise = None;
    let mut duration = None;
    let mut local_route_geometry = false;
    let mut seen = std::collections::BTreeSet::new();
    let mut i = 1;
    while i < args.len() {
        let option = args[i].as_str();
        if option != "--closed-edge" && !seen.insert(option) {
            return Err(format!("repeated import-osm option {option}").into());
        }
        if option == "--local-route-geometry" {
            local_route_geometry = true;
            i += 1;
            continue;
        }
        let value = args.get(i + 1).ok_or("option requires a value")?;
        match option {
            "--input" => input = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--origin-lat" => origin_lat = Some(value.parse::<f64>()?),
            "--origin-lon" => origin_lon = Some(value.parse::<f64>()?),
            "--default-half-width" => half_width = value.parse()?,
            "--scenario-output" => scenario_output = Some(PathBuf::from(value)),
            "--start" => start = Some(value.clone()),
            "--goal" => goal = Some(value.clone()),
            "--closed-edge" => closed.push(value.clone()),
            "--cruise-speed" => cruise = Some(value.parse::<f64>()?),
            "--duration" => duration = Some(value.parse::<f64>()?),
            _ => return Err(format!("invalid import-osm option {}", args[i]).into()),
        }
        i += 2;
    }
    if scenario_output.is_none()
        && (start.is_some()
            || goal.is_some()
            || !closed.is_empty()
            || local_route_geometry
            || cruise.is_some()
            || duration.is_some())
    {
        return Err("routing options require --scenario-output".into());
    }
    let document = osm_input::read_document(fs::File::open(input.ok_or("--input is required")?)?)?;
    let output = output.ok_or("--output map filename is required")?;
    let (network, report) = osm::import(
        document,
        ImportOptions {
            origin_lat: origin_lat.ok_or("--origin-lat is required")?,
            origin_lon: origin_lon.ok_or("--origin-lon is required")?,
            default_half_width: half_width,
        },
    )?;
    let scenario = if scenario_output.is_some() {
        let start = start.ok_or("--start is required with --scenario-output")?;
        let goal = goal.ok_or("--goal is required with --scenario-output")?;
        let plan = RoadNetwork::new(network.clone())?.shortest_route(&start, &goal, &closed)?;
        let mut scenario: Scenario = serde_json::from_value(serde_json::json!({
            "name":"OSM imported road route", "duration":duration.unwrap_or(180.0),
            "road_length":plan.distance_m, "half_width":plan.route.half_width,
            "expected":"goal", "cruise_speed":cruise.unwrap_or(2.0),
            "motion_limits":{"max_acceleration_m_s2":1.0,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.0},
            "navigation":{"network":network,"start":start,"goal":goal,"closed_edges":closed},
            "objects":[]
        }))?;
        scenario.local_route_geometry = local_route_geometry;
        scenario.validate()?;
        Some(scenario)
    } else {
        None
    };
    let report_json = serde_json::to_string_pretty(&report)?;
    let report_path = output.with_extension("import.json");
    let mut destinations = vec![output.as_path()];
    destinations.extend(scenario_output.as_deref());
    destinations.push(report_path.as_path());
    let staged = osm_output::ImportOutput::new(&destinations)?;
    staged.json(0, &network)?;
    if let Some(scenario) = scenario {
        staged.json(1, &scenario)?;
    }
    staged.json(destinations.len() - 1, &report)?;
    staged.publish()?;
    println!("{report_json}");
    Ok(true)
}
fn run() -> Result<bool, Box<dyn Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "--help" || args[0] == "-h" {
        println!(
            "RustDriving sensor-driven simulator\nUsage:\n  rustdriving run --scenario FILE [--seed N] [--output DIR]\n  rustdriving replay --log FILE [--output DIR]\n  rustdriving validate-scenario --scenario FILE\n  rustdriving import-osm --input FILE --output MAP.json --origin-lat N --origin-lon E [--default-half-width METERS] [--scenario-output FILE --start osm-node-ID --goal osm-node-ID --closed-edge ID --cruise-speed MPS --duration SECONDS --local-route-geometry]\n  rustdriving road-route --map FILE (--start NODE | --start-edge EDGE --start-s METERS) (--goal NODE | --goal-edge EDGE --goal-s METERS) [--closed-edge ID] [--scenario-output FILE --cruise-speed MPS --duration SECONDS]\n  rustdriving lane-route --map FILE --start LANE --goal LANE [--closed-lane ID] [--local-speed-limits] [--change-side left|right --change-start METERS --change-length METERS --wait-for-gap --return-on-conflict --return-distance METERS --resume-after-return --resume-distance METERS] [--scenario-output FILE --cruise-speed MPS --duration SECONDS]\nExit 0: acceptance/replay/validation/import/lane routing passed; 1: scenario acceptance failed; 2: invalid input, replay mismatch or I/O failure."
        );
        return Ok(true);
    }
    if args[0] == "import-osm" {
        return import_osm(&args);
    }
    if args[0] == "lane-route" {
        return lane_route::run(&args);
    }
    if args[0] == "road-route" {
        return road_route::run(&args);
    }
    if args[0] == "validate-scenario" {
        if args.len() != 3 || args[1] != "--scenario" {
            return Err("validate-scenario requires exactly --scenario FILE".into());
        }
        let scenario: Scenario = serde_json::from_str(&fs::read_to_string(&args[2])?)?;
        scenario.validate()?;
        println!(
            "{}",
            serde_json::json!({"valid": true, "scenario": scenario.name})
        );
        return Ok(true);
    }
    let mut scenario = None;
    let mut log = None;
    let mut seed = 7;
    let mut output = PathBuf::from("artifacts/demo");
    let mut i = 1;
    while i < args.len() {
        let value = args.get(i + 1).ok_or("option requires a value")?;
        match args[i].as_str() {
            "--scenario" if args[0] == "run" => scenario = Some(PathBuf::from(value)),
            "--seed" if args[0] == "run" => seed = value.parse()?,
            "--log" if args[0] == "replay" => log = Some(PathBuf::from(value)),
            "--output" => output = PathBuf::from(value),
            _ => return Err(format!("invalid option {} for {}", args[i], args[0]).into()),
        }
        i += 2;
    }
    match args[0].as_str() {
        "run" => {
            let publication = RunOutput::reference(
                &output,
                &scenario.as_deref().into_iter().collect::<Vec<_>>(),
            )?;
            let scenario: Scenario = serde_json::from_str(&fs::read_to_string(
                scenario.ok_or("--scenario is required")?,
            )?)?;
            let run = simulate(scenario, seed)?;
            let summary = publication.finish(&run, None::<&()>)?;
            println!("{summary}");
            Ok(run.summary.passed)
        }
        "replay" => {
            let json = replay_output::run(log.as_deref(), &output)?;
            println!("{json}");
            Ok(true)
        }
        _ => Err(
            "expected 'run', 'replay', 'validate-scenario', 'import-osm', 'road-route' or 'lane-route' subcommand"
                .into(),
        ),
    }
}
fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => process::exit(1),
        Err(e) => {
            eprintln!("rustdriving: {e}");
            process::exit(2);
        }
    }
}
