use rustdrive_pipeline::replay::verify;
use rustdrive_routing::{
    RoadNetwork,
    osm::{self, ImportOptions, OverpassDocument},
};
use rustdrive_sim::{Scenario, simulate};
use std::{
    env,
    error::Error,
    fs,
    io::{BufReader, BufWriter},
    path::PathBuf,
    process,
};
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
    let mut cruise = 2.0;
    let mut duration = 180.0;
    let mut local_route_geometry = false;
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--local-route-geometry" {
            local_route_geometry = true;
            i += 1;
            continue;
        }
        let value = args.get(i + 1).ok_or("option requires a value")?;
        match args[i].as_str() {
            "--input" => input = Some(PathBuf::from(value)),
            "--output" => output = Some(PathBuf::from(value)),
            "--origin-lat" => origin_lat = Some(value.parse::<f64>()?),
            "--origin-lon" => origin_lon = Some(value.parse::<f64>()?),
            "--default-half-width" => half_width = value.parse()?,
            "--scenario-output" => scenario_output = Some(PathBuf::from(value)),
            "--start" => start = Some(value.clone()),
            "--goal" => goal = Some(value.clone()),
            "--closed-edge" => closed.push(value.clone()),
            "--cruise-speed" => cruise = value.parse::<f64>()?,
            "--duration" => duration = value.parse::<f64>()?,
            _ => return Err(format!("invalid import-osm option {}", args[i]).into()),
        }
        i += 2;
    }
    let input = input.ok_or("--input is required")?;
    if fs::metadata(&input)?.len() > 16 * 1024 * 1024 {
        return Err("OSM input exceeds 16 MiB".into());
    }
    let document: OverpassDocument =
        serde_json::from_reader(BufReader::new(fs::File::open(input)?))?;
    let output = output.ok_or("--output map filename is required")?;
    if scenario_output
        .as_ref()
        .is_some_and(|path| path == &output || path == &output.with_extension("import.json"))
    {
        return Err("map, import report and scenario output paths must differ".into());
    }
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
            "name":"OSM imported road route", "duration":duration,
            "road_length":plan.distance_m, "half_width":plan.route.half_width,
            "expected":"goal", "cruise_speed":cruise,
            "motion_limits":{"max_acceleration_m_s2":1.0,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.0},
            "navigation":{"network":network,"start":start,"goal":goal,"closed_edges":closed},
            "objects":[]
        }))?;
        scenario.local_route_geometry = local_route_geometry;
        scenario.validate()?;
        Some(scenario)
    } else {
        if start.is_some() || goal.is_some() || !closed.is_empty() || local_route_geometry {
            return Err("routing options require --scenario-output".into());
        }
        None
    };
    for path in [&output].into_iter().chain(scenario_output.iter()) {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
    }
    fs::write(&output, serde_json::to_vec_pretty(&network)?)?;
    fs::write(
        output.with_extension("import.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    if let (Some(path), Some(scenario)) = (scenario_output, scenario) {
        fs::write(path, serde_json::to_vec_pretty(&scenario)?)?;
    }
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(true)
}
fn run() -> Result<bool, Box<dyn Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "--help" || args[0] == "-h" {
        println!(
            "RustDrive sensor-driven simulator\nUsage:\n  rustdrive run --scenario FILE [--seed N] [--output DIR]\n  rustdrive replay --log FILE [--output DIR]\n  rustdrive import-osm --input FILE --output MAP.json --origin-lat N --origin-lon E [--default-half-width METERS] [--scenario-output FILE --start osm-node-ID --goal osm-node-ID --closed-edge ID --cruise-speed MPS --duration SECONDS --local-route-geometry]\nExit 0: acceptance/replay/import passed; 1: scenario acceptance failed; 2: invalid input, replay mismatch or I/O failure."
        );
        return Ok(true);
    }
    if args[0] == "import-osm" {
        return import_osm(&args);
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
            let scenario: Scenario = serde_json::from_str(&fs::read_to_string(
                scenario.ok_or("--scenario is required")?,
            )?)?;
            let run = simulate(scenario, seed)?;
            fs::create_dir_all(&output)?;
            fs::write(output.join("run.json"), serde_json::to_vec(&run)?)?;
            run.sensor_log
                .as_ref()
                .ok_or("missing sensor log")?
                .write(BufWriter::new(fs::File::create(
                    output.join("sensors.jsonl"),
                )?))?;
            let summary = serde_json::to_string_pretty(&run.summary)?;
            fs::write(output.join("summary.json"), &summary)?;
            println!("{summary}");
            Ok(run.summary.passed)
        }
        "replay" => {
            fs::create_dir_all(&output)?;
            // Reports are written only on success; stale evidence cannot masquerade as this run.
            let summary = output.join("replay.json");
            if summary.exists() {
                fs::remove_file(&summary)?;
            }
            let reader = BufReader::new(fs::File::open(log.ok_or("--log is required")?)?);
            let report = verify(
                reader,
                BufWriter::new(fs::File::create(output.join("outputs.jsonl"))?),
            )?;
            let json = serde_json::to_string_pretty(&report)?;
            fs::write(summary, &json)?;
            println!("{json}");
            Ok(true)
        }
        _ => Err("expected 'run' or 'replay' subcommand".into()),
    }
}
fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => process::exit(1),
        Err(e) => {
            eprintln!("rustdrive: {e}");
            process::exit(2);
        }
    }
}
