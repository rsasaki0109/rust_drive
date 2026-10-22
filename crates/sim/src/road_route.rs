//! Compile an explicit directed map request into a driveable scenario.
use rustdriving_routing::{EdgePosition, RoadNetwork, RoadNetworkSpec, RouteEndpoint};
use rustdriving_sim::Scenario;
use std::{
    error::Error,
    fs,
    io::{Read, Write},
    path::PathBuf,
};

#[derive(Default)]
struct EndpointOptions {
    node: Option<String>,
    edge: Option<String>,
    distance: Option<f64>,
}
impl EndpointOptions {
    fn finish(self) -> Result<RouteEndpoint, &'static str> {
        match (self.node, self.edge, self.distance) {
            (Some(id), None, None) => Ok(RouteEndpoint::Node(id)),
            (None, Some(edge), Some(s_m)) => Ok(RouteEndpoint::Edge(EdgePosition { edge, s_m })),
            _ => Err("each endpoint requires either a node ID or both an edge ID and arc distance"),
        }
    }
}

pub(crate) fn run(args: &[String]) -> Result<bool, Box<dyn Error>> {
    let mut input = None;
    let (mut start, mut goal) = (EndpointOptions::default(), EndpointOptions::default());
    let mut closed = vec![];
    let mut scenario_output = None;
    let mut cruise = None;
    let mut duration = None;
    let mut seen = std::collections::BTreeSet::new();
    for pair in args[1..].chunks(2) {
        let option = &pair[0];
        let value = pair.get(1).ok_or("road-route option requires a value")?;
        if option != "--closed-edge" && !seen.insert(option.clone()) {
            return Err(format!("repeated road-route option {option}").into());
        }
        match option.as_str() {
            "--map" => input = Some(PathBuf::from(value)),
            "--start" => start.node = Some(value.clone()),
            "--start-edge" => start.edge = Some(value.clone()),
            "--start-s" => start.distance = Some(value.parse()?),
            "--goal" => goal.node = Some(value.clone()),
            "--goal-edge" => goal.edge = Some(value.clone()),
            "--goal-s" => goal.distance = Some(value.parse()?),
            "--closed-edge" => closed.push(value.clone()),
            "--scenario-output" => scenario_output = Some(PathBuf::from(value)),
            "--cruise-speed" => cruise = Some(value.parse::<f64>()?),
            "--duration" => duration = Some(value.parse::<f64>()?),
            _ => return Err(format!("invalid road-route option {option}").into()),
        }
    }
    if scenario_output.is_none() && (cruise.is_some() || duration.is_some()) {
        return Err("--cruise-speed and --duration require --scenario-output".into());
    }
    let start = start.finish()?;
    let goal = goal.finish()?;
    let mut bytes = vec![];
    fs::File::open(input.ok_or("--map is required")?)?
        .take(16 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 16 * 1024 * 1024 {
        return Err("road map exceeds 16 MiB".into());
    }
    let spec: RoadNetworkSpec = serde_json::from_slice(&bytes)?;
    let compiled = RoadNetwork::new(spec.clone())?
        .shortest_route_between_with_speed_zones(&start, &goal, &closed)?;
    let scenario = if scenario_output.is_some() {
        let scenario: Scenario = serde_json::from_value(serde_json::json!({
            "name":"Directed road position route", "duration":duration.unwrap_or(120.),
            "road_length":compiled.plan.distance_m,"half_width":compiled.plan.route.half_width,
            "expected":"goal","cruise_speed":cruise.unwrap_or(2.),"local_route_geometry":true,
            "motion_limits":{"max_acceleration_m_s2":1.,"max_deceleration_m_s2":2.5,"max_lateral_acceleration_m_s2":1.},
            "navigation":{"network":spec,"start":start,"goal":goal,"closed_edges":closed},"objects":[]
        }))?;
        scenario.validate()?;
        Some(serde_json::to_vec_pretty(&scenario)?)
    } else {
        None
    };
    let report = serde_json::to_string_pretty(&serde_json::json!({
        "schema_version":1,"start":start,"goal":goal,"closed_edges":closed,
        "plan":compiled.plan,"speed_zones":compiled.speed_zones
    }))?;
    // Validate everything before creation. create_new also rejects aliases,
    // dangling links and existing input/output files without overwriting them.
    if let (Some(path), Some(bytes)) = (scenario_output, scenario) {
        if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
            fs::create_dir_all(parent)?;
        }
        fs::File::create_new(path)?.write_all(&bytes)?;
    }
    println!("{report}");
    Ok(true)
}
