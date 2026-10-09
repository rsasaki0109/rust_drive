use rustdriving_rne::{
    Plant, run, run_with_scene, run_with_scene_adaptive_terrain_objects, run_with_scene_ground,
    run_with_scene_ground_body, run_with_scene_ground_body_precise, run_with_scene_lidar_3d,
    run_with_scene_multi_height, run_with_scene_terrain_objects, scene::Scene,
};
use rustdriving_sim::Scenario;
use std::{env, error::Error, fs, io::BufWriter, path::PathBuf, process};
fn main() {
    if let Err(e) = execute() {
        eprintln!("rustdriving-rne: {e}");
        process::exit(2);
    }
}
fn execute() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "--help" {
        println!(
            "rustdriving-rne --scenario FILE [--scene FILE] [--multi-height | --lidar-3d [--terrain-objects [--adaptive-terrain] | --ground-segmentation [--vehicle-body [--precise-capsule-rays]]]] [--plant kinematic|dynamic] [--seed N] [--output DIR]"
        );
        return Ok(());
    }
    let mut input = None;
    let mut scene_input = None;
    let mut multi_height = false;
    let mut lidar_3d = false;
    let mut ground_segmentation = false;
    let mut vehicle_body = false;
    let mut precise_capsule_rays = false;
    let mut terrain_objects = false;
    let mut adaptive_terrain = false;
    let mut plant = Plant::Kinematic;
    let mut seed = 7;
    let mut output = PathBuf::from("artifacts/rne");
    let mut index = 0;
    while index < args.len() {
        let option = &args[index];
        if option == "--multi-height" {
            multi_height = true;
            index += 1;
            continue;
        }
        if option == "--lidar-3d" {
            lidar_3d = true;
            index += 1;
            continue;
        }
        if option == "--ground-segmentation" {
            ground_segmentation = true;
            index += 1;
            continue;
        }
        if option == "--vehicle-body" {
            vehicle_body = true;
            index += 1;
            continue;
        }
        if option == "--precise-capsule-rays" {
            precise_capsule_rays = true;
            index += 1;
            continue;
        }
        if option == "--adaptive-terrain" {
            adaptive_terrain = true;
            index += 1;
            continue;
        }
        if option == "--terrain-objects" {
            terrain_objects = true;
            index += 1;
            continue;
        }
        let value = args.get(index + 1).ok_or("each option requires a value")?;
        match option.as_str() {
            "--scenario" => input = Some(PathBuf::from(value)),
            "--scene" => scene_input = Some(PathBuf::from(value)),
            "--plant" => {
                plant = match value.as_str() {
                    "kinematic" => Plant::Kinematic,
                    "dynamic" => Plant::Dynamic,
                    _ => return Err("unknown plant".into()),
                }
            }
            "--seed" => seed = value.parse()?,
            "--output" => output = PathBuf::from(value),
            _ => return Err("unknown option".into()),
        }
        index += 2;
    }
    if multi_height && scene_input.is_none() {
        return Err("--multi-height requires --scene".into());
    }
    if lidar_3d && scene_input.is_none() {
        return Err("--lidar-3d requires --scene".into());
    }
    if multi_height && lidar_3d {
        return Err("--multi-height and --lidar-3d are mutually exclusive".into());
    }
    if ground_segmentation && !lidar_3d {
        return Err("--ground-segmentation requires --lidar-3d".into());
    }
    if vehicle_body && !ground_segmentation {
        return Err("--vehicle-body requires --ground-segmentation".into());
    }
    if precise_capsule_rays && !(vehicle_body && ground_segmentation && lidar_3d) {
        return Err(
            "--precise-capsule-rays requires --lidar-3d --ground-segmentation --vehicle-body"
                .into(),
        );
    }
    if adaptive_terrain && !terrain_objects {
        return Err("--adaptive-terrain requires --terrain-objects and --lidar-3d".into());
    }
    if terrain_objects && ground_segmentation {
        return Err("--terrain-objects and --ground-segmentation are mutually exclusive".into());
    }
    if terrain_objects && !lidar_3d {
        return Err("--terrain-objects requires --lidar-3d and a physical ground scene".into());
    }
    if terrain_objects && vehicle_body {
        return Err("--terrain-objects and --vehicle-body are not supported together".into());
    }
    let scenario: Scenario =
        serde_json::from_str(&fs::read_to_string(input.ok_or("--scenario required")?)?)?;
    let (result, scene_evidence) = if let Some(path) = scene_input {
        let scene = Scene::from_json(&fs::read_to_string(path)?)?;
        let (result, evidence) = if adaptive_terrain {
            run_with_scene_adaptive_terrain_objects(scenario, seed, plant, scene)?
        } else if terrain_objects {
            run_with_scene_terrain_objects(scenario, seed, plant, scene)?
        } else if vehicle_body {
            if precise_capsule_rays {
                run_with_scene_ground_body_precise(scenario, seed, plant, scene)?
            } else {
                run_with_scene_ground_body(scenario, seed, plant, scene)?
            }
        } else if ground_segmentation {
            run_with_scene_ground(scenario, seed, plant, scene)?
        } else if lidar_3d {
            run_with_scene_lidar_3d(scenario, seed, plant, scene)?
        } else if multi_height {
            run_with_scene_multi_height(scenario, seed, plant, scene)?
        } else {
            run_with_scene(scenario, seed, plant, scene)?
        };
        (result, Some(evidence))
    } else {
        (run(scenario, seed, plant)?, None)
    };
    fs::create_dir_all(&output)?;
    let scene_output = output.join("scene.json");
    if let Some(evidence) = scene_evidence {
        fs::write(&scene_output, serde_json::to_vec(&evidence)?)?;
    } else if scene_output.exists() {
        // Reusing an output directory must not leave stale scene acceptance.
        fs::remove_file(scene_output)?;
    }
    fs::write(output.join("run.json"), serde_json::to_vec(&result)?)?;
    result
        .sensor_log
        .as_ref()
        .unwrap()
        .write(BufWriter::new(fs::File::create(
            output.join("sensors.jsonl"),
        )?))?;
    let summary = serde_json::to_string_pretty(&result.summary)?;
    fs::write(output.join("summary.json"), &summary)?;
    println!("{summary}");
    if !result.summary.passed {
        process::exit(1);
    }
    Ok(())
}
