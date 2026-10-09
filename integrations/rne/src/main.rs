use rustdrive_rne::{Plant, run, run_with_scene, run_with_scene_multi_height, scene::Scene};
use rustdrive_sim::Scenario;
use std::{env, error::Error, fs, io::BufWriter, path::PathBuf, process};
fn main() {
    if let Err(e) = execute() {
        eprintln!("rustdrive-rne: {e}");
        process::exit(2);
    }
}
fn execute() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "--help" {
        println!(
            "rustdrive-rne --scenario FILE [--scene FILE] [--multi-height] [--plant kinematic|dynamic] [--seed N] [--output DIR]"
        );
        return Ok(());
    }
    let mut input = None;
    let mut scene_input = None;
    let mut multi_height = false;
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
    let scenario: Scenario =
        serde_json::from_str(&fs::read_to_string(input.ok_or("--scenario required")?)?)?;
    let (result, scene_evidence) = if let Some(path) = scene_input {
        let scene = Scene::from_json(&fs::read_to_string(path)?)?;
        let (result, evidence) = if multi_height {
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
