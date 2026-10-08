use rustdrive_rne::{Plant, run};
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
            "rustdrive-rne --scenario FILE [--plant kinematic|dynamic] [--seed N] [--output DIR]"
        );
        return Ok(());
    }
    let mut input = None;
    let mut plant = Plant::Kinematic;
    let mut seed = 7;
    let mut output = PathBuf::from("artifacts/rne");
    for pair in args.chunks(2) {
        if pair.len() != 2 {
            return Err("each option requires a value".into());
        }
        match pair[0].as_str() {
            "--scenario" => input = Some(PathBuf::from(&pair[1])),
            "--plant" => {
                plant = match pair[1].as_str() {
                    "kinematic" => Plant::Kinematic,
                    "dynamic" => Plant::Dynamic,
                    _ => return Err("unknown plant".into()),
                }
            }
            "--seed" => seed = pair[1].parse()?,
            "--output" => output = PathBuf::from(&pair[1]),
            _ => return Err("unknown option".into()),
        }
    }
    let scenario: Scenario =
        serde_json::from_str(&fs::read_to_string(input.ok_or("--scenario required")?)?)?;
    let result = run(scenario, seed, plant)?;
    fs::create_dir_all(&output)?;
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
