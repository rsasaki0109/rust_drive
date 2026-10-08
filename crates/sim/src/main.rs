use rustdrive_pipeline::replay::verify;
use rustdrive_sim::{Scenario, simulate};
use std::{
    env,
    error::Error,
    fs,
    io::{BufReader, BufWriter},
    path::PathBuf,
    process,
};
fn run() -> Result<bool, Box<dyn Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "--help" || args[0] == "-h" {
        println!(
            "RustDrive sensor-driven simulator\nUsage:\n  rustdrive run --scenario FILE [--seed N] [--output DIR]\n  rustdrive replay --log FILE [--output DIR]\nExit 0: acceptance/replay passed; 1: scenario acceptance failed; 2: invalid input, replay mismatch or I/O failure."
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
