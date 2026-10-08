use rustdrive_sim::{Scenario, simulate};
use std::{env, error::Error, fs, path::PathBuf, process};
fn run() -> Result<bool, Box<dyn Error>> {
    let args: Vec<_> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "--help" || args[0] == "-h" {
        println!(
            "RustDrive deterministic simulator\nUsage: rustdrive run --scenario FILE [--seed N] [--output DIR]\nExit 0: acceptance passed; 1: acceptance failed; 2: invalid invocation/input."
        );
        return Ok(true);
    }
    if args[0] != "run" {
        return Err("expected 'run' subcommand".into());
    }
    let mut scenario = None;
    let mut seed = 7;
    let mut output = PathBuf::from("artifacts/demo");
    let mut i = 1;
    while i < args.len() {
        let value = args.get(i + 1).ok_or("option requires a value")?;
        match args[i].as_str() {
            "--scenario" => scenario = Some(PathBuf::from(value)),
            "--seed" => seed = value.parse()?,
            "--output" => output = PathBuf::from(value),
            _ => return Err(format!("unknown option {}", args[i]).into()),
        };
        i += 2;
    }
    let scenario: Scenario = serde_json::from_str(&fs::read_to_string(
        scenario.ok_or("--scenario is required")?,
    )?)?;
    let run = simulate(scenario, seed)?;
    fs::create_dir_all(&output)?;
    fs::write(output.join("run.json"), serde_json::to_vec(&run)?)?;
    let summary = serde_json::to_string_pretty(&run.summary)?;
    fs::write(output.join("summary.json"), &summary)?;
    println!("{summary}");
    Ok(run.summary.passed)
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
