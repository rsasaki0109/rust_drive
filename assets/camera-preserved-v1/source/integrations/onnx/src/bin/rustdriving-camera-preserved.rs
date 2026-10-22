//! Explicit bounded model-free fusion diagnostic; never inference attestation.
#[path = "../camera_preserved.rs"]
mod camera_preserved;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    env, fs,
    io::{Read, Write},
    path::Path,
};
fn sources() -> BTreeMap<&'static str, String> {
    macro_rules! source {
        ($name:literal) => {
            (
                $name,
                include_bytes!(concat!("../../../../", $name)).as_slice(),
            )
        };
    }
    [
        source!("integrations/onnx/src/camera_preserved.rs"),
        source!("integrations/onnx/src/bin/rustdriving-camera-preserved.rs"),
        source!("integrations/onnx/src/lib.rs"),
        source!("integrations/onnx/Cargo.toml"),
        source!("integrations/onnx/Cargo.lock"),
        source!("integrations/onnx/profile-preserved.json"),
        source!("rust-toolchain.toml"),
    ]
    .into_iter()
    .map(|(n, b)| (n, format!("{:x}", Sha256::digest(b))))
    .collect()
}
fn publish(path: &Path, bytes: &[u8]) -> Result<(), String> {
    if bytes.len() > 4 * 1024 * 1024 {
        return Err("complete report exceeds4MiB".into());
    }
    let parent = path.parent().ok_or("output parent required")?;
    let temp = parent.join(format!(".preserved-{}.tmp", std::process::id()));
    let mut file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temp)
        .map_err(|e| e.to_string())?;
    struct Cleanup(std::path::PathBuf);
    impl Drop for Cleanup {
        fn drop(&mut self) {
            let _ = fs::remove_file(&self.0);
        }
    }
    let _cleanup = Cleanup(temp.clone());
    file.write_all(bytes).map_err(|e| e.to_string())?;
    file.sync_all().map_err(|e| e.to_string())?;
    drop(file);
    fs::hard_link(&temp, path).map_err(|e| format!("publish new complete report: {e}"))?;
    Ok(())
}
fn run() -> Result<(), String> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    let [flag, input, out, destination] = args.as_slice() else {
        return Err("expected --fixture FILE --output NEW".into());
    };
    if flag != "--fixture" || out != "--output" {
        return Err("expected --fixture FILE --output NEW".into());
    }
    let path = Path::new(input);
    if !fs::symlink_metadata(path)
        .map_err(|e| e.to_string())?
        .is_file()
    {
        return Err("fixture must be regular and not a symlink".into());
    }
    let mut bytes = Vec::new();
    fs::File::open(path)
        .map_err(|e| e.to_string())?
        .take(4 * 1024 * 1024 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > 4 * 1024 * 1024 {
        return Err("fixture exceeds4MiB".into());
    }
    let input: camera_preserved::Input =
        serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    let result = camera_preserved::combine(&input)?;
    let report = serde_json::json!({"schema_version":1,"algorithm":"baseline_preserving_camera_fixture_v1","compiled_sources":sources(),"profile_sha256":format!("{:x}",Sha256::digest(include_bytes!("../../profile-preserved.json"))),"source_fixture_sha256":format!("{:x}",Sha256::digest(bytes)),"model_read":false,"images_read":false,"model_forwards":0,"control_permission":false,"result":result});
    let body = serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?;
    publish(Path::new(destination), &body)
}
fn main() {
    if let Err(e) = run() {
        eprintln!("camera preserved: {e}");
        std::process::exit(2);
    }
}
