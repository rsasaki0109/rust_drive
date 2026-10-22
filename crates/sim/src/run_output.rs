//! Complete run artifacts, with summary.json published last as the completion marker.
use crate::Run;
use serde::Serialize;
use std::{
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const DATA: [&str; 3] = ["run.json", "sensors.jsonl", "scene.json"];
static NEXT_STAGE: AtomicU64 = AtomicU64::new(0);

/// Owns only its exclusively created staging directory. Call after parsing
/// options and before loading scenario/scene inputs or starting simulation.
/// Keep the output directory and its ancestors stable during the invocation.
pub struct RunOutput {
    output: PathBuf,
    stage: PathBuf,
    native_scene: bool,
}

impl RunOutput {
    /// Reserve the reference run's three artifacts, protecting all input paths.
    pub fn reference(output: &Path, inputs: &[&Path]) -> Result<Self> {
        Self::prepare(output, inputs, false)
    }

    /// Also reserve scene.json, removing stale native scene evidence even when
    /// the new invocation has no scene. Both scenario and scene are inputs.
    pub fn native(output: &Path, inputs: &[&Path]) -> Result<Self> {
        Self::prepare(output, inputs, true)
    }

    fn prepare(output: &Path, inputs: &[&Path], native_scene: bool) -> Result<Self> {
        fs::create_dir_all(output)?;
        let names = ["summary.json", DATA[0], DATA[1], DATA[2]];
        for input in inputs {
            let canonical = match fs::canonicalize(input) {
                Ok(path) => path,
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
                Err(error) => return Err(error.into()),
            };
            #[cfg(unix)]
            let input_metadata = fs::metadata(&canonical)?;
            for name in names.iter().take(if native_scene { 4 } else { 3 }) {
                let destination = output.join(name);
                let resolved = match fs::canonicalize(&destination) {
                    Ok(path) => path,
                    Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
                    Err(error) => return Err(error.into()),
                };
                let same = canonical == resolved;
                #[cfg(unix)]
                let same = {
                    use std::os::unix::fs::MetadataExt;
                    let metadata = fs::metadata(&resolved)?;
                    same || (input_metadata.dev() == metadata.dev()
                        && input_metadata.ino() == metadata.ino())
                };
                if same {
                    return Err(format!(
                        "run output {} aliases input {}",
                        destination.display(),
                        input.display()
                    )
                    .into());
                }
            }
        }
        // Invalidate the previous completion marker first. Unlink entries,
        // including dangling links, without truncating any linked target.
        // Directories cause an error; their contents are never removed here.
        for name in names.iter().take(if native_scene { 4 } else { 3 }) {
            match fs::remove_file(output.join(name)) {
                Ok(()) => {}
                Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
                Err(error) => return Err(error.into()),
            }
        }
        for _ in 0..128 {
            let id = NEXT_STAGE.fetch_add(1, Ordering::Relaxed);
            let stage = output.join(format!(".run-stage-{}-{id}", std::process::id()));
            match fs::create_dir(&stage) {
                Ok(()) => {
                    return Ok(Self {
                        output: output.to_owned(),
                        stage,
                        native_scene,
                    });
                }
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => return Err(error.into()),
            }
        }
        Err("cannot allocate run staging directory".into())
    }

    fn json(&self, name: &str, value: &impl Serialize) -> Result<()> {
        let mut writer = BufWriter::new(fs::File::create_new(self.stage.join(name))?);
        serde_json::to_writer(&mut writer, value)?;
        writer.flush()?;
        writer.get_ref().sync_all()?;
        Ok(())
    }

    /// Write a complete run, including completed runs with failed acceptance.
    /// Ordinary errors clean up staging. Publication requires hard-link support;
    /// this is not a concurrent multi-file transaction or crash-durability API.
    /// Consuming self closes/removes staging before a CLI may call process::exit.
    pub fn finish<T: Serialize>(self, run: &Run, scene: Option<&T>) -> Result<String> {
        if scene.is_some() && !self.native_scene {
            return Err("scene evidence requires native run outputs".into());
        }
        let log = run.sensor_log.as_ref().ok_or("missing sensor log")?;
        self.json(DATA[0], run)?;
        let mut writer = BufWriter::new(fs::File::create_new(self.stage.join(DATA[1]))?);
        log.write(&mut writer)?;
        writer.flush()?;
        writer.get_ref().sync_all()?;
        drop(writer);
        if let Some(scene) = scene {
            self.json(DATA[2], scene)?;
        }
        let summary = serde_json::to_string_pretty(&run.summary)?;
        let mut file = fs::File::create_new(self.stage.join("summary.json"))?;
        file.write_all(summary.as_bytes())?;
        file.sync_all()?;
        drop(file);
        self.publish(scene.is_some())?;
        Ok(summary)
    }

    fn publish(&self, has_scene: bool) -> std::io::Result<()> {
        let mut published = Vec::with_capacity(4);
        for name in DATA
            .into_iter()
            .filter(|name| *name != DATA[2] || has_scene)
            .chain(["summary.json"])
        {
            if let Err(error) = fs::hard_link(self.stage.join(name), self.output.join(name)) {
                let mut cleanup_errors = Vec::new();
                for previous in published.into_iter().rev() {
                    if let Err(cleanup) = fs::remove_file(self.output.join(previous)) {
                        cleanup_errors.push(format!("{previous}: {cleanup}"));
                    }
                }
                return if cleanup_errors.is_empty() {
                    Err(error)
                } else {
                    Err(std::io::Error::other(format!(
                        "run publication failed: {error}; cleanup failed: {}",
                        cleanup_errors.join(", ")
                    )))
                };
            }
            published.push(name);
        }
        Ok(())
    }
}

impl Drop for RunOutput {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.stage);
    }
}

#[cfg(test)]
mod tests;
