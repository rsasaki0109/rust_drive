//! Create-only OSM exports. The import report is published after the map/scenario.
use serde::Serialize;
use std::{
    collections::HashSet,
    error::Error,
    fs,
    io::{self, BufWriter, Write},
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

static NEXT_STAGE: AtomicU64 = AtomicU64::new(0);

struct Artifact {
    destination: PathBuf,
    stage: PathBuf,
}

/// Keep destination directories and their ancestors stable during publication.
pub(super) struct ImportOutput {
    artifacts: Vec<Artifact>,
}

impl ImportOutput {
    pub(super) fn new(paths: &[&Path]) -> io::Result<Self> {
        let mut destinations = Vec::with_capacity(paths.len());
        for path in paths {
            let name = path.file_name().ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidInput, "OSM output needs a filename")
            })?;
            let parent = path
                .parent()
                .filter(|p| !p.as_os_str().is_empty())
                .unwrap_or(Path::new("."));
            fs::create_dir_all(parent)?;
            destinations.push(fs::canonicalize(parent)?.join(name));
        }
        let mut distinct = HashSet::new();
        for destination in &destinations {
            if !distinct.insert(destination) {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "map, import report and scenario output paths must differ",
                ));
            }
            // symlink_metadata also rejects dangling links. Existing files,
            // input aliases, directories and foreign link targets stay intact.
            match fs::symlink_metadata(destination) {
                Ok(_) => {
                    return Err(io::Error::new(
                        io::ErrorKind::AlreadyExists,
                        format!("OSM output already exists: {}", destination.display()),
                    ));
                }
                Err(error) if error.kind() == io::ErrorKind::NotFound => {}
                Err(error) => return Err(error),
            }
        }
        let mut output = Self {
            artifacts: Vec::with_capacity(paths.len()),
        };
        for destination in &destinations {
            let parent = destination.parent().expect("canonical destination parent");
            let mut allocated = None;
            for _ in 0..128 {
                let id = NEXT_STAGE.fetch_add(1, Ordering::Relaxed);
                let stage = parent.join(format!(".osm-stage-{}-{id}", std::process::id()));
                if destinations.contains(&stage) {
                    continue;
                }
                match fs::create_dir(&stage) {
                    Ok(()) => {
                        allocated = Some(stage);
                        break;
                    }
                    Err(error) if error.kind() == io::ErrorKind::AlreadyExists => continue,
                    Err(error) => return Err(error),
                }
            }
            output.artifacts.push(Artifact {
                destination: destination.clone(),
                stage: allocated
                    .ok_or_else(|| io::Error::other("cannot allocate OSM staging directory"))?,
            });
        }
        Ok(output)
    }

    pub(super) fn json(&self, index: usize, value: &impl Serialize) -> Result<(), Box<dyn Error>> {
        let mut writer = BufWriter::new(fs::File::create_new(
            self.artifacts[index].stage.join("data.json"),
        )?);
        serde_json::to_writer_pretty(&mut writer, value)?;
        writer.flush()?;
        writer.get_ref().sync_all()?;
        Ok(())
    }

    /// Same-filesystem hard links make each complete file visible exclusively.
    /// Ordinary late failures remove our earlier outputs, never a colliding entry.
    /// This is not an atomic multi-file or power-loss transaction.
    pub(super) fn publish(self) -> io::Result<()> {
        for (index, artifact) in self.artifacts.iter().enumerate() {
            if let Err(error) =
                fs::hard_link(artifact.stage.join("data.json"), &artifact.destination)
            {
                let mut cleanup_errors = Vec::new();
                for previous in self.artifacts[..index].iter().rev() {
                    if let Err(cleanup) = fs::remove_file(&previous.destination) {
                        cleanup_errors
                            .push(format!("{}: {cleanup}", previous.destination.display()));
                    }
                }
                return if cleanup_errors.is_empty() {
                    Err(error)
                } else {
                    Err(io::Error::other(format!(
                        "OSM publication failed: {error}; cleanup failed: {}",
                        cleanup_errors.join(", ")
                    )))
                };
            }
        }
        Ok(())
    }
}

impl Drop for ImportOutput {
    fn drop(&mut self) {
        for artifact in &self.artifacts {
            // Only directories exclusively allocated by this invocation are owned.
            let _ = fs::remove_dir_all(&artifact.stage);
        }
    }
}

#[cfg(test)]
mod tests;
