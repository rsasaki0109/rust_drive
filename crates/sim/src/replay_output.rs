//! Publish only fully verified replay outputs; the report is the commit marker.
use rustdriving_pipeline::replay::verify;
use std::{
    error::Error,
    fs,
    io::{BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

fn validate_replay_destinations(log: &Path, destinations: &[&Path]) -> Result<(), Box<dyn Error>> {
    let input = match fs::canonicalize(log) {
        Ok(path) => path,
        // Missing input still invalidates unrelated previous results below.
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(()),
        Err(error) => return Err(error.into()),
    };
    #[cfg(unix)]
    let input_metadata = fs::metadata(&input)?;
    for destination in destinations {
        let resolved = match fs::canonicalize(destination) {
            Ok(path) => path,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => continue,
            Err(error) => return Err(error.into()),
        };
        let same = input == resolved;
        #[cfg(unix)]
        let same = {
            use std::os::unix::fs::MetadataExt;
            let metadata = fs::metadata(&resolved)?;
            same || (input_metadata.dev() == metadata.dev()
                && input_metadata.ino() == metadata.ino())
        };
        if same {
            return Err(format!(
                "replay output {} aliases input log {}",
                destination.display(),
                log.display()
            )
            .into());
        }
    }
    Ok(())
}

fn remove_replay_artifact(path: &Path) -> std::io::Result<()> {
    // Unlink the artifact itself, including dangling links, without touching its target.
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error),
    }
}

static NEXT_STAGE: AtomicU64 = AtomicU64::new(0);

struct StagedReplay {
    directory: PathBuf,
}
impl StagedReplay {
    fn new(output: &Path) -> std::io::Result<Self> {
        for _ in 0..128 {
            let id = NEXT_STAGE.fetch_add(1, Ordering::Relaxed);
            let directory = output.join(format!(".replay-stage-{}-{id}", std::process::id()));
            match fs::create_dir(&directory) {
                Ok(()) => return Ok(Self { directory }),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => return Err(error),
            }
        }
        Err(std::io::Error::other(
            "cannot allocate replay staging directory",
        ))
    }

    fn publish(&self, outputs: &Path, summary: &Path) -> std::io::Result<()> {
        // Same-filesystem hard links publish complete files without replacing
        // an entry that appeared since invalidation. The report is linked last.
        fs::hard_link(self.directory.join("outputs.jsonl"), outputs)?;
        if let Err(error) = fs::hard_link(self.directory.join("replay.json"), summary) {
            if let Err(cleanup) = fs::remove_file(outputs) {
                return Err(std::io::Error::other(format!(
                    "report publication failed: {error}; output cleanup failed: {cleanup}"
                )));
            }
            return Err(error);
        }
        Ok(())
    }
}
impl Drop for StagedReplay {
    fn drop(&mut self) {
        // Only this invocation's exclusively created directory is owned here.
        let _ = fs::remove_dir_all(&self.directory);
    }
}

pub(super) fn run(log: Option<&Path>, output: &Path) -> Result<String, Box<dyn Error>> {
    fs::create_dir_all(output)?;
    let summary = output.join("replay.json");
    let outputs = output.join("outputs.jsonl");
    if let Some(input) = log {
        validate_replay_destinations(input, &[&summary, &outputs])?;
    }
    // Invalidate both old artifacts even when the new input is missing. Alias
    // rejection above preserves the input and both old artifacts unchanged.
    remove_replay_artifact(&summary)?;
    remove_replay_artifact(&outputs)?;
    let reader = BufReader::new(fs::File::open(log.ok_or("--log is required")?)?);
    let staged = StagedReplay::new(output)?;
    let mut writer = BufWriter::new(fs::File::create_new(
        staged.directory.join("outputs.jsonl"),
    )?);
    let report = verify(reader, &mut writer)?;
    writer.flush()?;
    writer.get_ref().sync_all()?;
    // Close before publication and cleanup, including on Windows.
    drop(writer);
    let json = serde_json::to_string_pretty(&report)?;
    let mut report_file = fs::File::create_new(staged.directory.join("replay.json"))?;
    report_file.write_all(json.as_bytes())?;
    report_file.sync_all()?;
    drop(report_file);
    staged.publish(&outputs, &summary)?;
    Ok(json)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn publication_collisions_preserve_new_entries_and_roll_back_own_outputs() {
        for occupied in ["outputs.jsonl", "replay.json"] {
            let root = std::env::temp_dir().join(format!(
                "rustdriving-publish-{}-{occupied}",
                std::process::id()
            ));
            fs::create_dir(&root).unwrap();
            {
                let stage = StagedReplay::new(&root).unwrap();
                fs::write(stage.directory.join("outputs.jsonl"), b"complete stream").unwrap();
                fs::write(stage.directory.join("replay.json"), b"complete report").unwrap();
                fs::write(root.join(occupied), b"new unrelated entry").unwrap();
                assert!(
                    stage
                        .publish(&root.join("outputs.jsonl"), &root.join("replay.json"))
                        .is_err()
                );
                assert_eq!(
                    fs::read(root.join(occupied)).unwrap(),
                    b"new unrelated entry"
                );
                let other = if occupied == "outputs.jsonl" {
                    "replay.json"
                } else {
                    "outputs.jsonl"
                };
                assert!(!root.join(other).exists());
            }
            assert_eq!(fs::read_dir(&root).unwrap().count(), 1);
            fs::remove_dir_all(root).unwrap();
        }
    }
}
