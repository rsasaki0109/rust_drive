//! One-image offline analysis and complete, no-clobber report publication.
use super::{yolox_s, yolox_tiles, yolox_vehicle_consistency};
use image::RgbImage;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    fs::{self, File, OpenOptions},
    io::{self, Cursor, Read, Write},
    path::{Path, PathBuf},
    sync::atomic::{AtomicU64, Ordering},
};

pub const REPORT_LIMIT: usize = 4 * 1024 * 1024;
const IMAGE_LIMIT: usize = 16 * 1024 * 1024;
static NEXT_STAGE: AtomicU64 = AtomicU64::new(0);

pub fn read_image(path: &Path) -> Result<(RgbImage, String), String> {
    if !fs::symlink_metadata(path)
        .map_err(|e| e.to_string())?
        .file_type()
        .is_file()
    {
        return Err("image must be a regular non-symlink file".into());
    }
    let file = File::open(path).map_err(|e| e.to_string())?;
    if file.metadata().map_err(|e| e.to_string())?.len() > IMAGE_LIMIT as u64 {
        return Err("image encoded-byte budget exceeded".into());
    }
    let mut bytes = Vec::new();
    file.take(IMAGE_LIMIT as u64 + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > IMAGE_LIMIT {
        return Err("image encoded-byte budget exceeded".into());
    }
    let reader = image::ImageReader::new(Cursor::new(&bytes))
        .with_guessed_format()
        .map_err(|e| e.to_string())?;
    let (width, height) = reader.into_dimensions().map_err(|e| e.to_string())?;
    if width == 0 || height == 0 || u64::from(width) * u64::from(height) > 16_000_000 {
        return Err("image pixel budget exceeded".into());
    }
    let image = image::load_from_memory(&bytes)
        .map_err(|e| e.to_string())?
        .into_rgb8();
    Ok((image, format!("{:x}", Sha256::digest(bytes))))
}

pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = yolox_vehicle_consistency::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/camera_analysis.rs",
            include_bytes!("camera_analysis.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-analyze.rs",
            include_bytes!("bin/rustdriving-camera-analyze.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-analysis.json",
            include_bytes!("../profile-analysis.json").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

#[derive(Serialize)]
pub struct Execution {
    pub model_read: bool,
    pub images_read: usize,
    pub model_forwards: usize,
    pub inference_run: bool,
}

#[derive(Serialize)]
pub struct Report {
    pub schema_version: u32,
    pub algorithm: &'static str,
    pub compiled_sources: BTreeMap<&'static str, String>,
    pub source_image_sha256: String,
    pub execution: Execution,
    pub base_prediction: serde_json::Value,
    pub result: yolox_vehicle_consistency::ConsistencyResult,
}

pub fn analyze(
    detector: &yolox_s::Detector,
    image: &RgbImage,
    source_sha: String,
) -> Result<Report, String> {
    let base = yolox_tiles::detect(detector, image, source_sha.clone())?;
    let forwards = base.views.len();
    let base_prediction = serde_json::to_value(base).map_err(|e| e.to_string())?;
    let fixture = yolox_vehicle_consistency::producer_fixture(&base_prediction)?;
    let result = yolox_vehicle_consistency::reconcile(&fixture)?;
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_s_image_analysis_v1",
        compiled_sources: compiled_sources(),
        source_image_sha256: source_sha,
        execution: Execution {
            model_read: true,
            images_read: 1,
            model_forwards: forwards,
            inference_run: true,
        },
        base_prediction,
        result,
    })
}

/// A sibling staging file is reserved before reading the model or image.
/// A hard link publishes the finished bytes without replacing any destination.
/// Normal failures remove staging; a killed process can leave a .partial file.
pub struct AtomicReport {
    destination: PathBuf,
    stage: PathBuf,
    file: Option<File>,
}

impl AtomicReport {
    pub fn reserve(destination: &Path) -> Result<Self, String> {
        match fs::symlink_metadata(destination) {
            Ok(_) => return Err("fresh output required before input reads".into()),
            Err(e) if e.kind() == io::ErrorKind::NotFound => (),
            Err(e) => return Err(format!("inspect output: {e}")),
        }
        let parent = destination
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        for _ in 0..128 {
            let stage = parent.join(format!(
                ".rustdriving-camera-{}-{}.partial",
                std::process::id(),
                NEXT_STAGE.fetch_add(1, Ordering::Relaxed)
            ));
            if stage.file_name() == destination.file_name() {
                continue;
            }
            match OpenOptions::new().write(true).create_new(true).open(&stage) {
                Ok(file) => {
                    return Ok(Self {
                        destination: destination.to_path_buf(),
                        stage,
                        file: Some(file),
                    });
                }
                Err(e) if e.kind() == io::ErrorKind::AlreadyExists => continue,
                Err(e) => return Err(format!("prepare output: {e}")),
            }
        }
        Err("output staging names exhausted".into())
    }

    pub fn publish(mut self, value: &impl Serialize) -> Result<(), String> {
        let file = self.file.as_mut().ok_or("output staging closed")?;
        let writer = BoundedWriter {
            inner: file,
            remaining: REPORT_LIMIT,
        };
        serde_json::to_writer_pretty(writer, value).map_err(|e| format!("write report: {e}"))?;
        self.file
            .as_ref()
            .ok_or("output staging closed")?
            .sync_all()
            .map_err(|e| format!("sync report: {e}"))?;
        self.file.take();
        fs::hard_link(&self.stage, &self.destination)
            .map_err(|e| format!("publish report: {e}"))?;
        Ok(())
    }
}

impl Drop for AtomicReport {
    fn drop(&mut self) {
        self.file.take();
        let _ = fs::remove_file(&self.stage);
    }
}

struct BoundedWriter<W> {
    inner: W,
    remaining: usize,
}
impl<W: Write> Write for BoundedWriter<W> {
    fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
        if bytes.len() > self.remaining {
            return Err(io::Error::other("4 MiB report budget exceeded"));
        }
        let written = self.inner.write(bytes)?;
        self.remaining -= written;
        Ok(written)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.inner.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Directory(PathBuf);
    impl Directory {
        fn new() -> Self {
            let path = std::env::temp_dir().join(format!(
                "rustdriving-camera-test-{}-{}",
                std::process::id(),
                NEXT_STAGE.fetch_add(1, Ordering::Relaxed)
            ));
            fs::create_dir(&path).unwrap();
            Self(path)
        }
    }
    impl Drop for Directory {
        fn drop(&mut self) {
            fs::remove_dir_all(&self.0).unwrap();
        }
    }

    #[test]
    fn publication_race_preserves_the_other_writer_and_removes_staging() {
        let directory = Directory::new();
        let path = directory.0.join("report.json");
        let reserved = AtomicReport::reserve(&path).unwrap();
        fs::write(&path, b"other writer").unwrap();
        assert!(
            reserved
                .publish(&serde_json::json!({"complete": true}))
                .is_err()
        );
        assert_eq!(fs::read(&path).unwrap(), b"other writer");
        assert_eq!(fs::read_dir(&directory.0).unwrap().count(), 1);
    }

    #[test]
    fn oversized_serialization_never_publishes_a_truncated_report() {
        let directory = Directory::new();
        let path = directory.0.join("report.json");
        let reserved = AtomicReport::reserve(&path).unwrap();
        assert!(reserved.publish(&"x".repeat(REPORT_LIMIT)).is_err());
        assert!(!path.exists());
        assert_eq!(fs::read_dir(&directory.0).unwrap().count(), 0);
    }

    #[test]
    fn abandoning_reserved_output_removes_staging() {
        let directory = Directory::new();
        let path = directory.0.join("report.json");
        drop(AtomicReport::reserve(&path).unwrap());
        assert!(!path.exists());
        assert_eq!(fs::read_dir(&directory.0).unwrap().count(), 0);
    }
}
