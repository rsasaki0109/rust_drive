//! Complete grid in joined waves of at most four std scoped view workers.
use super::{serial_camera_analysis, yolox_s, yolox_tiles, yolox_vehicle_consistency};
use image::RgbImage;
use serde::Serialize;
pub use serial_camera_analysis::{AtomicReport, read_image};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, thread, time::Instant};

/// Preserve source order and join every started job before any success/error.
fn ordered_wave<T: Send>(
    count: usize,
    job: impl Fn(usize) -> Result<T, String> + Sync,
) -> Result<Vec<T>, String> {
    if !(1..=4).contains(&count) {
        return Err("one to four fixed views required".into());
    }
    if count == 1 {
        return Ok(vec![job(0)?]);
    }
    thread::scope(|scope| {
        let job = &job;
        let mut handles = Vec::new();
        let mut error = None;
        for index in 0..count {
            match thread::Builder::new()
                .name(format!("camera-view-{index}"))
                .spawn_scoped(scope, move || job(index))
            {
                Ok(handle) => handles.push(handle),
                Err(e) => {
                    error = Some(format!("start view worker: {e}"));
                    break;
                }
            }
        }
        let mut results = Vec::new();
        for handle in handles {
            match handle.join() {
                Ok(value) => results.push(value),
                Err(_) => error = Some("view worker panicked".into()),
            }
        }
        if let Some(error) = error {
            return Err(error);
        }
        results.into_iter().collect()
    })
}
fn ordered<T: Send>(
    count: usize,
    job: impl Fn(usize) -> Result<T, String> + Sync,
) -> Result<Vec<T>, String> {
    if !(1..=yolox_tiles::MAX_VIEWS).contains(&count) {
        return Err("one to sixteen complete views required".into());
    }
    let mut result = Vec::with_capacity(count);
    for start in (0..count).step_by(4) {
        result.extend(ordered_wave((count - start).min(4), |index| {
            job(start + index)
        })?);
    }
    Ok(result)
}
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    super::compiled_sources()
}

#[derive(Serialize)]
struct Base {
    schema_version: u32,
    algorithm: &'static str,
    compiled_sources: BTreeMap<&'static str, String>,
    source_image_sha256: String,
    model_sha256: &'static str,
    width: u32,
    height: u32,
    score_threshold: f32,
    nms_iou_threshold: f32,
    views: Vec<yolox_tiles::View>,
    #[serde(flatten)]
    merged: yolox_tiles::Merged,
}
#[derive(Serialize)]
pub struct Execution {
    model_read: bool,
    images_read: usize,
    pub model_forwards: usize,
    inference_run: bool,
    view_workers: usize,
    shared_model_plans: usize,
    analysis_wall_s: f64,
}
#[derive(Serialize)]
pub struct Report {
    schema_version: u32,
    algorithm: &'static str,
    compiled_sources: BTreeMap<&'static str, String>,
    source_image_sha256: String,
    pub execution: Execution,
    control_permission: bool,
    original_graph_reference: bool,
    profile_sha256: String,
    base_prediction: serde_json::Value,
    pub result: yolox_vehicle_consistency::ConsistencyResult,
}
pub fn analyze(
    detector: &yolox_s::Detector,
    image: &RgbImage,
    source_sha: String,
) -> Result<Report, String> {
    let started = Instant::now();
    let rectangles = yolox_tiles::rectangles(image.width(), image.height())?;
    let count = rectangles.len();
    let infer = |index: usize| {
        let rectangle = rectangles[index];
        let cropped = image::imageops::crop_imm(
            image,
            rectangle.x,
            rectangle.y,
            rectangle.width,
            rectangle.height,
        )
        .to_image();
        let prediction = detector.detect(&cropped, 0.3, 0.45)?;
        Ok(yolox_tiles::View {
            rectangle,
            rgb_sha256: format!("{:x}", Sha256::digest(cropped.as_raw())),
            prediction,
        })
    };
    let views = if detector.is_reference() {
        (0..count).map(infer).collect::<Result<Vec<_>, String>>()?
    } else {
        ordered(count, infer)?
    };
    // These are original typed measurements; no external report profile is
    // rewritten or admitted as if it were produced by the serial executable.
    let fixture = yolox_tiles::Fixture {
        width: image.width(),
        height: image.height(),
        views: views
            .iter()
            .map(|v| yolox_tiles::FixtureView {
                rectangle: v.rectangle,
                detections: v.prediction.detections.clone(),
            })
            .collect(),
    };
    let merged = yolox_tiles::merge(&fixture)?;
    let result = yolox_vehicle_consistency::reconcile(&fixture)?;
    let base_prediction = serde_json::to_value(Base {
        schema_version: 1,
        algorithm: "pinned_yolox_nano_grid_fixed_views_v1",
        compiled_sources: compiled_sources(),
        source_image_sha256: source_sha.clone(),
        model_sha256: yolox_s::MODEL_SHA256,
        width: image.width(),
        height: image.height(),
        score_threshold: 0.3,
        nms_iou_threshold: 0.45,
        views,
        merged,
    })
    .map_err(|e| e.to_string())?;
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_nano_grid_image_analysis_v1",
        compiled_sources: compiled_sources(),
        source_image_sha256: source_sha,
        control_permission: false,
        original_graph_reference: detector.is_reference(),
        profile_sha256: format!(
            "{:x}",
            Sha256::digest(include_bytes!("../profile-grid.json"))
        ),
        execution: Execution {
            model_read: true,
            images_read: 1,
            model_forwards: count,
            inference_run: true,
            view_workers: if detector.is_reference() {
                1
            } else {
                count.min(4)
            },
            shared_model_plans: 1,
            analysis_wall_s: started.elapsed().as_secs_f64(),
        },
        base_prediction,
        result,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{
        Condvar, Mutex,
        atomic::{AtomicUsize, Ordering},
    };
    #[test]
    fn concurrently_completed_views_keep_original_order() {
        let arrivals = (Mutex::new(0usize), Condvar::new());
        let results = ordered(4, |index| {
            let mut count = arrivals.0.lock().unwrap();
            *count += 1;
            arrivals.1.notify_all();
            let (count, timed) = arrivals
                .1
                .wait_timeout_while(count, std::time::Duration::from_secs(1), |n| *n < 4)
                .unwrap();
            if timed.timed_out() || *count != 4 {
                return Err("view jobs did not overlap".into());
            }
            drop(count);
            if index == 0 {
                thread::sleep(std::time::Duration::from_millis(2));
            }
            Ok(index)
        })
        .unwrap();
        assert_eq!(results, vec![0, 1, 2, 3]);
    }
    #[test]
    fn worker_error_waits_for_all_other_views() {
        let completed = AtomicUsize::new(0);
        let result = ordered(4, |index| {
            completed.fetch_add(1, Ordering::SeqCst);
            if index == 0 {
                Err("view inference failed".into())
            } else {
                thread::sleep(std::time::Duration::from_millis(2));
                Ok(index)
            }
        });
        assert!(result.is_err());
        assert_eq!(completed.load(Ordering::SeqCst), 4);
    }
    #[test]
    fn worker_panic_is_a_joined_error_including_multiple_panics() {
        let completed = AtomicUsize::new(0);
        let result = ordered(4, |index| {
            completed.fetch_add(1, Ordering::SeqCst);
            assert!(index >= 2, "injected view panic");
            Ok(index)
        });
        assert!(result.is_err());
        assert_eq!(completed.load(Ordering::SeqCst), 4);
    }
    #[test]
    fn square_single_view_uses_the_calling_thread() {
        let caller = thread::current().id();
        assert_eq!(yolox_tiles::rectangles(8, 8).unwrap().len(), 1);
        assert_eq!(
            ordered(1, |_| Ok(thread::current().id())).unwrap(),
            vec![caller]
        );
    }
    #[test]
    fn out_of_budget_view_counts_start_no_jobs() {
        for count in [0, 17, usize::MAX] {
            let result: Result<Vec<()>, String> =
                ordered(count, |_| panic!("invalid count started a worker"));
            assert!(result.is_err());
        }
    }
}

#[cfg(test)]
mod grid_worker_tests {
    use super::*;
    use std::sync::{
        Barrier,
        atomic::{AtomicUsize, Ordering},
    };
    #[test]
    fn three_joined_waves_preserve_all_twelve_measurements() {
        let barrier = Barrier::new(4);
        let active = AtomicUsize::new(0);
        let peak = AtomicUsize::new(0);
        let result = ordered(12, |index| {
            let current = active.fetch_add(1, Ordering::SeqCst) + 1;
            peak.fetch_max(current, Ordering::SeqCst);
            barrier.wait();
            active.fetch_sub(1, Ordering::SeqCst);
            Ok(index)
        })
        .unwrap();
        assert_eq!(result, (0..12).collect::<Vec<_>>());
        assert_eq!(peak.load(Ordering::SeqCst), 4);
        assert_eq!(active.load(Ordering::SeqCst), 0);
    }
    #[test]
    fn failed_wave_is_drained_without_starting_later_waves() {
        let completed = AtomicUsize::new(0);
        assert!(
            ordered(12, |index| {
                completed.fetch_add(1, Ordering::SeqCst);
                if index == 1 {
                    Err("injected".into())
                } else {
                    Ok(index)
                }
            })
            .is_err()
        );
        assert_eq!(completed.load(Ordering::SeqCst), 4);
    }
}
