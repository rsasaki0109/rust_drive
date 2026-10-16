//! Measured serial replay. Original source offsets, real host processing clock.
use super::{camera_analysis, camera_freshness, camera_sequence, image_tracking, yolox_s};
use camera_freshness::{Decision, Event, Guard, Input, Output, Policy};
use camera_sequence::{CheckedEvent, Sequence};
use image_tracking::Detection;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    path::Path,
    thread,
    time::{Duration, Instant},
};

#[derive(Serialize)]
pub struct Slot {
    event_index: usize,
    source_timestamp_text: String,
    source_timestamp_s: f64,
    release_offset_s: f64,
    source_image_sha256: Option<String>,
}
pub fn plan(sequence: &Sequence, checked: &[CheckedEvent]) -> Result<Vec<Slot>, String> {
    if sequence.events.len() != checked.len() || checked.is_empty() {
        return Err("complete nonempty preflight required".into());
    }
    let origin = checked[0].timestamp_s;
    let mut slots = Vec::new();
    for (index, check) in checked.iter().enumerate() {
        let offset = check.timestamp_s - origin;
        if !offset.is_finite() || !(0.0..=2.0).contains(&offset) {
            return Err("measured replay schedule exceeds 2 seconds".into());
        }
        slots.push(Slot {
            event_index: index,
            source_timestamp_text: check.timestamp_text.clone(),
            source_timestamp_s: check.timestamp_s,
            release_offset_s: offset,
            source_image_sha256: check.image_sha256.clone(),
        });
    }
    bindings(sequence, &slots)?;
    Ok(slots)
}
fn bindings(sequence: &Sequence, slots: &[Slot]) -> Result<(), String> {
    if sequence.events.len() != slots.len() || slots.is_empty() {
        return Err("complete nonempty preflight required".into());
    }
    for (index, (event, slot)) in sequence.events.iter().zip(slots).enumerate() {
        let image = match &event.input {
            camera_sequence::Input::Image { sha256, .. } => Some(sha256),
            _ => None,
        };
        if index != slot.event_index
            || event.timestamp_s != slot.source_timestamp_text
            || image != slot.source_image_sha256.as_ref()
        {
            return Err("sequence differs from checked clock/image bindings".into());
        }
    }
    Ok(())
}
pub fn image_count(slots: &[Slot]) -> usize {
    slots
        .iter()
        .filter(|slot| slot.source_image_sha256.is_some())
        .count()
}
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = camera_sequence::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/camera_latency.rs",
            include_bytes!("camera_latency.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-latency.rs",
            include_bytes!("bin/rustdriving-camera-latency.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-latency.json",
            include_bytes!("../profile-latency.json").as_slice(),
        ),
        (
            "integrations/camera-tracking/src/freshness.rs",
            include_bytes!("../../camera-tracking/src/freshness.rs").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

trait Clock {
    fn checkpoint(&mut self) -> Result<f64, String>;
}
struct WallClock {
    origin: Instant,
    last: Option<f64>,
}
impl WallClock {
    fn new() -> Self {
        Self {
            origin: Instant::now(),
            last: None,
        }
    }
    fn wait_until(&self, offset_s: f64) {
        let left = offset_s - self.origin.elapsed().as_secs_f64();
        if left > 0. {
            thread::sleep(Duration::from_secs_f64(left));
        }
    }
}
impl Clock for WallClock {
    fn checkpoint(&mut self) -> Result<f64, String> {
        for _ in 0..128 {
            let now = self.origin.elapsed().as_secs_f64();
            if !now.is_finite() || !(0.0..=1e12).contains(&now) {
                return Err("invalid host monotonic clock".into());
            }
            if self.last.is_none_or(|last| now - last >= 0.000001) {
                self.last = Some(now);
                return Ok(now);
            }
            // Wait for an actually observed clock advance; never synthesize time.
            thread::yield_now();
        }
        Err("host monotonic clock did not advance one microsecond".into())
    }
}
fn advances(earlier: f64, later: f64) -> Result<(), String> {
    if !earlier.is_finite()
        || earlier < 0.
        || !later.is_finite()
        || later > 1e12
        || later - earlier < 0.000001
    {
        return Err("processing checkpoints must advance at least one microsecond".into());
    }
    Ok(())
}
#[derive(Serialize)]
pub struct Dispatch {
    event: Event,
    output: Output,
}
#[derive(Serialize)]
pub struct FrameTiming {
    processing_started_s: f64,
    sample_submitted_s: f64,
    checked_after_tracking_s: f64,
    read_analysis_and_probe_wall_s: f64,
    sample_guard_wall_s: f64,
    source_age_at_start_s: Option<f64>,
    source_age_at_check_s: Option<f64>,
    acquisition_deadline_s: Option<f64>,
    deadline_missed_at_check: Option<bool>,
}
#[derive(Serialize)]
pub struct Measured<T> {
    precheck_decision: Option<Decision>,
    analysis: Option<T>,
    timing: FrameTiming,
    guard_events: [Dispatch; 2],
}
fn process<T>(
    guard: &mut Guard,
    clock: &mut impl Clock,
    mut input: Input,
    policy: &Policy,
    analyze: impl FnOnce() -> Result<(T, Vec<Detection>), String>,
) -> Result<Measured<T>, String> {
    // Commit neither a probe nor a partial frame after a detector/clock failure.
    let mut candidate = guard.clone();
    let started = clock.checkpoint()?;
    let source = match &input {
        Input::Sample { acquired_s, .. } => Some(*acquired_s),
        _ => None,
    };
    let precheck = if source.is_some() {
        Some(
            candidate
                .clone()
                .step(&Event {
                    now_s: started,
                    input: input.clone(),
                })?
                .decision,
        )
    } else {
        None
    };
    let analysis = if precheck == Some(Decision::Accepted) {
        let (report, detections) = analyze()?;
        if let Input::Sample {
            detections: payload,
            ..
        } = &mut input
        {
            *payload = detections;
        }
        Some(report)
    } else {
        None
    };
    let submitted = clock.checkpoint()?;
    advances(started, submitted)?;
    let event = Event {
        now_s: submitted,
        input,
    };
    let output = candidate.step(&event)?;
    let checked = clock.checkpoint()?;
    advances(submitted, checked)?;
    let poll = Event {
        now_s: checked,
        input: Input::Tick,
    };
    let final_output = candidate.step(&poll)?;
    *guard = candidate;
    Ok(Measured {
        precheck_decision: precheck,
        analysis,
        timing: FrameTiming {
            processing_started_s: started,
            sample_submitted_s: submitted,
            checked_after_tracking_s: checked,
            read_analysis_and_probe_wall_s: submitted - started,
            sample_guard_wall_s: checked - submitted,
            source_age_at_start_s: source.map(|t| started - t),
            source_age_at_check_s: source.map(|t| checked - t),
            acquisition_deadline_s: source.map(|t| t + policy.max_age_s),
            deadline_missed_at_check: source.map(|t| checked > t + policy.max_age_s),
        },
        guard_events: [
            Dispatch { event, output },
            Dispatch {
                event: poll,
                output: final_output,
            },
        ],
    })
}

#[derive(Serialize)]
pub struct EventReport {
    source: Slot,
    measured: Measured<camera_analysis::Report>,
}
#[derive(Serialize)]
pub struct Summary {
    image_events: usize,
    inferred_images: usize,
    skipped_images: usize,
    inferred_but_late_images: usize,
    late_at_check_images: usize,
    available_at_check_images: usize,
    maximum_source_age_at_check_s: Option<f64>,
    deadline_gate_pass: Option<bool>,
}
#[derive(Serialize)]
pub struct Report {
    schema_version: u32,
    algorithm: &'static str,
    compiled_sources: BTreeMap<&'static str, String>,
    source_sequence_sha256: String,
    camera_id: String,
    width: u32,
    height: u32,
    config: image_tracking::Config,
    policy: Policy,
    clock: &'static str,
    control_permission: bool,
    manifest_and_preflight_wall_s: f64,
    model_load_wall_s: f64,
    startup_before_schedule_wall_s: f64,
    schedule_wall_s: f64,
    model_loads: usize,
    preflight_image_reads: usize,
    inference_image_reads: usize,
    model_forwards: usize,
    summary: Summary,
    events: Vec<EventReport>,
}
pub fn run(
    sequence: &Sequence,
    slots: Vec<Slot>,
    base: &Path,
    model: &Path,
    source_sha: String,
    startup: Instant,
    preflight_wall_s: f64,
) -> Result<Report, String> {
    bindings(sequence, &slots)?;
    let policy = Policy::default();
    let mut guard = Guard::new(
        sequence.width,
        sequence.height,
        sequence.config.clone(),
        policy.clone(),
    )?;
    let images = image_count(&slots);
    let load_started = Instant::now();
    let detector = if images > 0 {
        Some(yolox_s::Detector::load(model).map_err(|e| format!("load model: {e}"))?)
    } else {
        None
    };
    let load_wall_s = if detector.is_some() {
        load_started.elapsed().as_secs_f64()
    } else {
        0.
    };
    let mut events = Vec::with_capacity(sequence.events.len());
    let startup_wall_s = startup.elapsed().as_secs_f64();
    let mut clock = WallClock::new();
    let mut forwards = 0;
    for (index, (event, slot)) in sequence.events.iter().zip(slots).enumerate() {
        clock.wait_until(slot.release_offset_s);
        let input = match &event.input {
            camera_sequence::Input::Image { sha256, .. } => Input::Sample {
                acquired_s: slot.release_offset_s,
                image_sha256: sha256.clone(),
                detections: vec![],
            },
            camera_sequence::Input::Missing => Input::Tick,
            camera_sequence::Input::Failed => Input::Failed,
        };
        let measured = process(&mut guard, &mut clock, input, &policy, || {
            let camera_sequence::Input::Image { path, sha256 } = &event.input else {
                return Err("non-image inference requested".into());
            };
            let (image, sha) = camera_analysis::read_image(&base.join(path))?;
            if sha != *sha256
                || (image.width(), image.height()) != (sequence.width, sequence.height)
            {
                return Err("inference image changed after preflight".into());
            }
            let report = camera_analysis::analyze(
                detector.as_ref().ok_or("missing model plan")?,
                &image,
                sha,
            )?;
            let detections = report
                .result
                .detections
                .iter()
                .map(|d| Detection {
                    class_index: d.class_index,
                    confidence: f64::from(d.confidence),
                    bbox: d.bbox.map(f64::from),
                })
                .collect();
            forwards += report.execution.model_forwards;
            Ok((report, detections))
        })
        .map_err(|e| format!("event {index}: {e}"))?;
        events.push(EventReport {
            source: slot,
            measured,
        });
    }
    let schedule_wall_s = clock.origin.elapsed().as_secs_f64();
    let image_events: Vec<_> = events
        .iter()
        .filter(|event| event.source.source_image_sha256.is_some())
        .collect();
    let inferred = image_events
        .iter()
        .filter(|event| event.measured.analysis.is_some())
        .count();
    let late = |event: &EventReport| event.measured.timing.deadline_missed_at_check == Some(true);
    let late_images = image_events.iter().filter(|event| late(event)).count();
    let summary = Summary {
        image_events: images,
        inferred_images: inferred,
        skipped_images: images - inferred,
        inferred_but_late_images: image_events
            .iter()
            .filter(|e| e.measured.analysis.is_some() && late(e))
            .count(),
        late_at_check_images: late_images,
        available_at_check_images: image_events
            .iter()
            .filter(|e| e.measured.guard_events[1].output.ready)
            .count(),
        maximum_source_age_at_check_s: image_events
            .iter()
            .filter_map(|e| e.measured.timing.source_age_at_check_s)
            .reduce(f64::max),
        deadline_gate_pass: if images > 0 {
            Some(late_images == 0)
        } else {
            None
        },
    };
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_s_measured_serial_replay_v1",
        compiled_sources: compiled_sources(),
        source_sequence_sha256: source_sha,
        camera_id: sequence.camera_id.clone(),
        width: sequence.width,
        height: sequence.height,
        config: sequence.config.clone(),
        policy,
        clock: "std::time::Instant_serial_replay",
        control_permission: false,
        manifest_and_preflight_wall_s: preflight_wall_s,
        model_load_wall_s: load_wall_s,
        startup_before_schedule_wall_s: startup_wall_s,
        schedule_wall_s,
        model_loads: usize::from(detector.is_some()),
        preflight_image_reads: images,
        inference_image_reads: inferred,
        model_forwards: forwards,
        summary,
        events,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    struct Scripted(VecDeque<f64>);
    impl Clock for Scripted {
        fn checkpoint(&mut self) -> Result<f64, String> {
            self.0.pop_front().ok_or("clock exhausted".into())
        }
    }
    fn times(values: &[f64]) -> Scripted {
        Scripted(values.iter().copied().collect())
    }
    fn guard() -> Guard {
        Guard::new(
            100,
            100,
            image_tracking::Config::default(),
            Policy::default(),
        )
        .unwrap()
    }
    fn input(time: f64, id: u64) -> Input {
        Input::Sample {
            acquired_s: time,
            image_sha256: format!("{id:064x}"),
            detections: vec![],
        }
    }
    fn analysis() -> Result<((), Vec<Detection>), String> {
        Ok((
            (),
            vec![Detection {
                class_index: 2,
                confidence: 0.8,
                bbox: [10., 10., 30., 30.],
            }],
        ))
    }
    fn measured(guard: &mut Guard, time: f64, id: u64, values: &[f64]) -> Measured<()> {
        process(
            guard,
            &mut times(values),
            input(time, id),
            &Policy::default(),
            analysis,
        )
        .unwrap()
    }
    #[test]
    fn fast_frames_recover_once_without_consuming_probe_ids_or_hashes() {
        let mut state = guard();
        assert!(
            !measured(&mut state, 0., 1, &[0.01, 0.03, 0.031]).guard_events[1]
                .output
                .ready
        );
        let second = measured(&mut state, 0.1, 2, &[0.11, 0.12, 0.121]);
        assert!(second.guard_events[1].output.ready);
        assert_eq!(second.guard_events[1].output.tracks[0].track_id, 1);
        assert_eq!(second.guard_events[1].output.recovery_count, 2);
    }
    #[test]
    fn analysis_crossing_deadline_is_measured_and_never_accepted_as_fresh() {
        let result = measured(&mut guard(), 0., 1, &[0.01, 0.25, 0.251]);
        assert!(result.analysis.is_some());
        assert_eq!(result.precheck_decision, Some(Decision::Accepted));
        assert_eq!(
            result.guard_events[0].output.decision,
            Decision::StaleSample
        );
        assert_eq!(result.timing.deadline_missed_at_check, Some(true));
        assert!(!result.guard_events[1].output.ready);
    }
    #[test]
    fn already_old_images_skip_the_analyzer_entirely() {
        let result: Measured<()> = process(
            &mut guard(),
            &mut times(&[0.21, 0.211, 0.212]),
            input(0., 1),
            &Policy::default(),
            || panic!("stale image must not invoke analyzer"),
        )
        .unwrap();
        assert!(result.analysis.is_none());
        assert_eq!(result.precheck_decision, Some(Decision::StaleSample));
    }
    #[test]
    fn tracking_work_crossing_deadline_clears_a_provisionally_available_result() {
        let mut state = guard();
        measured(&mut state, 0., 1, &[0.001, 0.01, 0.011]);
        measured(&mut state, 0.05, 2, &[0.051, 0.06, 0.061]);
        let result = measured(&mut state, 0.1, 3, &[0.11, 0.12, 0.31]);
        assert!(result.guard_events[0].output.ready);
        assert!(!result.guard_events[1].output.ready);
        assert_eq!(
            result.guard_events[1].output.hold_reason,
            Some(camera_freshness::HoldReason::DeadlineExceeded)
        );
        assert_eq!(
            result.guard_events[1].output.backend_updates[0]
                .snapshot
                .retired[0]
                .track_id,
            1
        );
    }
    #[test]
    fn expired_backlog_cannot_resurrect_ids_and_two_new_frames_are_required() {
        let mut state = guard();
        measured(&mut state, 0., 1, &[0.001, 0.01, 0.011]);
        measured(&mut state, 0.05, 2, &[0.051, 0.06, 0.061]);
        let stale = measured(&mut state, 0.1, 3, &[0.4, 0.401, 0.402]);
        assert!(stale.analysis.is_none());
        assert_eq!(stale.guard_events[0].output.decision, Decision::BeforeFault);
        assert!(
            !measured(&mut state, 0.41, 4, &[0.411, 0.42, 0.421]).guard_events[1]
                .output
                .ready
        );
        let recovered = measured(&mut state, 0.45, 5, &[0.451, 0.46, 0.461]);
        assert!(recovered.guard_events[1].output.ready);
        assert_eq!(recovered.guard_events[1].output.tracks[0].track_id, 2);
    }
    #[test]
    fn clock_and_analyzer_failures_leave_complete_guard_state_unchanged() {
        let mut state = guard();
        let before = state.clone();
        for values in [
            [0.01, 0.03, 0.03],
            [0.01, 0.03, f64::NAN],
            [0.01, 0.03, 0.02],
        ] {
            assert!(
                process(
                    &mut state,
                    &mut times(&values),
                    input(0., 1),
                    &Policy::default(),
                    analysis
                )
                .is_err()
            );
            assert_eq!(state, before);
        }
        let failure = process::<()>(
            &mut state,
            &mut times(&[0.01]),
            input(0., 1),
            &Policy::default(),
            || Err("decoder failed".into()),
        );
        assert!(failure.is_err());
        assert_eq!(state, before);
    }
    #[test]
    fn polls_and_failure_do_not_invoke_analysis() {
        let result: Measured<()> = process(
            &mut guard(),
            &mut times(&[0.01, 0.02, 0.03]),
            Input::Failed,
            &Policy::default(),
            || panic!("non-image event must not invoke analyzer"),
        )
        .unwrap();
        assert!(result.analysis.is_none());
        assert_eq!(result.guard_events[1].output.fault_epoch_s, Some(0.02));
    }
    #[test]
    fn original_epoch_offsets_and_schedule_bound_are_explicit() {
        fn sequence(end: &str) -> Sequence {
            serde_json::from_str(&format!(r#"{{"schema_version":1,"camera_id":"clock","width":100,"height":100,"events":[{{"timestamp_s":"1341845824.165751","input":{{"kind":"missing"}}}},{{"timestamp_s":"{end}","input":{{"kind":"failed"}}}}]}}"#)).unwrap()
        }
        let input = sequence("1341845824.197930");
        let checked = camera_sequence::preflight(&input, Path::new("missing")).unwrap();
        let slots = plan(&input, &checked).unwrap();
        assert_eq!(slots[0].release_offset_s, 0.);
        assert_eq!(
            slots[1].release_offset_s,
            "1341845824.197930".parse::<f64>().unwrap()
                - "1341845824.165751".parse::<f64>().unwrap()
        );
        let too_long = sequence("1341845826.165752");
        let checked = camera_sequence::preflight(&too_long, Path::new("missing")).unwrap();
        assert!(plan(&too_long, &checked).is_err());
    }
}
