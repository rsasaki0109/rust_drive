//! Bounded latest-ready selection with the original measured frame controller.
use super::{
    camera_analysis, camera_freshness, camera_latency, camera_sequence, image_tracking, yolox_s,
};
use camera_freshness::{Decision, Event, Guard, Input, Output, Policy};
use camera_sequence::{CheckedEvent, Sequence};
use image_tracking::Detection;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    path::Path,
    thread,
    time::{Duration, Instant},
};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Slot {
    event_index: usize,
    source_timestamp_text: String,
    source_timestamp_s: f64,
    release_offset_s: f64,
    source_image_sha256: Option<String>,
}
pub fn plan(sequence: &Sequence, checked: &[CheckedEvent]) -> Result<Vec<Slot>, String> {
    // Internal conversion retains the validated original schedule exactly.
    serde_json::from_value(
        serde_json::to_value(camera_latency::plan(sequence, checked)?)
            .map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())
}
pub fn image_count(slots: &[Slot]) -> usize {
    slots
        .iter()
        .filter(|s| s.source_image_sha256.is_some())
        .count()
}
pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = camera_latency::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/camera_latest.rs",
            include_bytes!("camera_latest.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-latest.rs",
            include_bytes!("bin/rustdriving-camera-latest.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-latest.json",
            include_bytes!("../profile-latest.json").as_slice(),
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

// Non-image events are barriers: dispatch the first due barrier before any
// queued image, retaining that event and discarding only its preceding images.
fn choose(slots: &[Slot], next: usize, now: f64) -> Result<usize, String> {
    if !now.is_finite()
        || !(0.0..=1e12).contains(&now)
        || next >= slots.len()
        || now < slots[next].release_offset_s
    {
        return Err("selection requires a released source and finite observed clock".into());
    }
    let mut selected = next;
    for (index, slot) in slots.iter().enumerate().skip(next) {
        if slot.release_offset_s > now {
            break;
        }
        selected = index;
        if slot.source_image_sha256.is_none() {
            break;
        }
    }
    Ok(selected)
}
trait ReplayClock: Clock {
    fn wait_for(&mut self, offset: f64);
}
impl ReplayClock for WallClock {
    fn wait_for(&mut self, offset: f64) {
        self.wait_until(offset);
    }
}
#[derive(Serialize)]
pub struct EventReport<T> {
    selection_checked_s: f64,
    source: Slot,
    superseded_sources: Vec<Slot>,
    measured: Measured<T>,
}
fn replay<T>(
    sequence: &Sequence,
    slots: &[Slot],
    guard: &mut Guard,
    clock: &mut impl ReplayClock,
    mut analyze: impl FnMut(usize) -> Result<(T, Vec<Detection>), String>,
) -> Result<Vec<EventReport<T>>, String> {
    let mut events = Vec::new();
    let mut next = 0;
    let mut previous = None;
    let policy = Policy::default();
    while next < slots.len() {
        clock.wait_for(slots[next].release_offset_s);
        let selection = clock.checkpoint()?;
        if let Some(last) = previous {
            advances(last, selection)?;
        }
        let index = choose(slots, next, selection)?;
        let slot = &slots[index];
        let input = match &sequence.events[index].input {
            camera_sequence::Input::Image { sha256, .. } => Input::Sample {
                acquired_s: slot.release_offset_s,
                image_sha256: sha256.clone(),
                detections: vec![],
            },
            camera_sequence::Input::Missing => Input::Tick,
            camera_sequence::Input::Failed => Input::Failed,
        };
        let mut candidate = guard.clone();
        let measured = process(&mut candidate, clock, input, &policy, || analyze(index))
            .map_err(|e| format!("event {index}: {e}"))?;
        advances(selection, measured.timing.processing_started_s)?;
        previous = Some(measured.timing.checked_after_tracking_s);
        *guard = candidate;
        events.push(EventReport {
            selection_checked_s: selection,
            source: slot.clone(),
            superseded_sources: slots[next..index].to_vec(),
            measured,
        });
        next = index + 1;
    }
    Ok(events)
}
#[derive(Serialize)]
pub struct Summary {
    source_images: usize,
    dispatched_images: usize,
    superseded_images: usize,
    inferred_images: usize,
    inference_excluded_images: usize,
    inferred_but_late_images: usize,
    late_dispatched_images: usize,
    available_at_dispatched_checks: usize,
    dispatched_deadline_pass: Option<bool>,
    all_source_images_inferred: bool,
    all_source_results_available: bool,
}
fn summarize<T>(events: &[EventReport<T>], images: usize) -> Summary {
    let dispatched: Vec<_> = events
        .iter()
        .filter(|e| e.source.source_image_sha256.is_some())
        .collect();
    let inferred = dispatched
        .iter()
        .filter(|e| e.measured.analysis.is_some())
        .count();
    let late = dispatched
        .iter()
        .filter(|e| e.measured.timing.deadline_missed_at_check == Some(true))
        .count();
    let available = dispatched
        .iter()
        .filter(|e| e.measured.guard_events[1].output.ready)
        .count();
    Summary {
        source_images: images,
        dispatched_images: dispatched.len(),
        superseded_images: events.iter().map(|e| e.superseded_sources.len()).sum(),
        inferred_images: inferred,
        inference_excluded_images: dispatched.len() - inferred,
        inferred_but_late_images: dispatched
            .iter()
            .filter(|e| {
                e.measured.analysis.is_some()
                    && e.measured.timing.deadline_missed_at_check == Some(true)
            })
            .count(),
        late_dispatched_images: late,
        available_at_dispatched_checks: available,
        dispatched_deadline_pass: (!dispatched.is_empty()).then_some(late == 0),
        all_source_images_inferred: images > 0 && inferred == images,
        all_source_results_available: images > 0 && available == images,
    }
}
#[derive(Serialize)]
pub struct Report {
    schema_version: u32,
    algorithm: &'static str,
    compiled_sources: BTreeMap<&'static str, String>,
    source_sequence_sha256: String,
    model_sha256: &'static str,
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
    source_slots: Vec<Slot>,
    summary: Summary,
    events: Vec<EventReport<camera_analysis::Report>>,
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
    let startup_wall_s = startup.elapsed().as_secs_f64();
    let mut clock = WallClock::new();
    let mut forwards = 0;
    let events = replay(sequence, &slots, &mut guard, &mut clock, |index| {
        let camera_sequence::Input::Image { path, sha256 } = &sequence.events[index].input else {
            return Err("non-image inference requested".into());
        };
        let (image, sha) = camera_analysis::read_image(&base.join(path))?;
        if sha != *sha256 || (image.width(), image.height()) != (sequence.width, sequence.height) {
            return Err("inference image changed after preflight".into());
        }
        let report =
            camera_analysis::analyze(detector.as_ref().ok_or("missing model plan")?, &image, sha)?;
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
    })?;
    let summary = summarize(&events, images);
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_nano_latest_ready_replay_v1",
        compiled_sources: compiled_sources(),
        source_sequence_sha256: source_sha,
        model_sha256: yolox_s::MODEL_SHA256,
        camera_id: sequence.camera_id.clone(),
        width: sequence.width,
        height: sequence.height,
        config: sequence.config.clone(),
        policy,
        clock: "std::time::Instant_latest_ready_replay",
        control_permission: false,
        manifest_and_preflight_wall_s: preflight_wall_s,
        model_load_wall_s: load_wall_s,
        startup_before_schedule_wall_s: startup_wall_s,
        schedule_wall_s: clock.origin.elapsed().as_secs_f64(),
        model_loads: usize::from(detector.is_some()),
        preflight_image_reads: images,
        inference_image_reads: summary.inferred_images,
        model_forwards: forwards,
        source_slots: slots,
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
            self.0.pop_front().ok_or("script exhausted".into())
        }
    }
    impl ReplayClock for Scripted {
        fn wait_for(&mut self, _: f64) {}
    }
    fn state() -> Guard {
        Guard::new(
            100,
            100,
            image_tracking::Config::default(),
            Policy::default(),
        )
        .unwrap()
    }
    fn fixture(kinds: &[&str]) -> (Sequence, Vec<Slot>) {
        let events: Vec<_> = kinds.iter().enumerate().map(|(i, kind)| {
            let input = if *kind == "image" {
                serde_json::json!({"kind":"image","path":format!("{i}.png"),"sha256":format!("{:064x}",i+1)})
            } else { serde_json::json!({"kind":kind}) };
            serde_json::json!({"timestamp_s":(i as f64 * 0.033).to_string(),"input":input})
        }).collect();
        let sequence: Sequence =
            serde_json::from_value(serde_json::json!({"schema_version":1,"camera_id":"test",
            "width":100,"height":100,"events":events}))
            .unwrap();
        let slots = sequence
            .events
            .iter()
            .enumerate()
            .map(|(i, event)| Slot {
                event_index: i,
                source_timestamp_text: event.timestamp_s.clone(),
                source_timestamp_s: i as f64 * 0.033,
                release_offset_s: i as f64 * 0.033,
                source_image_sha256: match &event.input {
                    camera_sequence::Input::Image { sha256, .. } => Some(sha256.clone()),
                    _ => None,
                },
            })
            .collect();
        (sequence, slots)
    }
    fn boxes() -> Vec<Detection> {
        vec![Detection {
            class_index: 2,
            confidence: 0.8,
            bbox: [10., 10., 30., 30.],
        }]
    }
    fn times(values: &[f64]) -> Scripted {
        Scripted(values.iter().copied().collect())
    }

    #[test]
    fn latest_is_released_and_each_due_boundary_is_inclusive() {
        let (_, slots) = fixture(&["image"; 8]);
        assert_eq!(choose(&slots, 0, 0.).unwrap(), 0);
        assert_eq!(choose(&slots, 1, slots[3].release_offset_s).unwrap(), 3);
        assert_eq!(
            choose(&slots, 1, slots[3].release_offset_s - 0.000010).unwrap(),
            2
        );
        assert_eq!(choose(&slots, 6, 1.).unwrap(), 7);
        for value in [f64::NAN, f64::INFINITY, -1., 1e13] {
            assert!(choose(&slots, 0, value).is_err());
        }
        assert!(choose(&slots, 1, 0.).is_err());
        assert!(choose(&slots, 8, 1.).is_err());
    }
    #[test]
    fn due_missing_and_fault_events_are_never_crossed_or_discarded() {
        let (_, slots) = fixture(&["image", "missing", "image", "failed", "image"]);
        assert_eq!(choose(&slots, 0, 1.).unwrap(), 1);
        assert_eq!(choose(&slots, 2, 1.).unwrap(), 3);
        assert_eq!(choose(&slots, 3, 1.).unwrap(), 3);
        assert_eq!(choose(&slots, 4, 1.).unwrap(), 4);
    }
    #[test]
    fn coalescing_reduces_age_without_refreshing_discarded_sources() {
        let (sequence, slots) = fixture(&["image"; 8]);
        let mut calls = Vec::new();
        let rows = replay(
            &sequence,
            &slots,
            &mut state(),
            &mut times(&[
                0.000010, 0.000020, 0.080030, 0.080040, 0.080050, 0.080060, 0.160070, 0.160080,
                0.160090, 0.160100, 0.240110, 0.240120, 0.240130, 0.240140, 0.320150, 0.320160,
            ]),
            |i| {
                calls.push(i);
                Ok(((), boxes()))
            },
        )
        .unwrap();
        assert_eq!(calls, vec![0, 2, 4, 7]);
        let s = summarize(&rows, 8);
        assert_eq!(s.superseded_images, 4);
        assert_eq!(s.inferred_images, 4);
        assert_eq!(s.available_at_dispatched_checks, 3);
        assert_eq!(s.dispatched_deadline_pass, Some(true));
        assert!(!s.all_source_results_available && !s.all_source_images_inferred);
        assert_eq!(
            rows[2].measured.guard_events[1]
                .output
                .last_accepted_acquired_s,
            Some(slots[4].release_offset_s)
        );
        assert_eq!(
            rows.iter()
                .flat_map(|r| r.superseded_sources.iter().map(|s| s.event_index))
                .collect::<Vec<_>>(),
            vec![1, 3, 5, 6]
        );
        // Same synthetic 80 ms work with FIFO reaches only one ready result.
        let mut fifo = state();
        let mut ready = 0;
        let mut now = 0.;
        for slot in &slots {
            let start = now + 0.000020_f64;
            let done = start
                + if slot.release_offset_s <= 0.2 {
                    0.080010
                } else {
                    0.000010
                };
            let result = process(
                &mut fifo,
                &mut times(&[start, done, done + 0.000010]),
                Input::Sample {
                    acquired_s: slot.release_offset_s,
                    image_sha256: slot.source_image_sha256.clone().unwrap(),
                    detections: vec![],
                },
                &Policy::default(),
                || Ok(((), boxes())),
            )
            .unwrap();
            ready += usize::from(result.guard_events[1].output.ready);
            now = done + 0.000010;
        }
        assert_eq!(ready, 1);
    }
    #[test]
    fn due_fault_preempts_backlog_and_pre_fault_image_cannot_recover() {
        let (sequence, slots) = fixture(&["image", "image", "failed", "image", "image"]);
        let mut calls = Vec::new();
        let rows = replay(
            &sequence,
            &slots,
            &mut state(),
            &mut times(&[
                0.100010, 0.100020, 0.100030, 0.100040, 0.110010, 0.110020, 0.110030, 0.110040,
                0.140010, 0.140020, 0.150030, 0.150040,
            ]),
            |i| {
                calls.push(i);
                Ok(((), boxes()))
            },
        )
        .unwrap();
        assert_eq!(calls, vec![4]);
        assert_eq!(rows[0].source.event_index, 2);
        assert_eq!(rows[0].superseded_sources.len(), 2);
        assert_eq!(
            rows[0].measured.guard_events[0].output.decision,
            Decision::AcquisitionFailed
        );
        assert_eq!(
            rows[1].measured.precheck_decision,
            Some(Decision::BeforeFault)
        );
        assert_eq!(rows[2].measured.guard_events[1].output.recovery_count, 1);
        assert!(!rows[2].measured.guard_events[1].output.ready);
    }
    #[test]
    fn missing_event_does_not_infer_or_count_as_recovery() {
        let (sequence, slots) = fixture(&["image", "missing", "image"]);
        let rows = replay(
            &sequence,
            &slots,
            &mut state(),
            &mut times(&[
                0.100010, 0.100020, 0.100030, 0.100040, 0.110010, 0.110020, 0.120030, 0.120040,
            ]),
            |_| Ok(((), boxes())),
        )
        .unwrap();
        assert!(rows[0].measured.analysis.is_none());
        assert_eq!(rows[0].measured.guard_events[1].output.recovery_count, 0);
        assert_eq!(rows[1].measured.guard_events[1].output.recovery_count, 1);
        assert!(!rows[1].measured.guard_events[1].output.ready);
    }
    #[test]
    fn no_images_never_invoke_the_analyzer() {
        let (sequence, slots) = fixture(&["missing", "failed"]);
        let rows: Vec<EventReport<()>> = replay(
            &sequence,
            &slots,
            &mut state(),
            &mut times(&[
                0.000010, 0.000020, 0.000030, 0.000040, 0.040010, 0.040020, 0.040030, 0.040040,
            ]),
            |_| panic!("non-image inference"),
        )
        .unwrap();
        let s = summarize(&rows, 0);
        assert_eq!(s.inferred_images, 0);
        assert_eq!(s.dispatched_deadline_pass, None);
        assert!(!s.all_source_results_available);
    }
    #[test]
    fn selection_and_start_clock_failure_do_not_commit_guard_state() {
        let (sequence, slots) = fixture(&["image"]);
        for values in [vec![-1.], vec![0.1, 0.09, 0.11, 0.12]] {
            let mut guard = state();
            let before = guard.clone();
            let result = replay(
                &sequence,
                &slots,
                &mut guard,
                &mut Scripted(values.into()),
                |_| Ok(((), boxes())),
            );
            assert!(result.is_err());
            assert_eq!(guard, before);
        }
        let mut guard = state();
        let before = guard.clone();
        let result: Result<Vec<EventReport<()>>, String> = replay(
            &sequence,
            &slots,
            &mut guard,
            &mut times(&[0.000010, 0.000020]),
            |_| Err("analyzer failed".into()),
        );
        assert!(result.is_err());
        assert_eq!(guard, before);
    }
}
