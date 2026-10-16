//! Offline camera inference with an independently polled, original freshness guard.
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
    sync::mpsc::{self, RecvTimeoutError},
    thread,
    time::{Duration, Instant},
};

pub const POLL_S: f64 = 0.01;
const WALL_BUDGET_S: f64 = 10.;
const TRACE_BUDGET: usize = 8192;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Slot {
    pub event_index: usize,
    source_timestamp_text: String,
    source_timestamp_s: f64,
    pub release_offset_s: f64,
    source_image_sha256: Option<String>,
}
pub fn plan(sequence: &Sequence, checked: &[CheckedEvent]) -> Result<Vec<Slot>, String> {
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
            "integrations/onnx/src/camera_supervised.rs",
            include_bytes!("camera_supervised.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-supervised.rs",
            include_bytes!("bin/rustdriving-camera-supervised.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-supervised.json",
            include_bytes!("../profile-supervised.json").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

#[derive(Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum Change<T> {
    Source {
        event_index: usize,
    },
    Superseded {
        event_index: usize,
        reason: &'static str,
    },
    Started {
        event_index: usize,
        precheck_decision: Decision,
    },
    Excluded {
        event_index: usize,
        precheck_decision: Decision,
    },
    Finished {
        event_index: usize,
        worker_started_s: f64,
        worker_finished_s: f64,
        analysis: Option<T>,
        error: Option<String>,
    },
    Guard {
        reason: &'static str,
        event_index: Option<usize>,
        event: Event,
        output: Output,
    },
}
#[derive(Serialize)]
struct Record<T> {
    observed_s: f64,
    active_source_index: Option<usize>,
    change: Change<T>,
}
struct State<T> {
    guard: Guard,
    pending: Option<usize>,
    active: Option<(usize, f64)>,
    next_source: usize,
    last_observed: Option<f64>,
    trace: Vec<Record<T>>,
    last_output: Option<Output>,
}
impl<T> State<T> {
    fn new(sequence: &Sequence) -> Result<Self, String> {
        Ok(Self {
            guard: Guard::new(
                sequence.width,
                sequence.height,
                sequence.config.clone(),
                Policy::default(),
            )?,
            pending: None,
            active: None,
            next_source: 0,
            last_observed: None,
            trace: vec![],
            last_output: None,
        })
    }
    fn record(&mut self, now: f64, change: Change<T>) -> Result<(), String> {
        if !now.is_finite()
            || !(0.0..=WALL_BUDGET_S).contains(&now)
            || self.last_observed.is_some_and(|last| now < last)
            || self.trace.len() >= TRACE_BUDGET
        {
            return Err("invalid/regressed supervisor clock or exhausted trace budget".into());
        }
        self.last_observed = Some(now);
        self.trace.push(Record {
            observed_s: now,
            active_source_index: self.active.map(|a| a.0),
            change,
        });
        Ok(())
    }
    fn apply(
        &mut self,
        now: f64,
        input: Input,
        reason: &'static str,
        index: Option<usize>,
    ) -> Result<(), String> {
        let event = Event { now_s: now, input };
        let output = self.guard.step(&event)?;
        self.last_output = Some(output.clone());
        self.record(
            now,
            Change::Guard {
                reason,
                event_index: index,
                event,
                output,
            },
        )
    }
    fn drop_pending(&mut self, now: f64, reason: &'static str) -> Result<(), String> {
        if let Some(event_index) = self.pending.take() {
            self.record(
                now,
                Change::Superseded {
                    event_index,
                    reason,
                },
            )?;
        }
        Ok(())
    }
    fn release(
        &mut self,
        sequence: &Sequence,
        slots: &[Slot],
        index: usize,
        now: f64,
    ) -> Result<(), String> {
        if index != self.next_source || now < slots[index].release_offset_s {
            return Err("source must be released in order, never from the future".into());
        }
        self.record(now, Change::Source { event_index: index })?;
        match &sequence.events[index].input {
            camera_sequence::Input::Image { .. } => {
                self.drop_pending(now, "newer_released_image")?;
                self.pending = Some(index);
            }
            other => {
                self.drop_pending(now, "source_barrier")?;
                self.apply(
                    now,
                    if matches!(other, camera_sequence::Input::Failed) {
                        Input::Failed
                    } else {
                        Input::Tick
                    },
                    "source_barrier",
                    Some(index),
                )?;
            }
        }
        self.next_source += 1;
        Ok(())
    }
    fn input(
        sequence: &Sequence,
        slots: &[Slot],
        index: usize,
        detections: Vec<Detection>,
    ) -> Result<Input, String> {
        let camera_sequence::Input::Image { sha256, .. } = &sequence.events[index].input else {
            return Err("image required".into());
        };
        Ok(Input::Sample {
            acquired_s: slots[index].release_offset_s,
            image_sha256: sha256.clone(),
            detections,
        })
    }
    fn start(
        &mut self,
        sequence: &Sequence,
        slots: &[Slot],
        now: f64,
    ) -> Result<Option<usize>, String> {
        if self.active.is_some() {
            return Ok(None);
        }
        let Some(index) = self.pending.take() else {
            return Ok(None);
        };
        let input = Self::input(sequence, slots, index, vec![])?;
        let decision = self
            .guard
            .clone()
            .step(&Event {
                now_s: now,
                input: input.clone(),
            })?
            .decision;
        if decision == Decision::Accepted {
            self.active = Some((index, now));
            self.record(
                now,
                Change::Started {
                    event_index: index,
                    precheck_decision: decision,
                },
            )?;
            Ok(Some(index))
        } else {
            self.record(
                now,
                Change::Excluded {
                    event_index: index,
                    precheck_decision: decision,
                },
            )?;
            self.apply(now, input, "excluded_image", Some(index))?;
            Ok(None)
        }
    }
    fn finish(
        &mut self,
        sequence: &Sequence,
        slots: &[Slot],
        work: Work<T>,
        now: f64,
    ) -> Result<(), String> {
        let (index, dispatch) = self.active.ok_or("unsolicited inference completion")?;
        if index != work.index
            || !work.started.is_finite()
            || !work.finished.is_finite()
            || work.started < dispatch
            || work.finished < work.started
            || now < work.finished
        {
            return Err("inference completion identity or measured clocks differ".into());
        }
        let (analysis, error, input) = match work.result {
            Ok((report, detections)) => (
                Some(report),
                None,
                Self::input(sequence, slots, index, detections)?,
            ),
            Err(error) => (None, Some(error), Input::Failed),
        };
        self.record(
            now,
            Change::Finished {
                event_index: index,
                worker_started_s: work.started,
                worker_finished_s: work.finished,
                analysis,
                error,
            },
        )?;
        self.active = None;
        self.apply(now, input, "worker_result", Some(index))
    }
    fn drained_until(&self, slots: &[Slot]) -> f64 {
        let end = slots.last().map_or(0., |s| s.release_offset_s);
        self.last_output
            .as_ref()
            .and_then(|o| o.last_accepted_acquired_s)
            .map_or(end, |a| end.max(a + Policy::default().max_age_s))
            + POLL_S
    }
}
struct Work<T> {
    index: usize,
    started: f64,
    finished: f64,
    result: Result<(T, Vec<Detection>), String>,
}
struct Clock {
    origin: Instant,
    last: Option<f64>,
}
impl Clock {
    fn checkpoint(&mut self) -> Result<f64, String> {
        for _ in 0..128 {
            let now = self.origin.elapsed().as_secs_f64();
            if !now.is_finite() || now > WALL_BUDGET_S {
                return Err("supervisor wall-clock budget exhausted".into());
            }
            if self.last.is_none_or(|last| now - last >= 0.000001) {
                self.last = Some(now);
                return Ok(now);
            }
            thread::yield_now();
        }
        Err("host monotonic clock did not advance one microsecond".into())
    }
}

// The worker cannot access the guard. No channel can queue a second active job.
fn supervise<T: Send>(
    sequence: &Sequence,
    slots: &[Slot],
    mut analyze: impl FnMut(usize) -> Result<(T, Vec<Detection>), String> + Send,
) -> Result<(State<T>, f64), String> {
    let mut state = State::new(sequence)?;
    let mut clock = Clock {
        origin: Instant::now(),
        last: None,
    };
    let origin = clock.origin;
    let (commands, receive) = mpsc::sync_channel::<usize>(1);
    let (results, completed) = mpsc::sync_channel::<Work<T>>(1);
    thread::scope(|scope| {
        let worker = scope.spawn(move || {
            while let Ok(index) = receive.recv() {
                let started = origin.elapsed().as_secs_f64();
                let result = analyze(index);
                let finished = origin.elapsed().as_secs_f64();
                if results
                    .send(Work {
                        index,
                        started,
                        finished,
                        result,
                    })
                    .is_err()
                {
                    break;
                }
            }
        });
        let result: Result<(), String> = (|| {
            let mut completion = None;
            let mut next_poll = 0.;
            loop {
                let mut now = clock.checkpoint()?;
                // Due failures precede result delivery, even if both wake this loop.
                while state.next_source < slots.len()
                    && slots[state.next_source].release_offset_s <= now
                {
                    state.release(sequence, slots, state.next_source, now)?;
                    now = clock.checkpoint()?;
                }
                if now >= next_poll {
                    state.apply(now, Input::Tick, "periodic_poll", None)?;
                    next_poll = now + POLL_S;
                    now = clock.checkpoint()?;
                }
                if slots
                    .get(state.next_source)
                    .is_some_and(|s| s.release_offset_s <= now)
                {
                    continue;
                }
                if let Some(work) = completion.take() {
                    state.finish(sequence, slots, work, now)?;
                    now = clock.checkpoint()?;
                    state.apply(now, Input::Tick, "post_tracking_poll", None)?;
                    now = clock.checkpoint()?;
                }
                if slots
                    .get(state.next_source)
                    .is_some_and(|s| s.release_offset_s <= now)
                {
                    continue;
                }
                if state.active.is_none() && state.pending.is_some() {
                    if let Some(index) = state.start(sequence, slots, now)? {
                        commands
                            .send(index)
                            .map_err(|_| "inference worker disconnected")?;
                    }
                    now = clock.checkpoint()?;
                }
                if state.next_source == slots.len()
                    && state.pending.is_none()
                    && state.active.is_none()
                    && now > state.drained_until(slots)
                {
                    state.apply(now, Input::Tick, "final_poll", None)?;
                    return Ok(());
                }
                let wake = slots
                    .get(state.next_source)
                    .map_or(next_poll, |s| next_poll.min(s.release_offset_s));
                let wait = Duration::from_secs_f64((wake - now).clamp(0., POLL_S));
                match completed.recv_timeout(wait) {
                    Ok(work) => completion = Some(work),
                    Err(RecvTimeoutError::Timeout) => (),
                    Err(RecvTimeoutError::Disconnected) => {
                        return Err("inference worker terminated before shutdown".into());
                    }
                }
            }
        })();
        drop(commands);
        // Disconnect a completed send before joining on any fatal clock error.
        drop(completed);
        worker.join().map_err(|_| "inference worker panicked")?;
        result
    })?;
    Ok((state, origin.elapsed().as_secs_f64()))
}

#[derive(Serialize)]
pub struct Summary {
    source_images: usize,
    superseded_images: usize,
    excluded_images: usize,
    started_images: usize,
    completed_images: usize,
    worker_errors: usize,
    accepted_result_images: usize,
    available_result_images: usize,
    inferred_but_late_images: usize,
    delivered_source_barriers: usize,
    source_barriers_observed_while_worker_active: usize,
    periodic_polls_while_worker_active: usize,
    max_observed_poll_gap_s: f64,
    max_source_barrier_delay_s: Option<f64>,
    final_ready: bool,
    all_source_results_available: bool,
}
fn summarize<T>(state: &State<T>, slots: &[Slot]) -> Summary {
    let mut s = Summary {
        source_images: image_count(slots),
        superseded_images: 0,
        excluded_images: 0,
        started_images: 0,
        completed_images: 0,
        worker_errors: 0,
        accepted_result_images: 0,
        available_result_images: 0,
        inferred_but_late_images: 0,
        delivered_source_barriers: 0,
        source_barriers_observed_while_worker_active: 0,
        periodic_polls_while_worker_active: 0,
        max_observed_poll_gap_s: 0.,
        max_source_barrier_delay_s: None,
        final_ready: false,
        all_source_results_available: false,
    };
    let mut last_poll = None;
    for row in &state.trace {
        match &row.change {
            Change::Superseded { .. } => s.superseded_images += 1,
            Change::Excluded { .. } => s.excluded_images += 1,
            Change::Started { .. } => s.started_images += 1,
            Change::Finished { error, .. } => {
                s.completed_images += 1;
                s.worker_errors += usize::from(error.is_some());
            }
            Change::Guard {
                reason,
                event_index,
                event,
                output,
            } => {
                if *reason == "source_barrier" {
                    s.delivered_source_barriers += 1;
                    s.source_barriers_observed_while_worker_active +=
                        usize::from(row.active_source_index.is_some());
                    let delay = row.observed_s - slots[event_index.unwrap()].release_offset_s;
                    s.max_source_barrier_delay_s =
                        Some(s.max_source_barrier_delay_s.map_or(delay, |d| d.max(delay)));
                }
                if *reason == "worker_result" {
                    s.accepted_result_images += usize::from(output.decision == Decision::Accepted);
                    if let Input::Sample { acquired_s, .. } = event.input {
                        s.inferred_but_late_images +=
                            usize::from(row.observed_s > acquired_s + Policy::default().max_age_s);
                    }
                }
                if *reason == "post_tracking_poll" {
                    s.available_result_images += usize::from(output.ready);
                }
                if *reason == "periodic_poll" {
                    s.periodic_polls_while_worker_active +=
                        usize::from(row.active_source_index.is_some());
                    if let Some(last) = last_poll {
                        s.max_observed_poll_gap_s =
                            s.max_observed_poll_gap_s.max(row.observed_s - last);
                    }
                    last_poll = Some(row.observed_s);
                }
            }
            _ => (),
        }
    }
    s.final_ready = state.last_output.as_ref().is_some_and(|o| o.ready);
    s.all_source_results_available =
        s.source_images > 0 && s.available_result_images == s.source_images;
    s
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
    poll_period_s: f64,
    in_flight_capacity: usize,
    pending_image_capacity: usize,
    model_loads: usize,
    preflight_image_reads: usize,
    inference_image_reads: usize,
    model_forwards: Option<usize>,
    successful_model_forwards: usize,
    manifest_and_preflight_wall_s: f64,
    model_load_wall_s: f64,
    startup_before_schedule_wall_s: f64,
    schedule_wall_s: f64,
    source_slots: Vec<Slot>,
    summary: Summary,
    trace: Vec<Record<camera_analysis::Report>>,
}
pub fn run(
    sequence: &Sequence,
    slots: Vec<Slot>,
    base: &Path,
    model: &Path,
    sha: String,
    startup: Instant,
    preflight: f64,
) -> Result<Report, String> {
    let images = image_count(&slots);
    let load = Instant::now();
    let detector = if images > 0 {
        Some(yolox_s::Detector::load(model).map_err(|e| format!("load model: {e}"))?)
    } else {
        None
    };
    let load_wall = if detector.is_some() {
        load.elapsed().as_secs_f64()
    } else {
        0.
    };
    let startup_wall = startup.elapsed().as_secs_f64();
    let (state, schedule_wall) = supervise(sequence, &slots, |index| {
        let camera_sequence::Input::Image { path, sha256 } = &sequence.events[index].input else {
            return Err("non-image inference requested".into());
        };
        let (image, hash) = camera_analysis::read_image(&base.join(path))?;
        if hash != *sha256 || (image.width(), image.height()) != (sequence.width, sequence.height) {
            return Err("image changed after preflight".into());
        }
        let report =
            camera_analysis::analyze(detector.as_ref().ok_or("missing model plan")?, &image, hash)?;
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
        Ok((report, detections))
    })?;
    let summary = summarize(&state, &slots);
    let forwards = state
        .trace
        .iter()
        .filter_map(|r| match &r.change {
            Change::Finished {
                analysis: Some(a), ..
            } => Some(a.execution.model_forwards),
            _ => None,
        })
        .sum();
    Ok(Report {
        schema_version: 1,
        algorithm: "yolox_nano_supervised_replay_v1",
        compiled_sources: compiled_sources(),
        source_sequence_sha256: sha,
        model_sha256: yolox_s::MODEL_SHA256,
        camera_id: sequence.camera_id.clone(),
        width: sequence.width,
        height: sequence.height,
        config: sequence.config.clone(),
        policy: Policy::default(),
        clock: "std::time::Instant_supervised_replay",
        control_permission: false,
        poll_period_s: POLL_S,
        in_flight_capacity: 1,
        pending_image_capacity: 1,
        model_loads: usize::from(detector.is_some()),
        preflight_image_reads: images,
        inference_image_reads: summary.completed_images,
        model_forwards: (summary.worker_errors == 0).then_some(forwards),
        successful_model_forwards: forwards,
        manifest_and_preflight_wall_s: preflight,
        model_load_wall_s: load_wall,
        startup_before_schedule_wall_s: startup_wall,
        schedule_wall_s: schedule_wall,
        source_slots: slots,
        summary,
        trace: state.trace,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(kinds: &[(&str, f64)]) -> (Sequence, Vec<Slot>, State<()>) {
        let events: Vec<_> = kinds.iter().enumerate().map(|(i, (kind, time))| {
            let input = if *kind == "image" { serde_json::json!({"kind":"image","path":format!("{i}.png"),"sha256":format!("{:064x}",i+1)}) }
                else { serde_json::json!({"kind":kind}) };
            serde_json::json!({"timestamp_s":time.to_string(),"input":input})
        }).collect();
        let sequence: Sequence = serde_json::from_value(serde_json::json!({"schema_version":1,"camera_id":"unit","width":100,"height":100,"events":events})).unwrap();
        let slots = kinds
            .iter()
            .enumerate()
            .map(|(i, (kind, time))| Slot {
                event_index: i,
                source_timestamp_text: time.to_string(),
                source_timestamp_s: *time,
                release_offset_s: *time,
                source_image_sha256: if *kind == "image" {
                    Some(format!("{:064x}", i + 1))
                } else {
                    None
                },
            })
            .collect();
        let state = State::new(&sequence).unwrap();
        (sequence, slots, state)
    }
    fn done(index: usize, start: f64, finish: f64) -> Work<()> {
        Work {
            index,
            started: start,
            finished: finish,
            result: Ok(((), vec![])),
        }
    }
    #[test]
    fn fault_is_delivered_while_work_remains_active_and_old_result_cannot_recover() {
        let (q, s, mut g) = fixture(&[("image", 0.), ("image", 0.01), ("failed", 0.02)]);
        g.release(&q, &s, 0, 0.).unwrap();
        assert_eq!(g.start(&q, &s, 0.001).unwrap(), Some(0));
        g.release(&q, &s, 1, 0.01).unwrap();
        g.release(&q, &s, 2, 0.02).unwrap();
        assert_eq!(g.active, Some((0, 0.001)));
        assert_eq!(g.pending, None);
        assert_eq!(g.last_output.as_ref().unwrap().fault_epoch_s, Some(0.02));
        g.finish(&q, &s, done(0, 0.002, 0.08), 0.081).unwrap();
        let output = g.last_output.as_ref().unwrap();
        assert_eq!(output.decision, Decision::BeforeFault);
        assert!(!output.ready && output.tracks.is_empty());
        let summary = summarize(&g, &s);
        assert_eq!(summary.source_barriers_observed_while_worker_active, 1);
        assert_eq!(summary.superseded_images, 1);
    }
    #[test]
    fn watchdog_expires_an_accepted_acquisition_during_the_next_forward() {
        let (q, s, mut g) = fixture(&[("image", 0.), ("image", 0.1)]);
        g.release(&q, &s, 0, 0.).unwrap();
        g.start(&q, &s, 0.001).unwrap();
        g.finish(&q, &s, done(0, 0.002, 0.03), 0.031).unwrap();
        g.release(&q, &s, 1, 0.1).unwrap();
        g.start(&q, &s, 0.101).unwrap();
        g.apply(0.20001, Input::Tick, "periodic_poll", None)
            .unwrap();
        assert_eq!(g.last_output.as_ref().unwrap().fault_epoch_s, Some(0.2));
        assert_eq!(g.active, Some((1, 0.101)));
        g.finish(&q, &s, done(1, 0.102, 0.21), 0.211).unwrap();
        assert_eq!(
            g.last_output.as_ref().unwrap().decision,
            Decision::BeforeFault
        );
        assert_eq!(
            g.last_output.as_ref().unwrap().last_accepted_acquired_s,
            Some(0.)
        );
    }
    #[test]
    fn exactly_two_strictly_post_fault_acquisitions_recover_then_final_drain_expires() {
        let (q, s, mut g) = fixture(&[("failed", 0.), ("image", 0.05), ("image", 0.1)]);
        g.release(&q, &s, 0, 0.).unwrap();
        for (index, now) in [(1, 0.05), (2, 0.1)] {
            g.release(&q, &s, index, now).unwrap();
            g.start(&q, &s, now + 0.001).unwrap();
            g.finish(&q, &s, done(index, now + 0.002, now + 0.01), now + 0.011)
                .unwrap();
            assert_eq!(g.last_output.as_ref().unwrap().ready, index == 2);
            assert_eq!(g.last_output.as_ref().unwrap().recovery_count, index as u32);
        }
        assert!(g.drained_until(&s) > 0.3);
        g.apply(0.3101, Input::Tick, "final_poll", None).unwrap();
        assert!(!g.last_output.as_ref().unwrap().ready);
    }
    #[test]
    fn bounded_pending_replacement_neither_renews_acquisition_nor_counts_recovery() {
        let (q, s, mut g) = fixture(&[("image", 0.), ("image", 0.01), ("image", 0.02)]);
        g.release(&q, &s, 0, 0.).unwrap();
        g.start(&q, &s, 0.001).unwrap();
        g.release(&q, &s, 1, 0.01).unwrap();
        g.release(&q, &s, 2, 0.02).unwrap();
        assert_eq!(g.pending, Some(2));
        assert!(g.start(&q, &s, 0.021).unwrap().is_none());
        assert!(g.last_output.is_none());
        g.finish(&q, &s, done(0, 0.002, 0.04), 0.041).unwrap();
        assert_eq!(g.start(&q, &s, 0.042).unwrap(), Some(2));
        g.finish(&q, &s, done(2, 0.043, 0.06), 0.061).unwrap();
        assert_eq!(
            g.last_output.as_ref().unwrap().last_accepted_acquired_s,
            Some(0.02)
        );
        assert_eq!(summarize(&g, &s).superseded_images, 1);
    }
    #[test]
    fn worker_error_latches_failure_without_a_successful_detection() {
        let (q, s, mut g) = fixture(&[("image", 0.)]);
        g.release(&q, &s, 0, 0.).unwrap();
        g.start(&q, &s, 0.001).unwrap();
        let mut w = done(0, 0.002, 0.03);
        w.result = Err("changed input".into());
        g.finish(&q, &s, w, 0.031).unwrap();
        let out = g.last_output.as_ref().unwrap();
        assert_eq!(out.decision, Decision::AcquisitionFailed);
        assert!(out.last_accepted_acquired_s.is_none() && !out.ready && out.tracks.is_empty());
        assert_eq!(summarize(&g, &s).worker_errors, 1);
    }
    #[test]
    fn stale_pending_image_is_excluded_without_forward_or_fake_acquisition() {
        let (q, s, mut g) = fixture(&[("image", 0.)]);
        g.release(&q, &s, 0, 0.).unwrap();
        assert_eq!(g.start(&q, &s, 0.21).unwrap(), None);
        assert_eq!(
            g.last_output.as_ref().unwrap().decision,
            Decision::StaleSample
        );
        assert_eq!(summarize(&g, &s).started_images, 0);
    }
    #[test]
    fn future_sources_unsolicited_completions_and_wrong_worker_clocks_are_fatal() {
        let (q, s, mut g) = fixture(&[("image", 0.1)]);
        assert!(g.release(&q, &s, 0, 0.09).is_err());
        assert!(g.finish(&q, &s, done(0, 0.1, 0.11), 0.12).is_err());
        g.release(&q, &s, 0, 0.1).unwrap();
        g.start(&q, &s, 0.101).unwrap();
        for (i, start, finish, now) in [
            (1, 0.102, 0.11, 0.12),
            (0, 0.1, 0.11, 0.12),
            (0, 0.102, 0.11, 0.105),
            (0, f64::NAN, 0.11, 0.12),
        ] {
            assert!(g.finish(&q, &s, done(i, start, finish), now).is_err());
        }
    }
    #[test]
    fn missing_source_is_a_delivered_tick_and_drops_only_pending_work() {
        let (q, s, mut g) = fixture(&[("image", 0.), ("missing", 0.01)]);
        g.release(&q, &s, 0, 0.).unwrap();
        g.release(&q, &s, 1, 0.01).unwrap();
        assert!(g.active.is_none() && g.pending.is_none());
        let out = g.last_output.as_ref().unwrap();
        assert_eq!(out.decision, Decision::Polled);
        assert_eq!(out.fault_epoch_s, None);
        assert!(!out.ready);
    }
}
