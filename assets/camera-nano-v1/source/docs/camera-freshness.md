# Camera-result freshness and recovery

`rustdriving-camera-guard` adds separate acquisition and delivery clocks around
the existing image-space tracker. Polling predicts an available track to the
current time without renewing its observation. A delayed result can still be
accepted after a poll when its acquisition time advances and remains fresh.
An expired result or explicit acquisition failure drains active tracks. Queued
pre-fault images cannot restore availability; consecutive fresh acquisitions
after the fault are required. The tracker, detector and their prior source pins
are unchanged.

This is an optional **offline JSON CLI**, outside vehicle sensing/planning.
`ready` describes camera-result availability, including healthy images with
zero detections. It supplies no vehicle-control permission or safety guarantee.

## Run the saved real-report example

```bash
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/camera-tracking/Cargo.toml
mkdir -p artifacts
integrations/camera-tracking/target/release/rustdriving-camera-guard \
  --sequence assets/camera-freshness-v1/verification/cases/recorded-explicit-fault-recovery-input.json \
  --output artifacts/camera-freshness-example.json
```

Choose a new output path for every run. This example consumes eight saved
detector reports from the original TUM RGB indices 102–109, preserving image
identities and original acquisition times. Its receive times, intervening poll,
queued report and explicit failure are **injected controls**, not observed
sensor faults or measured latency. No image, model, depth, pose or ground-truth
file is read; no inference is performed. The derived reports carry
[TUM attribution and CC BY 4.0 terms](../assets/camera-freshness-v1/LICENSE-data.txt).

The failed event clears output and retires existing IDs. Re-delivery of an
image acquired before that failure is excluded. The first new acquisition
remains held; the second restores availability with new IDs. A separate saved
deadline trace intentionally finishes held: only one post-deadline recorded
image is available, which cannot satisfy two-image recovery.

## Input and clock contract

Sequence JSON has `schema_version:1`, `camera_id`, fixed `width`/`height`,
optional tracker `config`, optional `policy` and 1–128 `events`.
Each event has `now_s` and an `input` of one of these forms:

```json
{"now_s":12.05,"input":{"kind":"analysis","acquired_s":12.0,"report":"frame.json","sha256":"<64 lowercase hex digits>"}}
{"now_s":12.10,"input":{"kind":"tick"}}
{"now_s":12.15,"input":{"kind":"failed"}}
```

For synthetic controls, `kind:"detections"` instead takes `acquired_s`,
`image_sha256` and `detections` with the existing class/confidence/pixel-box
contract. A report path is relative to the sequence file. SHA-256, pinned
analyzer sources, geometry and retained-measurement lineage are verified by
the unchanged report adapter.

Both clocks are caller-supplied seconds in the **same time domain**, finite in
[0, 1e12]. Delivery time must advance by at least 1 microsecond. Accepted
acquisition times must advance by at least 1 microsecond, never exceed delivery
time, and be strictly after the latched fault. A poll advances only delivery
time. A sample may therefore be older than the last poll while still usable.
The caller owns timestamp association and synchronization: saved image-analysis
reports do not independently authenticate acquisition time. This CLI neither
estimates clock offsets nor measures or guarantees execution latency.

Policy defaults are `max_age_s:0.2` and `recovery_samples:2`; supported ranges
are 0.001–2 seconds and 1–10 samples. These are authored limits, not calibrated
camera or driving requirements. An age exactly at the computed deadline is
allowed. On the first event beyond it, failure is recorded once at
`last_accepted_acquired_s + max_age_s`. Later polls do not move this barrier.
An explicit failure instead latches its delivery time. Recovery needs the
configured count of consecutive accepted, fresh post-fault acquisitions.
Rejected samples during a hold reset that count; polls do not count or reset it.

Readiness and track confirmation are separate. Empty images can restore camera
availability. Output tracks must also be confirmed/coasting and each individual
observation must be no older than `min(max_age_s, config.coast_seconds)`.
Coordinates are pixels; velocities are pixels/second. Projection uses the last
observed box and velocity at its acquisition time. Observation timestamps and
confidence remain unchanged. This extrapolation has no calibrated uncertainty,
metric depth or ego-motion compensation.

## Output and failure behavior

The report retains input/source SHA-256 receipts, effective policy/config and
one result for every delivery/poll. Each result gives its admission `decision`,
`ready`, hold reason, fault epoch, recovery count, source age and projected
tracks. `backend_updates` preserve every underlying acquisition/failure frame
and complete tracker snapshot. One late fresh arrival can contain two updates:
expiry at the previous deadline followed by the new acquisition.

Semantic sample rejections (future, pre-fault, out-of-order, stale or invalid
payload/reused image) are reported without renewing accepted acquisition time.
An already available result remains usable only to its original deadline.
Invalid delivery clocks, JSON/schema, provenance or file access abort the batch
with exit 2 and publish no result. Exit 0 means the trace was evaluated; it does
not mean its final result is ready.

Sequence JSON is bounded to 8 MiB, each analysis report to 4 MiB and final output
to 16 MiB; regular non-symlink inputs are required. Fresh output is reserved
before source reads. Only a completed report is published, without replacing
an existing file or dangling symlink. Handled failures remove hidden staging.
Abrupt termination can leave staging; local Linux verification does not
establish power-loss durability or cross-platform behavior.

## Verification and remaining limits

```bash
python3 integrations/camera-tracking/freshness_check.py \
  --output artifacts/camera-freshness-new
```

The checker needs Python's standard library and both built tracking binaries.
It runs **46 synthetic sequences / 1,029 events** plus four injected-control
sequences on the eight unchanged recorded reports, each twice. Fifty extracted
backend traces reproduce the standalone tracker's snapshots exactly. An
independent motion/lifecycle reconstruction and residual min-cost-flow solver
check every backend update; the solver is cross-checked against exhaustive
enumeration on 96 small graphs. Healthy/delayed recorded acquisitions reproduce
the earlier one-process sequencer's tracking states exactly.

Ten new Rust tests cover the two clocks, inclusive epoch-time deadlines,
one-shot expiry, recovery, invalid-clock atomicity, rejected samples and
per-object age. The 11 existing tracking tests also pass. Thirty fatal input/
report-budget controls and two existing-output protections pass. Raw inputs,
results, replays, failures and source pins are in the
[evidence index](../assets/camera-freshness-v1/files.json). Sparse/special files
used by rejection controls are represented by descriptors in the packet.
The dedicated image-tracking CI job now runs this check without downloading
images/weights; a remote run has not been observed for this change.

The indoor slice is already viewed and lasts 0.232288 seconds. No real sensor
fault rate, sustained automotive availability, independent object-identity
accuracy or vehicle integration is established. Detection quality is unchanged.
The subjective OSS-relative maturity estimate stays **about 20%**; the 30%
waypoint remains unmet.
