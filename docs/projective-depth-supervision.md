# Explicit-clock supervision for depth odometry

The optional depth library now supplies a `supervision::Supervisor` wrapper
for [projective depth odometry](projective-depth-rust.md). Caller-driven ticks
detect stale estimates even when no new acquisition arrives. Fault holds
require a successful acquisition strictly after the fault receipt time;
queued older depth cannot clear them. Expired tracking cannot automatically
restart or establish a new map origin.

This is temporal supervision of offline optical-camera estimates. A `fresh`
status is not a position-accuracy guarantee or permission to drive. The wrapper
does not feed vehicle localization or controls. The recorded position-accuracy
failures remain unchanged: room **64/179**, desk2 **161/179**.

## Clock and failure contract

`Supervisor::tick(now, Observation)` uses a finite, nonnegative, strictly
advancing caller clock, in the same monotonic epoch as depth acquisition time.
The implementation reads no wall clock and starts no threads or timers.
Callers must continue providing ticks while sensors are silent. Duplicate,
regressing, nonfinite or oversized receipt clocks return an error before any
state mutation; callers must hold on that error.

`Observation` is `Idle`, `Failed`, or `Depth { timestamp, depth_m }`.
The underlying estimator receives actual depth acquisitions only; idle ticks,
explicit failure notifications and rejected metadata do not fabricate a
measurement or refresh the accepted acquisition clock.

| Temporal status | Meaning and estimate availability |
|---|---|
| `waiting` | No accepted origin yet; no cached estimate |
| `fresh` | Last accepted acquisition is at most 0.20 s old and no fault hold remains; cached estimate available with its acquisition timestamp |
| `hold` | Explicit failure, invalid/old acquisition or estimator rejection; cached estimate withheld |
| `lost` | Accepted pose older than 0.20 s; cached estimate withheld and loss terminal for this instance |

Expiry is checked against the last successful **acquisition** timestamp, before
handling an arriving observation. Idle ticks, repeated errors and failure
notifications cannot postpone that deadline. There is no reset/relocalization
operation. Creating a new instance would explicitly establish a new session,
rather than recovering the old root.

Depth acquisition timestamps must be finite, nonnegative, strictly new and no
later than `now`. Acquisition age above 0.20 s is rejected. Explicit failures,
invalid metadata and estimator rejections record the current receipt time as
the fault epoch. While held, an acquisition at or before that epoch is
quarantined before fitting; it cannot update the pose, depth model or reference.
Quarantined backlog does not advance the fault epoch. Only a successful fit
from an acquisition strictly after the epoch clears the hold.

The cached `first_from_last_accepted` pose maps the last accepted optical camera
into the original optical camera. Its timestamp remains
`last_accepted_timestamp`; an idle tick does not predict or relabel this pose
as a pose at `now`. During holds/loss this field is absent. `measurement`, when
present, contains actual underlying estimator work and its separate acceptance
decision. Frames, calibration, algorithm gates and covariance limitations remain
those of the original depth library.

## Actual checks

The [design](../assets/projective-depth-supervision-v1/design.json) fixes the
clock rules, acquisition selection and fault offsets before the recorded
supervised execution. The underlying ICP/fusion implementation is unchanged;
the library adds an exported wrapper module and a separate CLI.

Six added Rust tests exercise silent expiry without fabricated acquisitions,
terminal loss, backlog quarantine and recovery, malformed-depth state retention,
invalid receipt clocks, future/duplicate/stale acquisitions and repeated faults
that cannot postpone expiry. Together with the original six depth tests,
all **12** pass locally. The analytic CLI check compares recovered estimates
with a separate clean estimator and rejects thirteen invalid transport/clock
requests, including both work caps.

The first analytic check found that serde's internally tagged unit-event form
ignored unknown fields. Empty struct event variants now enforce rejection of
unknown fields on idle and failure events. The original failing CLI source
and [failure receipt](../assets/projective-depth-supervision-v1/first-protocol-failure.json)
are retained. This admission repair changes no ICP parameter or accuracy gate.

Both viewed TUM recordings retain all 180 original acquisitions. Each accepted
measurement and its full normal-equation trace is compared with the immutable
prior sensor ledger, and an idle tick follows each acquisition. These synthetic
receipt clocks use a declared 1 ms delivery offset and a tick 10 ms after
acquisition; these are test inputs, not measured latency bounds. A final idle
tick at 201 ms age produces loss without another acquisition.

A separate first-20-acquisition branch injects an empty depth buffer at offset
5 and an explicit acquisition failure at offset 11. Offset 11's real depth,
acquired before the failure receipt time, is then delivered late and quarantined.
Acquisitions 6 and 12 must recover using fresh, successfully fitted data.
A clean estimator receives exactly the 18 surviving acquisitions. Every
common returned pose and complete fitting trace must agree, demonstrating
that malformed/quarantined data did not alter the map, reference or later fits.
After silence expires this branch, the genuine later acquisition at offset
27 must leave it lost, without fitting or resetting.

These are viewed-data failure-injection regressions. Numeric ground truth and
RGB pixels are not decoded. They validate temporal/state behavior, rather than
new physical pose accuracy or transport reliability. Full results and replay
checks are in the [validation receipt](../assets/projective-depth-supervision-v1/validation.json).

## API and reproduction

```rust
use rustdriving_projective_depth::supervision::{Observation, Supervisor};
// Use explicitly calibrated Camera and a single shared monotonic epoch.
// let mut supervisor = Supervisor::new(camera)?;
// let update = supervisor.tick(now, Observation::Depth { timestamp, depth_m: &depth })?;
// let later = supervisor.tick(later_now, Observation::Idle)?;
```

The `rustdriving-projective-depth-supervised` JSON-lines evaluator accepts one
`Configure` followed by `Tick` requests with nested `Idle`, `Failed` or `Depth`
events. It limits requests to 8 MiB each, 1,000 ticks and 200 depth frames.
Protocol errors and invalid receipt clocks exit 2. Valid transport can exit 0
with held/lost outcomes; its exit alone is no freshness or accuracy assertion.

With the existing pinned room/desk2 acquisition and NumPy/Pillow environment:

```sh
source scripts/env.sh
cargo build --release --bins --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-depth-supervision.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-depth-supervised \
  --base-binary integrations/projective-depth/target/release/rustdriving-projective-depth \
  --output artifacts/projective-depth-supervision/local
```

Use a fresh output directory. `--analytic-only` requires no recordings and is
included in the Ubuntu analytic CI job. Recorded checks were run on this Linux
host; no production timing or cross-platform result is claimed. Supplying a
trustworthy shared clock and independent handling of caller silence remain
responsibilities of a future live integration.

The subjective maturity estimate remains **about 20%**. This closes a concrete
temporal-supervision gap, while sustained physical accuracy, independent
automotive validation, calibrated uncertainty and vehicle integration remain
unresolved; the **30% waypoint remains unmet**.
