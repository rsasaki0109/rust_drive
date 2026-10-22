# Motion continuity and latched localization loss

With recovery disabled, the shared driving pipeline stops permanently after a
post-initial control interval cannot be propagated from valid wheel/gyro motion. Previously,
a missing interval could enter scan-pose history as no movement, retain object
identities and later resume on fresh scalar observations. A new GNSS position
or wheel speed does not reconstruct the lost rigid-body displacement/heading.

## Operational contract

The existing accepted-motion cache remains usable for at most 0.15 seconds.
A brief malformed delivery still brakes until a fresh post-fault acquisition;
if accepted cached motion covers every interval, that temporary hold can recover.
The first pipeline step anchors the externally supplied initial pose and needs
no preceding motion interval. Missing initial motion still commands braking.

Any subsequent missing/expired motion, clock interval over 0.25 seconds or
failed bounded-history prediction latches `HealthIssue::MotionDiscontinuity`.
The optional `motion_discontinuity` output records `last_covered_time`,
`detected_at`, the last accepted wheel acquisition stamp and the original cause.
It remains fixed while the enclosing control clock continues advancing.

The latch clears scan-pose history, tracks and predictor evidence, stops GNSS
fusion and prohibits new LiDAR/map admission. The EKF pose, speed, covariance,
GNSS accepted/observed clocks and counts stay at the last covered state.
**These estimate values belong to that old epoch and are invalid as a current
pose/speed, even after the physical vehicle stops.** Historical occupancy cells
remain developer evidence; no new rays update them. Commands remain acceleration
−6 m/s² and steering zero. Fresh GNSS, LiDAR, map or wheel observations cannot
clear the latch or reconnect object identities.

By default, restart requires an externally supplied current pose calibration
and a new `DrivingPipeline` instance.
A separate explicit [stopped regional map policy](regional-relocalization.md)
now provides measured reinitialization within a bounded region for supported
fixed-route/XY-LiDAR configurations. The default guard and the original results
below retain their stopping-only contract.

Normal covered-motion frames omit the new field. JSONL schema remains 1;
historical fault logs that expected automatic resume after a motion gap now
need new expectations, while their original evidence must remain unchanged.

## Independent checks and fixed physical protocol

A parent-commit regression retains two failures: old scene evidence survives a
clock gap, and new wheel observations resume after an expired cache. The repair
also checks 150 ms grace, temporary malformed-input recovery, map exclusion,
complete pre-gap posterior retention and replay rejection of a forged latch epoch.
Simulator-only odometry transport preserves acquisition stamps/values, with
bounded delay and nonoverlapping dropout windows. It remains outside the
operational header and is independent of GNSS/LiDAR failure handling.

All **36** scenario inputs and hashes are fixed before their first execution.
Two plants (reference 2D and native RNE dynamic), straight/curved road geometry,
and LiDAR delay 0/2 ticks provide:

- **24 dropout cases:** seeds 1/7/42; missing motion acquisitions on `[8,11)` s.
- **8 healthy goal controls:** seed 7 on every plant/world/LiDAR combination.
- **4 persistent-delay controls:** straight roads; wheel delay 4 ticks (0.2 s),
  outside the normal accepted-motion age.

The authored scenes explicitly widen the original corridor half-width from 3.5
to 5.5 m and add stationary measured circles at road coordinates `(45,+4)` and
`(70,-4)`m, radius 0.5 m. Original route curves, 45 s deadline, ≥1 m swept clearance
and source GNSS transport (straight 8 ticks, curve 4) remain. Existing pipeline
pose, GNSS and native chassis calibrations remain supplied defaults.

Healthy controls require original goal/deadline, RMSE ≤0.2 m and maximum position
error ≤0.4 m. Fault cases require no goal, an actual stop ≤0.05 m/s during
`[10.5,11.2)`s, and every physical frame after 12 s stopped. Returned fresh joint
observations after 13 s must not resume dropout cases. They qualify **stopping
policy, not localization accuracy or recovery**; every invalid-epoch pose error
remains in the full-interval scores.

An independent raw-clock/motion pass locates the first uncovered interval before
examining latch outputs. A separate joint-matrix chronological filter verifies
every covered prefix. All later states, covariance and GNSS diagnostics must
remain exactly that independently verified last state, with no tracks or motion
history. An independent swept-clearance calculation and full-frame truth errors
check physical outcomes. Truth enters only synthesis and evaluation.

## Recorded outcomes

The fixed stop/healthy contract passes **36/36**: all **8 healthy controls**
reach their original goals, and all **28 motion-loss controls** hold an actual
stop of **0 m/s**. Every physical frame after 12 s remains stopped, including
returned fresh wheel/GNSS/LiDAR deliveries in all 24 dropout cases. Healthy worst
RMSE is **0.0838074 m**, with maximum position error **0.247965 m**. Independent
covered-prefix reconstruction, held-state checks, full swept-clearance scoring
and exact replay verify **28,937 control ticks**.

The general CLI still marks **24/24 dropout runs failed** because its global
1 m position gate also scores invalid-epoch estimates; their maximum errors
**3.098921–3.324089 m** remain visible. Only the eight healthy and four stationary
persistent-delay cases pass that general summary. **No dropout case qualifies
current localization accuracy or automatic recovery.**

The first independent checker accidentally imposed that global summary gate
on the stopping-only cases, recording 12 passes and 24 failures. Its original
complete report and source remain archived. A checker-only repair implements
the already frozen split: only the exact independently reconstructed invalid-
epoch position failure can explain CLI exit 1. Valid-prefix errors above 1 m,
healthy-case errors, collisions and every other source failure still reject.
Mutation tests check this distinction. All first raw streams and general CLI
failures remain unchanged; re-auditing performs **no repeated physical runs**
and changes no driving algorithm or frozen gate.

The final exact `bash scripts/check.sh` passes **710 Rust tests**, **53 Python
tests** and all **49 standard drive/replay pairs**; all **245 standard files**
retain their preceding bytes. Native release formatting, strict all-target
Clippy and **63 tests** pass. All **434 previous recordings** retain exact
replay output bytes, including earlier failed/expired controls. Both final
runtime binaries match the physical candidates; locks, toolchains and engine
revision remain unchanged. The comparative engineering estimate is **about 26%**,
subjective against Autoware, Apollo and openpilot; the 30% waypoint remains open.
[Complete first outcomes, failures and provenance](../assets/motion-continuity-results.json).

All raw first outcomes remain losslessly archived as XZ with decoded/stored
SHA-256. The earlier 92-case GNSS recovery packet underwent a separate verified
gzip-to-XZ storage migration; its raw bytes, first outcomes and original report
are preserved with a migration receipt. Four large historical desk2 JSON streams
from the descent/subpixel research packets also have verified lossless XZ
mappings; decoding restores their original byte hashes and paths. No acceptance gate or first outcome is
removed by storage compaction.

## Reproduction and limits

```sh
source scripts/env.sh
bash scripts/check.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/check_motion_continuity.py --output artifacts/motion-continuity
```

Use a new output directory. `--freeze-only` writes every input before execution;
`--smoke --plant reference-2d` or `--smoke --plant rne-dynamic` selects 10 cases.
All 36 first cohort recordings, mathematical checks, resource settings and
source/binary receipts remain under `artifacts/motion-continuity-v1/`.

These are authored planar Linux x86_64 simulation results. Holding the last
state does not restore localization; braking depends on the modeled plant and
is not a real-vehicle stopping guarantee. Delayed odometry, live-clock faults,
measured automotive uncertainty, automatic initialization and availability
under sustained sensor loss remain open. The 30% maturity waypoint remains unmet.
