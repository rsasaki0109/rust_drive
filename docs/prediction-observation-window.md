# Preserve contrary measurements in prediction history

The predictor previously selected observations near 0.2, 0.4 and 0.6 seconds
before the latest acquisition, then checked only those spaced intervals. A stop,
turn, backward position step or reacceleration at an intervening 10 Hz acquisition
could disappear from this evidence. Net decreasing speed at the anchors could
therefore reinstate the braking hypothesis despite a measured acceleration.

## Evidence used by the predictor

The original spaced-anchor requirements remain. Braking needs three intervals
with at least 0.3 m/s² net deceleration and velocity-direction cosine at least
0.98. Experimental slow-motion confirmation needs four observations, forward
progress of at least 0.1 m/s over each spaced interval, velocity-direction cosine
at least 0.95 and displacement-direction cosine at least 0.9. Anchor lookup
retains its existing 25 ms tolerance.

The same selected earliest anchor now bounds a check of **every intervening
acquisition** through the current track:

- Braking rejects any observed speed increase beyond 1e-9 m/s or velocity
  direction outside the 0.98 cosine allowance. Its existing immediate veto on
  the latest non-decreasing speed remains. A preceding plateau alone does not
  imply acceleration.
- Slow-motion confirmation rejects any speed below 0.1 m/s, velocity direction
  outside the 0.95 cosine allowance, or adjacent position increment outside the
  0.9 displacement-direction allowance, with 1e-9 m roundoff tolerance. Zero
  increments are allowed; spaced intervals still need their minimum progress.

This is a window check, not a permanent identity blacklist. Once contrary samples
and intervals leave the selected window, new evidence can support the hypothesis
again. In particular, a backward displacement needs both of its endpoints inside
the window to remain an observed contrary interval. Duplicate control calls do
not insert new acquisitions or erase contradictory samples.

History remains bounded to 0.8 seconds / 32 samples per present identity. The
predictor still receives only tracked observations in the continuous wheel/gyro
frame and the current clock. No actor policy, future motion, physical acceptance
or simulator identity enters it. The one-second half-deceleration budget, 2 m/s²
cap, acquisition-time origin, stale-age withdrawal and eight-second forecast
horizon remain unchanged. High-speed contrary braking falls back to constant
velocity. Unsupported slow motion retains the existing static deadband.

## Independent checks

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-prediction --test observation_window
python3 scripts/test_prediction_evidence.py
bash scripts/check.sh
```

Three new Rust regressions cover intervening stops, turns, backward steps and
reacceleration, rotated directions, cached calls and evidence recovery. They
check full forecast positions against analytic constant velocity or bounded
braking, including acquisition age. All three fail on the original predictor;
the original source, failed executable and output remain retained locally.

The independent Python auditor reconstructs chronological observations and checks
all intervening samples. Three adversarial standard-library tests are now part
of the required check. They accept the withdrawn hypothesis and reject old
braking/slow-motion forecast mutations. These handcrafted arithmetic fixtures are
not sensor replay or physical driving evidence. The original auditor fails them;
its source and failure output remain retained.

## Reference measurements

Local validation passes **578 Rust tests, three Python auditor tests and 49
standard scenario/replay pairs**. The fixed-world comparison covers **80 physical
episodes: 78 passes and two retained insufficient-range failures**. All 80 replay
exactly. Independent coordinate, tracker-interval and forecast audits cover
62,504 sensor ticks; 31 episodes retain every physical tick for a separate
swept-clearance recomputation. [Compact results, tested sources and raw inputs](prediction-observation-window-results.json).

All 15 experimental low-speed crossings and all six follower-deadline cases
retain their original physical gates. The six deadlines still complete in
63.45–63.85 seconds, including eight seconds of physical residence. All 245
standard output files are byte-identical to the preceding `b018885` results.
The 245 older historical live outputs were separately restored with distinct
live/backup inodes.

This does **not** improve every clearance result. Experimental `slow-mirror`,
seed 42, falls from 1.276578 to 1.179197 m, a 0.097380 m reduction against the
unchanged 1 m floor. It is the largest passing reduction. Experimental
`slow-crossing` seed 42 and `slow-mirror` seed 1 also lose clearance. Dense
position evidence can withhold motion through noisy increments, while the
unchanged unsupported-motion fallback remains static. These decreases remain in
the comparison; this is evidence hardening, not general prediction-accuracy
qualification. Slow mode remains opt-in.

Five additional authored raw-circle inputs each contain 111 pipeline steps,
with a 0/2/4/6/8 cm position pulse at 3.7 seconds during measured deceleration.
All **555 steps** replay exactly. The 4 cm input exercises two intervening-sample
braking vetoes; the old binary rejects its new expectations at tick 84 (4.2 s).
The unchanged control and other three inputs replay on both binaries, including
normal braking forecasts. These are raw sensing/tracking/replay checks, not
physical driving episodes or measured automotive data. Their parameters,
source/lock hashes and compressed input recordings are retained locally. The
probe uses the root lockfile's dependency versions.

The adversarial auditor tests are included in `scripts/check.sh` and configured
in the existing Linux/macOS/Windows CI matrix, after Python setup. The exact CI
command and workflow YAML were checked locally. No new hosted or cross-platform
CI execution is claimed. Python's standard library suffices; algorithm crates
retain their existing Rust dependencies.

## Limits and adoption

This repairs evidence handling; it does not estimate calibrated prediction
uncertainty or guarantee future braking. Unobserved starts, turns and acceleration
can invalidate a forecast. Small/noisy displacements may withhold slow-motion
confirmation, whose static fallback can underpredict genuine motion. No general
traffic accuracy or safe default adoption follows from the authored cases.
Slow-motion mode therefore remains explicit opt-in. The default 0.7 m/s deadband
is retained. This change has no new RNE, external automotive benchmark,
real-vehicle or real-time qualification.
