# Pose covariance in world forecasts

Optional `forecast_pose_sigma: 3.0` adds current localization uncertainty to
world-coordinate obstacle forecasts. It requires a declared `forecast_envelope`
and `absolute_heading_fusion: true`. Omission preserves preceding behavior and
serialization. The multiplier must be finite and between 1 and 5.

## Why world occupancy needs another term

The preceding detour qualification exposed a gap between sensor-relative shape
fits and world-coordinate occupancy. In the reference seed-7 acquisition at
5.8 s, the fitted sensor-frame circle is close to the physical circle, but the
world track is about 0.62 m from its physical center. An orientation error at a
range of about 45 m accounts for much of that displacement. The existing declared
motion envelope adds 0.262 m at 0.2 s; it explicitly excludes ego-pose uncertainty.
Changing the circle solver would not address the missing coordinate uncertainty.

The new term uses the current EKF posterior covariance, including x/y/yaw cross
terms. Simulator positions, sensor-noise schedules and future closure events
never enter this calculation. The selected multiplier is a caller assumption;
neither the EKF covariance nor a three-sigma ellipsoid has measured automotive
confidence calibration in this prototype.

## Conditional geometric bound

Let `P` be the ENU covariance of `(x, y, yaw)` in m², m·rad and rad². For a
nominal forecast center with offset `q` from the current estimated ego position,
the point-position Jacobian is `J = [[1, 0, -q.y], [0, 1, q.x]]`. The added radius is

```text
sigma * sqrt(lambda_max(J P Jᵀ)) + |q| * sigma² * P[yaw,yaw] / 2
```

The first term encloses the projected joint pose ellipsoid, retaining translation
and orientation correlation. The second bounds the exact rotation's Taylor
remainder. It is necessary even when correlated linear terms cancel. The bound
is conditional on the supplied joint ellipsoid and a point fixed in continuous
tracking coordinates. It does not estimate object shape, unobserved object
acceleration, association uncertainty, future ego covariance or Gaussian tail
probability.

The operation adds to every existing additional-radius knot after the forecast
is transformed into world ENU. Nominal centers, base track radii, acquisition
timestamps, motion alternatives and declared position/velocity/acceleration
budgets remain intact. Along a segment, the projected norm and rotation remainder
are convex in `q`; linear interpolation of the endpoint margins encloses the
intermediate bound. The eight-second forecast horizon and existing endpoint
hold remain unchanged.

The reusable prediction-crate function accepts a pose, covariance and multiplier,
without depending on localization or simulator crates. It validates finite
occupancy and symmetric positive semidefinite covariance. An error leaves its
input prediction unchanged. The pipeline responds to an expansion failure with
localization-uncertain braking and empty malformed forecasts; it cannot silently
continue using nominal occupancy after a failed expansion.

## Acquisition and guard ownership

The continuous-frame forecast-consistency guard runs before world projection and
before this new radius is added. A world-pose margin cannot hide a newly acquired
proxy disc escaping an earlier continuous-frame forecast. The guard retains its
original identities, acquisition clocks, stopping and recovery rules.

Cached scans retain their acquisition time. A later GNSS or heading update may
change the current posterior and therefore the world margin without renewing
object support. The absolute-heading freshness and other health guards continue
to govern braking. The planner consumes the expanded world occupancy through its
ordinary collision checks; there is no separate relaxed path through the planner.

## Frozen evaluation and independent checks

Before implementation, 48 supported inputs were frozen from the preceding
versioned result packets: all 44 detour conditions and four original mission
conditions that already declare absolute-heading fusion. The other 28 progress
conditions lack this prerequisite and are retained as existing disabled-mode
replay evidence. Support selection uses configuration, not a successful score.
Only `forecast_pose_sigma: 3.0` is added. Original worlds, sensor schedules,
noise, motion budgets, goals/deadlines and acceptance gates stay fixed. Changed
commands can change later acquired returns in the new physical executions.

The Python oracle independently reconstructs the complete localization posterior
from delivered wheel/gyro, GNSS and heading observations. It projects a Cholesky
factor of that covariance, whereas Rust directly forms the point covariance.
Removing only this independently calculated margin must reproduce every original
nominal-motion radius and center. The independent continuous guard uses those
unexpanded forecasts. Local planning and physical coverage use the expanded
world forecasts. Corrupted padding, covariance or multipliers are rejected.

Rust geometry tests compare exact nonlinear joint pose perturbations at knots
and between knots, rather than testing another copy of the linearized formula.
They cover world-frame rotations/translations, correlated cancellation, zero
uncertainty, invalid covariance and transactional overflow. Pipeline tests retain
tracking, the raw continuous guard and cached acquisition clocks, and reject
replay tampering. Existing strict physical, accuracy, goal and forecast gates
remain separate from the new covariance/occupancy provenance contract.

```sh
python3 scripts/test_pose_forecasts.py
python3 scripts/check_pose_forecasts.py --output artifacts/pose-forecasts
bash scripts/check.sh
```

## Recorded result

All 48 supported conditions pass the independent posterior, pose-occupancy,
continuous-guard and complete replay contracts, covering 60,735 control ticks
and 1,800,720 expanded forecast knots. Every previously passing gate is retained.
Physical forecast-coverage failures fall **21 → 8**, and strict qualification
improves **16 → 28/48**. All 48 now pass 0.2 s coverage; all 21,812 eligible
forecasts at that horizon enclose their associated physical circle. The original
99% requirement and first-acquisition scoring remain unchanged.

All eight newly authored observed-detour conditions retain their goals and now
pass forecast coverage. The four known-closure detours do too. Total arrivals
across enabled/disabled/control conditions remain 20. Eight moving-body controls
still fail the 1/2/4/8 s coverage requirements; their cold measured velocity does
not support the physical motion. Eight deliberately disabled detour controls
and four supported original missions retain their missed-goal failures. The
other four original mission conditions lack heading support and remain unmodified
in the preceding replay evidence. All eight original mission failures stay open.

Minimum physical swept clearance is 4.527637 m, minimum active-road margin is
0.763385 m, maximum position error is 0.247889 m and maximum heading error is
0.017222 rad. The largest added pose radius is 2.445685 m. These are correlated
samples from the stated authored worlds, not held-out automotive calibration.
Default adoption still fails and the option stays off.

The first 48-run audit correctly verifies covariance and geometry, but its
parent-specific goal classifier initially treats the added qualification-name
suffix as a different control mode. That adds a spurious classification failure
to eight already-unsolved disabled controls. The report is retained. The final
auditor passes the original mode to the existing classifier and re-audits those
eight archived executions; the other forty complete audits are unchanged. No
physical trial, input, gate, strict score or mission failure is discarded.

The unchanged-source `bash scripts/check.sh` passes 958 Rust tests, 254 Python
tests and 49 standard driving/replay scenarios. All 245 standard output files
retain exact preceding bytes. Native formatting, 67 release tests and strict
all-target Clippy pass. All 700 preceding sensor logs, covering 521,643 control
ticks, retain exact replay output bytes. All 240 physical recording streams have
their encoded and decoded SHA-256 verified; final re-audits reference the same
archives. CI includes both plants; remote and cross-platform results are unverified.

The [result packet](../assets/pose-forecast-envelope-results.json) records all
48 original comparisons, fixed inputs, failed gates, first classifier errors,
complete replay evidence and tested source/binary/lock hashes. Before/after
counts are derived directly from the versioned baseline rows.

The feature is off by default. Measured covariance calibration, physical
shape confidence, cold moving-object velocity, incomplete original missions,
broader domains and real-time/vehicle operation remain open. Comparative
engineering maturity remains about 29%; the 30% waypoint is unmet.
