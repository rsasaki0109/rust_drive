# Recent acquired motion fits

`fit_recent_track_motion` optionally refits moving-object position and velocity
from a bounded recent acquisition window. Ordinary alpha-beta tracking can lag
observed acceleration or braking, even when the acquired circular center is
accurate. The new policy improves responsiveness after measured motion support.
It defaults to false and requires `enclose_track_measurements`; it excludes the
separate `confirm_track_initialization` policy.

The window uses raw associated detections in continuous wheel/gyro coordinates.
It contains the shortest recent suffix spanning at least 0.2 s, with at least
three actual observations, at most 32 samples and at most 0.6 s total span. A
per-identity acquisition gap over 0.35 s resets both the window and moving support.
Cached ticks, empty scans and rejected inputs do not add samples. Track expiry
and measured localization reset discard the identity's history.

The existing stable-shape least-squares calculation supplies the fitted current
position and velocity. Radius range must be at most 0.12 m, fit RMS at most 0.12 m
and speed at most 15 m/s. All intervening admitted samples contribute. A valid
fit must reach the existing ordinary moving-object cutoff of 0.7 m/s before it
can replace the ordinary update. Subsequent stopping fits can continue for at
most 0.6 s after that moving acquisition. Low-speed fits and cached control ticks
cannot renew this clock. Unsupported windows retain the ordinary correction.

These are declared estimation heuristics. A line through three equally spaced
samples of constant acceleration estimates velocity at the middle sample, with
a one-sample lag; it does not recover instantaneous acceleration or future
intent. Cold tracks still begin with zero velocity. Uncertainty, segmentation
and body extent remain uncalibrated. The supplied future error budgets remain
0.2 m position, 0.3 m/s velocity and 0.1 m/s² acceleration.

## Independent verification

The Rust path reuses a centered online-moment fit. The Python oracle independently
selects the bounded suffix, evaluates dense centered sums/direct residuals and
folds the acquired moving-support clock. It reconstructs ordinary fallback,
fitted motion, radius enclosure and cached acquisition ownership without
producer fit diagnostics or actor truth. The existing sparse-geometry,
prediction, forecast-guard, stop, road and physical-clearance checks remain active.

Rust tests cover the analytic constant-acceleration lag, irregular/20 Hz
acquisitions, coordinate invariance, unchanged static/slow filtering, stopping
support expiry, bounded oversampling, shape/speed/residual rejection, gap/identity
reset, rejected inputs, absolute pose corrections, delayed turning-ego scans,
replay tampering and invalid policy combinations. Eleven Python checks cover
independent reconstruction, cache/gap/expiry ownership, corruption, bounded
support and unchanged source worlds/deadlines/error budgets. Both plant CI jobs
run the fixed comparison.

## First candidate and refinement

The first all-window fitter passes numerical/replay/braking checks in all 32
conditions, but changes ordinary static filtering. Reference mission seed 7 with
heading falls to 604/614 coverage at 0.2 s (98.37%, below the unchanged 99% gate).
Forecast-coverage failures rise 26 → 27, so regression and adoption fail.
The first required check also stops at Clippy's `items_after_test_module` rule;
the test module is moved after the implementation in the final source.

The final policy adds the observed moving-speed cutoff and bounded stopping
support above. Its 32 scenario input files are byte-identical to the first
candidate's files; all original goals, sensing/noise, deadlines, budgets and
acceptance gates remain fixed. The new gate declaration is frozen before the
second physical executions. The first 32 runs, source snapshot, binaries, failed
check and all outputs remain retained. This is a viewed same-seed refinement,
with no held-out or automotive generalization claim.

## Complete physical results

The complete matrix retains both plants, seeds 7/42, the eight original obstacle
missions, permanent obstruction, release/fault recovery, slow crossings, lead
stop/resume and priority crossings. All first acquisitions remain scored.

| Measure | Preceding sparse-circle policy | Final recent-motion policy |
|---|---:|---:|
| Conditions retaining every previously passed gate | — | 32/32 |
| Strict complete qualification | 0/32 | 0/32 |
| Goal arrivals | 16 | 16 |
| Original incomplete missions | 8 | 8 |
| Forecast-coverage failures | 26 | 26 |
| Localization-accuracy failures | 5 | 5 |
| Acquired proxy misses against older forecasts | 25 | 13 |
| Forecast-fault braking control ticks | 116 | 74 |
| Full control ticks | 27,416 | 27,347 |

All 3,141 applied fits are independently reconstructed. Static filtering remains
unchanged in all twelve mission/permanent-obstruction conditions: complete
operational output bytes, physical frames, summary and original sensor inputs
match their preceding recordings exactly. Initial unsupported and unconfirmed
slow observations retain ordinary tracking. Minimum physical swept clearance is
2.876505 m and minimum road margin is 0.643533 m, with zero collisions or road
violations.

Across 160 per-condition horizon fractions, 73 improve, 11 worsen and 76 stay
equal. The largest reduction is 0.026923. All counts, unmatched observations,
first acquisitions and end-of-record censoring remain in the packet. Different
driving/termination changes the acquired sample sets; pooled fractions are not
a fixed-sample causal comparison.

### Velocity evidence

An evaluation-only scorer associates the acquisition to a physical actor under
the unchanged 2 m / 0.5 m ambiguity rule, then derives world velocity from
centered differences of 20 Hz physical positions, using one-sided differences
at record boundaries. This derivative is a scoring approximation and does not
enter the driver or replace the original forecast-coverage gate.

| Full-record metric | Preceding | Final |
|---|---:|---:|
| Acquired observations | 12,310 | 12,282 |
| Matched observations | 12,234 | 12,206 |
| Retained unmatched observations | 76 | 76 |
| Matched velocity errors above 0.3 m/s | 545 | 99 |
| Mean velocity error | 0.057567 m/s | 0.037162 m/s |
| RMS velocity error | 0.219020 m/s | 0.133157 m/s |
| 95th percentile error | 0.245122 m/s | 0.123641 m/s |
| Maximum error, including cold tracks | 4.0 m/s | 4.0 m/s |

A separate read-only comparison on the **same original** 12,310 acquisitions and
associations supports 3,175 gated fits. It keeps unsupported/unmatched samples
and reduces velocity-budget exceedances 545 → 123, mean error
0.057567 → 0.038922 m/s and RMS 0.219020 → 0.135202 m/s. The initial ungated
proposal lowers exceedances to 96 but raises mean error to 0.064515 m/s; this
tradeoff and its physical forecast regression remain recorded.

## Evidence and remaining work

The [result packet](../assets/recent-track-motion-results.json) retains the full
first/final matrices, frozen inputs, unchanged gates, source/binary hashes,
fixed-acquisition proposal summaries, physical velocity/static-preservation
audit and verification receipts. Local raw data and complete proposal samples
remain in `artifacts/recent-track-motion-v1`. Native RNE remains pinned to
`df6007aa40315e81d12ae00fc1f60369e393a178`.

The final `bash scripts/check.sh` passes 929 Rust tests, 218 Python tests and
49 standard scenario/replay pairs; all 245 standard output files retain exact
bytes. Native formatting, 67 release tests and strict all-target Clippy pass.
All 592 supported preceding logs replay with exact output bytes across 411,440
ticks on the final binary. All 320 first/final physical/replay archives have
both compressed and decoded SHA-256 verified. The 1,086 operational source files
remain unchanged throughout the final required check; Cargo locks and pinned
toolchains remain unchanged.

Absolute pose/heading error remains a major short-horizon issue: in the preceding
matrix, 511 of 529 matched 0.2 s misses also fail the acquired world-frame proxy
check, while none fails sensor-local acquired shape containment. This follow-up
improves observed motion tracking; world projection, unobserved future motion,
association and calibrated uncertainty remain open.

Default adoption remains unqualified. All eight original incomplete missions,
five accuracy failures and 26 forecast-coverage failures remain scored. Comparative
maturity stays **about 29%**, and the 30% waypoint remains unmet.
