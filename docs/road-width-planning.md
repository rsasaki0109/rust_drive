# Side-path candidates within the supplied road width

Optional `lateral_candidate_margin_m` fits the existing three lateral candidates
to a supplied constant-width road. With an additional edge reserve `m`, a road
half-width `w` and a vehicle's circular radius `r`, the side magnitude is
`max(0, min(3.5, w - r - m))` meters. The candidates remain center, left and
right; when the side magnitude is at most 0.1 m, only center is useful.

The configuration accepts a finite reserve from 0.1 through 1.0 m. It is
recorded in the sensor-only header and replayed. Omitted/null keeps the preceding
fixed ±3.5 m policy and its output bytes. The candidate count remains at most
three. This is bounded width fitting rather than a continuous lateral search.

The trigger is geometric: a 4.5 m half-width road and a 1.25 m vehicle radius
exclude both old 3.5 m side candidates. With a declared 0.35 m reserve, 2.9 m
side candidates can pass a small obstacle. A 5.5 m half-width road still uses
3.5 m. The extra reserve is a target allowance, not a guarantee of physical
tracking error or a measured road-edge uncertainty distribution.

Every fitted candidate still passes the original smooth geometry, heading
join, steering/curvature, reachable speed profile, complete collision sweep,
retimed stop and road containment checks. A blocked side is rejected through
those checks; fitting cannot clamp an unsafe trajectory into acceptance. A
fully blocked road must still stop. Forecasts, collision margins, vehicle radius
and the supplied road remain unchanged. Existing route-dependent resets retain
the configuration; measured localization resets reconstruct it from the header.

The option is independent of `forecast_envelope` and
`settle_obstruction_stops`. It stays off by default. Initial calibration,
constant-width circular road geometry and the existing tracking controller
remain assumptions; lane topology and continuous offsets remain future work.

## Chassis passing continuity follow-up

A follow-up repairs the remaining calibrated native right-pass regression. A
blocked centerline braking prefix previously replaced the side maneuver's
original anchor. Combined with a fresh full-length chassis error join on each
replan, this delayed the actual shift past the obstacle.

For local chassis/course mode with fitted sides below 3.5 m, a temporary blocked
center brake now retains the passing intent. After the original completion arc,
the residual joins over the wheelbase / declared lateral-acceleration recovery
span; an increased residual may extend that span. The initial shift keeps its
original join. A clear return or side change replaces the intent; route reset
clears it. Every candidate still passes the measured-course join, original road
containment, reachable speeds and complete retimed collision/hold sweep.
Point-reference recovery, unfitted/default modes and unshortened wide-road
candidates retain their preceding arithmetic.

The first attempted repair shortened joins during unfinished shifts too. It
strengthened a transient early left choice and stopped at 29.219359 m without
reaching the goal. That complete 45 s recording and exact replay remain failed.
The final policy is evaluated with the **same 64 input bytes and unchanged
numerical gates** as the original calibrated cohort; these are viewed authored
regressions, not held-out automotive evidence.

| Check | Follow-up result |
|---|---|
| Primary native right-pass seed 42 clearance | 0.845780 → **1.250174 m**, unchanged 1 m floor |
| Primary minimum lateral distance across progress 33–37 m | 2.386459 → **2.989554 m**, unchanged 2.6 m floor |
| Primary arrival / minimum actual road margin / max yaw error | 27.20 s / 0.245439 m / 0.092298 rad |
| Policy comparison | **32/32 pairs**, previously 31/32 |
| Narrow physical-pass requirements / goals | **18/18 / 18/18**, previously 17/18 / 18/18 |
| Strict fitted qualification | **23/32**, previously 22/32 |
| Blocked holds | 6/6, actual 0 m/s over 14–20 s |
| Minimum swept clearance / actual circular road margin, all 64 | 1.068972 m / 0.159867 m; no contacts or road/rule violations |
| Unchanged reference/default/wide/mission cases | 52/52, all five physical/replay streams byte-identical |

Five native heading failures and four inherited incomplete missions still fail
strict qualification, including the two original forecast-coverage failures.
The option remains **off by default**. No collision margin, road, actor, speed,
deadline, startup prior or acceptance threshold was changed.

The [follow-up result asset](../assets/fitted-chassis-recovery-results.json)
retains both focused attempts, the complete 64-case report, frozen inputs,
source/binary fingerprints and lossless stream receipts. Its 66 recordings
include two development repeats; qualification remains the 64-case matrix.
All 330 streams have encoded and decoded SHA-256 verification. The 64-case matrix
contains 45,474 ticks and 4,536 sampled planned-path certificates through
342,298 interval bounds. The previous 52 compatible logs also replay with exact
outputs. The required check passes 801 Rust tests, 112 Python tests and 49
standard drive/replay pairs with all 245 output files byte-identical; the adapter
passes 63 native tests, formatting, release build and strict Clippy locally.
The native CI job now includes all three road-width seeds, including seed 42,
and preserves the complete evidence on failure; remote success is unverified.
Comparative engineering maturity remains **about 29%**.

## Original frozen paired protocol

Before the first new physical trial, the protocol freezes **64 episodes**:

- Straight, curved and occupied-left-refuge roads, plus a fully blocked control:
  seeds 7/42/101, reference/native plants and fixed/fitted policy pairs.
- Wide-road controls and the preceding failed envelope mission: seeds 7/42,
  both plants and both policies.

Every within-world pair differs only in the new candidate reserve. The new
roads retain a 45 s goal deadline (20 s for the blocked control), 4 m/s cruise,
0.5 m obstacle radius and 4.5 m half-width. The curved road has 2 m amplitude;
the right-pass case adds a 0.5 m obstacle in the left refuge. The blocked
control uses a 3.3 m obstacle radius. Wide controls use 5.5 m half-width. The
inherited mission keeps its original geometry, actors, envelope budgets and
65 s deadline. All cases use full-rate physical recording and the preceding
obstruction-stop option.

The first cohort omits local/chassis calibration. Its attempted 3.3 m blocked
actor exceeds the scenario's 3.0 m radius limit, rejecting twelve inputs before
driving. Those errors and every valid failed physical recording remain retained.
Native turns also expose the preceding point/chassis reference mismatch.

The viewed follow-up keeps all numerical gates and the 0.35 m candidate reserve.
It explicitly enables the existing `local_route_geometry` mode for both
partners in the four narrow-road worlds, activating declared chassis/course
calibration in native dynamic driving. The blocked radius becomes a valid 2 m,
still too large to pass. Wide and inherited-mission controls remain unchanged.
The follow-up tests that explicit calibration composition; uncalibrated native
results remain failed. The current evaluator reproduces this follow-up.
The declared forward/braking/lateral limits are 2.0/2.5/2.5 m/s². An intermediate
follow-up omitted these required limits; the new preflight rejected its 48
narrow-road inputs before any driving, with all diagnostics retained.

The fixed physical requirements are no contact/road/rule violations, at least
1 m swept obstacle clearance and at least 0.1 m actual circular road margin.
A completed new pass must cover road progress 33–37 m at least 2.6 m from the
centerline; the occupied-left case must use the right. Blocked inputs must stay
at most 0.05 m/s over 14–20 s. Accuracy remains 0.3 m RMSE, 0.6 m maximum position
error and 0.1 rad maximum yaw error, including launch. The inherited mission's
failed goals and 99% forecast-coverage gates remain scored.

Independent scoring reconstructs finite-segment projection and the full
circular footprint at every physical tick. Every emitted trajectory point is
checked against the unchanged corridor, and every selected lateral target is
checked against the declared policy. Every tenth control tick additionally
receives the independent accelerated-path / growing-radius collision
certificate described in [obstruction stopping](obstruction-stops.md).

The policy-regression gate requires fitted functional success without any new
qualification failure relative to its fixed partner. Wide and inherited-mission
partners must retain identical output and summary bytes. The original inherited
mission failures are required to remain visible. Complete positive qualification
separately includes every accuracy/source/forecast failure. Disabled controls
retain their unsuccessful goals; these are authored viewed regressions rather
than held-out or automotive evidence.

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-planning --test road_width
cargo test --locked -p rustdriving-pipeline --test road_width
python3 scripts/test_road_width.py
# Both release executables must be built; choose a fresh output directory.
python3 scripts/check_road_width.py --check-policy-regression \
  --output artifacts/road-width
bash scripts/check.sh
```

Omit `--check-policy-regression` to require complete positive qualification.
Both exit conditions retain all row-level gates and losslessly archived raw
streams with encoded and decoded SHA-256 receipts.

`rustdriving validate-scenario --scenario FILE` checks scenario and pipeline
configuration without running a physical episode or creating output files.
The evaluator now validates its complete frozen cohort before starting any
physical job. Unsafe independent swept-clearance results are recorded
numerically and fail the same 1 m gate, without aborting before other metrics.

## Original recorded results and unmet adoption

The [complete result asset](../assets/road-width-results.json) contains every
frozen input, original classification, numerical reaudit, preflight rejection,
final outcome and source/binary/archive receipt. The final 64-case follow-up
retains the same numerical gates; its explicit calibration composition is
described above.

| Check | Result |
|---|---|
| Narrow-road goals, fixed / fitted | 0/18 / 18/18; fitted arrivals 26.80–27.45 s |
| Fitted narrow physical-pass requirements | 17/18 |
| Paired policy regression | 31/32; native right-pass seed 42 fails |
| Strict fitted qualification | 22/32, including all accuracy/source/coverage gates |
| Fitted blocked-road holds | 6/6, actual 0 m/s over 14–20 s |
| Wide and inherited-mission comparisons | 8/8 pairs retain identical output and summary bytes |
| Final minimum swept clearance | 0.845780 m; fails the unchanged 1 m floor |
| Final minimum actual circular road margin | 0.159867 m; no contacts or road/rule violations |

The remaining native right pass reaches its goal but has not completed the
shift at the obstacle. At the closest recorded tick (9.9 s), actual lateral
position is -2.584655 m and the selected target is -2.9 m. Actual lateral
distance over the complete passing window falls to 2.386459 m against the
2.6 m requirement. Heading stays within the 0.1 rad gate in this case. The
ordinary predicted-path collision certificate does not establish the separate
1 m physical clearance floor; controller feasibility and uncertainty remain
necessary work. At that revision, `--check-policy-regression` correctly exits 1.
The continuity follow-up above repairs this one physical failure.

Five fitted native launch-heading failures and all four inherited failed
missions also remain scored. The policy stays off by default. These authored
regressions do not qualify general narrow-road driving or automotive operation.

The first 64 attempts retain 52 valid physical recordings and twelve invalid
radius rejections. Their two native clearance failures (0.817371 and 0.850465 m)
remain in the original report. A numerical reaudit uses those same raw bytes
and creates no new physical evidence. Intermediate preflight rejects 48 missing
motion-calibration inputs and starts no physical jobs. All 64 final inputs pass
preflight before driving.

Every valid physical recording replays exactly: 116 episodes and 86,086 control
ticks across both physical cohorts. All 580 stored streams pass encoded and
decoded SHA-256 verification. The final cohort contains 64 recordings and
45,474 ticks. Independent checks verify every selected offset and emitted path
point, and certify 4,537 sampled planned paths through 341,653 interval bounds.
All 56 preceding recordings retain identical outputs over 42,447 ticks.

The final exact `bash scripts/check.sh` passes 798 Rust tests, 112 Python tests
and 49 standard drive/replay pairs, with all 245 standard outputs byte-identical.
The native adapter passes 63 tests, formatting, release build and strict Clippy;
its linked Rust sources remain unchanged during the root CLI/auditor follow-up.
An earlier complete check and an intentionally cancelled intermediate check
remain recorded alongside the final success. These are local Linux results.
Comparative engineering maturity remains **about 29%**.
