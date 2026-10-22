# Observed obstructions and stopped detours

The subsequent [pose covariance forecast envelope](pose-forecast-envelope.md)
repairs the eight new detours' physical forecast-coverage failures under the same
worlds and gates. The original qualification below is retained unchanged.

`navigation.avoid_observed_obstructions: true` lets a supplied road graph offer
another route when acquired tracks persistently block every current nominal
local candidate. It requires `navigation.hold_for_handover: true`. The option is
false by default and is omitted from older configurations and outputs.

## Acquisition and ownership

The observer uses the last accepted LiDAR acquisition, current estimated tracks
and the preceding local planning decisions. Every nominal candidate must record
a conflict with the same identity, and no candidate may offer a complete path.
The preceding decision must be at most 0.15 s old. The matching track must belong
to the current scan, move at no more than 0.2 m/s, and have radius at least 0.4 m.

At least three distinct acquisitions must span 0.2 s, with gaps and current age
at most 0.35 s. The measured center must remain within 0.15 m of the first
supporting center. Cached scans cannot vote; incompatible cached evidence clears
the probe. Faults, newer external updates and non-following navigation withdraw
support, consuming the acquisition so that a reused scan cannot seed recovery.

The track must project ahead of the estimated vehicle by its radius plus 2 m.
Exactly one active directed traversal must contain the measured disc inside its
road band, with radius plus 1 m of interior reserve at each traversal endpoint.
Association uses map-edge arc coordinates, including partial origin edges.
These observations propose an advisory mission-local edge preference; they do
not establish a legal closure or prove that all possible paths are blocked.

At most eight preferences are retained for the current mission. There is no
automatic expiry or reopening. A new pipeline/mission starts a new ledger.
External closure snapshots retain their own revision and `closed_edges` values;
route search uses the union of external closures and local preferences. External
reopening cannot erase an acquired preference. `NavigationStatus.observed_obstructions`
records the original acquired evidence and the latest proposal decision.

## Stopped route change

The navigator previews the exclusion transactionally using the existing directed
graph search, original endpoints, legal turns, widths and mapped speed limits.
It also checks normal braking reachability before the first divergent/excluded
traversal. `NoAlternative`, `UnreachableDivergence` and `CapacityReached` leave
the active route, stop ownership and external closure knowledge unchanged.

A valid proposal enters the ordinary braking handover. Three fresh, healthy,
stopped wheel/gyro observations and an aligned estimated pose are still required
before switching. The existing brake hold remains active during confirmation.
An irrelevant newer external snapshot resets votes while preserving a pending
route if a fresh graph search still chooses that same route. A changed route or
invalid snapshot follows the existing replan/fault behavior.

The proposal never authorizes movement by itself. The existing collision-checked
local planner and controller still govern motion after switching. Collision
margins, eight-second terminal checks, forecast budgets and motion limits remain
unchanged. Planner decisions can be captured internally without emitting the
large optional diagnostic output. Regional relocalization composition is
rejected; automatic live-position map matching and moving handover remain open.

## Evaluation protocol

The paired protocol freezes 44 authored conditions: two plants, seeds 7 and 42,
and eleven variants. Positive pairs have a static acquired obstruction on the
preferred road and a supplied alternate road. Negative pairs remove the
alternate, introduce the obstruction after the fork, or use a moving crossing
body. Separate external-update pairs close and reopen an unrelated road. A
known-closure positive control supplies an explicit main-road closure.

Each enabled/disabled pair changes only the new navigation flag. Original body,
map, sensors, calibration, 70-second deadline (90 s for the moving control),
1 m physical clearance and acceptance remain identical. The alternate geometry
is an explicitly new authored fixture; it does not repair the eight preceding
single-route mission failures. Before implementation, all 24 disabled/control
conditions were run with parent commit `d625482` and retained in full.

The independent Python observer reconstructs acquisition prefixes, stable
identity/geometry and prospective graph exclusions. A separate navigation fold
checks every external revision, pending route, stopping boundary and switch.
Local candidate decisions are independently checked against each reconstructed
active route. Physical audits retain full pose errors, relative-body swept
clearance, active-road reserve, true stopped switching, acquired-disc geometry,
forecast coverage and braking holds. Expected unsolved controls retain their
original failed goal gate in the strict score.

```sh
python3 scripts/test_observed_detours.py
python3 scripts/check_observed_detours.py --output artifacts/observed-detours
bash scripts/check.sh
```

The checker distinguishes the observed-route contract and paired regression
result from strict driving qualification and default-adoption acceptance.
Negative pairs must preserve every operational output byte after lossless
removal of the optional navigation report, as well as physical frames, summaries
and acquired inputs. An optional `--parent-cohort` additionally checks all five
complete recording streams of each disabled run against retained parent bytes.

## Recorded result

All 44 final physical recordings pass the acquired-route, independent navigation,
local-candidate, physical clearance, road, pose and exact replay contracts,
covering 55,509 control ticks. Eight enabled detour/external-update conditions
reach their goals versus zero paired disabled conditions. Observed proposals
occur at 6.4–7.6 s; the ordinary navigation fold independently verifies every
stopped switch. Maximum true/estimated speed at any switch is 0.003372 m/s.

All twelve moving/no-alternative/late pairs preserve 15,349 operational output
lines byte for byte after removal of the final optional navigation field.
Physical frames, complete summaries and acquired inputs also match. All 24
disabled/control recordings preserve all five parent recording streams exactly.
External closure and cancellation pairs retain the acquired main-road preference
without forging an external revision. No-alternative and late proposals leave
the original route intact, and moving-body controls do not propose exclusions.

Strict qualification is **16/44**. Twenty conditions fail the unchanged forecast
coverage gate; eight disabled detour controls retain their missed-goal failure.
All eight newly completed detours introduce a forecast-coverage failure relative
to their stationary disabled controls. **Paired regression and default adoption
fail**. Total goal arrivals across enabled/disabled/control recordings are 20;
this is not an acceptance rate. Minimum physical swept clearance is 4.594923 m,
minimum active-road margin is 0.879141 m, maximum position error is 0.275414 m
and maximum heading error is 0.017222 rad. These are correlated samples from
authored conditions, with no measured automotive calibration claim.

For the eight new detours, the failing 0.2 s coverage fraction is 91.83–98.02%
against the unchanged 99% requirement. Their denominators are 252–257 acquired
forecasts, versus 641 in the stopped controls; missed samples are retained in
both. The reference seed-7 pair has five misses versus four, and native seed 7
has 21 versus six. Short-horizon physical shape/center coverage needs work;
remaining stopped longer would change the denominator without repairing those
misses.

The first canary already reached its goal, but its reader rejected serde's
omission of an empty update schedule. The first 44-run reader then rejected 40
records because initial route-history events are present only with external
updates. Both failed reader reports and raw executions remain retained. Final
audits normalize that documented serialization and independently reconstruct
the initial route from the frozen request and physical frames. They re-audit
the same recorded executions, without another physical trial or changed inputs.

CI uses `--check-contract` explicitly to verify the route/physical and unchanged
negative-control contracts while retaining `regression_passed: false`,
`passed: false` and `adoption_passed: false` in the report. The default checker
still returns failure for those retained regressions. The option stays off.

The unchanged-source `bash scripts/check.sh` passes 949 Rust tests, 244 Python
tests and 49 standard driving/replay scenarios. All 245 standard output files
preserve prior bytes. Native formatting, 67 release tests and strict all-target
Clippy pass. All 656 preceding logs, covering 466,134 control ticks, retain exact
replay output bytes. All 345 archived streams from the 24 parent controls, first
canary and 44 final executions have their encoded and decoded SHA-256 verified;
the final re-audit references the same 220 execution archives. Remote CI and
cross-platform results remain unverified.

The [result packet](../assets/observed-detours-results.json) includes the complete
44-condition matrix, frozen inputs, all strict failures, paired comparisons,
parent byte proofs, reader failures and tested source/binary/lock hashes.

## Limits

This is a planar simulation feature on supplied directed maps. Stable proxy
discs are not calibrated physical shape estimates, and forecast coverage still
requires independent evaluation. Retained preferences can exclude a road after
an obstruction moves away. Broader topology, repeated obstructions, measured
automotive sensing, real-time performance and real-vehicle operation are
unverified. Comparative engineering maturity remains about 29%; the 30% waypoint
is still open.
