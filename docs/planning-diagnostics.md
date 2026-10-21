# Recorded planning decisions

`record_planning_diagnostics: true` optionally records why the planner accepts
or rejects each enumerated lateral candidate. It belongs to simulation scenarios
and the sensor-only `PipelineConfig`; it stays off by default. Independent callers
use `LatticePlanner::set_diagnostics_enabled` and `diagnostics()` without changing
the core `Planner` trait or adding an algorithm dependency.

## Record contract

`PipelineOutput.planning_diagnostics` contains at most three or nine candidate
records, in their original evaluation order. Every enumerated candidate receives
one record, including candidates discarded before collision checking. Dispositions
distinguish an unusable side corridor, an opposite-side transition during a pass,
geometry outside the corridor, an unreachable speed profile, a blocked refined
path, failure to find a checked stop, a complete path and a checked stopping path.

The first conflict on each nominal path names the measured prediction identity,
relative conflict time, occupied radius, forecast horizon and clearance-policy
floor. A zero floor retains the original time/motion reserve; refined proposals
retain the existing 1 m floor. These are conservative sweep bounds: chord and
radius inflation can reject a path before its actual centers touch. They are
neither physical collision witnesses nor evidence that every continuous path
through the road is impossible. Stop retiming occurs after that nominal conflict.

Accepted candidates retain a bounded profile summary: number of points, final
route progress, final relative time, terminal speed and whether the stop-search
fallback produced the profile. The selected index binds that summary to the
actual trajectory. Published progress uses the supplied active route, including
any mapped-duty prefix; local fitting can use a separate internal planning arc.
Relative times start at the current planning call. The scene acquisition and
refinement confirmation stamps use the original accepted sensor clock.

The record also retains fresh refinement confirmation and obstruction braking/
holding duties. Invalid calls replace the preceding record with an explicit
failure and no candidate list. `NoSelectableCandidate` can include a complete
refinement still awaiting confirmation. Route reset and disabling capture clear
old records. Disabled capture allocates no candidate records.

These records describe the planner proposal. Existing pipeline faults and mapped
duties still own final mode/command overrides; the ordinary `health`, `emergency`
and command fields retain that evidence. Capture does not change candidates,
costs, motion limits, forecast budgets, collision checks or stop ownership.

## Stop diagnosis and discarded experiments

Four earlier diagnostic runtime variants tested all eight incomplete mission
conditions: a later initial stop, an expanded 3.9 m side cap, both changes, and
fine checked-stop sampling with a forward-progress preference. All 32 executions
retain the original world, sensing, forecast budgets, 65-second goal deadline,
1 m physical clearance, 0.1 m road reserve and localization/coverage gates.
Every proposed stopping trajectory still passes the original complete sweep
and eight-second stationary hold. **None reaches the goal.** Some variants also
add forecast-coverage failures. Their sources, frozen inputs, binaries, complete
failed outcomes and raw recordings remain retained; these planning changes are
discarded from the delivered runtime.

The recorded reference seed-7 no-heading example at 20 s explains the retained
stop directly. All nine nominal candidates encounter forecast identity 1,
with first conservative conflicts 2.834–3.211 s into the proposed motion.
Their occupied radii at those times are 2.578–2.805 m, including the declared
future-motion envelope. The three original choices retain checked stationary
holds; all six intermediate paths fail their complete nominal sweep. The chosen
center hold ends at supplied-route progress 45.399260 m and retains its full
eight-second tail. No refined path is waiting for confirmation in that frame.
This identifies the present lattice's forecast bottleneck without assuming an
unseen actor intention or weakening its uncertainty budget.

## Qualification and inspection

The fixed qualification adds only the capture flag to all 44 preceding input
conditions. It checks complete sensor replay, candidate bounds, forecast identity/
radius/horizon provenance, confirmation clocks and selected-profile binding.
Removing the final diagnostic JSON field preserves every original operational
output byte without reserializing floats. Physical frames and complete summaries
are compared with the preceding archived runs. Original mission, clearance and
forecast failures remain scored separately from capture compatibility.

All 44 capture comparisons pass across both plants: 28,887 control ticks,
233,919 candidate records, 117,920 nominal forecast conflicts and 784 fault-braking
ticks. Every projected operational output byte, physical frame and complete
summary matches the preceding recording. Strict physical qualification remains
28/44: eight paired three-target controls retain clearance failures and eight
original missions remain incomplete, including three forecast-coverage failures.
All 7,208 healthy standstill ticks at or after 20 s in those eight missions report
conflicts on all nine nominal candidates, with no pending refinement confirmation.
These repeated time samples are correlated evidence from eight viewed conditions.

`inspect_planning.py` validates and summarizes fixed-route recordings without
navigation, lane changes or mapped stop prefixes. Those changing-route numerical
compositions remain outside this independent inspector's scope; the runtime API
still records its actual supplied-route progress and complete replay reconstructs
the captured output. The inspector checks record provenance, not the mathematical
completeness of the candidate search. A closed-form Rust test separately checks
the conservative conflict time on a straight accelerating approach. Other tests
cover fault override, stale-record clearing, acquisition confirmation and replay
rejection after a changed reason or capture policy.

The unchanged-source `bash scripts/check.sh` passes 878 Rust tests, 172 Python
tests and 49 standard drive/replay cases; all 245 standard outputs retain exact
bytes. Native formatting, 65 tests and strict release all-target Clippy pass.
All 420 preceding supported sensor logs retain exact replay output bytes.
All 380 archives across 76 physical executions have encoded and decoded SHA-256
verified. The standalone inspector audits the supplied tick stream; complete
Rust replay separately verifies transport and footer integrity. CI includes the
fixed-route comparisons on Linux; remote results for this branch remain unverified.

```sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/test_planning_diagnostics.py
# Both commands require fresh output directories.
python3 scripts/check_refined_lateral.py --output artifacts/decision-baseline
python3 scripts/check_planning_diagnostics.py --baseline-output artifacts/decision-baseline --output artifacts/planning-decisions
python3 scripts/inspect_planning.py --log artifacts/planning-decisions/episodes/reference-2d/mission-search-seed-7/run/sensors.jsonl.xz
bash scripts/check.sh
```

The [result packet](../assets/planning-diagnostics-results.json) retains the
complete comparison, discarded variants, source inventories and archive receipts.
The preceding eight missions remain incomplete. This adds observable decision
evidence; it does not repair forecast calibration or qualify vehicle, real-time
or cross-platform operation. Comparative maturity stays **about 29%**, with
the 30% waypoint still open.
