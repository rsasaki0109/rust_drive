# Acquired planning progress status

`monitor_planning_progress: true` adds an optional sensor-only
`PipelineOutput.planning_progress` status. An application can distinguish a
prolonged stationary forecast block from ordinary progress without parsing
thousands of trajectories or waiting for the simulation deadline. The monitor
observes the current lattice; it does not reroute, release a stop, alter health,
change a command or establish that every possible route is infeasible.

## Evidence contract

The state becomes `StationaryForecastBlocked` only after at least three distinct
accepted scene acquisitions span two seconds. Consecutive acquisitions must be
at most 0.35 s apart, and the latest must be no more than 0.35 s old. Control
ticks that reuse a scan neither add acquisitions nor extend the supported
duration. A scan consumed while evidence is invalid cannot seed a new prefix
when reused later.

Every supporting decision must select a checked stopping path. None of the
enumerated candidates may offer a complete path, including a complete refined
candidate awaiting confirmation. At least one nominal path must have a recorded
forecast conflict. The sorted set of first-conflict identities must stay the
same; it contains at most nine identities. This reports the available lattice's
decisions rather than a complete search of free space.

Measured speed and every proposed point speed must be at most 0.2 m/s. All
proposed positions must remain within 0.2 m of the current estimated position.
The estimated position and supplied-route progress must also stay within 0.2 m
of their first supporting acquisition's anchor. A chain of small movements
cannot accumulate standstill credit. Remaining route length must exceed 2 m,
the selected mode must be `Yield`, the actual command must be nonpositive, and
the pipeline must be healthy without an emergency or temporary mapped stopping
route.

A fault, goal stop, mapped stop, complete candidate, acceleration command,
motion, identity change or unsupported acquisition clears the status. Position,
progress and blocker changes withdraw support even on cached control ticks.
They cannot start a replacement prefix until a new acquisition arrives. Route
handover and lane-route changes reset the evidence. Localization epoch reset
creates a fresh monitor with fresh identity scope.

`Observing` may include a still-maturing prefix. The report exposes its first
and last supporting acquisition times, acquisition count, acquired duration and
sorted conflict identities. These are observation times, not simulated future
collision times or control-clock dwell. Estimated standstill does not certify
physical immobility outside the evaluated conditions.

## Integration

The flag is false by default and omitted when false. The output field is omitted
when disabled. The monitor requests bounded internal planner decisions even if
`record_planning_diagnostics` is false; that mode exposes only the small progress
status. Both flags can be enabled when the application needs candidate details.
The extra decision sweeps have bounded size but no real-time latency claim.

The independent inspector requires both flags and planar fixed-route logs.
It reconstructs supporting prefixes from acquired inputs, poses, selected paths
and conflict records, checks every published field, and uses the existing
independent scan/forecast intersection reconstruction to exclude mapped yield
waits. Navigation, lane changes, stop signs/signals, layered/XYZ sensing and
localization reset remain outside the inspector's supported scope. The runtime
still resets or suppresses its observer using the active pipeline context.
The inspector checks the supplied ticks. Complete Rust replay separately checks
transport, footer counts and exact recomputation of the status and commands.

```sh
python3 scripts/inspect_planning_progress.py --log PATH/sensors.jsonl.xz
python3 scripts/test_planning_progress.py
python3 scripts/check_recent_track_motion.py --output artifacts/progress-parent
python3 scripts/check_planning_progress.py --baseline-output artifacts/progress-parent --output artifacts/progress-check
bash scripts/check.sh
```

The qualification freezes all 32 preceding conditions before physical execution,
adding only the monitor and decision-capture flags. Removing the two final
optional JSON fields preserves the original float spelling and every operational
output byte. Complete physical frames, summaries and acquired inputs are
compared with the preceding archives. Original mission deadlines, sensing,
forecast budgets, clearance, road, accuracy and coverage gates remain scored.

## Retained development checks

The first full execution completed all 32 conditions. Its inspector accepted
28, but rejected the four mapped intersection recordings because their context
was outside its initial scope. Those failures and their raw recordings remain
retained. The final inspector reconstructs mapped yield ownership from actual
scan clocks, poses and forecasts before suppressing evidence.

Review also found that a cached tick could retain an old status after a pose or
blocker change. A new regression fails against the retained first Rust source;
the first Python inspector accepts the stale status, while the final inspector
rejects it and accepts immediate withdrawal. Final runtime tests cover both
cached pose and identity changes, acquisition gaps, over-sampling, faults,
goals, movement, complete alternatives, replay tampering and disabled output
compatibility. None of these refinements changes the driving policy.

## Final qualification

All 32 final comparisons pass the monitor, independent acquisition-prefix,
mapped-yield context, complete replay and passive-capture contracts across the
reference and native dynamic plants, with seeds 7 and 42. Every one of 27,347
operational outputs preserves its original bytes after lossless removal of the
two optional fields. Every physical frame, complete summary and acquired input
also matches the preceding recording. All prior semantic scores, failed gates
and coverage denominators remain identical.

Twenty conditions produce 9,872 notified control ticks: all eight incomplete
original missions, four permanent obstructions, and eight temporary obstruction
release cases. Original missions notify at 11.6–12.6 s and retain their checked
stops through the unchanged deadline. All eight temporary cases withdraw the
notification at 12.2 s and subsequently reach their original goals. The twelve
other conditions produce no notification, including the four mapped intersection
cases. Actual speed is zero at every notified physical sample in this cohort.
These repeated frames are correlated evidence from the stated conditions.

Source goals remain **16**, strict driving qualification remains **0/32**, and
the eight incomplete missions, 26 forecast-coverage failures and five accuracy
failures remain open. Minimum physical swept clearance is 2.876505 m; minimum
road margin is 0.643533 m. Both values are unchanged. This feature makes the
prolonged block observable without repairing its planning/forecast cause.

Native formatting, 67 release tests and strict all-target Clippy pass. All 624
preceding sensor logs, covering 438,787 control ticks, preserve exact replay
output bytes. All 320 archived streams across both development cohorts have
their encoded and decoded SHA-256 checked. CI includes both plant comparisons;
remote and cross-platform results remain unverified.

The unchanged-source `bash scripts/check.sh` passes 939 Rust tests, 230 Python
tests and 49 standard driving/replay scenarios. All 245 standard output files
preserve exact preceding bytes. The operational source inventory remains
unchanged throughout the required check.

The [result packet](../assets/planning-progress-results.json) records complete
outcomes, frozen inputs, retained failures, source/binary hashes and regression
controls. Comparative maturity stays **about 29%**; the 30% waypoint is unmet.
