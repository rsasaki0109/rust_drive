# Braked standstill confirmation for navigation

`navigation.hold_for_handover: true` keeps requesting braking once the first new healthy stopped observation qualifies a pending route change. It prevents acceleration during the three-observation confirmation, including immediately after a measured localization reset. False/omitted preserves preceding behavior and serialization.

A confirmation starts only with a reachable candidate route, estimated and measured speed at most 0.05 m/s, permitted progress before the divergence, body/corridor margin and heading error below 0.35 rad. Cached motion does not start a hold or add a vote. Three distinct healthy stopped acquisitions are required to switch. Faults and localization admission retain their existing guards.

The hold latches through missing/unhealthy motion, movement, misalignment and a newly blocked replacement. These interruptions discard votes; health faults request ordinary emergency braking. An accepted closure cancellation returning to the active route or successful confirmed handover releases the hold. An explicit new localization epoch clears pose-dependent holds and votes, and must confirm again. Reordered snapshots cannot release a hold. There is no timer that authorizes acceleration when observations fail: the vehicle waits until the existing observation and route conditions pass or a valid external cancellation is accepted.

This changes command retention only. It supplies no additional sensing, moving-route handover, traffic-control recovery or automotive calibration. The simulation uses truth only for synthesis and independent physical scoring.

## Qualification

The complete preceding 32 navigation-recovery inputs are frozen again with only the new flag enabled. Original map geometry, endpoints, closures, sensor schedules, seeds, mapped speed limits and physical gates stay fixed. Twelve prior late-closure episodes form byte-verified baselines; their old physical streams are retained rather than rerun. The twelve late-closure cases must stop-confirm the detour within 0.5 s of measured reset and retain physical speed <=0.05 m/s during confirmation. Each paired arrival must improve by at least 2 s. Four already-switched, four no-route, four reopened, four healthy and four disabled controls retain their previous contracts. Seeds are regression inputs, not unseen automotive data.

The independent navigation state fold reconstructs closure ownership, paths, pose admission, motion acquisition watermarks, votes, holds and releases from the measured input stream; complete localization epoch/covariance and physical swept-speed/clearance checks remain active. Full exact replay and tamper tests verify the real pipeline and command stream. Invalid-epoch accuracy errors remain scored.

## Results and the missed comparison gate

The first complete 32 physical episodes pass their independent navigation,
localization-epoch, speed, clearance and outcome checks. They cover 30,010
control steps with zero contacts or road violations. Twenty recovered missions
reach their original goals; four recovered no-route missions stop. Four healthy
goals and four disabled-recovery holds retain their existing outcomes.

| Fixed check | Observed result |
| --- | --- |
| Late-closure reset to switch <=0.5 s | All 12 switch in 0.20 s; prior paired runs take 7.55–8.40 s. |
| Physical confirmation speed <=0.05 m/s | All 12 hold at 0 m/s during confirmation. |
| Arrival improvement >=2 s in every pair | **5/12 pass; seven miss.** All 12 improve by 1.90–2.85 s. |
| Original mapped speed excess <=0.15 m/s | Maximum 0.080641 m/s; zero violating intervals. |
| Valid-epoch RMSE / maximum position error <=0.2 / 0.4 m | Worst 0.012401 / 0.072098 m. |

**The full preregistered comparison remains unmet.** Its two-second threshold
has not changed. The old run travels along the common prefix during much of its
handover delay, so the arrival gain is smaller than the reduction in switch
latency. These are regression seeds and authored maps.

Twenty-four general CLI accuracy failures remain scored, including errors while
motion is invalid; the global maximum is 3.275095 m. Passing the valid epochs
and stopping checks does not remove those failures.

## Fault recovery repair

A separate fixed sequence exposed a gap after malformed navigation input:
`Braking` with a retained hold, then `Fault`, then a valid still-closed snapshot
became `Blocked` and requested +2.0 m/s². The first implementation had discarded
the hold during the fault. That complete failed input/output recording is saved.

The final implementation retains an armed hold through the navigation fault.
The same inputs now request -6.0 m/s² during `Fault` and -0.5 m/s² after the
valid still-closed snapshot. A later valid cancellation releases the hold; a
subsequent detour still requires three new stopped observations. A separate
raw-input state fold verifies all seven observations, and exact replay verifies
the complete pipeline. This sequence checks command retention; the 32 original
episodes supply physical motion coverage.

The exact required `bash scripts/check.sh` passes 747 Rust tests, 73 Python
auditor tests and 49 ordinary drive/replay pairs. All 245 standard outputs keep
their preceding bytes. The native build, 63 tests and strict Clippy also pass.
The source, failed fixture, original physical binaries and streams remain
preserved in `artifacts/stopped-handover-v1`; the fault repair does not repeat
or replace the physical episodes or change their gates.

The final runtime reproduces all 578 preceding recordings and all 32 first
physical episode logs: **610/610 exact output streams and replay reports**.
This checks the original physical outputs across the separate fault repair.
[Published cases, missed comparison gates, hashes and validation](../assets/navigation-handover-results.json).

## Reproduce

After the ordinary root/native builds, install the pinned regional-recovery
Python dependencies and run the per-episode checks:

```sh
python3 -m pip install -r scripts/requirements-relocalization.txt
python3 scripts/check_navigation_handover.py --output artifacts/navigation-handover
```

`--plant reference-2d` / `--plant rne-dynamic` selects a plant. `--smoke` selects
12 cases per plant; the full run has 32 across both. The CLI scores individual
physical/epoch/navigation checks. The separate paired arrival comparison uses
the preserved preceding 12 physical recordings, whose frozen inputs differ only
by the explicit hold flag. The published evidence records that comparison's
seven failures alongside the individual checks.

To regenerate the preceding default baseline from this revision, also run:

```sh
python3 scripts/check_navigation_relocalization.py --output artifacts/navigation-handover-baseline
```

Match the twelve `late-closure` rows by plant, world and seed. Subtract the new
`summary.simulated_seconds` from the corresponding default value and require
at least 2 s for each pair. Both materializers retain identical inputs after
removing `navigation.hold_for_handover`; a fixed-input unit test checks this.

Comparative engineering maturity remains **about 29%**, a subjective comparison
with Autoware, Apollo and openpilot. This improves consistency and availability
within the existing authored mission domain; the automotive 30% waypoint is
still open.
