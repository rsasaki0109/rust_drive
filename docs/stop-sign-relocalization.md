# Preserve mapped stop obligations through measured recovery

`relocalization.preserve_stop_signs: true` lets a stopped, measured map reset retain fixed-route stop duties. A served stop behind the newly measured vehicle front remains served. Every unfinished stop, and every served stop now ahead of the measured front, requires a new continuous two-second healthy stop. The option defaults to false/omitted; ordinary regional recovery still rejects mapped stops without the explicit flag.

```json
{
  "stop_signs": [{"id": "stop-1", "route_s_m": 35.0}],
  "relocalization": {"preserve_stop_signs": true}
}
```

Supply the fixed route and surveyed `localization_map` as usual. The existing regional search requires fresh stopped wheel/gyro samples and three consistent synchronous XY scan fits. Acquisition-time GNSS, bounded map-only support and confirmed GNSS handoff retain their existing independent policies. Signals require the separate [signal-preservation option](signal-relocalization.md). This does not enable live route changes with mapped controls, yielding intersections, lane changes or XYZ recovery.

## State and clock ownership

At the measured reset, stop admission uses the original route and vehicle radius. Estimated speed must be at most 0.05 m/s; heading error must be below 0.2 rad and the measured center-to-route distance plus body radius and 0.2 m must fit strictly within the corridor. Every unserved stop must leave at least 0.5 m before the measured vehicle front. A violated condition latches `InvalidStopSignRecovery` and ordinary emergency braking. Later healthy GNSS cannot clear this fault; a separate admissible measured reset is required.

`Released` or `Passed` stops behind the measured front retain their phase and completed dwell. Those now ahead are rearmed. All other holds and elapsed times are discarded, and the acquisition watermark advances to the reset. The confirming acquisition cannot count twice. New health, route alignment, fresh motion and continuous near-line standstill are required for the entire two seconds; cached samples and missing intervals cannot credit dwell. `stop_signs.localization_reset` records the measured geometry, admission and retained/rearmed/too-close IDs. Exact replay recomputes that evidence and braking.

Fixed mapped stops also compose with planner-local route fitting. Stop constraints first truncate the original route at its surveyed arc; local fitting preserves that endpoint. Stop admission and physical crossing evaluation continue using the original route. This permits the native dynamic plant's existing delayed-GNSS chassis calibration without changing stop coordinates. Signals now use the same original-prefix contract in their separate [qualification](signal-relocalization.md); yielding remains excluded from local fitting. Rotated right-angle fixtures test stops before and after the corner with both chassis and rear-axle pose references; an unrepresentable temporary fitted prefix fails closed.

## Qualification

The protocol freezes 32 cases across reference bicycle and native RNE dynamic plants: straight and curved 120 m roads, seeds 1/7/42 before the first sign, and seed 7 for interrupted dwell, release before crossing, an already passed sign followed by another sign, healthy controls and disabled recovery. Cruise is 4 m/s with a 65 s limit. Wheel motion disappears for three seconds and GNSS for eight, at fixed delivery/acquisition schedules. Surveyed map points and stop signs enter the declared operational configuration; dropout schedules, physical truth and acceptance gates stay in synthesis/evaluation. Recovery receives timestamped sensor observations.

The independent checker reconstructs stop phases and timers from raw acquisitions and surveyed geometry. It also reconstructs the regional search and complete post-reset localization matrices, evaluates physical continuous dwell and crossings, and sweeps the physical route/clearance. It keeps every invalid-epoch error in the global CLI scores. A small physical error during a stationary outage still does not qualify the missing-motion interval.

All 32 cases in the follow-up protocol pass: 24 recovered goals, four healthy goals and four disabled-recovery holds, covering 28,719 control steps. There are zero contacts, road violations or stop-sign violations. Minimum physical clearance is 2.229210 m. Before each sign first satisfies the independent physical two-second stop rule, minimum front-line margin is 1.959108 m. The worst valid-epoch RMSE / maximum position error is 0.009676 / 0.031713 m, with maximum yaw error 0.001536 rad.

Every one of 24 post-reset healthy releases also has an independently verified new physical standstill of at least 2.00 s; maximum actual speed during those holds is 0.048474 m/s. Physical dwell requires actual speed at most 0.1 m/s and circular-front margin within 0–3.5 m. The original 1 m margin gate applies before first physical fulfillment, which persists across epochs. Previously served signs rearmed by the driver can reconfirm closer: minimum observed post-reset margin is **0.664137 m**, under the separate 0.5 m measured-reset admission bound and physical hold window. Stop state diagnostics do not supply physical acceptance.

**The original first protocol remains unmet.** After repairing the omitted-zero schema, 14/16 first reference cases passed; the straight interrupted-hold case had only 0.10 s dwell and the intended released case was still holding. The follow-up freezes faults at 11.2 / 12.9 s for these two modes, using already viewed healthy timing. All gates remain unchanged. Twelve unchanged reference streams are reused, while four distinct follow-up reference and sixteen native cases supply the remaining evidence. All 36 physical recordings, first exceptions, first failed coverage checks and native launch logs remain preserved. These are authored regressions, without an unseen-data claim.

20 general CLI accuracy failures remain scored, including missing-motion epochs; the global maximum error is 1.547922 m. Passing the new-epoch checks does not remove those failures.

The exact required `bash scripts/check.sh` passes 755 Rust tests, 78 Python auditor tests and 49 standard drive/replay pairs. All 245 standard output files retain their previous bytes. Native fmt/build, 63 tests and strict Clippy pass. The final runtime reproduces all 610 preceding and all 36 current physical recordings exactly: **646/646 output streams and replay reports**. Separate measured-reset sensor fixtures also retain exact replay. Validation was observed on Linux x86_64 only.

The first reference classification incorrectly required an explicitly serialized zero violation count. The final checker respects the existing omitted-zero schema; all original physical streams and exceptions remain retained, and the same streams are reclassified without changing inputs or gates. The first native launch rejected the existing delayed-GNSS/local-route/stop composition before producing physical streams. The final implementation admits fixed stops using unchanged surveyed prefixes; the original launch logs remain retained.

A separate measured-reset input sequence verifies a full new dwell, while another places an unserved sign behind the measured front and verifies that fresh GNSS cannot release emergency braking. These are sensor-input regressions, separate from physical driving qualification.

[Published per-case outcomes, hashes and validation](../assets/stop-sign-relocalization-results.json) identify the immutable local packet `artifacts/stop-sign-relocalization-v1`.

## Reproduce

After the ordinary root and native builds:

```sh
python3 -m pip install -r scripts/requirements-relocalization.txt
python3 scripts/check_stop_relocalization.py --output artifacts/stop-relocalization
```

`--plant reference-2d` or `--plant rne-dynamic` selects one plant. `--smoke` selects 12 cases per plant; the full run has 32 across both. CI runs both plant smoke protocols. Exact input/output replay and negative checker tests remain part of validation.

Comparative engineering maturity stays **about 29%** against Autoware, Apollo and openpilot. This extends authored simulation recovery into fixed stop rules. It does not establish independent automotive sensing, measured uncertainty, real-time performance or vehicle operation; the automotive 30% waypoint remains unmet.
