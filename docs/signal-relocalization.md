# Fresh signal permission after measured localization recovery

`relocalization.preserve_traffic_signals: true` lets a measured, stopped map reset retain fixed-route signal duties. It preserves previously committed lines behind the new measured front and rearms committed lines now ahead. Every accepted color and its acquisition clock are cleared: a green acquired at or before the reset cannot release a pending line.

```json
{
  "stop_lines": [{"id": "main", "route_s_m": 55.0}],
  "relocalization": {"preserve_traffic_signals": true}
}
```

The supplied route, surveyed map and synchronous XY scans retain the existing [regional recovery](regional-relocalization.md) contract. The option defaults to false/omitted; without it, regional recovery still rejects mapped signals. Stop signs require their own explicit preservation flag. Live navigation with mapped controls, yielding intersections, lane changes, XYZ/multiheight recovery and global initialization remain outside this mode.

## Ownership and observation time

Admission uses the newly measured pose against the original route. Estimated speed must be at most 0.05 m/s, heading error below 0.2 rad, and route distance plus vehicle radius plus 0.2 m strictly below corridor half width. Every remaining line must leave at least 0.5 m before the measured circular front. A missed uncommitted line, close remaining line or invalid geometry latches `InvalidTrafficSignalRecovery` and -6 m/s² emergency braking. Fresh GNSS and green snapshots cannot pardon that admission failure; a separate admissible measured reset is required.

A previously committed line behind the measured front retains its passage flag; its newly observed red cannot send the vehicle back. A previously committed line now ahead loses permission and must be considered again. Current green never supplies evidence of a past legal crossing. `traffic_controls.localization_reset` records the measured admission and retained/rearmed/too-close identities, and exact replay recomputes them.

All signal permission is withdrawn at reset, including a valid green delivered in the confirming frame. Only a complete snapshot with an acquisition stamp strictly newer than the reset can establish new permission. The existing 0.5-second accepted-age bound still applies. Cached/reordered green cannot refresh the clock or replace newer red. Missing/expired observations become unknown; malformed snapshots retain their separate fault until later valid evidence. Base sensor/localization health still gates motion and passage commitment. An independent latched stop-duty admission failure also blocks signal passage commitment.

The nearest pending nonpermissive line constrains the original surveyed route before planner-local corner fitting, preserving its endpoint. This composes with the native dynamic plant's declared chassis calibration and with explicitly preserved fixed stop signs. Rotated right-angle controls check original stop coordinates before and after a corner with both rear-axle and chassis references. An unrepresentable temporary fitted prefix fails closed.

## Fixed qualification

The frozen 28 cases use two plants (reference bicycle/native RNE dynamics), straight/curved 120 m roads, seed 7, 4 m/s cruise and a 65-second deadline. Seven modes cover red-to-green, expired last green with missing infrastructure observations, permanent red, an already passed line followed by another signal, a stop-sign composition, healthy sensing and disabled recovery. Fault cases lose wheel motion at 8–11 s and GNSS acquisitions at 8–16 s. Green returns or red changes at 22 s; signal-loss cases withhold acquisitions at 8–22 s. The passed-line case uses original arcs 20/80 m; the mixed case adds a stop at 35 m before its 55 m signal.

Map geometry and acquired complete infrastructure snapshots are declared operational inputs. Signal phase schedules, dropout windows, actor truth and acceptance gates remain in synthesis/evaluation. The independent auditor folds raw acquisitions and measured route projections to reconstruct colors, clocks, passage ownership and admission. It separately reconstructs regional search and full localization matrices, and checks actual fronts against both true and newly observed permission, original-corridor containment and continuous collision clearance. Invalid-epoch errors remain in full-interval CLI scores.

All **28/28** fixed cases pass: 16 recovered goals, 4 healthy goals, four permanent-red holds and four disabled-recovery holds, covering 26,598 control steps. Contacts, road violations and signal violations are zero. Minimum physical clearance is **2.236097 m**; minimum actual-front margin under true or acquired nonpermissive permission is **1.953930 m**, against the fixed 1 m floor. Permanent-red controls remain stopped from 30 s through 65 s, with maximum actual speed **0.000000 m/s**. All four mixed stop-sign cases also pass their independently reconstructed dwell and physical crossing checks.

Worst valid-epoch RMSE / maximum position error is **0.009611 / 0.032818 m**; maximum yaw error is **0.001204 rad**. The 24 general CLI accuracy failures remain scored, including invalid motion epochs; global maximum position error is **1.678619 m**. The feature protocol does not qualify those missing-motion estimates.

The exact required `bash scripts/check.sh` passes **764 Rust tests**, **85 Python auditor tests** and **49 standard drive/replay pairs**. All **245** standard output files retain their preceding bytes. Native fmt/build, **63 tests** and strict all-targets Clippy pass. The final binary reproduces all **646 preceding physical recordings** (451,255 control ticks) with identical output-stream and replay-report bytes; all **28 current physical recordings** retain identical replay-stream/report bytes with the final binary and the stricter independent signal oracle. Three separate 94-tick measured-reset fixtures independently verify old/reset-time green withdrawal, newer red ownership, stale expiry, continued -6 m/s² braking after an unserved crossing, and blocked signal commitment while another stop-duty admission is faulted. Their complete final-binary replay passes. Validation was observed on Linux x86_64 only.

An additional combined-control input first exposed **18** signal passage commitments while a stop-duty admission failure already required emergency braking. The original recording and failure remain saved. Revision B includes that independent fault in signal passage health; it removes the invalid commitments while preserving every command in that sensor sequence. The original 28 physical inputs, gates and streams are unchanged: all are re-audited for strict signal ownership and reproduce identical replay bytes with the final runtime. No additional physical run or physical braking improvement is claimed for this repair.

The preliminary design mistakenly placed permanent-red close-line standstill at 15 s, conflating the earlier motion-fault stop with the later approach to the 55 m line. A recorded pre-physical review corrected that gate to 30 s before running any of these cases; the original design and review remain saved. No physical inputs or gates were adjusted after observing this cohort. Initial formatting and unit-fixture errors (a point still inside the corridor and overlapping rule arcs) remain in development logs; corrected fixtures pass the required checks.

[Per-case results, source and raw-stream hashes](../assets/signal-relocalization-results.json) identify the local packet `artifacts/signal-relocalization-v1`.

## Limits

These are authored seed-7 simulation regressions, without an unseen automotive-data claim. The 0.5 m admission threshold and 1 m physical fixture margin are different checks and do not calibrate pose uncertainty or vehicle geometry. A freshly misclassified green still lacks an independent recognition channel. Signal perception, V2X transport, dilemma-zone decisions, lane topology, general priority negotiation, live-route signal remapping, measured real-time latency and vehicle operation remain unqualified. [Comparative engineering maturity](maturity.md) stays about **29%**, versus the scope and evidence of Autoware, Apollo and openpilot; this is subjective and the 30% waypoint remains unmet.
