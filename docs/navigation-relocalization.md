# Preserve live navigation through measured relocalization

An explicit `relocalization.preserve_navigation: true` now lets a stopped,
measured localization reset retain its live navigation mission. Earlier versions
reject this configuration. The mission keeps the original graph, start,
destination, accepted closure revision, active route, mapped speed limits and
route-switch count. It can receive closures while motion is invalid and recover
without replacing that knowledge with its startup map snapshot.

```json
{
  "gnss_latency_compensation": true,
  "relocalization": {
    "map_only_grace_s": 5.0,
    "confirm_gnss_handoff": true,
    "preserve_navigation": true
  },
  "gnss_recovery": {
    "max_position_correction_m": 0.4,
    "max_yaw_correction_rad": 0.1,
    "confirmation_fixes": 3,
    "max_hold_s": 5.0
  }
}
```

Supply the existing `navigation` graph/endpoint request, surveyed
`localization_map`, and acquisition-time GNSS calibration as usual. The new flag
requires a navigation mission and defaults to false/omitted. Lane changes,
mapped signals/stop signs/intersections and XYZ/multi-height relocalization
remain excluded from live navigation. Fixed routes can separately opt into
[stop preservation](stop-sign-relocalization.md), without enabling navigation
with controls. This extends the authored directed planar-map domain.

## Ownership and route admission

The existing [regional reset](regional-relocalization.md) still requires fresh
stopped wheel/gyro observations, consistent raw scan/map fits and a measured
pose/full covariance. It constructs fresh sensing, filter, scene and control
state and brakes on the confirming frame.

The navigator retains its semantic mission state. Construction validates the
original mission before restoring its actual active plan and speed policy;
a previously switched detour never becomes the startup route again. At the new
measured pose, Euclidean distance to the active route plus body radius and a
0.2 m margin must be strictly inside its half-width. Heading error must be
less than 0.35 rad and measured initialization speed at most 0.05 m/s. Rejection
records `navigation.localization_reset.admitted: false` and latches navigation
braking through later closure updates. A new measured initialization is needed.
This admission checks the retained path; it does not select an arbitrary nearby
edge or establish global map uniqueness.

An accepted reset reruns pending/blocked-route reachability against the new
pose and current accepted closures, preserving the original incoming/traversal
semantics. A branch now behind the vehicle stays blocked. Every old stopped
handover vote is cleared, the motion watermark advances to the reset stamp,
and three subsequent fresh healthy stopped acquisitions are required to change
routes. A repeated acquisition cannot count. Prior navigation faults persist;
ordinary valid newer external snapshots retain their existing recovery rules.
Failed localization-to-route admission stays latched through those snapshots.

A subsequent optional [`navigation.hold_for_handover`](navigation-handover.md)
retains braking during those new stopped votes. This prevents the common-prefix
planner from accelerating before it can finish confirmation after recovery.
The original 32-case qualification below uses the preceding default behavior;
the linked evaluation records the separate fixed comparison.

Optional map-only grace and confirmed GNSS handoff retain their separate
acquisition clocks, transaction ownership and reset-anchored deadlines.
Navigation permission never fabricates accepted GNSS. Holding, route closure,
missing sensors, expired confirmation or excessive uncertainty may still brake.

## Frozen evaluation

All 32 inputs are frozen before the first physical run. Both reference 2D and
pinned native RNE dynamic plants use authored 140 m straight/curved main routes,
3.5 m half-width, 6 m/s cruise, a 60 s deadline and original external landmark
maps. Four-node directed graphs have a common 70 m head, 40 m main/detour branch
and 30 m tail. The detour has a smooth 4 m lateral excursion. Whole-edge caps
are 6 m/s on head/main, 3 m/s on detour and 4 m/s on the tail. Static landmarks
remain attached to the initial road's offline positions through route changes.

| Cases | Fixed protocol |
| --- | --- |
| 12 late closures | Seeds 1/7/42; wheel loss [8,11) s, GNSS loss [8,16) s; main closes at 9 s while localization is invalid; retain closure, recover, stop-confirm detour and reach the original destination. |
| 4 already-switched detours | Seed 7; main closes at 3 s; wheel loss [16,19) and GNSS loss [16,24); retain the actual changed route, switch count and 3 m/s cap through reset. |
| 4 no-route controls | Both branches close at 9 s; recover measured localization and approach the closed boundary, then hold a physical stop. |
| 4 reopened missions | Both branches close at 9 s; after at least 2 s of physical standstill, detour reopens at 24 s; accept revision 2, stop-confirm and reach the original goal. |
| 4 disabled controls | Same late-closure inputs with localization recovery omitted; the existing motion-continuity guard keeps the vehicle stopped. |
| 4 healthy controls | No motion/GNSS loss; perform the stopped detour normally without any localization epoch reset. |

All recovered missions must stop before reset and travel at least 3 m during
[reset+1, reset+3.8] s. Goal cases must exercise every active edge speed interval.
All cases require zero contact/road/closed-edge violations, swept clearance
at least 1 m, physical speed excess at most 0.15 m/s, and valid-epoch RMSE at
most 0.2 m, maximum position error 0.4 m and yaw error 0.1 rad. No-route controls
hold at <=0.05 m/s after 32 s; disabled controls hold after loss+4 s.

An independent exhaustive path enumerator reconstructs every selected route,
clipped traversal and original speed interval. A separate raw-input state fold
checks each closure revision, reset admission, pending path, stopped vote and
route switch. Physical swept scoring checks collision/closure/speed behavior.
The preceding raw-point regional proof and full-history joint-matrix oracle
reconstruct the localization epochs, proposals, covariance and freshness.
Normal local map observations retain full exact-replay verification. Truth is
used for synthesis/scoring only; no fault schedule enters operational headers.
Seeds were used in previous development and are regression inputs, not unseen
automotive holdouts. Invalid-epoch errors and general CLI judgments remain in
all raw records.

## Recorded outcomes

All **32/32 cases** meet the frozen feature protocol on Linux x86_64:

| Cases | Recorded outcome |
| --- | --- |
| 12 late closures | Retain the closure received during invalid motion, recover, perform a stopped detour handover and reach the original goal in **43.50–44.45 s**. |
| 4 already-switched detours | Preserve the active detour, revision 1, switch count 1 and its 3 m/s cap through reset; reach the goal in **43.40–43.85 s**. |
| 4 no-route controls | Recover and approach the original closed boundary, then hold at <=0.05 m/s after 32 s; final speed is **0 m/s**, with no closed-edge entry. |
| 4 reopened missions | Physically hold for **4.20–4.40 s**, retain revision 2 on reopening, stop-confirm the detour at **24.10 s**, and reach the original goal in **47.85–48.00 s**. |
| 4 disabled controls | No reset or route switch; hold the original motion-loss stop, with final speed **0 m/s**. |
| 4 healthy controls | Reach the goal in **38.30–38.55 s**, with one ordinary stopped handover and no localization reset. |

That is **20 recovered goals**, four recovered closed-mission stops, four healthy
goals and four unchanged disabled holds. Recovered missions travel
**8.024–12.498 m** during [reset+1, reset+3.8] s. All goal cases exercise every
active speed interval. Maximum physical speed excess is **0.079838 m/s** against
the original 0.15 m/s tolerance; no speed/road/closed-edge/contact violations
occur. Independent epoch, navigation and exact-replay checks cover all
**30,525 control ticks**. Worst valid-epoch RMSE is **0.012573 m**, maximum
position error **0.069474 m** and yaw error **0.001588 rad**.

The general CLI retains **24 accuracy failures**, with full-interval maximum
position error up to **3.275094 m**. Its remaining eight aggregate judgments pass;
the four slower already-switched missions still mark the missing-motion epoch
invalid even when the CLI's global error threshold is not crossed. Every raw
error, health flag, failure string and exit code is preserved. Valid-epoch feature
acceptance does not qualify pose accuracy during missing motion.

The final exact `bash scripts/check.sh` passes **739 Rust tests**, **68 Python
tests** and **49 standard drive/replay pairs**, preserving exact bytes for all
**245 standard files**. Native release formatting, strict all-target Clippy and
**63 tests** pass. All **546 previous logs** reproduce their historical replay
outputs exactly, including previous failed protocols. Three actual-pipeline
synthetic checks separately verify changed-plan retention, fresh post-reset
handover confirmations and latched failed-route admission. Rust 1.90.0/root,
Rust 1.95.0/native, locks and pinned RNE remain unchanged; other CI platforms
were not observed locally.

## Retained first classifications

The first complete 32-case cohort has **20 classified passes and 12 checker
exceptions**. Eight ordinary blocked/reopening holds correctly request the
existing **-0.5 m/s²** minimum brake while stopped; the first oracle incorrectly
required emergency **-6 m/s²**. Faults retain that emergency requirement. Four
healthy cases also expose normal Serde omission of empty GNSS/wheel dropout
arrays. The repaired checker normalizes those declared empty defaults while
rejecting nonempty changes, and separately tests both braking contracts.

All first reports, original checker bytes and raw streams are retained.
**No feature physical run is repeated**. Inputs, runtime binaries and physical
speed/stop/clearance/deadline gates remain unchanged. Final classification
rechecks encoded and decoded SHA-256 for all **160 archives**. One superseded
mandatory-check run was interrupted before Rust tests executed, preserving its
67 passed Python tests and compilation log; the final exact command above then
passes completely.

[Published inputs, all outcomes and hashes](../assets/navigation-relocalization-results.json)
identify the local packet `artifacts/navigation-relocalization-v1`. It retains
both runtime binaries, full operational/scoring/replay streams, first failures
and all final receipts. Two older large offline JSON records were migrated
losslessly to XZ with encoded/decoded hash receipts to preserve disk capacity.

Comparative engineering maturity increases from **about 28% to about 29%**:
measured recovery now composes with live directed missions and their actual
closure/speed state. This is a subjective scope estimate against Autoware,
Apollo and openpilot, not a benchmark or vehicle-safety score.


## Reproduce

Complete the [native adapter setup](../integrations/rne/README.md) for the pinned
RNE checkout, then use a fresh output directory:

```sh
source scripts/env.sh
bash scripts/check.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 -m pip install -r scripts/requirements-relocalization.txt
python3 scripts/check_navigation_relocalization.py --output artifacts/navigation-relocalization
```

`--freeze-only` writes all 32 inputs first. `--smoke --plant reference-2d` or
`--smoke --plant rne-dynamic` runs 12 cases per plant. CI retains both protocols'
raw records. At most two checker workers run concurrently. NumPy is an offline
checker dependency; the driving stack gains no runtime dependency.

This evaluates bounded recovery on authored supplied maps. Moving route
handover, global initialization, live mission controls during recovery, measured
automotive uncertainty, real-time performance and vehicle operation remain open.
The 30% maturity waypoint remains unmet.
