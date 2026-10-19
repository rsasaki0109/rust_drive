# Bounded map support after a measured reset

The optional `relocalization.map_only_grace_s` setting lets a newly measured
localization epoch drive briefly before a new GNSS acquisition arrives. It
accepts 0.5–5 seconds. Omitting it preserves the existing post-reset GNSS hold.
This experimental policy supports supplied fixed routes, surveyed maps and
ordinary synchronous body-XY LiDAR, under the existing
[stopped regional recovery contract](regional-relocalization.md).

```json
{
  "relocalization": {
    "map_only_grace_s": 5.0
  }
}
```

Place that policy beside the externally supplied `localization_map`. The
simulator's dropout schedule is evaluation input and never enters the operational
pipeline header. Delayed map scans are rejected when this option is enabled.
Signals/yielding controls and XYZ/multi-height exclusions remain. Fixed stop
signs need [explicit stop preservation](stop-sign-relocalization.md); live
navigation needs the [mission-preservation option](navigation-relocalization.md).

## Freshness and control

A measured reset still requires fresh stopped wheel/gyro samples, three
consistent bounded raw scan/map fits and a new full pose/covariance. It clears
scene/filter/control history and brakes on the confirming frame. A GNSS or
LiDAR acquisition at or before that reset cannot enter the new epoch again.

The absolute deadline is `measured_reset_stamp + map_only_grace_s`. Only a
subsequent accepted synchronous map correction, at most 0.2 seconds old, can
support control. The same qualified map acquisition stamp reaches both the
pipeline health decision and the final independent control guard. Later scans
cannot move the deadline. LiDAR acquisition/freshness, motion coverage, numeric
and covariance guards still apply. Missing scans cannot renew map age and support
expires after 0.2 seconds. Repeated, delayed, empty or rejected observations
cannot provide support; a new accepted observation may restore it within the
original deadline.

Malformed/rejected GNSS or uncovered motion interrupts this permission. Ordinary
fresh GNSS acceptance may recover through the existing gates. Without it, the
first frame after the absolute deadline brakes, even when every new map fit
continues to succeed. Expiry alone does not prevent a later genuine GNSS fix
from recovering normal localization health.

The separate opt-in [confirmed GNSS handoff](confirmed-gnss-handoff.md) arms the
configured recovery policy at the measured reset. It requires several bounded
fresh returning fixes and gives confirmation its own reset-anchored, latched
deadline. When enabled, expiry of that confirmation budget prevents late GNSS
from resuming control. The ordinary late-fix behavior above applies with this
additional option omitted.

`relocalization.map_only_support` records the reset, fixed deadline, actual last
accepted map stamp and `WaitingForMap`, `Active`, `Expired`, `Interrupted` or
`GnssAccepted` status. Actual GNSS acquisition clocks, accepted/rejected counts
and history are preserved: they remain empty after reset until real fixes arrive.
The existing `map_localization.gnss_outage_covered` flag retains its actual-GNSS
basis. After a real fix is accepted, ordinary GNSS/map outage limits apply.
Optional fields remain absent from older/default log serialization.

## Frozen protocol and independent checks

The 24 inputs use the previous authored 120 m straight/curved routes, 3.5 m
half-width, 6 m/s cruise and 45 s deadline, with the same offline landmark maps.
Both reference 2D and native RNE dynamic plants lose wheel/gyro observations
during [8,11) seconds. GNSS acquisition is unavailable during [8,16) seconds in
12 handoff cases (seeds 1/7/42) and four matching default-option controls.
Four permanent-GNSS-loss cases test absolute expiry; four add LiDAR loss at
14 seconds. All inputs are frozen before execution under each revision.

Original gates require an actual stop during [10.5,11.2), measured reset during
[11,15], swept clearance of at least 1 m, no contact/road violation and valid-epoch
RMSE≤0.2 m, maximum position error≤0.4 m and yaw error≤0.1 rad. Handoff cases
must travel at least 5 m during [12.3,16] seconds with no accepted post-reset
GNSS, then accept actual GNSS and reach the original goal. Matching defaults
must hold at ≤0.05 m/s during that interval, then reach their goals on GNSS.
Expiry must brake on the first frame after the fixed deadline and stop within
1.5 s; LiDAR loss must brake during (14.1,14.3] s and hold after 15.6 s.

Revision B additionally requires every negative stop control to exceed 1 m/s
after reset and before expiry/LiDAR loss. A stationary vehicle cannot qualify
the effectiveness of a braking transition. Original inputs and gates are unchanged.

The raw-point regional checker reconstructs the measured reset independently.
That proved reset supplies the map-only deadline to a separate full-history
joint-matrix filter. The filter checks every valid prefix/post-reset pose,
covariance, GNSS count/clock, map age and permission status. Raw scan fitting has
the preceding [independent regional checks](regional-relocalization.md); normal
local map-registration outputs are checked by exact full-log replay. Physical
truth is used for synthesis and scoring, including actual motion and swept
clearance. Missing-motion pose errors and the general CLI's full-interval
accuracy failures remain in the records.

## First integration failure

Revision A produced `Active` map-support diagnostics while the final control
guard still received the absent GNSS clock and kept braking. All six classified
handoff trials failed the fixed movement gate at **0 m**. Three classified
controls passed their original conditions. The obsolete run was stopped with
**9/24 cases classified**; all completed raw streams, six raw files from two
additional in-flight runs, first reports, binaries and source bytes are retained.
The other 13 A cases were not started. Revision B connects the accepted map
timestamp to the final guard and adds unit assertions on the final positive
acceleration command. Its stop controls also gain the explicit pre-fault
actual-speed gate above.

## Recorded revision B results

All **24/24** frozen cases meet the feature protocol on Linux x86_64:

| Cases | Recorded behavior |
| --- | --- |
| 12 handoffs | Travel **13.798–13.932 m** during [12.3,16] s before any accepted post-reset GNSS; then accept actual fixes and reach the original goal. |
| 4 matching defaults | Travel **0 m** in that interval, then reach the goal after actual GNSS returns. |
| 4 absolute-expiry controls | Brake at **17.15 s**, from **5.996–6.008 m/s**, and hold at **0 m/s** after 18.6 s. |
| 4 LiDAR-loss controls | Brake at **14.15 s**, from **3.9 m/s**, and hold at **0 m/s** after 15.6 s. |

The four seed-7 paired goal times improve by **3.90–4.25 s**, with the same
original route, width and 45 s deadline. Genuine GNSS acceptance occurs at
**16.20–16.40 s**. Independent reconstruction and exact replay cover all
**16,562 control ticks**. Worst valid-epoch RMSE is **0.056889 m**, maximum
position error **0.113939 m** and maximum yaw error **0.001536 rad**. No physical
case is repeated within revision B and no gate changes after its first run.

All **24 general CLI accuracy summaries still fail** because they retain the
missing-motion interval's frozen poses, with maximum global error up to
**3.274999 m**. Those errors, failure strings and exit codes remain in the raw
records. Passing this stopped-recovery feature protocol does not qualify
accuracy during that invalid interval.

The final exact `bash scripts/check.sh` passes **724 Rust tests**, **60 Python
tests** and **49 standard drive/replay pairs**; all **245 standard files** retain
exact bytes. Native release formatting, strict all-target Clippy and **63 tests**
pass. All **498 earlier logs** reproduce their original replay outputs exactly,
including their historical failures. A separate synthetic reconstruction checks
100 actual positive map-only commands, fixed expiry and later genuine GNSS
handoff. Root Rust 1.90.0, native Rust 1.95.0, locks and the pinned RNE revision
remain unchanged. Other configured CI platforms have not been observed locally.

[Published inputs, hashes, all outcomes and first failures](../assets/relocalization-map-only-results.json)
link the local immutable packet at `artifacts/relocalization-map-grace-v1`.
It preserves raw input/output/truth/replay streams in decoded-hash-verified XZ,
both binaries, first-revision source bytes and the interrupted obsolete check.
Older large JSON histories were migrated losslessly, with original/encoded SHA
receipts and decoded verification, to keep the workspace within its disk limit.

## Reproduce

Complete the [native adapter setup](../integrations/rne/README.md) for its pinned
RNE checkout before building the native binary.

```sh
source scripts/env.sh
bash scripts/check.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 -m pip install -r scripts/requirements-relocalization.txt
python3 scripts/check_relocalization_map_only.py --output artifacts/relocalization-map-only
```

Use a new output directory. `--freeze-only` writes all fixed inputs before
running; `--smoke --plant reference-2d` or `--smoke --plant rne-dynamic` runs
eight cases per plant. The checker uses at most two concurrent workers.
NumPy is an offline checker dependency and adds no driver dependency.

This does not establish global map uniqueness, measured automotive uncertainty,
live mission recovery, moving sensor timing calibration, real-time performance
or real-vehicle operation. The map and stopping measurements remain authored
simulation assumptions. The 30% maturity waypoint remains unmet.
