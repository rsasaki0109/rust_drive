# Confirmed GNSS handoff after measured recovery

`relocalization.confirm_gnss_handoff: true` arms the configured GNSS recovery
policy at a stopped, independently measured localization reset. A single
otherwise admissible returning fix can no longer resume normal GNSS control.
The option defaults to false and requires `map_only_grace_s`, `gnss_recovery`
and acquisition-time GNSS fusion. The existing synchronous body-XY, supplied
map and synchronous-XY restrictions still apply. A separate
[navigation-preservation option](navigation-relocalization.md) can retain the
actual live mission through the measured reset. Fixed stop signs use the
separate [stop-preservation option](stop-sign-relocalization.md).

```json
{
  "gnss_latency_compensation": true,
  "relocalization": {
    "map_only_grace_s": 5.0,
    "confirm_gnss_handoff": true
  },
  "gnss_recovery": {
    "max_position_correction_m": 0.4,
    "max_yaw_correction_rad": 0.1,
    "confirmation_fixes": 3,
    "max_hold_s": 5.0
  }
}
```

## Candidate and deadline ownership

The measured reset starts `Holding` with zero observed/accepted/rejected fixes.
Fresh map corrections may support bounded driving while GNSS remains absent.
Each genuinely new, otherwise admissible GNSS fix produces a complete
acquisition-time filter/history proposal. The existing recovery policy compares
its current-time position and heading correction with the current posterior.
Until the configured count is met, the whole candidate is discarded: pose,
covariance, accepted acquisition clock and retained history do not change.
`RecoveryHeld` records the actual observation and its NIS/rejected count.
Only the last confirming real fix commits. Holding interrupts map-only control
and brakes; normal actual-GNSS-based freshness resumes on confirmation.

Malformed, rejected, excessive, stale, duplicate or pre-reset acquisitions cannot
supply missing confirmations. They retain the existing individual admission
and sequence-interruption rules. A confirming reset scan initializes once; it
cannot renew map freshness or GNSS time.

The existing `gnss_recovery.max_hold_s` budget starts at the measured reset,
even before any GNSS arrives. Its deadline is never renewed by a scan or fix.
`GnssHandoffExpired` explicitly brakes on the first frame after expiry, including
when the map-only grace period is longer. Expiry stays latched: late healthy
GNSS cannot reopen this handoff. A new independently measured initialization is
required. With this option omitted, the preceding map-only behavior remains:
a late genuine GNSS fix may recover ordinary health after map-grace expiry.

## Frozen physical protocol

The 24 inputs retain the preceding authored 120 m straight/curved routes,
3.5 m half-width, 6 m/s cruise and 45 s deadline. Reference 2D and native RNE
dynamic plants lose wheel/gyro observations during [8,11) seconds and GNSS
acquisitions during [8,16). Twelve guarded cases use seeds 1/7/42; four paired
seed-7 controls omit only the new flag while retaining the same recovery policy.
Their first returning acquisition at 16 s has a 0.45 m ENU-Y bias, lasting just
one acquisition. Four healthy controls omit that spike. Four expiry controls
return GNSS at 18 s, after the reset-anchored five-second confirmation budget.

Every case must physically stop before reset, then travel at least 5 m during
[12.3,16] s on actual map support before any accepted post-reset GNSS. Guarded
and healthy cases require three independently reconstructed bounded proposals, at least two
held observations, first acceptance during [16.6,17.1] s and the original goal.
Expiry controls must brake at the absolute deadline, stop within 1.5 s and remain
stopped despite later healthy acquisitions. Original valid-epoch accuracy gates
are RMSE <=0.2 m, maximum position error <=0.4 m and yaw error <=0.1 rad;
clearance is >=1 m with zero contacts/road violations.

The independent raw-point regional checker proves the stopped reset. A separate
full-history joint-matrix filter then reconstructs every valid pose/covariance,
proposal, acquisition NIS, withheld transaction, ownership counter, map-only
permission and deadline. Normal scan-registration outputs retain exact full-log
replay checks; the raw-point proof covers regional reinitialization. Simulator
truth is used only for synthesis/scoring, including acquisition-time noise/bias,
physical motion and swept clearance. No fault schedule enters operational config.
The missing-motion interval remains in all raw streams and general CLI accuracy
summaries; separate valid-epoch gates cannot remove those failures.

## Recorded outcomes

All **24 expected outcomes** are recorded on Linux x86_64. The new policy's
**20 cases pass** their quality gates. The four otherwise paired unguarded
controls retain **four `unconfirmed_handoff` quality failures**; those failures
are expected comparisons and are not counted as quality passes.

| Cases | Recorded behavior |
| --- | --- |
| 12 guarded returns and 4 healthy controls | Hold exactly two complete proposals, commit the third at **16.60–16.80 s**, and reach the original goal. |
| 4 unguarded paired controls | Accept the first returning acquisition at **16.20–16.40 s**, before the configured confirmations; reach the goal while failing the handoff gate. |
| 4 expiry controls | Brake at **17.15 s** from **5.993–6.003 m/s**; hold **0 m/s** after 18.6 s despite 134–135 later real GNSS deliveries per case. |

Every case travels **13.798–13.932 m** on actual map support during [12.3,16] s,
with no accepted post-reset GNSS. Epoch reconstruction and exact replay check
all **14,985 control ticks**. Worst valid-epoch RMSE is **0.010541 m**, maximum
position error **0.036030 m** and yaw error **0.001536 rad**, with zero contacts
or road violations. These results qualify the fixed confirmation protocol;
the experiment does not establish general bias-detection accuracy.

All **24 general CLI accuracy summaries still fail**, retaining frozen estimates
in the missing-motion interval and global errors up to **3.274999 m**. The
original failure strings and exit codes remain scored in the full recordings.

The final exact `bash scripts/check.sh` passes **730 Rust tests**, **63 Python
tests** and **49 standard drive/replay pairs**. All **245 standard files** retain
exact bytes. Native release formatting, strict all-target Clippy and **63 tests**
pass. All **522 previous logs** reproduce their original replay bytes, including
historical failures. Root Rust 1.90.0, native Rust 1.95.0, locks and the pinned
RNE revision remain unchanged. Other configured CI platforms were not observed
locally.

## Retained first attempts

The initial evaluation copies lacked executable permissions. All 24 launch
attempts failed before a driving process started. Restoring executable mode
preserved both binary hashes and every frozen input/gate.

The first complete physical cohort then exposed a checker composition defect:
its recovery oracle required a current wheel sample even during the original
0.15-second validity of the last raw sample. All 24 first classifications retain
that exception. The checker now reconstructs that existing short window from
raw observations and still rejects stale motion. A closed-form boundary test
covers both outcomes. The same 24 archived runs are reclassified; **no physical
run is repeated**, and no binary, scenario or gate changes. The initial sources,
reports, trace, launch evidence and corrected classifications remain preserved.

[Published inputs, outcomes and hashes](../assets/gnss-handoff-results.json)
identify the local packet at `artifacts/confirmed-gnss-handoff-v1`. It retains
both candidate binaries, raw operational/scoring/replay streams and earlier
classifications. All **120 archives** have their encoded and decoded SHA-256
reverified during final classification.


## Reproduce

Complete the [native adapter setup](../integrations/rne/README.md), retaining its
pinned RNE checkout, before building the native binary.

```sh
source scripts/env.sh
bash scripts/check.sh
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 -m pip install -r scripts/requirements-relocalization.txt
python3 scripts/check_gnss_handoff.py --output artifacts/gnss-handoff
```

Use a new directory. `--freeze-only` writes all 24 fixed inputs before execution;
`--smoke --plant reference-2d` or `--smoke --plant rne-dynamic` runs eight cases.
CI runs those smoke protocols and retains raw outcomes, including expected
unguarded failures. NumPy belongs to the offline checker only.

This confirms candidate ownership for authored simulation assumptions. Repeated
bounded biased fixes can still satisfy the configured policy; general GNSS bias
detection, measured automotive uncertainty, real-time performance and vehicle
operation remain unverified. Comparative maturity remains **about 28%**, a
subjective planning estimate against Autoware, Apollo and openpilot. The 30%
waypoint is unmet.
