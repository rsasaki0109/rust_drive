# Earlier obstruction-stop search

`search_obstruction_stops: true` optionally repairs a rejected stopping proposal
without changing the predicted occupancy, road or braking authority. It requires
`settle_obstruction_stops: true`; both options remain off by default. The
[fixed results](../assets/stop-search-results.json) retain every first failure.

## Planning contract

The ordinary planner proposes a stop four meters before its nominal first
contact. Slowing changes arrival times, so its complete braking path and
eight-second stationary tail can still intersect a growing or moving forecast.
Previously that single rejected proposal could produce emergency braking even
when an earlier reachable stop passed the same sweep.

The new option first preserves that ordinary proposal. If it fails, it reuses
the existing four-meter heuristic to propose earlier stopping locations, then
checks up to 16 alternatives down to the physically reachable braking distance.
Each alternative rebuilds its acceleration-limited speed profile and repeats
the full collision check, including every intervening forecast knot and the
entire hold. This finite search does not assume that feasibility is monotonic
in stopping distance. It may miss a feasible interval; no candidate bypasses
the existing collision or road checks. The four-meter heuristic is not a
calibrated uncertainty bound.

A selected search while moving arms a retained braking duty. Its stopping
anchor is expressed in supplied-route progress, with explicit conversion to
the corrected candidate's actual arc length. Subsequent blocked plans can stop
earlier but cannot move that anchor forward. Searched profiles cap cruise speed
at measured initial speed; the final command guard prohibits positive
acceleration while the duty persists. At a measured speed at most 0.2 m/s,
the existing immediate brake and full stationary hold apply, with acceleration
at most −0.5 m/s². A complete clear nominal candidate can release it, including
a checked lateral escape. Another retimed blocked approach cannot.

Route reset clears the anchor's old coordinates while retaining brake intent;
without a usable anchor the planner requests a reachable immediate stop.
Invalid inputs cannot clear the duty. Sensor faults retain the existing
−6 m/s² emergency command. Positions, measured speed, supplied geometry and
observed predictions are the operational inputs; simulator actor truth is used
only by the independent post-run scorer.

## Frozen evaluation and first failures

Forty unchanged preceding inputs cover both reference and native RNE plants:
eight curved-mission configurations with heading enabled/disabled, 12 calibrated
narrow-road passes, 12 blocked/releasing/GNSS-fault cases, and eight heading
loss/jump/recovery cases. Seeds, additional sensors, 65/45/20-second deadlines,
startup priors, uncertainty budgets and physical gates are unchanged. The only
new scenario field is the search option. Earlier raw inputs match byte for byte
after removing that field.

The independent checker retains the 1 m physical clearance floor, 0.1 m road
reserve, 0.3/0.6 m localization RMSE/maximum gates, 0.1 rad heading gate and 99%
forecast coverage at 0.2/1/2/4/8 seconds. It reconstructs a braking commitment
from complete emitted geometry rather than a producer latch, certifies every
partial braking/hold path through continuous accelerated-motion interval bounds,
and independently measures actual physical separation and standstill. A
separate improvement gate requires at least 80% fewer mission emergency steps
and actual standstill without positive commands from 20 to 65 seconds. That gate
does not replace the original goal or coverage requirements.

The first focused revision repeatedly moved the stopping point forward:
53 emergency steps became **306**. Revision B reduced all eight mission runs to
zero emergency steps but stopped two reference narrow-road cases at only
7.94/7.56 m, failing the original minimum-progress requirement. Both failures
remain in the results. Revision C retains the checked stopping anchor and
restores those cases to 21.93/21.86 m under the same requirement.

| Final gate or measurement | Result |
|---|---:|
| No new failed acceptance gates against exact preceding inputs | 40/40 |
| Stable mission-stop improvement | 8/8 |
| Mission emergency control steps, preceding → current | 355 → 3 (99.2% fewer) |
| Mission conditions with no emergency control steps | 5/8 |
| Original scenario acceptance | 32/40 |
| Full goal/accuracy/forecast qualification | 20/40 |
| Minimum swept physical clearance | 1.109367 m |
| Minimum actual road margin | 0.221750 m |

Eight configured mission runs still miss their original 65-second goal, 17 cases
fail forecast coverage, and three retain native heading-error failures. Five of
the eight missions have zero emergency steps; the other three have one each.
Stable braking does not qualify a completed mission or calibrated prediction.

All **89 new physical executions**, including development repeats, first failed
trials and four disabled controls, retain **59,809 ticks / 445 losslessly verified
execution archives**. Later recordings reside under `/tmp/rustdrive-stop-search-evidence`
through workspace artifact symlinks because the workspace filesystem was nearly
full; earlier failed recordings remain in place. These are authored, viewed
simulation worlds, without held-out automotive calibration. A separate traced
111-tick historical prefix was rejected for its missing count footer; it is
retained as a diagnostic, not counted as successful replay or physical execution.

## Reproduction and limits

```sh
source scripts/env.sh
python3 scripts/test_stop_search.py
cargo test --locked -p rustdriving-planning --test stop_search
# Requires built root/native executables. Full qualification intentionally exits 1.
python3 scripts/check_stop_search.py --output artifacts/stop-search
# Compare retained failures and require the separate stable-stopping improvement.
python3 scripts/check_stop_search.py --check-regression --output artifacts/stop-search-regression
bash scripts/check.sh
```

Use a fresh output directory. Inputs are frozen before execution and every raw
stream is archived with encoded/decoded SHA-256 receipts. The public result
packet includes the original classifications, all final audit rows, fixed inputs,
source reconstruction and binary provenance. The first complete required check's
Rust 1.90 Clippy type-complexity failure remains recorded; a private named
selection structure replaces the nested tuple.

The public result file is now a small manifest with the unchanged comparison
inputs. Its `full_evidence.parts` list contains ordered pieces of the complete
original JSON packet, each at most 32,000 bytes. Concatenation reconstructs the
original bytes exactly; the manifest records both per-piece and complete SHA-256
digests. This packaging avoids oversized publication requests without removing
any first failure, source reconstruction or raw-stream receipt. To restore it:

```sh
python3 - <<'PY'
import hashlib, json
from pathlib import Path
assets = Path('assets')
manifest = json.loads((assets/'stop-search-results.json').read_text())['full_evidence']
pieces = [(assets/p['path']).read_bytes() for p in manifest['parts']]
assert all(len(b) == p['bytes'] and hashlib.sha256(b).hexdigest() == p['sha256']
           for b, p in zip(pieces, manifest['parts']))
packet = b''.join(pieces)
assert len(packet) == manifest['bytes']
assert hashlib.sha256(packet).hexdigest() == manifest['sha256']
Path('/tmp/stop-search-results-full.json').write_bytes(packet)
PY
```

The final exact `bash scripts/check.sh` passes **826 Rust tests, 125 Python tests
and 49 standard drive/replay pairs**, retaining all 245 preceding output files
byte exactly. Native format, locked release build, **64 native tests** and strict
all-target Clippy pass. All **134 preceding logs** retain exact replay outputs;
four disabled native physical runs retain all five stream hashes, including
their original heading failures. CI adds the adversarial Python checks; remote
CI success is unverified.

General motion/shape uncertainty, mission availability, coupled lane/rule/reset
compositions, automotive sensing, real-time operation, cross-platform behavior
and real-vehicle operation are unqualified. Comparative engineering maturity
remains **about 29%** versus Autoware/Apollo/openpilot, a subjective planning
estimate. The **30% waypoint remains unmet**.
