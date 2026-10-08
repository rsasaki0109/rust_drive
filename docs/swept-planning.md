# Continuous candidate collision validation

The lateral lattice now checks circular envelopes throughout each trajectory interval, not only at its 81 sample points. This implementation runs inside the shared sensor-only pipeline on both the reference and CPU-only RNE plants.

## Collision model and timing

A candidate still spans up to 40 m and uses `distance / max(ego_speed, 3 m/s)` as its arrival-time approximation. Ego position is linear between candidate points. Each object's forecast is linear between prediction knots and stays at its final position after the forecast horizon. Every candidate interval is split at intervening prediction knots, so a forecast turn cannot disappear into an endpoint chord.

On each subinterval, relative position is `r(u) = r0 + u * delta`, for `u` in `[0, 1]`. The checker solves `|r(u)| <= R` for the earliest entry. `R` includes both circular radii and the existing uncertainty margin `0.30 + 0.06 * min(time, 5)` meters. Using the subinterval's end-time margin conservatively covers the growing envelope. Initial overlap and tangency count as contact. Empty, non-finite or invalid-radius/timestep forecasts produce an emergency trajectory instead of a clear-road result.

The first contact time determines the blockage distance used for candidate selection and braking. Swept geometry alone does not make the approximate arrival times dynamically feasible: acceleration-aware timing, tire-force coupling and controller tracking guarantees remain unimplemented.

## Stop, wait and resume

When a candidate's blockage is within `ego_speed² / (2 * calibrated_deceleration) + 4 m`, the planner requests zero speed. Once selected, this stop remains held while the candidate is blocked. A clear candidate releases it. This avoids accelerating again merely because reduced ego speed makes the instantaneous braking distance smaller.

A yield stop requests the controller's existing normal deceleration bound of 4 m/s²; goal stops retain 3.5 m/s². RNE continues to limit actual longitudinal actuation by `mu * 9.81`; the plant and acceptance criteria were not loosened. These are simulator control policies, not semantic traffic-priority reasoning or a certified braking controller.

The anchored quintic maneuver retains its progress, but a decaying quintic correction joins it from the current estimated lateral position. The first trajectory point is the current estimate. A stopped or lagging vehicle is no longer assumed to have already reached the previous maneuver's planned offset. Corrected candidates whose sampled lateral offsets exceed the road's circular-footprint margin are rejected. No ground-truth position is used for this correction.

## Reproduced regressions

- A supported 0.2 m circular ego footprint and a 0.2 m object moving toward it at 12 m/s meet between candidate samples. The previous planner returns `Cruise`; the new planner returns `Yield` with zero target speed. The same regression was executed against the previous committed planner in an isolated temporary test harness and failed there. This is a small-footprint algorithm regression, not the default vehicle's driving demo.
- Analytic tests cover contact entry, tangency, overlap, clear endpoints with an interior collision, forecast turns, static/terminal forecasts, and geometric crossings that occur at different times.
- A new opposing-crossings fixture initially reproduced 0.138 m of physical overlap with the continuous checker alone. Additional testing found the weaker stop command and a displaced trajectory origin. The stop/wait policy, normal-bound yield braking and estimated-position path join repair it without changing the fixture or collision criteria.

## Closed-loop fixtures and reproduction

```sh
bash scripts/check.sh
bash scripts/check-hazards.sh --output artifacts/swept-planning
```

The workspace check runs 56 tests, formatting, Clippy, a locked release build, and eight reference scenario/replay pairs. The hazard command builds both release binaries and runs **30 cases**, each with independent physical evaluation and full sensor-output replay:

- Reference: occluded crossing, cut-in, multiple blocked alternatives, and opposing crossings, each with seeds 1, 7 and 42.
- RNE dynamic: those four scenarios plus low-friction avoidance and stopping, with the same three seeds.

`multiple-blocked` has three stationary circles at route s=40 m and lateral offsets 0 and ±3.5 m on a road with half-width 5.5 m. The circles block all candidate lanes, so stopping is required. `opposing-crossings` has two initially visible circles crossing a narrow road from opposite sides at s=40/55 m; their motion starts at 5/7 s. Goal completion is required. Their schedules do not react to the ego or one another.

All 30 local runs passed with zero colliding evaluation steps, zero road violations and matching full replay. The RNE integration's existing 8 tests and Clippy also passed. The [compact results](swept-planning-results.json) retain individual summaries, replay counts, engine pin and a source fingerprint. Full evidence is in ignored `artifacts/swept-planning/`. The earlier [18-run record](hazard-validation.md) describes the previous implementation.

| Backend | Scenario | Passing seeds | Worst clearance (m) | Seed 7 simulated time (s) | Seed 7 replay ticks |
|---|---|---|---|---|---|
| reference | cut-in | 3/3 | 1.059 | 21.10 | 423 |
| reference | multiple-blocked | 3/3 | 9.140 | 25.00 | 501 |
| reference | occluded-crossing | 3/3 | 1.861 | 20.90 | 419 |
| reference | opposing-crossings | 3/3 | 0.268 | 24.10 | 483 |
| rne-dynamic | cut-in | 3/3 | 0.624 | 22.80 | 457 |
| rne-dynamic | low-friction | 3/3 | 0.862 | 29.50 | 591 |
| rne-dynamic | low-friction-stop | 3/3 | 10.949 | 40.00 | 801 |
| rne-dynamic | multiple-blocked | 3/3 | 7.063 | 25.00 | 501 |
| rne-dynamic | occluded-crossing | 3/3 | 1.197 | 25.70 | 515 |
| rne-dynamic | opposing-crossings | 3/3 | 0.823 | 26.40 | 529 |

The stop hold is conservative: some fixtures stop well before an obstruction. This is not an optimized speed profile. Physical evaluation remains independent of the planner and uses swept circular bodies between simulator ticks. Planar routes, circular targets, supplied calibration, prediction uncertainty heuristics, scheduled actors and a constant-speed timing approximation remain the operating boundaries. No real-time throughput, interactive traffic behavior, real-vehicle safety or general collision-avoidance guarantee is established.

Sensor record schema 1 is unchanged. Algorithm outputs changed, so previously generated logs can correctly fail exact replay with this binary; regenerate recordings for the revised pipeline. The [replay contract](sensor-replay.md) distinguishes format compatibility from algorithm-version compatibility.
