# Acceleration-aware speed planning

The shared Rust pipeline now computes a distance-varying speed profile and derives collision-checking arrival times from it. Both the reference bicycle and CPU-only RNE adapter run this implementation. It replaces the constant-speed timing recorded in [the earlier swept-planning stage](swept-planning.md).

## Profile and control

Each candidate starts at the estimated position and speed. Its sampled geometry receives a desired cruise limit and, when calibrated, local circumcircle curvature limits. A backward pass propagates the available braking authority; a forward pass propagates the available acceleration authority. Cruise overspeed, including observation noise, recovers with bounded braking rather than changing the initial state instantly. A hard curvature or stop bound that cannot be reached from the initial speed rejects the candidate.

On each straight geometric segment, speed varies linearly in time. Travel time is `2 * segment_distance / (initial_speed + final_speed)`, equivalent to constant-acceleration integration. Zero-speed segments never invent forward motion or infinite arrival times. A stopping point is interpolated at its exact arc distance; the path ends there and adds an eight-second stationary hold. A goal stop is proposed one route meter before the endpoint; physical goal acceptance is unchanged.

A blocked free profile proposes a stop four meters before the first contact distance. That braking profile and stationary hold receive a new synchronized collision sweep. If slowing creates another encounter, the candidate is rejected. The planner does not assume the original collision check is still valid after retiming. A selected stopped destination is held while the free candidate remains blocked, and released when it clears. If every candidate is infeasible or obstructed, the planner returns an empty emergency trajectory.

The circular checker retains forecast-knot splitting and growing uncertainty margins. Constant acceleration differs from the temporal position chord by at most `abs(final_speed - initial_speed) * segment_duration / 8`; additional radius inflation covers this deviation. This is conservative collision checking for the supplied segment model, not exact curved-body dynamics.

For low calibrated lateral authority, the proposed quintic shift length increases using the normalized quintic second-derivative bound. The selected length persists across replans; actual sampled curvature and reachable speed bounds still decide feasibility. Estimated-position tracking correction remains active. Longitudinal control uses first-segment acceleration feedforward plus bounded PI speed feedback. A stationary hold commands no forward acceleration. Steering remains pure pursuit, and adapters still enforce their actual actuation limits.

## Regressions and independent verification

- Analytic acceleration from rest: with 2 m/s² authority, reaching 16 m takes 4 s and reaching 40 m with an 8 m/s cruise limit takes 7 s.
- Exact interpolated stops, strictly increasing finite times, reachable forward/braking bounds, stationary hold and rejection of an impossible stop.
- Local curvature slowing followed by speed recovery on a straight; noisy overspeed recovery without an initial speed jump.
- A crossing at x=16 m, t=4 s is detected with acceleration-aware timing. The old 3 m/s floor placed arrival at 5.33 s, after the crossing cleared. The same regression was executed against committed planner `38475ae`: it returned `Cruise`, whereas the current planner returns `Yield` with a reachable stop.
- A second actor crosses a braking destination after the original free path would have passed; revalidation rejects the stopped path.
- High-speed low-friction avoidance requires a longer reachable shift. During development, the unchanged low-friction fixture reproduced a stop from which a short lateral candidate could not escape. Longer persistent shifts repaired goal completion without relaxing physical acceptance.
- The earlier tiny-footprint oncoming regression now selects emergency braking because no feasible stopping profile exists before that encounter; no physically impossible zero-speed initial state is published.
- Controller tests cover feedforward starting from rest, planned braking, stationary hold and invalid profile time.

`scripts/check_hazards.py` independently reads recorded non-emergency trajectories and verifies their initial state, finite increasing times, speed-integrated segment distance and calibrated longitudinal acceleration bounds. These checks run in addition to the simulator's independent truth-based collision/road evaluation and full sensor-output replay. Emergency trajectories are excluded from profile feasibility claims.

## Reproduction and measured outcomes

```sh
bash scripts/check.sh
bash scripts/check-hazards.sh --output artifacts/speed-planning
```

Locally, formatting, Clippy with warnings denied, locked release builds, **68 workspace tests**, **8 RNE adapter tests**, and eight reference acceptance/replay pairs pass. The hazard suite passes all **30 runs** across seeds 1, 7 and 42, with zero colliding evaluation steps, zero road violations, matching full replay and valid recorded speed profiles. Low-friction actual longitudinal acceleration remains bounded by the adapter's `mu * 9.81` criterion.

The [result snapshot](speed-planning-results.json) retains individual metrics, profile checks, replay counts, engine pin and source fingerprint. Full logs are in ignored `artifacts/speed-planning/`. The RNE revision remains `df6007aa40315e81d12ae00fc1f60369e393a178`. Earlier numeric snapshots remain historical comparisons.

| Backend | Scenario | Passing seeds | Worst clearance (m) | Seed 7 time (s) | Seed 7 replay ticks | Seed 7 emergency steps |
|---|---|---|---|---|---|---|
| reference | cut-in | 3/3 | 0.794 | 20.60 | 413 | 56 |
| reference | multiple-blocked | 3/3 | 3.880 | 25.00 | 501 | 11 |
| reference | occluded-crossing | 3/3 | 1.873 | 21.15 | 424 | 34 |
| reference | opposing-crossings | 3/3 | 1.007 | 23.10 | 463 | 57 |
| rne-dynamic | cut-in | 3/3 | 0.870 | 23.05 | 462 | 28 |
| rne-dynamic | low-friction | 3/3 | 1.238 | 29.10 | 583 | 83 |
| rne-dynamic | low-friction-stop | 3/3 | 4.579 | 40.00 | 801 | 23 |
| rne-dynamic | multiple-blocked | 3/3 | 3.312 | 25.00 | 501 | 9 |
| rne-dynamic | occluded-crossing | 3/3 | 1.323 | 25.90 | 519 | 25 |
| rne-dynamic | opposing-crossings | 3/3 | 0.870 | 26.45 | 530 | 27 |

Emergency fallback still occurs, including repeated fallback in some low-friction or tracking states. The table records it explicitly: reaching the goal does not establish smooth or dynamically guaranteed controller tracking. Three fixed lateral offsets, sampled circumcircle curvature, separate longitudinal/lateral limits, heuristic uncertainty and constant-velocity forecasts remain limitations. No jerk bound, combined tire-force optimization, interaction model, traffic-priority reasoning, rectangular footprint, throughput guarantee or real-vehicle safety is established. Forecast endpoints remain held after eight seconds, which can be conservative or inaccurate for actors that continue moving.

The README opening GIF was regenerated from an actual RNE dynamic mission: 790 sensor ticks, 133 GIF frames, zero collisions and road violations, and matching full replay. Its [provenance](../assets/rne-demo.json) retains the scenario, engine revision and measured metrics. Reference and occluded-crossing assets were regenerated as well.

Sensor log schema 1 remains readable. Older calibrated motion-limit objects default the new forward acceleration field to 2 m/s²; supplied invalid limits fail construction. Output semantics and algorithm behavior changed, so old recordings can correctly fail exact replay. Regenerate logs with this implementation.
