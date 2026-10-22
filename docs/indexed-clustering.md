# Indexed planar LiDAR clustering

The ordinary planar detector now uses a spatial index for retained clouds with
more than 64 points. It preserves the preceding detector's connectivity,
breadth-first discovery order, fitting and output bits. No configuration option
or new output field is introduced. This improves computation for larger clouds;
it does not improve measured perception accuracy or mission completion.

## Preserving the detector contract

The original detector scans every retained point for every expanded point. Its
link threshold is `min(0.28 + 0.015 * range, 0.85)` meters, with a strict distance
comparison. The threshold belongs to the expanded point: links are directed.
Replacing this with symmetric connected components would change the algorithm.
Input seed order and ascending neighbor indices also determine discovery order,
which controls the floating-point sums used by the fitted center and circle.

The new index uses one-meter body-forward/body-left cells. Any point within the
maximum 0.85 m threshold must occupy the same cell or one of its eight neighbors.
Only undiscovered point indices remain in the buckets. Discovery removes an
index by swapping the last bucket member into its slot and updating that
member's location. Empty buckets are removed. Before testing distances, the
combined neighboring indices are sorted into original input order. The exact
preceding predicate then decides every link; cell membership cannot itself
admit a point.

This preserves both component membership and every queue element's order.
Finite/near-origin filtering, the minimum three-return component, ordinary and
sparse circle fits, fallback geometry and the ENU transform keep their old
operations. Both public detector modes use this traversal. Acquired clocks,
tracking identities, forecasts, planning and control retain their interfaces.
Neither labels nor simulator truth enter the index.

For clouds of at most 64 retained points, the iterator retains exhaustive
traversal. It also falls back for any floored coordinate outside the exact
integer domain `[-2^52, 2^52]`, avoiding saturating casts and unsafe neighbor
arithmetic. Those fallback clouds retain quadratic scanning. Index storage is
linear in point count, but no universal latency or worst-case linear-time
guarantee is claimed for arbitrary inputs.

## Independent correctness checks

The complete preceding detector and fit bodies are frozen from commit
`2da76ef2de409c829a13499d7a62593057bcd1fc` in a
[reference fixture](../crates/perception/tests/fixtures/legacy_cluster_detector.rs).
The qualification script pins its SHA-256. It is used only by tests and the
benchmark example, rather than as an operational alternate detector.

A separate full directed adjacency graph checks queue order over 512 seeded
dense/sparse clouds. Boundary tests rotate and reverse input order across
negative cell boundaries, duplicates and strict-radius neighbors. A deliberately
asymmetric two-point example rejects symmetric component implementations.
Exact-domain limits and extreme finite coordinates exercise fallback. Full
detector comparisons use ordinary and sparse circles, invalid/near-origin
filtering and three ENU poses. Permutations are compared to the same permuted
legacy input; the original order dependence is preserved.

Two 16,384-point tests enforce deterministic work bounds. Separated points
perform zero distance tests instead of the old `N*(N-1)/2`. A single compact
bucket visits only `N-1` candidate entries instead of repeatedly scanning `N`
entries for each of its `N` points. The old compact case already had only
`N-1` distance predicates; the removed work there is scanning visited entries.
Counters exist only in the test build.

## Production release measurement

The benchmark calls the public production detector and the complete frozen
detector in the same release executable. It compares every detection's x/y/radius
bits, including output order. Each fixed workload runs nine paired repetitions
with alternating execution order. Equality checks occur outside the timer.
The table reports local medians, including filtering, fitting and transforms.

| Synthetic workload | Points | Previous (ms) | Indexed (ms) | Speedup |
| --- | ---: | ---: | ---: | ---: |
| Small circle | 64 | 0.014211 | 0.015473 | 0.92× |
| Circle | 720 | 1.711716 | 0.288124 | 5.94× |
| Dense circle | 4,096 | 54.412398 | 1.769404 | 30.75× |
| Compact cloud | 8,192 | 30.929713 | 0.677049 | 45.68× |
| Separated cloud | 8,192 | 222.369586 | 4.163945 | 53.40× |
| Ordinary circular arcs | 512 | 0.899904 | 0.129605 | 6.94× |
| Sparse circular arcs | 512 | 0.913735 | 0.187062 | 4.88× |

The 64-point case is about 9% slower, an increase of about 1.26 microseconds.
The first benchmark used a grow-on-first-push queue allocation. Its complete
results are retained; the delivered version reserves one initial element, as
the original detector did. Both trials keep every workload and all nine
repetitions. No timing cutoff or point-count threshold was tuned after viewing
the measurements.

These clouds isolate computation, not physical detector accuracy. The large
separated workload extends beyond the native sensor's 45 m range; circular
workloads can also exercise fallback fitting. They are not recorded automotive
data, a calibrated shape-confidence model, a whole-stack speedup, or a real-time
qualification. CI checks output equivalence and retains timings without gating
on host-dependent speed ratios.

## Reproduction

```sh
source scripts/env.sh
cargo build --release --locked -p rustdriving-perception --example cluster_index
python3 scripts/check_cluster_index.py --output artifacts/cluster-index
bash scripts/check.sh
```

The regression qualification also uses the pinned native release tests,
formatting and Clippy, every latest 48-condition sensor recording, and fresh
reference/native seed-42 mission canaries compared against their predecessor
streams. The exact required check passes 1,009 Rust tests, 295 Python tests and
49 standard runs/replays, with 245 standard streams matching retained bytes.
Native formatting, Clippy and all 67 tests pass. All 48 latest recordings replay
exactly over 60,831 control ticks. Both original-mission canaries reproduce their
five complete streams; the original missed goals remain missed.

Those 48 recordings contain 30,432 distinct planar scans, with at most 35
retained points. They exercise the exhaustive small-cloud path. To cover actual
dense acquisition, the additional
[blocked-road fixture](../scenarios/dense-lidar-stop.json) uses a large parked
obstacle and a roadside circle. It retains the normal 720-ray sensor, noise,
20-second deadline and source stopping criteria, including at least 10 m of
progress before stopping. The same input and seed run through the predecessor
and delivered executables in both plants.

Both dense pairs meet the source stop, independent 1 m swept-clearance and
0.1 m actual-road-reserve gates. Each records 201 distinct scans. Reference
acquisition reaches 107 retained points, with 163 scans above 64; native reaches
108 points, with 162 above 64. All five raw/replay streams and both summaries
are byte-identical within each pair. This supplements the 48 original conditions
without replacing any of them or counting another completed mission. All six
specified physical runs retain 30 compressed streams with encoded/decoded hashes.

The [versioned result packet](../assets/indexed-clustering-results.json) binds
the frozen reference, both benchmark trials, operational sources, executables,
scan eligibility audit and physical/replay receipts. The dense fixture is a
local physical comparison. CI is configured for the graph/work-bound tests and
benchmark equivalence, with timings retained only as diagnostics; remote
execution is not claimed.

The preceding missed mission goals and uncertainty-calibration limits
remain open. The comparative maturity estimate stays about 29%, with the
30% waypoint unmet.
