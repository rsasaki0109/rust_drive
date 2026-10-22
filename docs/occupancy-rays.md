# Occupancy ray traversal

The default diagnostic occupancy grid now visits the cells crossed by a return
ray instead of sampling positions at roughly one-resolution intervals. The old
sampler could count a cell twice, skip a short cell intersection, and subtract
free-space evidence from the return's own hit cell. These errors affected the
exported map. The current planner consumes tracked objects rather than this
grid, so this repair does not establish improved driving availability.

## Update contract

For each measured return, the mapper transforms the body point with the supplied
acquisition pose, clips its segment to the grid, and walks crossed cell
boundaries. The grid uses half-open cells, including the lower coordinate and
excluding the upper coordinate on each axis.

- Each positive-length crossed cell receives one `-0.15` log-odds update, except
  the cell containing the endpoint.
- An in-grid endpoint receives one `+0.85` update. A zero-length return supplies
  only this endpoint evidence.
- A corner touch alone supplies no free-space evidence. A ray along a boundary
  follows the half-open cell on that boundary's upper-coordinate side.
- Separate returns still accumulate independently, in input order. Log odds
  retain the existing `[-4, 4]` saturation and occupied threshold `>1`.
- Empty scans supply no ray evidence. Nonfinite poses, points, or normalized
  transforms are ignored; the pipeline retains its existing scan-admission and
  acquisition-clock checks.

For example, a unit grid ray from `(0.25, 0.25)` to `(2.75, 2.75)` now updates
cells `(0,0)`, `(1,1)`, `(2,2)` by `-0.15`, `-0.15`, `+0.85`. Previously the
first cell received `-0.30` and the endpoint received about `+0.70`. A second
regression reproduces short traversed cells missed between distance samples.

The iterator uses constant auxiliary storage. Each iteration advances an axis
or finishes, limiting traversal to the grid width plus height regardless of
out-of-grid ray length. Boundary times are recomputed from the original segment
to avoid accumulated increment drift. This is a work bound, without a measured
real-time deadline claim.

Clipping and traversal use `f64`. Extremely remote exterior endpoints can lose
subcell distinctions; an intersection whose parameter interval rounds to zero
is omitted. The independent exact tests cover their declared dyadic input
domain, rather than establishing arbitrary-precision production geometry.

## Independent checks

The mapping crate tests every cell rectangle independently using exact rational
interval intersections. It does not call the production traversal, clipping,
or indexing helpers. The fixed seed generates 8,192 rays, and 19 hand-authored
edge, corner, parallel, exterior, reverse and zero-length rays run with three
translated/scaled grids. Together these compare 519,687 cell results.

Additional tests cover rotated body measurements, nonbinary resolution
boundaries and adjacent floating values, invalid transformations, independent
repeated returns, saturation, empty scans, and grid-bounded traversal with remote
endpoints. The two original bias regressions fail on the preceding implementation;
their failure logs are retained.

Reproduce the component and complete required checks with:

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-mapping
bash scripts/check.sh
```

## Local qualification

The exact required command completed with exit 0: 1,019 Rust tests, 295 Python
tests and all 49 standard runs with sensor-only replay. Across those runs, all
196 sensor, replay and summary streams retain their preceding bytes. Every
`run.json` retains all other decoded fields; 31 occupied-cell exports change.
Native formatting, strict release Clippy and all 67 native tests also pass.
The combined local Rust total is 1,086 tests. The ordinary CI workspace test
step includes the new component tests; remote CI was not run for this change.
All 48 latest archived sensor-only recordings also reproduce their preceding
outputs byte for byte over 60,831 control ticks. Their unsuccessful missions
remain unsuccessful. Encoded and decoded hashes verify all 59 newly archived
streams: 49 standard run files and ten streams from the two fresh physical runs.

Two fresh `dense-lidar-stop` executions use the unchanged preceding scenario,
seed 42 and both vehicle models. Both retain their original stopping,
physical-clearance, road-containment and localization gates. Each reproduces
all four operational streams and every other run field from its retained
predecessor recording. The reference occupied-cell export changes from 49 to
48 cells; the native export remains 61. Cell counts alone do not score map
accuracy or probability calibration.

The initial required check exhausted temporary storage during replay after
all component tests had passed. Its completed and partial records remain
losslessly archived. The final command reran on identical runtime sources,
releasing only rebuildable debug cache after tests and release compilation.
Its asynchronous archive collector initially missed the already completed
`blocked` case; a separate postprocessor checked those five remaining files
against the same parent and completed the 49-case collection. Both original
collector summaries and the repair receipt remain in the evidence. No runtime
source, scenario or acceptance gate changed during these storage and collection
repairs. [Result packet](../assets/occupancy-rays-results.json).

Dynamic-object ghost cells, localization corrections to historical cells,
no-return horizon clearing, inverse-sensor probability calibration and SLAM
remain open. The comparative maturity estimate stays about 29%, and the 30%
waypoint remains unmet.
