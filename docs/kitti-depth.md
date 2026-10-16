# Sparse depth from projected LiDAR points

The optional `rustdriving-kitti-depth` command converts a version-1
`rustdriving-kitti-project` JSON report into sparse depth cells. It reads the
already projected rows; it does not load images or acquire KITTI data.

Each in-frame pixel `(u, v)` belongs to cell `(floor(u), floor(v))` on the
original rectified canvas. The winning point has the smallest positive
**selected-camera axial Z in metres**, not Euclidean range. Exact depth ties
keep the smallest original point index; there is no epsilon tie interval.
Cells are emitted in row-major order. Each cell includes its winning source
index, depth and complete count of points projecting into it. Unobserved cells
are absent, without interpolation, filling or an inferred zero depth.

Every input point also has an assignment: `selected`, `occluded`, `outside`
or `behind`. Selected/occluded assignments link to the winning original row.
Here `occluded` means losing the discrete cell's depth comparison; it does not
prove physical occlusion or account for a moving scan or camera exposure.

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/kitti-depth/Cargo.toml
integrations/kitti-depth/target/release/rustdriving-kitti-depth \
  --projection artifacts/kitti/local-projection.json \
  --output artifacts/kitti/local-depth.json
```

The standalone crate enables correctly rounded JSON float parsing, so adjacent
binary64 depths remain distinct. Its dependencies and lockfile are separate
from the existing projection evaluator.

The input is bounded to 64 MiB and 1–200,000 consecutive point rows. Canvas
bounds match the projection tool: dimensions 1–8192, at most 16,777,216 pixels.
Storage scales with point rows and occupied cells, not canvas area. Nonfinite
coordinates/reflectance/pixels, invalid row IDs, contradictory status/depth/
canvas combinations, mismatching camera IDs and inconsistent summary counts
fail before rasterization. Other projection metadata remains in the source
report; its SHA-256 and byte count bind the exact supplied report. This tool
validates projected-row consistency, without re-estimating calibration or
authenticating the source projection's geometry.

Output must be a fresh path, without symlink parents or `.`/`..` components;
admission precedes input reads. An input failure may leave an empty reserved
output. Exit 0 means a valid transformation; exit 2 means a contract/I/O error.

The independent checker uses the existing authored four-camera calibration
fixture and reconstructs the full homogeneous projection from calibration
and raw float32 points before computing expected depth cells. Additional
hand-specified projected-row controls cover axial versus range ordering,
exact ties, half-open cells, raster collisions, empty observed canvases and
preservation of all point assignments. These are analytic checks, without
measured KITTI accuracy, image fusion or vehicle-pipeline integration.

The local Linux check passes **5 new Rust tests**, **371 core tests** and
**114 existing RGB-D tests**, with formatting and warnings-denied Clippy.
All 49 core scenarios/replays reproduce their 245 preceding files exactly;
the four frozen projection reports also remain byte-identical. The depth
checker passes 18 positive CLI runs, rejects 35 malformed/input/output CLI
controls and seven report corruptions, and reproduces seven depth outputs
byte-for-byte. Adjacent binary64 depth and pixel controls caught an initial
JSON-rounding failure; the standalone float-roundtrip dependency repairs it.

| Authored camera | Input points | Occupied cells | Points losing the cell comparison |
| --- | ---: | ---: | ---: |
| 0 | 18 | 7 | 3 |
| 1 | 18 | 5 | 3 |
| 2 | 18 | 4 | 1 |
| 3 | 18 | 7 | 5 |

The [validation record](../assets/kitti-depth-v1/validation.json) binds tested
source hashes, the [independent result](../assets/kitti-depth-v1/results.json),
a [camera-0 depth report](../assets/kitti-depth-v1/camera-0-depth.json) and the
[adjacent-depth regression](../assets/kitti-depth-v1/near-tie-depth.json).
The added CI job runs these checks; no remote run or other-host execution is
claimed by the local record.

```sh
python scripts/check-kitti-depth.py \
  --projection-binary integrations/rgbd/target/release/rustdriving-kitti-project \
  --depth-binary integrations/kitti-depth/target/release/rustdriving-kitti-depth \
  --output artifacts/kitti/depth-audit
```

Use the pinned NumPy environment in `scripts/requirements-visual.txt`. Every
output directory must be new. Existing [projection evidence](kitti-projection.md),
data-access and license boundaries still apply. No new sensor accuracy or
engineering-maturity claim follows from rasterization.

The [provided-file evaluator](kitti-depth-evaluation.md) now reports conditional
depth errors and missing-reference coverage under a frozen selection. Its
current KITTI run awaits authorized input files. A separate
[recorded Argoverse experiment](argoverse-sparse-depth.md) verifies in-frame
projection and all raster assignments on 278,790 measured points, retaining
two initial off-canvas numerical-gate failures and no physical accuracy claim.
