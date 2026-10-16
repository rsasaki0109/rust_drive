# Immutable measured-depth-map localization

The optional `FixedMapTracker` and `rustdriving-projective-fixed-map` binary
estimate local SE(3) pose directly against one immutable measured depth image.
They reuse the existing bounded projective point-to-plane fitter without
rolling fusion or reference replacement. The [fixed design](../assets/projective-fixed-map-v1/design.json)
tests whether removing accumulated map updates improves continuous accuracy.
The complete physical protocol **fails**, so this path is not adopted.

## Contract

The first qualified acquisition establishes an optical x-right/y-down/z-forward
map frame in metres. Initialization self-fits the measured image and requires
the same geometric support, residual and physical-information checks as later
fits. A rejected initial image establishes no map or pose. Map depth, pyramid,
frame and original acquisition timestamp remain unchanged after initialization.

Each subsequent fit uses that fixed pyramid and the last accepted absolute
first-camera-from-current pose as its prior. There is no motion extrapolation,
fusion, rekey, pose composition through replacement references or reset. The
original pyramid iteration caps, correspondence/normal policy, increment limits,
0.03 m residual limit and 0.5 m displacement limit remain in effect. That
displacement bound applies relative to the fixed map and limits its local
operating range. Information conditioning is a structural check, not calibrated
covariance or a guarantee of correct association.

The first accepted pose clock is the actual source acquisition time. Invalid,
duplicate, regressing or rejected frames cannot refresh it. More than 0.2 s
since the last accepted acquisition latches loss, including when the current
depth is malformed. A retained map after loss is historical state. The binary
permits one camera configuration and at most 200 bounded JSON-line frames.
It accepts depth only; no reference poses, simulator truth or driving commands
enter the runtime.

## Complete viewed results

The trial retains all 180 original acquisitions, depth indices 100–279, from
each of the same three previously viewed TUM recordings. Each denominator is
179 updates, including inaccurate and rejected updates. No trajectory is reset
or truncated after loss. Intrinsics remain 525/525 with center 319.5/239.5 and
5,000 raw units per metre. Numeric motion-capture references are read only
after **all three** native sensor ledgers have been saved.

| Recording | Accepted / 179 | Root accurate / 179 | Max accepted position error | Max accepted rotation error | Lost |
| --- | ---: | ---: | ---: | ---: | --- |
| Room | 24 | 24 | 0.041309 m | 0.035177 rad | Yes |
| Desk2 | 20 | 20 | 0.033323 m | 0.061126 rad | Yes |
| RPY | 31 | 17 | 0.534438 m | 0.704551 rad | Yes |

Room and desk2 meet the 0.1 m / 0.1 rad accuracy gates on accepted updates,
then lose availability. Their first rejections occur at source indices 125
and 121. RPY first rejects at 130, later accepts additional frames, and
contains 14 accepted inaccurate updates. Residual/support/displacement and
increment guards eventually reject all three trajectories; accepted-pose
expiry then latches loss. A small residual and finite information cannot
certify correct correspondence or physical pose.

The fixed pretrial adoption rule requires all 179 updates accepted and accurate
in **each** recording without loss. All three fail. The rolling default remains
unchanged, with its previous room/desk2/RPY accuracy counts 64/161/74. Neither
result establishes automotive map localization or the overall 30% milestone.

## Independent validation and reproduction

The [raw audit](../assets/projective-fixed-map-v1/raw-audit/summary.json)
reconstructs each depth pyramid from pinned PNGs and uses the original fixed
map for every correspondence. It verifies all **1,921** emitted normal systems,
six-axis increments, accepted/rejected state, acquisition clocks and complete
root scoring. Its integrity pass preserves the failed quality outcome.
Six independent ray-cast motion controls check optical direction and metric
scale; 30 altered equations, gradients, priors, poses and counts are rejected.
The standalone Rust library passes 62 tests, including three new fixed-map
state/expiry tests.

The rolling library's existing algorithm is unchanged apart from exposing the
new separate module. Because rebuilding changes the binary identity, actual
legacy depth, RGB-D and timing sensor replay checks all **1,440** historical
outcomes, including equations, fusion, poses and clocks. Numeric reference
poses are not read during that comparison. Source freezes, first reports,
failures, audits and compatibility receipts remain in the
[evidence packet](../assets/projective-fixed-map-v1/summary.json).

With the [existing recorded inputs and tool environment](projective-depth-rust.md#reproduction),
choose new output directories:

```sh
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/projective-depth/Cargo.toml
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-fixed-map.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-fixed-map \
  --output artifacts/fixed-map-example/controls
OPENBLAS_NUM_THREADS=1 python scripts/evaluate-projective-fixed-map.py \
  --binary integrations/projective-depth/target/release/rustdriving-projective-fixed-map \
  --design assets/projective-fixed-map-v1/design.json \
  --output artifacts/fixed-map-example/recorded
# Expected exit 1: all three complete physical accuracy/availability gates fail.
OPENBLAS_NUM_THREADS=1 python scripts/check-projective-fixed-map.py \
  --reports artifacts/fixed-map-example/recorded \
  --output artifacts/fixed-map-example/raw-audit
```

This is offline indoor research. It supplies no vehicle integration, qualified
uncertainty, global relocalization or fresh held-out result. TUM provenance and
data terms remain [documented separately](tum-source-provenance.md); raw sensor
files and binary executables are not redistributed in the evidence packet.
