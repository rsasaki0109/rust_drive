# Depth-supported recorded image matching

The optional `rustdriving-rgbd-supported` binary compares one sensor-only
matching change against the preserved [continuous room failure](temporal-recorded-motion.md).
All 180 images and their numerical reference source have already been viewed.
This is a **calibration regression**, not an unseen-recording or generalization
trial. The original sources, first outcomes and accuracy gates remain preserved.

## Candidate domain

The original extractor still supplies at most 400 FAST/BRIEF features in its
original order. Before descriptor matching, each feature's measured depth must
pass the existing validation: all nine pixels in the 3×3 patch are valid,
within 0.3–5 m, and span at most 0.05 m. This check uses the supplied depth
image and pinhole calibration; reference poses never enter matching.

Both directional nearest/second-nearest searches consider only those eligible
features. Strict `5*best < 4*second`, Hamming distance at most 64, mutual
association, tie rejection and the 256-match cap remain unchanged. A direction
with fewer than two eligible candidates cannot pass a ratio test. Matches map
back to original feature indices and retain the original deterministic order.
Reported eligibility lists are checked from original depth pixels by a separate
Python implementation.

This changes the candidate domain, and can change a ratio-test decision. It
does not create a depth value at a hole or relax a discontinuity threshold.
Filtering after the original matching would miss eligible associations whose
best or second-best competitor lacked valid depth. Malformed original features
and oversized original arrays still fail even when masked out.

The rigid consensus, original-inlier pixel refinement, reference replacement,
one initialization and 0.20 s accepted-pose expiry are unchanged. Rejected
updates have no current root output or clock renewal. Lost tracking remains
latched; there is no boundary reset, relocalization or coarse-pose fallback.
Ground truth is scored only after every operational fit, using the unchanged
0.1 m / 0.1 rad root gates.

## Diagnosis and evidence

The [measured-depth diagnosis](../assets/supported-depth-diagnosis.json) uses
the prior report's descriptors and six original depth images. With its last
accepted reference at index 158, the candidate domain supplies 15, 6, 5, 9 and
15 matches at 159–163, versus 13, 4, 4, 7 and 10 original depth-valid matches.
Every original supported pair in those five comparisons remains. Three still
have fewer than the unchanged minimum of 12; these candidate counts alone do
not establish a successful pose fit or recovered tracking. The prior loss also
involves substantial descriptor ambiguity, beyond missing depth.

The [declared design](../assets/recorded-supported/room-v1/design.json) fixes
this single change and retains indices 100–279 before candidate pose evaluation.
The source freeze, metadata qualification, report and independent audit are
stored alongside the design. All 179 updates, including failed and repeated
acquisitions, remain in the comparison denominator. The manifest's split tags
describe the original temporal trial's history; this variant explicitly treats
both parts as previously viewed.

## Measured outcome

The frozen calibration variant **fails, evaluator exit 1**. The independent
archived-source audit and viewed reproduction pass integrity checks; neither
turns the physical failure into a successful protocol.

| Same 179-update window | Original | Depth-supported |
| --- | ---: | ---: |
| Accepted updates | 58 | 59 |
| Root-accurate updates | 52 | 49 |
| Rejected updates | 121 | 120 |
| First accepted accuracy failure, source index | 149 | 149 |
| First latched tracking loss, source index | 164 | 170 |
| Maximum accepted translation error, m | 0.098846 | 0.118900 |
| Maximum accepted rotation error, rad | 0.110373 | 0.126653 |

Both runs initialize once and retain all 179 valid reference scores. The
candidate adds exactly one accepted acquisition, index 163. Fifteen supported
pairs yield 12 consensus inliers and renew the reference at age 0.169181 s.
That root already fails both accuracy gates. Indices 164–165 have 14 and 15
pairs but no consensus; 166–169 fall to 7, 4, 2 and 5 pairs. Accepted-pose
expiry latches loss at 170, age 0.231722 s. No current root is supplied thereafter.

This **does not improve root accuracy**: the accurate count drops by three,
despite one additional accepted fit and later loss. It remains an opt-in
research binary; the original method is unchanged. Descriptor stability,
accumulated rotation error and independently assessed recovery still require
work. More candidate pairs alone are insufficient.

The [failure detail](../assets/supported-failure-detail.json) retains the complete
chronology and evaluation-only pair diagnostics. Indices 155, 156 and 158 lose
their earlier accuracy classification before the additional fit at 163. The
next matching priority is bounded bidirectional image-patch tracking across
scales; it remains unimplemented, and accumulated drift remains a separate problem.

[Outcome](../assets/recorded-supported/room-v1/phase.json),
[full report](../assets/recorded-supported/room-v1/results.json),
[independent audit](../assets/recorded-supported/room-v1/audit.json),
[comparison](../assets/supported-comparison.json) and
[exact viewed reproduction](../assets/supported-regression.json) preserve this result.
The audit executes 49 positive contracts and 193 negative checks, including a
tracked 180-frame analytic-motion fixture accepted by the same continuous
auditor before corruption tests. Fixture poses are synthetic verification,
separate from recorded-camera evidence. Release tests pass all 49 optional
RGB-D tests, including 15 in the new binary. Workspace checks pass 371 tests
and 49 executions/replays; 245 generated outputs and 770 prior source/evidence
files match exactly. No new native execution is claimed.

## Reproduction

Use the existing optional RGB-D development environment; there is no new
dependency or toolchain. Original images remain ignored and are fetched
explicitly under the dataset's documented terms.

```sh
python3 -m pip install -r scripts/requirements-visual.txt
python3 scripts/fetch-temporal-dataset.py tum-fr1-room-temporal
source scripts/env.sh
cargo build --release --locked --manifest-path integrations/rgbd/Cargo.toml \
  --bin rustdriving-rgbd-supported
python3 scripts/check-recorded-supported-suite.py \
  --binary integrations/rgbd/target/release/rustdriving-rgbd-supported \
  --output artifacts/recorded-supported/local-reproduction
```

Use the actual binary path when `CARGO_TARGET_DIR` is set. The binary requires
`--regression` for both freeze preparation and execution. The suite prepares
and independently checks source-bound metadata before fitting, verifies the
preserved trial hashes, reproduces every non-timing report field, and independently
reconstructs image features, depth masks, descriptor associations, rigid fits,
refinement, clocks and physical errors. Evaluator exit 1 remains a failed
physical protocol; suite exit 0 means the declared outcome and its integrity
were reproduced. CI runs this path alongside the unchanged original temporal
evaluation.

This remains short indoor, offline camera odometry. It does not add scale
invariance, loop closure, vehicle sensor fusion, root covariance, a new physical
camera calibration or driving validation. See the original
[dataset provenance and license limits](../data/tum-fr1-room-temporal/SOURCE.md).
