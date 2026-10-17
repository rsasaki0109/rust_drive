# Frozen baseline-preserving camera comparison

`python -B assets/camera-preserved-v1/verify.py` independently reconstructs all
saved results with pinned NumPy/Pillow, without images, weights or inference.
It checks216 cumulative source files, all7958 previous evidence entries,245
core outputs, original annotations, full parent identities and fixture bytes.

The native component performs explicit model-free postprocessing: the complete
640px supported measurements remain an unchanged prefix; eligible416px
supported measurements append after overlap checks. Evaluation separately
verifies the two full original producers on the same image. See
`docs/camera-preserved.md` for the fixed protocol and adoption decision.

Fresh16 uses1056 new forwards. Viewed40 uses240 saved native parents with2640
old forwards, never recounted. All56 pairs have three native fixture passes:
168 runs, zero forwards. All432 new JSON outputs/fixtures are pinned before
new numeric labels are parsed. Every anchor measurement and lineage remains;
greedy ground-truth match retention is measured separately.

`files.json` pins exact bytes; `raw-files.json` pins decompressed original
bytes. Gzip has mtime0. Earlier packets hold207 prior sources; this packet
archives9 new top-level files. Raw images, pixel exports, weights, executables
and wheels are excluded. Original annotation JSON retains the separately
copied full BDD data terms; code remains Apache2. Passing integrity checks
does not imply passing quality gates. Remote CI is unobserved. No availability,
driving-control, physical safety or maturity gain is claimed.

The first scoring attempt rejects an original zero-height signal reference.
All model/fixture executions and prediction-prefix checks completed first.
The unchanged failure is reproduced; fresh16 quality remains unknown and
`adopt_candidate:false`. No annotation correction, image removal, gate change
or model rerun occurs. Viewed40 is scored separately. Integrity success does
not imply a successful fresh accuracy evaluation.
