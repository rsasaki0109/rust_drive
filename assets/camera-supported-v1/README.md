# Coarse-preserving support evidence v1

The new native rule keeps all original four-view measurements and appends only
distinct-crop-supported, non-conflicting supplemental hypotheses. On the new
sixteen images, four views score 118 TP / 72 FP / 151 FN, unrestricted twelve
score 151 / 113 / 118 and supported additions score 134 / 90 / 135. Support
removes some grid false positives but does not meet four-view precision or the
original absolute gates. The candidate is **not adopted**. All three policies
use the same measurements. [Method and full results](../../docs/camera-supported.md).

Native processing reuses 120 frozen reports on forty viewed images without
inference; their 1,440 historical forwards are not counted again. Sixteen new
images run twice and against a current original graph: 48 image reports / 576
new forwards. All measurements, immutable coarse prefixes, support witnesses,
decisions and counters pass independent reconstruction and repeat/reference
checks. The 16 original annotations are retained; previous 40 annotations,
native parent reports and pixel manifests are reused from earlier packets.

`verify.py` rebuilds the fixed selection, all three postprocessing policies and
all metrics without original images, model weights or executables. The initial
model-free checker and hash-verified `math` import addition are preserved;
both control sessions execute zero forwards. There are eleven unique valid
fixtures and nine rejected fixtures, repeated before the final comparison.
Initial native compile/test namespace corrections precede all model inference.

`files.json` indexes everything except itself. `raw-files.json` pins original
uncompressed bytes; gzip uses mtime zero. The 187-file source closure reuses
177 prior sources and archives ten new sources, preserving 80 prior indexes /
7,122 entries and 245 core output pins. Images, lossless pixel exports, model
weights, wheels and executables are excluded. BDD data and derived evidence
retain the included research/educational/not-for-profit terms separately from
Apache-2.0 code. Remote workflow execution is unobserved.

No new latency, availability, identity, depth or driving integration is claimed.
The subjective maturity estimate remains about 20%, with 30% unmet.

```sh
python -B assets/camera-supported-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_supported.py -v
```

Both commands execute no model and need no original images, weights or OpenCV.
