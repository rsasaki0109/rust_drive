# Preprocessing compatibility and new road-image evidence v1

The safe Rust YOLOX-style linear preprocessing matches a separately installed
OpenCV reference on 48 unique generated cases and all 24 actual source images.
Input-tensor hashes for all 96 real production crops match the reference.
Two reversed-order repetitions and current original-graph references produce
144 image reports / 576 unique forwards. All current tensor hashes and decoded
results match their repetitions and graph references.

The 16 newly selected BDD frames contain 268 mapped objects. Triangle scores
99 TP / 60 FP / 169 FN, versus Linear 101 / 62 / 167. Linear loses five original
matches while gaining seven; person/rider and traffic-light recall regress.
All frozen quality gates fail and fresh precision regresses. The candidate is
**not adopted**. The eight viewed-image results, all classes, misses and false
positives remain intact. [Full contract and commands](../../docs/camera-linear.md).

`verify.py` reconstructs the first-16 selection from the pinned complete Git
tree, verifies every original annotation, rebuilds all NumPy postprocessing and
recomputes both metric groups. It re-audits with the exact initial checker and
the corrected checker. Their results agree after JSON representation alone is
normalized. The first failed audit and one-line tuple/list correction remain
preserved; no extra inference or changed scores accompany that correction.

The initial fetch's HTTP 503 and the incomplete first model-free control attempt
are retained. Its 48 native generated-case results are byte-identical to the
final 48, pinned here once. The first model-free loop driver was not separately
snapshotted. Only the final complete control session is used to authorize the
model comparison. There are 48 unique generated cases executed twice, 24 actual
full-image pixel diagnostics, six fatal controls and one existing-output check;
none add valid model forwards. The generated PNGs and diagnostic pixel exports
are excluded along with original camera images, weights, wheels and executables.

The 167-file source closure reuses the 157 files from prior camera packets and
archives ten new sources. All 78 previous indexes / 6,510 entries and 245 core
outputs remain unchanged. `files.json` indexes every file except itself;
`raw-files.json` pins original uncompressed bytes. Gzip uses mtime zero.
Original annotations and derived reports retain the included BDD research,
educational and not-for-profit terms; code remains Apache-2.0.

No latency, availability, tracking-identity, live-camera or vehicle-integration
claim is made. The new workflow has not run remotely; its success would validate
execution and evidence integrity, not convert failed accuracy gates to passes.
The subjective maturity estimate remains about 20%, with 30% unmet.

```sh
python -B assets/camera-linear-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p 'test_camera_linear*.py' -v
```

These checks need no original images, weights or OpenCV and execute no inference.
