# Debian runtime and slice-traversal evidence v1

The existing depthwise wrapper is fully executed in pinned Debian 12 / glibc
2.36, with independently installed hashed wheels, read-only source, offline
networking and a four-CPU limit. Its first 404 forwards, including its actual
16-forward pilot, pass output and guard audits. The physical CPU/kernel and
host-built executables are shared; this is runtime/ABI evidence, not independent
hardware. The earlier 408 host forwards are not counted again.

A new candidate replaces indexed interior reads with zipped source slices while
reusing every other model/guard operation. Its 504-forward fixed comparison
verifies 192 raw pair comparisons / 57,919,680 f32 values, current serial/parallel
measurements and eight complete standalone guard replays. Serial aggregate time
falls only 1.69% (11/16 faster pairs). Supervised availability changes from 3/8
and 1/8 to 1/8 and 2/8. FIFO stays 1/8 in every trial. Both candidate poll gaps
exceed 10 ms. The candidate is not adopted. All first results and regressions
remain intact. [Contract and results](../../docs/camera-rowzip.md).

Total unique forwards for this increment: 404 + 504 = 908. There are sixteen
measured guard replays plus two model-free control replays. No new holdout,
accuracy score, identity score, driving integration or hard real-time claim is
made; inherited quality gates fail and maturity remains about 20% (subjective).

`files.json` indexes everything except itself. `raw-files.json` pins original
uncompressed bytes and deterministic gzip uses mtime 0. The 157-file source
closure reuses 149 previous sources from the five parent camera packets; only
eight new source files are copied. All 77 prior indexes / 6,204 entries and
245 core outputs are unchanged. Weights, executables, wheels, installed packages
and original camera images are excluded. Tiny authored PNG controls and invalid
image text are retained; nine-byte invalid-model controls are omitted by hash.

The original actually executed container scripts are preserved here. A separate
combined public helper has syntax validation and consists of those same phases;
it has not received a second full local execution. Remote CI is unobserved.
Model/code uses the original Apache-2.0 provenance. Complete TUM/BDD notices are
retained with derived measurements; there are no new images or annotations.

```sh
python -B assets/camera-rowzip-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_rowzip.py -v
```

Both checks execute no model and need neither Docker nor original images.
