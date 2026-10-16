# Current-CPU verification of the guarded Resize optimization

The optional implementation lowers only the pinned YOLOX-S model's two exact
nearest/asymmetric/floor 2× expansions to standard tract axis, broadcast and
reshape operators. Weights, pixels, thresholds, decoder, view geometry, merge
and controller are unchanged. See [implementation and reproduction](../../docs/camera-operators.md).

The v2 profiler independently decodes both the original and lowered plan and
publishes both merges and final decisions. Every one of the 714,000 output
f32 values per view must match bit for bit. All sixteen original images run
twice in alternating execution order. The separate parallel checker captures
sixteen current ordinary-parallel image references after the timed trials;
both lowered repeats and every inferred timed frame must match those references
exactly, excluding durations. Only historical confidence comparisons use the
separate bounded [CPU drift rule](../../docs/camera-ci.md).

| Latest local measurement | Original | Lowered |
|---|---:|---:|
| Serial four-view inference mean, 32 paired images | 2.180055 s | 2.026524 s |
| Parallel processing first pair | 0.650533 s | 0.631123 s |
| Parallel processing second pair | 0.674117 s | 0.599007 s |

Parallel processing averages **7.14% less time** (1.077×) in these two pairs.
The frozen first run measured 15.35%; both observations are preserved, and
these small samples on one host do not establish a stable speedup. All four
200 ms deadline gates still fail, with zero available results out of eight
per trace. Inherited BDD quality gates still fail: 78 TP / 31 FP / 60 FN.
These are previously viewed images, without new accuracy scoring or driving
integration. Profiling times include observer overhead on the lowered side.

`operator-validation/` records 264 real model forwards. `parallel-validation/`
records 208, including 64 ordinary-reference forwards; its four standalone guard
replays agree exactly. Twelve model-free regression tests use these actual
reports and reject a one-ULP current confidence change even when historical
comparison is enabled. Raw hashes, decisions, coordinates and view-order
corruption controls are also retained.

`probe/` and `avx2-*` document a separate eight-forward real AVX2/FMA check.
The isolated probe uses the already preserved two-line tract dispatch patch,
not a production dependency change. Its source receipt differs only in the
probe manifest and lockfile. The current original and lowered outputs still
match bit for bit while confidence differs from the historical report. It is
a numerical portability check, without a timing claim. This increment made
**480 actual model forwards**, separate from the frozen v1 observations.

## Verify without weights or images

```sh
python3 -B assets/camera-operators-v2/verify.py
python3 -m unittest discover -s integrations/onnx -p test_camera_operators.py -v
```

The verifier checks the complete packet, original/compressed byte hashes,
current and archived source pins, 4,901 previously indexed evidence entries,
current-reference comparisons, repeated reports, raw-output hashes and clock
and guard audits. It restores the frozen v1 source in a temporary tree to run
v1's own verifier, preserving its exact source contract. The original v1
verifier therefore belongs to that archived version; the command above is the
entry point for the current checkout. Neither command executes a model.

`before.json` records the starting commit, source pins and 71 prior evidence
indexes. `source/` preserves the complete current code/checker/documentation
closure. `checks/` preserves input controls, Rust tests, lint, Python regression
results and the required root check. `ci-status.json` identifies the passing
ten-job PR #21 CI run separately from this optimization's subsequent CI.
Reports larger than 64 KiB are deterministically gzipped without rewriting
their original JSON; `raw-files.json` retains exact original hashes.

Images, weights and native executables are not redistributed. Tiny input
controls are authored fixtures; invalid ONNX fixtures are omitted and indexed
separately. Code/reports use Apache-2.0, and original TUM/BDD attribution and
data terms are copied byte for byte. Peak memory, other hardware and sustained
load remain unmeasured. OSS-relative maturity remains about 20%; 30% is unmet.
