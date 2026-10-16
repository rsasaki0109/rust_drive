# Camera CPU confidence portability evidence

The original merged CI failed its exact comparison against historical camera
freshness output. An isolated AVX2/FMA fallback run on the same eight images
reproduces a historical mismatch: only confidence values differ, by at most
1.1920929e-7. Coordinates, clocks, classes, identities, decisions and projections
are unchanged. The original remote artifact's individual numeric difference
remains unavailable; the controlled reproduction and later CI runs are recorded
separately. [Implementation and comparison contract](../../docs/camera-ci.md).

Historical comparison now applies the existing 1e-5 absolute f32 bound only to
`confidence` and `last_observed_confidence`, rejecting score-threshold crossings
and all changes to other fields. Every accepted difference is recorded. The
current-host pipeline, guard/tracker replays, serial/parallel results and
normal/native-build results still require exact equality. Native implementation,
model, view geometry and all accuracy/freshness thresholds remain unchanged.

## Preserved observations

- `avx2-freshness.json.gz`: the complete eight-event fallback freshness trace,
  original published freshness reference and explicit raw-detection guard input.
  Regression tests replay it exactly in the original independent native guard.
- `baseline-healthy.json.gz`, `glibc-healthy.json.gz`, `avx2-healthy.json.gz`:
  complete original and probe outputs; comparisons retain all non-timing
  differences. The baseline was observed in the prior increment and is reused
  as a reference, not counted as a new model execution.
- `glibc-design.json`, `avx2-comparison.json`, `probe/`: frozen probe hypothesis,
  source/binary hashes, build logs and the two dispatch edits. All 312 library
  source files were checked; only those two edits differ. The first failed build
  lacked the copied toolchain file; that failure is preserved. This library
  patch is an isolated diagnostic and is absent from production dependencies.
- `pipeline-controls/`, `pipeline-validation/`: 28 fatal-input controls,
  original four replay protocols twice, 216 actual model forwards, exact repeats
  and standalone guard/tracker replays, and the historical difference ledger.
- `latency-controls/`, `latency-validation/`: actual measured-clock audit and
  exact guard replay; eight forwards, with both original 200 ms gates failing.
- `parallel-controls/`, `parallel-validation/`: 25 fatal inputs and two output
  protections; 208 forwards, including 64 current serial-reference forwards.
  Sixteen images twice match current serial results exactly.
- `cpu-controls/`, `cpu-validation/`: the normal/native-build comparison,
  25 fatal inputs, two output protections and six build-admission controls.
  Another 208 forwards include 64 current normal-build reference forwards;
  both native repeats match those references exactly.
- `checks/`: 12 regression tests including native replay, the required 371
  Rust tests / 49 scenario replays, source and evidence preservation, lint and
  packet verification logs. `ci-status.json` records observed remote runs.
- `source/`, `source-sha256.json`: the repair and native source closure.
  `before.json` retains original runtime pins and the 69 previously published
  evidence indexes. Every one of their 3,961 entries remains unchanged.

There are **704 new local model forwards**: 32 system-libm probes, 32 AVX2/FMA
probes, 216 pipeline, eight measured serial, 208 parallel and 208 native-build.
Remote CI forwards are separate. The recorded fallback fixture and packet
verification execute no model. Current timing checks retain failed 200 ms gates;
new performance or accuracy improvements are not claimed.

## Verify

```sh
python3 -B assets/camera-ci-v1/verify.py
```

This validates packet/source/previous-evidence hashes, the recorded fallback
comparison, all captured guard/tracker replay equalities, measured-clock audits,
current-host serial/parallel and normal/native image equivalence, and forward
counts. It needs no original images or weights. The native replay test command
in the linked implementation document additionally requires the original
standalone tracking binary; CI builds that binary first.

`files.json` covers every packet file except itself. `raw-files.json` records
original-byte sizes/SHA-256 for copied observations; JSON reports over 64 KiB
use deterministic gzip with `mtime=0`. No observation is rounded or relabeled.
`omitted-files.json` identifies invalid model fixtures excluded from the packet.
No original RGB/depth frames, poses, weights, executable binaries or patched
inference library are redistributed. Tiny control PNGs are existing authored
fixtures. Code/report and original TUM data terms are included separately.

The earlier failed CI and evidence remain intact. Passing this repair's checks
does not establish cross-platform inference identity, live-camera operation,
automotive accuracy or driving integration. Subjective OSS-relative maturity
remains about 20%, with 30% unmet.
