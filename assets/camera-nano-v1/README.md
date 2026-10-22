# Fixed Nano camera candidate evidence

The independent `rustdriving-camera-nano` command adds the existing fixed views,
parallel execution, merge, vehicle consistency and measured clocks to the
original pinned Nano detector. Read [the implementation and limits](../../docs/camera-nano.md).

The two first-image processing samples are **89.43 ms and 86.68 ms**, compared
with **542.21 ms and 527.27 ms** for the unchanged lowered-S baseline: a local
6.073× ratio of means. Both Nano deadline flags pass, while only **one result
out of eight is available** in each replay; four images infer and four skip.
The guard's original 200 ms limit and two-sample recovery are unchanged. This
does not establish continuous availability or automotive timing.

Original BDD annotation scoring gives Nano **55 TP / 39 FP / 83 FN**, precision
58.51%, recall 39.86%. The current original single-view Nano has 35 / 14 / 103;
the inherited S result is 78 / 31 / 60. All original quality gates still fail.
These sixteen TUM/BDD images were already viewed, without new holdout or
object-identity ground truth. No camera result enters vehicle control.

## Captured observations

- `before.json` pins the starting commit, original protocol, 72 prior indexes
  and previous source closure. All 5,171 previously indexed entries remain exact.
- `controls*` preserve three executions of 30 fatal input controls and two output
  protections. Model-free traces never load weights or invent detections.
- `validation-no-original-binary/` and `first-preflight-failure.log` retain an
  initial setup failure before any inference, when the original Nano executable
  was not built in this checkout. Its unchanged source was then built.
- `validation/` preserves the first 44 forwards: 40 timed S/Nano/Nano/S forwards
  and one four-view serial image. The first auditor compared threshold JSON
  decimals directly and rejected a value with the correct f32 bits.
- `validation-final/` imports those 44 observations and makes 204 new forwards.
  It retains all sixteen serial Nano reports, sixteen original full-image Nano
  reports and two complete parallel repeats per image. The next audit rejected
  typed-f32 versus promoted-f64 result text despite identical represented bits.
- `validation-checked/` imports and re-audits all **248 unique model forwards**
  with exact f32 bits, independently recomputes merges/consistency, reconstructs
  all clocks, verifies four standalone guard replays and scores original labels.
  This final audit performs **zero new model forwards**. Original timing reports
  and image outputs match their earlier bytes; no timing sample was replaced.
- Both failed auditors, their source bindings and errors remain in their original
  directories. Only declared f32 detection fields are canonicalized; there is no
  numerical tolerance. Clock values, types, counts and decisions remain exact.
- `annotations/` contains the eight original BDD annotation JSON files, checked
  against the existing pre-inference protocol hashes. The verifier independently
  reconstructs all reference boxes, matching identities, false detections, misses
  and aggregate counts without requiring original images.
- `source/` binds the native/checker/documentation/workflow closure, including
  the unchanged original Nano library and CLI; `checks/` retains test/build/lint
  and required root-check logs. `ci-status.json` separates observed PR #22 CI
  from this candidate's subsequent remote CI.

## Model-free verification

```sh
python -m pip install -r scripts/requirements-visual.txt
python -B assets/camera-nano-v1/verify.py
python -m unittest discover -s integrations/onnx -p test_camera_nano.py -v
```

Twelve regression tests include one-ULP confidence and box changes, exact
threshold bits, signed zero, integer/bool distinctions, producer/tensor/view
corruptions and one-f64-ULP clock, source-age, policy and readiness changes.
The optional native command also passes its 39 reused library/clock tests;
these are not 39 newly invented acceptance cases.

`files.json` covers every packet file except itself. Large JSON is deterministically
gzipped, with original-byte hashes in `raw-files.json`. Code and reports are
Apache-2.0; original BDD annotation metadata retains its data notice. TUM/BDD
notices are copied byte for byte. Original images, model weights and native
executables are excluded; tiny authored control PNGs accompany input cases.
Subjective maturity versus mature driving OSS remains about 20%; 30% is unmet.
