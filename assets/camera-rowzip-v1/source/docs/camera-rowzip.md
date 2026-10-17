# Debian runtime reproduction and rejected slice traversal

The existing Nano depthwise path now completes its full reproduction in a
pinned Debian 12 / glibc 2.36 Python container, using host-built Rust 1.90
executables. Source is mounted read-only, output has a separate writable mount,
networking is disabled, and the container has a four-CPU limit. Python 3.12.14,
NumPy 2.3.5 and Pillow 12.3.0 use separately installed, SHA-256-pinned wheels.
The builder uses glibc 2.41. This checks another user-space runtime on the same
physical CPU and kernel; it is not independent hardware or remote CI evidence.

An additional optional `rustdriving-camera-rowzip` executable changes only the
interior traversal of the safe depthwise rows. One source slice replaces each
per-pixel indexed read loop; zipped stride iteration retains bias-first,
row-major multiplication/addition order and the original scalar boundary loops.
The graph binder, model weights, Resize2x lowering, four views, preprocessing,
decoder, thresholds, postprocessing and guard are reused from the frozen source.
The 200 ms age, two-sample recovery and acquisition fault epochs are unchanged.

The candidate **is not adopted**: its small serial improvement does not produce
consistent availability. The original executables and all prior measurements
remain unchanged.

## First complete observations

The existing reproduction wrapper actually executes its 16-forward pilot and
388-forward main comparison, for **404 unique forwards** in Debian. All raw
output checks and eight independent standalone guard replays pass. The main
serial reduction against Resize-only execution is 13.01%, versus the earlier
host observation of 9.79%; neither is a portable speed guarantee.

| Existing depthwise result | Earlier host | First Debian reproduction |
|---|---:|---:|
| FIFO first available / 8 | 2 | 1 |
| FIFO repeat available / 8 | 2 | 1 |
| Supervised first available / 8 | 4 | 3 |
| Supervised repeat available / 8 | 4 | 4 |

Each of the two Debian depthwise FIFO trials has one inferred result miss its
own deadline. These failures remain in the report. The original unoptimized
supervisor in that comparison supplies 3/8 and 1/8 available results. A successful
execution/audit is distinct from passing availability or deadline gates.

The new candidate then receives its own frozen protocol before its first model
forward. FIFO and supervised replay each use indexed/slice/slice/indexed order.
All eight timed trials precede current references and profiles, with no candidate
warmup or timing-based retries. Both baselines are the same existing depthwise
executable, isolating the traversal change in both controllers.

| Traversal comparison | Inferred / 8 | Available / 8 |
|---|---:|---:|
| FIFO indexed first | 4 | 1 |
| FIFO slice first | 4 | 1 |
| FIFO slice repeat | 4 | 1 |
| FIFO indexed repeat | 4 | 1 |
| Supervised indexed first | 4 | 3 |
| Supervised slice first | 3 | 1 |
| Supervised slice repeat | 4 | 2 |
| Supervised indexed repeat | 3 | 1 |

No inferred candidate-comparison result misses its own deadline, but readiness
still depends on the previous acquisition, fault epochs and recovery samples.
Every original source remains in the denominator, including skipped/superseded
images. The slice supervisor's maximum poll gaps are 16.93/13.64 ms; both exceed
the 10 ms target. Its first availability result regresses, while its repeat
improves. The default stays unchanged.

On the same sixteen previously viewed images, paired serial profiles alternate
indexed/slice order. Eleven of sixteen pairs are faster; aggregate time falls
**1.69%**. Five regressions, including a 6.38% slower BDD pair, are preserved.
Each profile independently compares all raw values with the ordinary tract
operator, and the indexed/slice outputs are also compared directly. These are
192 pair comparisons of **57,919,680 f32 values**, without numerical tolerance;
shared/transitive comparisons are not additional model executions. All current
serial detections, parallel results, merges and decisions match. Independent
postprocessing and eight standalone guard replays check the new comparison.

The new comparison executes **504 forwards**: 120 timed, 256 paired profiles,
64 original Nano serial references and 64 parallel image forwards. Combined with
the separately recorded 404-forward existing reproduction, this increment has
**908 unique model forwards**. The earlier local 408-forward packet is historical
and is not added again. One new kernel edge/lane test supplements 64 reused native
tests; fourteen Python regressions check existing corruption controls and the
new producer markers. Twenty-seven fatal-input controls, two output protections
and a model-free supervised guard replay execute no valid model.

The mathematical algorithm/model-profile identifiers remain the existing
Nano depthwise identities because the algorithm is unchanged. Every direct
candidate report additionally requires `producer_variant =
"nano_depthwise_rowzip_v1"`, its separate profile hash, and a complete 68-file
compiled source receipt. Nested typed analyses retain that full source receipt.
Metadata describes internally computed work; the command accepts no external
report for relabelling. No custom runtime, unsafe code or dependency is added.

This adds no unseen recording, BDD accuracy score, identity evaluation, live
camera or driving integration. The inherited detector-quality gates still fail.
The required 371 root tests and 49 scenario replays pass; 245 core outputs,
149 previous frozen sources and 6,204 earlier indexed evidence entries are
unchanged. Subjective maturity stays about 20%, below the 30% waypoint.

## Reproduce

Build the five native executables and guard as in the new workflow. Cache the
unchanged models and sixteen images using the existing fetch commands. For a
Linux/amd64 host with Python 3.12 and Docker, create a fresh artifact directory:

```sh
python -m pip download --only-binary=:all: --no-deps \
  -r scripts/requirements-visual.txt --dest artifacts/rowzip-example/wheels
docker pull python@sha256:392307d22300de8b5986851a12d9176dfc0fc073e65bf6523ebd7dcbeb23564e
docker run --rm --network none --cpus 4 \
  --mount "type=bind,src=$(pwd),dst=$(pwd),readonly" \
  --mount "type=bind,src=$(pwd)/artifacts/rowzip-example,dst=$(pwd)/artifacts/rowzip-example" \
  --workdir "$(pwd)" \
  python@sha256:392307d22300de8b5986851a12d9176dfc0fc073e65bf6523ebd7dcbeb23564e \
  sh scripts/check-camera-rowzip-container.sh "$(pwd)/artifacts/rowzip-example"
```

The helper checks exact wheel hashes before offline installation and runs both
full phases. This combined helper is syntax checked; its constituent container
commands were executed separately for the first recorded observations. A second
full execution through the helper and remote CI are unobserved. Docker is only
an optional research reproduction tool, not a stack runtime dependency.

The candidate accepts the same image/profile/FIFO/latest/supervised CLI options
as the existing depthwise executable. The lower-level checker executes only its
reported new phase, without silently adding the previous reproduction count.

```sh
python -B assets/camera-rowzip-v1/verify.py
python -B -m unittest discover -s integrations/onnx -p test_camera_rowzip.py -v
```

These checks execute no model and require neither Docker nor original images.
The packet contains source, reports, authored tiny controls, scripts, runtime
metadata and wheel hashes. It omits weights, executables, wheels, installed
packages and original camera images.
