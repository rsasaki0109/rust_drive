# Complete run artifact publication

Both run CLIs previously wrote directly to final filenames. A valid scenario
named `run.json` inside its output directory was overwritten by the evaluation
result. Native `--scene scene.json` could similarly destroy its own input.
Malformed/missing input or a later write error could leave a previous successful
`summary.json` beside incomplete data. The new shared `RunOutput` publisher
protects inputs and publishes the completion marker after the complete record.

## CLI contract

The reference CLI manages `run.json`, `sensors.jsonl` and `summary.json`. The native
CLI also manages `scene.json`, including removal of stale scene evidence on a
subsequent run without a scene. The reference CLI leaves other filenames alone.
Option syntax and native mode prerequisites are checked before publication
preparation; help and argument-parsing errors do not start a new run.

After valid options are parsed, and before loading the scenario/scene or running
simulation, the publisher checks every existing managed destination against all
supplied inputs. Canonical paths catch direct and symbolic-link aliases; Unix
device/inode checks also reject hard-link aliases. An alias returns exit 2 and
preserves inputs and the entire previous managed artifact set. Distinct inputs
inside the output directory remain supported.

After these checks, the publisher removes the previous `summary.json` first,
then unlinks the other managed entries. Unlinking replaces an output link without
writing through it to its target, including dangling and cross-linked entries.
A directory at an artifact path causes an error and its contents are preserved.
An invalidation error can retain previous entries; exit 2 must be respected.

The complete run, bounded sensor log, optional native scene and pretty summary
are written with exclusive creation in a private `.run-stage-*` directory under
the output directory. Each file is flushed, synchronized and closed before
publication. Same-filesystem hard links publish complete files without replacing
entries that appeared since invalidation. Data files are published first and
`summary.json` last as the completion marker. A publication error attempts to
remove data files published by that invocation and reports cleanup errors.

The completion marker records the outcome, including an unsuccessful mission:

| Exit | Result |
|---|---|
| `0` | Complete run whose scenario acceptance passed. |
| `1` | Complete run whose scenario acceptance failed; its full evidence remains. |
| `2` | Input, simulation, serialization or I/O error. After successful invalidation, no new completion marker is published on an ordinary failure. |

The serialized bytes, scenario acceptance rules and driving algorithms retain
their preceding behavior. The shared publisher consumes its staging owner before
returning, so the native CLI's exit 1 cannot bypass ordinary staging cleanup.
Lower-level simulation APIs still return in-memory runs and do not write files.

## Independent verification

Subprocess fixtures exercise both actual CLIs against frozen authored scenario
bytes. They check input hashes, exit codes, prior artifact preservation on alias
rejection, unrelated link targets, directories, stale output removal and complete
failed-run recording. Published sensor logs are independently recomputed through
the sensor-only pipeline verifier. Native cases additionally protect scene inputs
and exercise scene-to-sceneless output reuse.

Deterministic library fault injection rejects a nonfinite last sensor record,
fails midway through scene serialization, and introduces destination collisions
at every publication position. These checks require staging cleanup, preservation
of colliding entries and rollback of already published files. A byte comparison
uses the preceding encoders for every artifact, including failed acceptance.

```sh
source scripts/env.sh
cargo test --locked -p rustdriving-sim --lib run_output
cargo test --locked -p rustdriving-sim --test run_output
cargo +1.95.0 test --release --locked --manifest-path integrations/rne/Cargo.toml --test run_output
bash scripts/check.sh
```

## Local results

The exact required command completed with exit 0: 1,030 Rust tests, 295 Python
tests and all 49 standard runs/replays. All 245 standard artifact files retain
their preceding SHA-256 identities. Native formatting, strict release Clippy
and all 77 native tests pass, for a combined 1,107 local Rust tests. The 21
added tests comprise four publisher fault/encoding checks, seven reference CLI
tests and ten native CLI tests. Ordinary workspace/native CI test steps discover
them; remote CI and other operating systems were not executed for this change.

The predecessor executables reproduce three input overwrites (two scenario
inputs and a native scene input) and two retained stale-success markers. The
fixed executables reject all three aliases with exit 2 and preserve input bytes;
both missing-input attempts remove the old managed result files. These fixtures
retain the original bytes, commands, output hashes and complete outcomes.

Three specified native physical executions retain the preceding dense stopping
scenario, seed and gates: one current plain run and a parent/current pair with
the same overhead scene. The five current plain artifacts reproduce the retained
parent recording; all six current scene artifacts reproduce the fresh parent
recording, including complete scene evidence and sensor-only replay. The 17
stored streams are verified by encoded and decoded hashes. Stopping acceptance,
physical clearance, road containment, localization and dense acquisition floors
remain satisfied. These are storage/compatibility checks on authored simulation;
the original mission failures remain open. [Result packet](../assets/run-publication-results.json).

## Limits

The output directory and its ancestors must remain stable during an invocation.
This does not serialize concurrent writers or provide an atomic multi-file
snapshot. Process termination or a cleanup error can leave a private staging
directory; future invocations skip existing private names instead of removing
unknown contents. File synchronization does not establish directory-entry
durability across power loss. The filesystem must support hard links; unsupported
publication returns exit 2.

This improves input preservation and evaluation reliability. It does not complete
the original eight obstacle missions, calibrate automotive uncertainty or add
real-vehicle validation. Comparative maturity remains about 29%, with the 30%
waypoint unmet.
