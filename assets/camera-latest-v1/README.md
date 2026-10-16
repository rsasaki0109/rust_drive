# First latest-ready scheduling comparison

The optional latest-ready consumer preserves the original Nano detector and
per-frame clock/guard while discarding queued images before inference. It
records every discarded acquisition and preserves missing/failure barriers.
This first real comparison fails to improve availability: FIFO supplies 0/8
and 1/8 available results, latest-ready 0/8 and 0/8. Zero late dispatched
latest images is not complete source coverage or continuous availability.

The packet preserves 88 actual local model forwards, all four actual timing
reports, eight current serial references, exact independent guard replays,
25 invalid-input controls, two output protections and model-free checks.
`initial-checker.py` binds the unchanged auditor that observed the first run.
The previous PR's decoded GitHub job log retains its separate 244 forwards,
failed Nano deadlines and 0/8 availability on that host.

Only new source files are copied here. `source-sha256.json` binds those copies
and the existing 110-file source archive in `assets/camera-nano-v1/source/`.
`before.json` binds all 73 previous indexes and 5,618 existing evidence entries;
all bytes remain unchanged. The 245 root replay outputs retain their hashes.
Reports are deterministically gzipped, with original byte hashes in
`raw-files.json`. Weights, original camera images and executables are excluded.
The tiny authored 8×6 input controls are included. The omitted malformed model
fixture is indexed separately. Apache code and TUM data notices are retained.

Run `python -B assets/camera-latest-v1/verify.py` from the repository root to
recompute source bindings, all selection choices, clock arithmetic, independent
postprocessing and guard outcomes without inference or original images.
See [commands and limits](../../docs/camera-latest.md).

This is a previously viewed indoor experiment, with no new detection accuracy
or driving integration. The subjective maturity estimate remains about 20%
versus mature driving OSS, with 30% unmet.
