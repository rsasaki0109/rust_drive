# OSM input and option contract

The importer previously checked `metadata.len()` before parsing an unrestricted
reader. A Linux stdin pipe reports zero bytes even when it supplies more than
16 MiB. A valid geographic document followed by whitespace could therefore
exceed the advertised limit and still produce a successful export. Single-value
options also silently used the last occurrence, and map-only requests silently
ignored explicit cruise speed and duration.

## Actual bytes and explicit requests

`import-osm` now reads at most **16 MiB + 1 bytes** before deserializing. The extra
byte distinguishes an allowed exact-size input from an oversized one. All bytes
count, including JSON whitespace. Exceeding 16 MiB returns exit 2 before import
or export preparation, regardless of file metadata. A read error is propagated
even after a complete JSON prefix; an incomplete read cannot become a valid map.
An endless reader is stopped after the extra byte. A stalled producer can still
block reading; byte bounds do not establish a wall-time bound.

Each single-value option may occur once, and `--local-route-geometry` may also
occur once. Repetition returns exit 2 before opening the input. **`--closed-edge`
remains repeatable** and retains all supplied closures. Routing-related options,
including explicit `--cruise-speed` and `--duration`, require `--scenario-output`;
their values are no longer silently ignored for map-only export. Scenario output
still defaults to 2 m/s cruise and 180 s duration. Projection, route validation,
export filenames and successful JSON encodings are unchanged.

Invalid requests do not prepare or publish exports. Existing destination
preservation and staged complete-file publication retain their separate
[export contract](osm-export.md).

## Verification

Seven added Rust tests cover actual reader consumption, a read error after a
complete JSON prefix, regular-file and Linux-pipe boundaries, all eleven
non-repeatable options/flags, ignored speed/duration requests and repeated
closures. The Linux pipe test observes the actual child stdin as a FIFO with
zero reported length. Exact-size inputs succeed; one extra byte causes exit 2
without output files or staging remnants. This is not a cross-platform claim.

Independent retained executables compare thirteen cases. The parent accepts an
oversized pipe, overrides a missing input with a repeated input and ignores two
scenario-only options. The fixed executable rejects all four with no export.
Nine other cases preserve their outcomes, including ordinary file rejection
above the cap. Eight positive controls retain **all nineteen export files and
all eight stdout streams** byte for byte: map-only, default/explicit scenario
settings, multiple closures and the file/pipe size boundaries. All source bytes
are preserved. Large padded inputs are retained as lossless XZ archives with
verified stored and decoded hashes rather than repeated raw whitespace copies.

`bash scripts/check.sh` passes **1,049 Rust tests and 295 Python tests**. All
49 standard runs and full sensor replays preserve the preceding 245 artifact
hashes. The external-map CLI test still reaches its reference physical goal and
recomputes every sensor tick. Only the reference importer binary and its tests
change; shared driving code and the native application retain their preceding
source hashes. Native physics was not requalified for this input-only change.
[Results and evidence bindings](../assets/osm-input-results.json).

These changes enforce the existing input contract and expose ambiguous requests.
They establish no new automotive accuracy or driving availability. All eight
original incomplete missions remain open. Comparative maturity remains about
29%, with the 30% waypoint unmet.
