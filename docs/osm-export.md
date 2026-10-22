# OSM export preservation

`import-osm` previously wrote directly to final paths. Using the input as the
map, report or scenario destination destroyed the original geographic document.
Different spellings of one destination, including a symbolic-link parent, could
replace the map with a scenario while returning success. A report/scenario write
failure could also leave a partial export set.

The CLI now creates exports only at absent destinations. Existing files,
directories, hard links and symbolic links (including dangling links) cause
exit 2 and are preserved. This intentionally changes repeated invocations:
choose fresh output filenames or a fresh directory for another import. There
is no overwrite option. Existing map/scenario/report bytes are never invalidated
by an unsuccessful replacement attempt.

The report path still uses `map_path.with_extension("import.json")`, and successful
JSON encodings are unchanged. For example, `map.json` produces `map.import.json`;
`map.import.json` produces `map.import.import.json`. Optional scenario generation,
geographic projection, restrictions and validation retain their existing behavior.

## Publication boundary

After input parsing, import and optional scenario validation, all destination
parents are created and canonicalized. Equal resolved destinations are rejected
before staging. Every existing destination entry is rejected without following
it. A missing output may therefore use a symbolic-link parent when it resolves
to a distinct destination.

Each destination has an exclusively allocated private `.osm-stage-*` directory
in its own parent. Pretty JSON is fully serialized, flushed, synchronized and
closed before publication starts. Same-filesystem hard links publish complete
files without replacing an entry that appeared after validation. The order is
map, optional scenario, then the import report as the completion marker. An
ordinary publication error rolls back this invocation's earlier published files;
cleanup failures are included in the returned error. Private staging directories
are removed on ordinary success or error. Newly created empty parents may remain.

Keep destination directories and their ancestors stable during invocation.
This is not an atomic multi-file transaction or a power-loss durability guarantee.
Observers can see data before the final report, and termination/crashes may leave
staging or partial publication. The filesystem must support hard links; each
stage is placed beside its destination so distinct output files need not share
one filesystem. Other operating systems and filesystems were not tested here.

## Local verification

Twelve added Rust tests cover source aliases at every artifact, regular/linked/
dangling/directory entries, resolved destination aliases, different parents,
map-only output, repeated imports and partial failures. Direct fault injection
checks serialization failure after a written prefix, collisions at every
publication position across different directories, and a missing final staged
payload. Earlier published files are removed and foreign entries survive.

The retained parent executable independently reproduces four source destructions,
a successful map/scenario alias overwrite, a dangling-link write, an unrelated
report overwrite and two partial exports. The fixed executable rejects all nine
with exit 2 and preserves every preexisting entry and its bytes. Three fresh
positive controls retain every output byte and stdout byte, including map-only
export and the valid `map.import.json` filename.

`bash scripts/check.sh` passes 1,042 Rust and 295 Python tests. Its 49 standard
run/replay pairs retain all 245 preceding artifact hashes. The existing external
OSM CLI test imports the pinned geographic extract, physically reaches its goal
in the reference plant and recomputes the complete sensor log. This change is
confined to the reference binary's importer; the native application and shared
driving implementation are unchanged. Native physics was not requalified for
this export-only change. [Results and evidence bindings](../assets/osm-export-results.json).

These checks improve input preservation and reproducibility. They add no driving
availability or measured automotive accuracy. The original eight incomplete
missions remain open; comparative maturity stays about 29%, with 30% unmet.
