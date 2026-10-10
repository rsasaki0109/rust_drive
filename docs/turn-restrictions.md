# Legal turns in known road graphs

Routing now understands unconditional directed junction transitions. A `no`
rule prohibits one outgoing edge after a particular incoming edge; an `only`
rule permits only the named outgoing edge. Edge closures still apply and never
override either rule. A route with no legal continuation fails explicitly.

```json
"turn_restrictions": [
  {
    "id": "no-main-from-approach",
    "from_edge": "approach",
    "via_node": "fork",
    "to_edge": "main",
    "kind": "no"
  }
]
```

This optional network field is omitted when empty. IDs must be unique, the
named edges must meet at the via node, and duplicate/contradictory transitions
or conflicting `only` rules are rejected. Networks accept at most 10,000 rules.
The route request starts at a node with no specified arrival edge; its first
edge therefore has no incoming transition. Routing from arbitrary mid-edge
vehicle positions remains unsupported.

A node-only shortest-path search cannot handle these rules: the shortest
arrival at a junction may prohibit the exit, while a longer arrival permits it.
The restricted search retains `(node, incoming edge)` states, with at most
one state per directed edge plus the initial state. Deterministic Dijkstra
selects a legal path; it may revisit a node through a different arrival edge.
Maps without restrictions retain the preceding search implementation and
serialized output.

## Bounded OpenStreetMap import

Overpass JSON relations now support motorcar-applicable, unconditional
node-via `no_left_turn`, `no_right_turn`, `no_straight_on`, `no_u_turn`,
`only_left_turn`, `only_right_turn` and `only_straight_on`. Directed edge
orientation, including `oneway=-1`, determines which transition is constrained.
Same-way U-turn prohibitions split the via node and constrain actual reverse
edges, preserving through traffic. Distinct from/to ways require unambiguous
directed incidence at the via node; turn names are map declarations, not a
geometric angle classifier.

Specific restriction tags take precedence in this order: `motorcar`,
`motor_vehicle`, `vehicle`, then the generic restriction. Relevant conditional
rules, via-way sequences, unknown qualifiers, ambiguous direction, missing
references and unsupported exception semantics are rejected. A relevant rule
is never silently discarded because the importer cannot interpret it.
Explicit nonmotor-only relations and proven filtered-road cases may be ignored;
an `only` rule pointing to a filtered road fails. Other relation types retain
their prior unsupported-element accounting. Positive integer member references
and decimal strings from the archived XML conversion are accepted; malformed
values fail. The existing attributed geographic extract remains unchanged.

## Actual driving and independent checks

[Results](../assets/turn-restriction-results.json) cover **24 real closed-loop
episodes**: four cases, reference/native dynamic plants and seeds 1/7/42.
They include prohibited direct turns, a detour-only approach, a shorter invalid
arrival at the final junction, and a complete public CLI OSM import/run path.
The OSM-format fixture is authored test geometry; it is not new surveyed road
data or independently verified Japanese road law.

All 17,158 sensor-only replay ticks are recomputed. Restricted maps are included
in the log header, so replay repeats map search and verifies its resolved route.
Removing a rule, supplying an invalid via node or corrupting the expected path
is rejected (**24 negative replay checks**). Both plants also reject closing the
only allowed turn without taking a prohibited fallback.

The checker independently validates edge transitions and closures, concatenated
geometry, actual recorded circular corridor containment, 20 Hz relative linear
segment clearance and stopped goal position. The three obstacle fixtures keep
the preceding **0.5 m** clearance floor; minimum measured segment clearances are
0.594587 m reference and 1.400077 m native. The imported fixture has no obstacles
and makes no obstacle-clearance measurement. These are authored planar checks,
not native contact response or a guarantee between unrecorded physics substeps.

Routing tests also compare every result with an independent exhaustive path
enumerator for 128 closure subsets and 12 start/goal pairs. The required
workspace checks pass 331 tests and 49 legacy scenario/replay pairs, retaining
all 245 prior output files byte-for-byte. A checker failure caused by an omitted
default field in the imported fixture is retained separately in the results.
No external dependency, toolchain or RNE revision changed.

```sh
bash scripts/setup-rne.sh
source scripts/env.sh
cargo build --workspace --release --locked
cargo +1.95.0 build --release --locked --manifest-path integrations/rne/Cargo.toml
python3 scripts/check-turn-restrictions.py --backend both --compact \
  --output artifacts/turn-restrictions
```

The new search integrates with the existing stopped closure handover policy.
General lane topology, conditional traffic laws, arbitrary mid-edge routing,
intersection negotiation and moving handover remain outside this feature.
Restricted-map replay currently uses the navigation controller and retains its
existing exclusion of simultaneously configured mapped stop controls. The
separate fixed-route city Hero uses timestamped mapped signals rather than
claiming general traffic-rule negotiation.
