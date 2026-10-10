//! Bounded OpenStreetMap Overpass JSON import. No network or simulator truth input.
use crate::{
    RoadEdge, RoadNetwork, RoadNetworkSpec, RoadNode, TurnRestriction, TurnRestrictionKind,
};
use rustdriving_core::Vec2;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, Deserialize)]
pub struct OverpassDocument {
    pub elements: Vec<OverpassElement>,
}
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "type")]
pub enum OverpassElement {
    #[serde(rename = "node")]
    Node {
        id: u64,
        lat: f64,
        lon: f64,
        #[serde(default)]
        tags: BTreeMap<String, String>,
    },
    #[serde(rename = "way")]
    Way {
        id: u64,
        nodes: Vec<u64>,
        #[serde(default)]
        tags: BTreeMap<String, String>,
    },
    #[serde(rename = "relation")]
    Relation {
        #[serde(default)]
        id: u64,
        #[serde(default)]
        members: Vec<RelationMember>,
        #[serde(default)]
        tags: BTreeMap<String, String>,
    },
    #[serde(other)]
    Other,
}
/// Raw OSM member kinds/roles remain strings so unsupported relevant relations fail explicitly.
#[derive(Clone, Debug, Deserialize)]
pub struct RelationMember {
    #[serde(default, rename = "type")]
    pub member_type: String,
    #[serde(default, rename = "ref", deserialize_with = "deserialize_member_ref")]
    pub reference: u64,
    #[serde(default)]
    pub role: String,
}
// XML-derived Overpass archives can represent member refs as decimal strings.
// Accept that representation without weakening integral identity or overflow checks.
fn deserialize_member_ref<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<u64, D::Error> {
    struct MemberRefVisitor;
    impl serde::de::Visitor<'_> for MemberRefVisitor {
        type Value = u64;
        fn expecting(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            formatter.write_str("a positive u64 or decimal-string OSM member reference")
        }
        fn visit_u64<E: serde::de::Error>(self, value: u64) -> Result<u64, E> {
            if value == 0 {
                return Err(E::custom("OSM member reference must be positive"));
            }
            Ok(value)
        }
        fn visit_str<E: serde::de::Error>(self, value: &str) -> Result<u64, E> {
            if value.is_empty() || !value.bytes().all(|byte| byte.is_ascii_digit()) {
                return Err(E::custom(
                    "OSM member reference string must contain decimal digits only",
                ));
            }
            let parsed = value.parse::<u64>().map_err(E::custom)?;
            self.visit_u64(parsed)
        }
    }
    deserializer.deserialize_any(MemberRefVisitor)
}

#[derive(Clone, Copy, Debug)]
pub struct ImportOptions {
    pub origin_lat: f64,
    pub origin_lon: f64,
    /// Explicit simulation corridor calibration when OSM has no metric width.
    pub default_half_width: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct ImportReport {
    pub input_nodes: usize,
    pub input_ways: usize,
    pub imported_ways: usize,
    pub filtered_ways: usize,
    pub clipped_short_ways: usize,
    pub node_restricted_ways: usize,
    pub unsupported_elements: usize,
    pub default_width_ways: usize,
    pub graph_nodes: usize,
    pub directed_edges: usize,
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub default_half_width: f64,
    #[serde(skip_serializing_if = "is_zero")]
    pub input_turn_relations: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub imported_turn_restrictions: usize,
    #[serde(skip_serializing_if = "is_zero")]
    pub ignored_turn_relations: usize,
}

fn is_zero(value: &usize) -> bool {
    *value == 0
}

fn ecef(lat: f64, lon: f64) -> Result<[f64; 3], String> {
    if !lat.is_finite()
        || !lon.is_finite()
        || !(-85.0..=85.0).contains(&lat)
        || !(-180.0..=180.0).contains(&lon)
    {
        return Err("OSM coordinates require finite latitude +/-85 and longitude +/-180".into());
    }
    let (lat, lon) = (lat.to_radians(), lon.to_radians());
    let e2 = 6.694_379_990_141_316_5e-3;
    let n = 6_378_137.0 / (1.0 - e2 * lat.sin().powi(2)).sqrt();
    Ok([
        n * lat.cos() * lon.cos(),
        n * lat.cos() * lon.sin(),
        n * (1.0 - e2) * lat.sin(),
    ])
}

/// WGS84 zero-height geodetic point projected onto the origin's local ENU plane.
/// Restricts the planar distance to 2 km; elevation/geoid corrections are absent.
pub fn project_enu(lat: f64, lon: f64, origin_lat: f64, origin_lon: f64) -> Result<Vec2, String> {
    let p = ecef(lat, lon)?;
    let o = ecef(origin_lat, origin_lon)?;
    let d = [p[0] - o[0], p[1] - o[1], p[2] - o[2]];
    let (a, b) = (origin_lat.to_radians(), origin_lon.to_radians());
    let result = Vec2::new(
        -b.sin() * d[0] + b.cos() * d[1],
        -a.sin() * b.cos() * d[0] - a.sin() * b.sin() * d[1] + a.cos() * d[2],
    );
    // Chord distance also excludes antipodal points projecting near the origin.
    if d.iter().map(|v| v * v).sum::<f64>().sqrt() > 2000.0 || !result.finite() {
        return Err("OSM extract exceeds the 2 km local projection radius".into());
    }
    Ok(result)
}

fn motor_road(tags: &BTreeMap<String, String>) -> bool {
    let road = tags.get("highway").is_some_and(|h| {
        matches!(
            h.as_str(),
            "motorway"
                | "motorway_link"
                | "trunk"
                | "trunk_link"
                | "primary"
                | "primary_link"
                | "secondary"
                | "secondary_link"
                | "tertiary"
                | "tertiary_link"
                | "unclassified"
                | "residential"
                | "living_street"
                | "service"
        )
    });
    if !road || tags.iter().any(|(k, _)| k.ends_with(":conditional")) {
        return false;
    }
    // More specific motorcar restrictions override general access tags.
    motor_access(tags)
}
fn motor_access(tags: &BTreeMap<String, String>) -> bool {
    let access = ["motorcar", "motor_vehicle", "vehicle", "access"]
        .iter()
        .find_map(|key| tags.get(*key));
    access.is_none_or(|a| matches!(a.as_str(), "yes" | "permissive" | "designated"))
}

#[derive(Clone, Debug)]
struct ImportedRestriction {
    kind: TurnRestrictionKind,
    reverse_only: bool,
}

/// Passenger motorcar precedence, without evaluating dates, access credentials or conditions.
fn restriction_kind(
    id: u64,
    tags: &BTreeMap<String, String>,
) -> Result<Option<ImportedRestriction>, String> {
    let other_modes = [
        "bicycle",
        "foot",
        "bus",
        "psv",
        "taxi",
        "hgv",
        "motorcycle",
        "moped",
        "emergency",
        "agricultural",
    ];
    for key in tags.keys().filter(|key| key.starts_with("restriction:")) {
        let suffix = key.trim_start_matches("restriction:");
        let mode = suffix.strip_suffix(":conditional").unwrap_or(suffix);
        if suffix != "conditional"
            && !matches!(mode, "motorcar" | "motor_vehicle" | "vehicle")
            && !other_modes.contains(&mode)
        {
            return Err(format!(
                "restriction {id} has unsupported mode qualifier {key}"
            ));
        }
    }
    if let Some(exceptions) = tags.get("except") {
        let exceptions: Vec<_> = exceptions.split(';').map(str::trim).collect();
        if exceptions.iter().any(|mode| {
            !matches!(
                *mode,
                "motorcar" | "motor_vehicle" | "vehicle" | "bicycle" | "foot"
            )
        }) {
            return Err(format!(
                "restriction {id} has unsupported or empty except mode"
            ));
        }
        if exceptions
            .iter()
            .any(|mode| matches!(*mode, "motorcar" | "motor_vehicle" | "vehicle"))
        {
            return Ok(None);
        }
    }
    // The first vehicle-specific layer overrides every less specific layer.
    // A condition at the selected layer cannot be decided by this offline importer.
    let mut selected = None;
    for key in [
        "restriction:motorcar",
        "restriction:motor_vehicle",
        "restriction:vehicle",
        "restriction",
    ] {
        if tags.contains_key(&format!("{key}:conditional")) {
            return Err(format!(
                "restriction {id} has unsupported applicable conditional"
            ));
        }
        if let Some(value) = tags.get(key) {
            selected = Some(value.as_str());
            break;
        }
    }
    let Some(value) = selected else {
        // Known mode-specific restrictions do not constrain a generic passenger car.
        let keys: Vec<_> = tags
            .keys()
            .filter(|k| k.starts_with("restriction:"))
            .collect();
        if !keys.is_empty()
            && keys.iter().all(|key| {
                let suffix = key.trim_start_matches("restriction:");
                let mode = suffix.strip_suffix(":conditional").unwrap_or(suffix);
                other_modes.contains(&mode)
            })
        {
            return Ok(None);
        }
        return Err(format!(
            "restriction {id} lacks a supported motorcar restriction tag"
        ));
    };
    let (kind, reverse_only) = match value {
        "no_left_turn" | "no_right_turn" | "no_straight_on" => (TurnRestrictionKind::No, false),
        "no_u_turn" => (TurnRestrictionKind::No, true),
        "only_left_turn" | "only_right_turn" | "only_straight_on" => {
            (TurnRestrictionKind::Only, false)
        }
        // An explicit higher-priority mode exemption overrides a generic restriction.
        "none" => return Ok(None),
        _ => return Err(format!("restriction {id} has unsupported value {value}")),
    };
    Ok(Some(ImportedRestriction { kind, reverse_only }))
}

fn relation_topology(id: u64, members: &[RelationMember]) -> Result<(u64, u64, u64), String> {
    if members.len() != 3 {
        return Err(format!(
            "restriction {id} requires exactly one from way, via node and to way"
        ));
    }
    let mut from = None;
    let mut via = None;
    let mut to = None;
    for member in members {
        if member.reference == 0 {
            return Err(format!("restriction {id} has zero member reference"));
        }
        let slot = match (member.role.as_str(), member.member_type.as_str()) {
            ("from", "way") => &mut from,
            ("via", "node") => &mut via,
            ("to", "way") => &mut to,
            _ => {
                return Err(format!(
                    "restriction {id} has unsupported member kind or role"
                ));
            }
        };
        if slot.replace(member.reference).is_some() {
            return Err(format!("restriction {id} repeats a member role"));
        }
    }
    match (from, via, to) {
        (Some(from), Some(via), Some(to)) => Ok((from, via, to)),
        _ => Err(format!("restriction {id} is missing from, via or to")),
    }
}

type RetainedRelation = (u64, u64, u64, u64, ImportedRestriction);

fn expand_restrictions(
    relations: &[RetainedRelation],
    edges: &[RoadEdge],
    edge_way_parts: &BTreeMap<String, (u64, usize, bool)>,
) -> Result<Vec<TurnRestriction>, String> {
    let mut result = Vec::new();
    for (id, from_way, via, to_way, rule) in relations {
        let via_node = format!("osm-node-{via}");
        let incoming: Vec<_> = edges
            .iter()
            .filter(|edge| edge_way_parts[&edge.id].0 == *from_way && edge.to == via_node)
            .collect();
        let outgoing: Vec<_> = edges
            .iter()
            .filter(|edge| edge_way_parts[&edge.id].0 == *to_way && edge.from == via_node)
            .collect();
        if from_way == to_way {
            if !rule.reverse_only || incoming.is_empty() {
                return Err(format!(
                    "restriction {id} has unsupported or directionally dangling same-way turn"
                ));
            }
            // A bidirectional way split at via can have two incoming directions.
            // Prohibit each edge's actual reverse, never its legal continuation.
            for from in incoming {
                let (_, part, reverse) = edge_way_parts[&from.id];
                let matches: Vec<_> = outgoing
                    .iter()
                    .copied()
                    .filter(|to| {
                        edge_way_parts[&to.id] == (*to_way, part, !reverse) && to.to == from.from
                    })
                    .collect();
                if matches.len() != 1 {
                    return Err(format!(
                        "restriction {id} has missing or ambiguous reverse edge"
                    ));
                }
                result.push(TurnRestriction {
                    id: format!("osm-restriction-{id}-{}", from.id),
                    from_edge: from.id.clone(),
                    via_node: via_node.clone(),
                    to_edge: matches[0].id.clone(),
                    kind: rule.kind,
                });
            }
        } else {
            if incoming.len() != 1 || outgoing.len() != 1 {
                return Err(format!(
                    "restriction {id} has missing or ambiguous directed from/to edge"
                ));
            }
            result.push(TurnRestriction {
                id: format!("osm-restriction-{id}"),
                from_edge: incoming[0].id.clone(),
                via_node,
                to_edge: outgoing[0].id.clone(),
                kind: rule.kind,
            });
        }
        if result.len() > 10_000 {
            return Err("OSM import exceeds 10000 directed turn restrictions".into());
        }
    }
    result.sort_by(|a, b| a.id.cmp(&b.id));
    Ok(result)
}

pub fn import(
    document: OverpassDocument,
    options: ImportOptions,
) -> Result<(RoadNetworkSpec, ImportReport), String> {
    ecef(options.origin_lat, options.origin_lon)?;
    if !options.default_half_width.is_finite()
        || !(1.2..=12.0).contains(&options.default_half_width)
    {
        return Err(
            "default-half-width must be 1.2..12 meters of explicit simulation calibration".into(),
        );
    }
    if document.elements.is_empty() || document.elements.len() > 50_000 {
        return Err("OSM extract requires 1..50000 elements".into());
    }
    let mut nodes = BTreeMap::new();
    let mut restricted_nodes = BTreeSet::new();
    let mut ways = BTreeMap::new();
    let mut unsupported = 0;
    let mut relations = BTreeMap::new();
    for element in document.elements {
        match element {
            OverpassElement::Node { id, lat, lon, tags } => {
                if !motor_access(&tags)
                    || tags.contains_key("barrier")
                    || tags.keys().any(|key| key.ends_with(":conditional"))
                {
                    restricted_nodes.insert(id);
                }
                if id == 0
                    || nodes
                        .insert(
                            id,
                            project_enu(lat, lon, options.origin_lat, options.origin_lon)?,
                        )
                        .is_some()
                {
                    return Err("zero or duplicate OSM node ID".into());
                }
            }
            OverpassElement::Way { id, nodes, tags } => {
                if id == 0 || ways.insert(id, (nodes, tags)).is_some() {
                    return Err("zero or duplicate OSM way ID".into());
                }
            }
            OverpassElement::Relation { id, members, tags } => {
                if tags.get("type").is_some_and(|kind| kind == "restriction") {
                    if id == 0 || relations.insert(id, (members, tags)).is_some() {
                        return Err("zero or duplicate OSM restriction relation ID".into());
                    }
                    if relations.len() > 10_000 {
                        return Err("OSM import exceeds 10000 restriction relations".into());
                    }
                } else if tags
                    .get("type")
                    .is_some_and(|kind| kind.starts_with("restriction:"))
                    || tags
                        .keys()
                        .any(|key| key == "restriction" || key.starts_with("restriction:"))
                {
                    return Err(format!("relation {id} has unsupported restriction type"));
                } else {
                    unsupported += 1;
                }
            }
            OverpassElement::Other => unsupported += 1,
        }
    }
    let input_ways = ways.len();
    let input_way_ids: BTreeSet<_> = ways.keys().copied().collect();
    ways.retain(|_, (_, tags)| motor_road(tags));
    let mut node_restricted_ways = 0;
    ways.retain(|_, (refs, _)| {
        if refs.iter().any(|node| restricted_nodes.contains(node)) {
            node_restricted_ways += 1;
            false
        } else {
            true
        }
    });
    let mut clipped_short_ways = 0;
    ways.retain(|_, (refs, _)| {
        if refs.len() == 1 {
            clipped_short_ways += 1;
            false
        } else {
            true
        }
    });
    if ways.is_empty() {
        return Err("OSM extract has no supported motor roads".into());
    }
    if ways
        .values()
        .try_fold(0_usize, |total, (refs, _)| total.checked_add(refs.len()))
        .is_none_or(|total| total > 200_000)
    {
        return Err("OSM import exceeds 200000 retained way-node references".into());
    }
    let mut ignored_turn_relations = 0;
    let mut retained_relations = Vec::new();
    let mut explicit_vias = BTreeSet::new();
    for (id, (members, tags)) in &relations {
        // Applicability is resolved before topology: explicitly nonmotor restrictions can
        // safely be ignored without requiring their unretained bicycle/foot geometry.
        let Some(kind) = restriction_kind(*id, tags)? else {
            ignored_turn_relations += 1;
            continue;
        };
        let (from, via, to) = relation_topology(*id, members)?;
        if !input_way_ids.contains(&from) || !input_way_ids.contains(&to) {
            return Err(format!("restriction {id} references a missing from/to way"));
        }
        if !nodes.contains_key(&via) {
            return Err(format!(
                "restriction {id} references missing via node {via}"
            ));
        }
        // A missing incoming road or an absent prohibited exit cannot constrain a
        // retained transition. An only-turn to a filtered road must NOT disappear:
        // it would prohibit every remaining exit, which this importer rejects.
        if !ways.contains_key(&from)
            || (!ways.contains_key(&to) && kind.kind == TurnRestrictionKind::No)
        {
            ignored_turn_relations += 1;
            continue;
        }
        if !ways.contains_key(&to) {
            return Err(format!(
                "restriction {id} only-turn points to a filtered road"
            ));
        }
        for way_id in [from, to] {
            if !ways[&way_id].0.contains(&via) {
                return Err(format!(
                    "restriction {id} via node does not belong to way {way_id}"
                ));
            }
        }
        explicit_vias.insert(via);
        retained_relations.push((*id, from, via, to, kind));
    }
    let mut usage: BTreeMap<u64, usize> = BTreeMap::new();
    for (id, (refs, _)) in &ways {
        if refs.len() < 2 || refs.len() > 4096 {
            return Err(format!("way {id} requires 2..4096 nodes"));
        }
        for n in refs.iter().copied().collect::<BTreeSet<_>>() {
            *usage.entry(n).or_default() += 1;
        }
        for pair in refs.windows(2) {
            let a = nodes
                .get(&pair[0])
                .ok_or_else(|| format!("way {id} references missing node {}", pair[0]))?;
            let b = nodes
                .get(&pair[1])
                .ok_or_else(|| format!("way {id} references missing node {}", pair[1]))?;
            if a.distance(*b) < 0.01 {
                return Err(format!("way {id} has coincident consecutive coordinates"));
            }
        }
        let unique = refs.iter().copied().collect::<BTreeSet<_>>().len();
        if unique + usize::from(refs.first() == refs.last()) != refs.len() {
            return Err(format!("way {id} repeats interior nodes"));
        }
    }
    let mut graph_nodes = BTreeMap::new();
    let mut edges = Vec::new();
    let mut default_width_ways = 0;
    let mut edge_way_parts = BTreeMap::new();
    for (id, (refs, tags)) in &ways {
        let half_width = if let Some(width) = tags.get("width") {
            let width = width
                .trim()
                .strip_suffix(" m")
                .unwrap_or(width.trim())
                .parse::<f64>()
                .map_err(|_| format!("way {id} has unsupported width; metric scalar required"))?
                / 2.0;
            if !width.is_finite() || !(1.2..=12.0).contains(&width) {
                return Err(format!("way {id} width out of supported range"));
            }
            width
        } else {
            default_width_ways += 1;
            options.default_half_width
        };
        let direction = match tags.get("oneway").map(String::as_str) {
            Some("yes" | "1" | "true") => 1,
            Some("-1" | "reverse") => -1,
            Some("no" | "0" | "false") => 0,
            None if tags.get("junction").is_some_and(|s| s == "roundabout") => 1,
            None if tags
                .get("highway")
                .is_some_and(|s| matches!(s.as_str(), "motorway" | "motorway_link")) =>
            {
                1
            }
            None => 0,
            Some(_) => return Err(format!("way {id} has unsupported oneway value")),
        };
        let mut cuts: BTreeSet<usize> = [0, refs.len() - 1].into_iter().collect();
        for (index, n) in refs.iter().enumerate() {
            if usage[n] > 1 || explicit_vias.contains(n) {
                cuts.insert(index);
            }
        }
        if refs.first() == refs.last() && cuts.len() == 2 {
            cuts.insert(refs.len() / 2);
        }
        let cuts: Vec<_> = cuts.into_iter().collect();
        for (part, pair) in cuts.windows(2).enumerate() {
            let from = refs[pair[0]];
            let to = refs[pair[1]];
            if from == to {
                return Err(format!("way {id} has unsplit self-loop"));
            }
            for n in [from, to] {
                graph_nodes.insert(n, nodes[&n]);
            }
            let points: Vec<_> = refs[pair[0]..=pair[1]].iter().map(|n| nodes[n]).collect();
            for reverse in [false, true] {
                if (reverse && direction == 1) || (!reverse && direction == -1) {
                    continue;
                }
                let mut points = points.clone();
                if reverse {
                    points.reverse();
                }
                let edge_id = format!(
                    "osm-way-{id}-{part}-{}",
                    if reverse { "reverse" } else { "forward" }
                );
                edge_way_parts.insert(edge_id.clone(), (*id, part, reverse));
                edges.push(RoadEdge {
                    id: edge_id,
                    from: format!("osm-node-{}", if reverse { to } else { from }),
                    to: format!("osm-node-{}", if reverse { from } else { to }),
                    points,
                    half_width,
                });
                if edges.len() > 10_000 {
                    return Err("OSM import exceeds 10000 directed edges".into());
                }
            }
        }
    }
    let turn_restrictions = expand_restrictions(&retained_relations, &edges, &edge_way_parts)?;
    let network = RoadNetworkSpec {
        nodes: graph_nodes
            .into_iter()
            .map(|(id, position)| RoadNode {
                id: format!("osm-node-{id}"),
                position,
            })
            .collect(),
        edges,
        turn_restrictions,
    };
    RoadNetwork::new(network.clone())?;
    let report = ImportReport {
        input_nodes: nodes.len(),
        input_ways,
        imported_ways: ways.len(),
        filtered_ways: input_ways - ways.len(),
        clipped_short_ways,
        node_restricted_ways,
        unsupported_elements: unsupported,
        default_width_ways,
        graph_nodes: network.nodes.len(),
        directed_edges: network.edges.len(),
        origin_lat: options.origin_lat,
        origin_lon: options.origin_lon,
        default_half_width: options.default_half_width,
        input_turn_relations: relations.len(),
        imported_turn_restrictions: network.turn_restrictions.len(),
        ignored_turn_relations,
    };
    Ok((network, report))
}

#[cfg(test)]
mod tests;
