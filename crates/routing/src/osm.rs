//! Bounded OpenStreetMap Overpass JSON import. No network or simulator truth input.
use crate::{RoadEdge, RoadNetwork, RoadNetworkSpec, RoadNode};
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
    #[serde(other)]
    Other,
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
            OverpassElement::Other => unsupported += 1,
        }
    }
    let input_ways = ways.len();
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
            if usage[n] > 1 {
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
                edges.push(RoadEdge {
                    id: format!(
                        "osm-way-{id}-{part}-{}",
                        if reverse { "reverse" } else { "forward" }
                    ),
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
    let network = RoadNetworkSpec {
        nodes: graph_nodes
            .into_iter()
            .map(|(id, position)| RoadNode {
                id: format!("osm-node-{id}"),
                position,
            })
            .collect(),
        edges,
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
    };
    Ok((network, report))
}

#[cfg(test)]
mod tests;
