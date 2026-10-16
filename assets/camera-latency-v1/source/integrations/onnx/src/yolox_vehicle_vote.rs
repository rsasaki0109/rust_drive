//! Bounded reconciliation of conflicting cross-view rigid-road-vehicle labels.
use super::yolox_tiles::{self, Fixture, FixtureView, Origin};
use rustdriving_camera_detect::{Detection, intersection_over_union};
use serde::Serialize;
use sha2::{Digest, Sha256};

const CLASSES: [usize; 3] = [2, 5, 7];

#[derive(Serialize)]
pub struct Vote {
    pub view: usize,
    pub source_rank: usize,
    pub class_index: usize,
    pub confidence: f32,
}
#[derive(Serialize)]
pub struct Resolution {
    pub base_ranks: Vec<usize>,
    pub selected_base_rank: usize,
    pub class_index: usize,
    pub votes: Vec<Vote>,
    pub full_image_tie_break: bool,
}
#[derive(Serialize)]
pub struct Ambiguity {
    pub base_ranks: Vec<usize>,
    pub reason: &'static str,
}
#[derive(Serialize)]
pub struct Result {
    pub base_merged: yolox_tiles::Merged,
    pub detections: Vec<Detection>,
    pub detection_origins: Vec<Origin>,
    pub retained_base_ranks: Vec<usize>,
    pub resolutions: Vec<Resolution>,
    pub ambiguities: Vec<Ambiguity>,
    pub comparison_work: usize,
}

pub fn reconcile(fixture: &Fixture) -> std::result::Result<Result, String> {
    let base = yolox_tiles::merge(fixture)?;
    let mut work = base.merge.comparisons;
    let mut restored = Vec::new();
    for view in 0..fixture.views.len() {
        let single = Fixture {
            width: fixture.width,
            height: fixture.height,
            views: fixture
                .views
                .iter()
                .enumerate()
                .map(|(index, row)| FixtureView {
                    rectangle: row.rectangle,
                    detections: if index == view {
                        row.detections.clone()
                    } else {
                        Vec::new()
                    },
                })
                .collect(),
        };
        let result = yolox_tiles::merge(&single)?;
        work += result.merge.comparisons;
        restored.push(result);
    }
    let mut adjacent = vec![vec![]; base.detections.len()];
    for (i, a) in base.detections.iter().enumerate() {
        for (j, b) in base.detections[..i].iter().enumerate() {
            if CLASSES.contains(&a.class_index)
                && CLASSES.contains(&b.class_index)
                && a.class_index != b.class_index
                && base.detection_origins[i].view != base.detection_origins[j].view
            {
                work += 1;
                if intersection_over_union(a.bbox, b.bbox) > 0.45 {
                    adjacent[i].push(j);
                    adjacent[j].push(i);
                }
            }
        }
    }
    let mut seen = vec![false; base.detections.len()];
    let mut removed = vec![false; base.detections.len()];
    let mut resolutions = Vec::new();
    let mut ambiguities = Vec::new();
    for start in 0..base.detections.len() {
        if seen[start] {
            continue;
        }
        let mut stack = vec![start];
        let mut component = Vec::new();
        seen[start] = true;
        while let Some(index) = stack.pop() {
            component.push(index);
            for &neighbor in &adjacent[index] {
                if !seen[neighbor] {
                    seen[neighbor] = true;
                    stack.push(neighbor);
                }
            }
        }
        component.sort_unstable();
        if component.len() < 2 {
            continue;
        }
        if component.len() > CLASSES.len()
            || component.iter().any(|&i| {
                component
                    .iter()
                    .any(|&j| i != j && !adjacent[i].contains(&j))
            })
        {
            ambiguities.push(Ambiguity {
                base_ranks: component,
                reason: "not a clique of distinct classes and views",
            });
            continue;
        }
        let mut votes = Vec::new();
        for (view, proposals) in restored.iter().enumerate() {
            for (rank, candidate) in proposals.detections.iter().enumerate() {
                if !component
                    .iter()
                    .any(|&i| base.detections[i].class_index == candidate.class_index)
                {
                    continue;
                }
                let supported = component.iter().all(|&i| {
                    work += 1;
                    intersection_over_union(candidate.bbox, base.detections[i].bbox) > 0.45
                });
                if supported {
                    votes.push(Vote {
                        view,
                        source_rank: proposals.detection_origins[rank].rank,
                        class_index: candidate.class_index,
                        confidence: candidate.confidence,
                    });
                    break;
                }
            }
        }
        if votes.is_empty() {
            ambiguities.push(Ambiguity {
                base_ranks: component,
                reason: "no eligible per-view vote",
            });
            continue;
        }
        let count = |kind| votes.iter().filter(|v| v.class_index == kind).count();
        let maximum = CLASSES
            .iter()
            .map(|&kind| count(kind))
            .max()
            .expect("three fixed classes");
        let tied: Vec<_> = CLASSES
            .iter()
            .copied()
            .filter(|&kind| count(kind) == maximum)
            .collect();
        let full = votes
            .iter()
            .find(|v| v.view == 0 && tied.contains(&v.class_index));
        let full_image_tie_break = tied.len() > 1 && full.is_some();
        let winner = if let Some(full) = full {
            full.class_index
        } else {
            votes
                .iter()
                .filter(|v| tied.contains(&v.class_index))
                .max_by(|a, b| {
                    a.confidence
                        .total_cmp(&b.confidence)
                        .then(b.class_index.cmp(&a.class_index))
                })
                .expect("nonempty eligible votes")
                .class_index
        };
        let selected = *component
            .iter()
            .find(|&&i| base.detections[i].class_index == winner)
            .ok_or("winning vote has no original representative")?;
        for &index in &component {
            removed[index] = index != selected;
        }
        resolutions.push(Resolution {
            base_ranks: component,
            selected_base_rank: selected,
            class_index: winner,
            votes,
            full_image_tie_break,
        });
    }
    if work > 1_200_000 {
        return Err("bounded comparison work exceeded".into());
    }
    let retained_base_ranks: Vec<_> = (0..base.detections.len())
        .filter(|&i| !removed[i])
        .collect();
    Ok(Result {
        detections: retained_base_ranks
            .iter()
            .map(|&i| base.detections[i].clone())
            .collect(),
        detection_origins: retained_base_ranks
            .iter()
            .map(|&i| base.detection_origins[i].clone())
            .collect(),
        base_merged: base,
        retained_base_ranks,
        resolutions,
        ambiguities,
        comparison_work: work,
    })
}

pub fn compiled_sources() -> std::collections::BTreeMap<&'static str, String> {
    let mut sources = yolox_tiles::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/yolox_vehicle_vote.rs",
            include_bytes!("yolox_vehicle_vote.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-detect-vehicle-vote.rs",
            include_bytes!("bin/rustdriving-camera-detect-vehicle-vote.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-vehicle-vote.json",
            include_bytes!("../profile-vehicle-vote.json").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Fixture {
        Fixture {
            width: 1280,
            height: 720,
            views: yolox_tiles::rectangles(1280, 720)
                .unwrap()
                .into_iter()
                .map(|rectangle| FixtureView {
                    rectangle,
                    detections: vec![],
                })
                .collect(),
        }
    }
    fn add(f: &mut Fixture, view: usize, class_index: usize, confidence: f32, x: f32) {
        let offset = f.views[view].rectangle.x as f32;
        f.views[view].detections.push(Detection {
            class_index,
            confidence,
            bbox: [x - offset, 100., x + 100. - offset, 200.],
        });
    }
    #[test]
    fn full_context_breaks_a_tie_without_changing_scores_or_geometry() {
        let mut input = fixture();
        add(&mut input, 0, 2, 0.6, 400.);
        add(&mut input, 2, 5, 0.9, 400.);
        let result = reconcile(&input).unwrap();
        assert_eq!(result.detections.len(), 1);
        assert_eq!(result.detections[0].class_index, 2);
        assert_eq!(result.detections[0].confidence, 0.6);
        assert!(result.resolutions[0].full_image_tie_break);
        assert_eq!(result.detection_origins[0], Origin { view: 0, rank: 0 });
    }
    #[test]
    fn distinct_views_vote_once_and_non_clique_overlap_remains_ambiguous() {
        let mut input = fixture();
        add(&mut input, 0, 2, 0.6, 400.);
        add(&mut input, 1, 2, 0.7, 400.);
        add(&mut input, 2, 5, 0.9, 400.);
        let result = reconcile(&input).unwrap();
        assert_eq!(result.detections[0].class_index, 2);
        assert!(!result.resolutions[0].full_image_tie_break);
        let mut chain = fixture();
        add(&mut chain, 0, 2, 0.9, 400.);
        add(&mut chain, 1, 7, 0.8, 430.);
        add(&mut chain, 2, 5, 0.7, 460.);
        let result = reconcile(&chain).unwrap();
        assert_eq!(result.detections.len(), 3);
        assert_eq!(result.ambiguities.len(), 1);
        assert!(result.resolutions.is_empty());
    }
}
