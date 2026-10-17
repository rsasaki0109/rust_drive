//! Preserve coarse hypotheses and require distinct-crop support for additions.
use super::{camera_supported_coarse as coarse, yolox_tiles as grid, yolox_vehicle_consistency};
use rustdriving_camera_detect::{Detection, intersection_over_union};
use serde::Serialize;

const MAX_WORK: usize = 27_000_000;
#[derive(Clone)]
struct Candidate {
    detection: Detection,
    origin: grid::Origin,
}
#[derive(Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Reason {
    CoarseOverlap,
    VehicleAmbiguity,
    InsufficientSupport,
    SuppressedAddition,
    OutputCap,
    Added,
}
#[derive(Serialize)]
pub struct Decision {
    pub origin: grid::Origin,
    pub witnesses: Vec<grid::Origin>,
    pub conflicting_vehicle_label: bool,
    pub coarse_rank: Option<usize>,
    pub added_rank: Option<usize>,
    pub reason: Reason,
}
#[derive(Default, Serialize)]
pub struct Stats {
    pub supplemental_input_boxes: usize,
    pub interior_edge_rejected: usize,
    pub proposals: usize,
    pub added: usize,
    pub iou_comparisons: usize,
}
#[derive(Serialize)]
pub struct SupportedResult {
    pub naive: yolox_vehicle_consistency::ConsistencyResult,
    pub coarse: coarse::yolox_vehicle_consistency::ConsistencyResult,
    pub detections: Vec<Detection>,
    pub detection_origins: Vec<grid::Origin>,
    pub decisions: Vec<Decision>,
    pub stats: Stats,
}

fn overlaps(a: &Detection, b: &Detection, work: &mut usize) -> Result<bool, String> {
    if *work >= MAX_WORK {
        return Err("supplemental IoU comparison budget exceeded".into());
    }
    *work += 1;
    Ok(intersection_over_union(a.bbox, b.bbox) > 0.45)
}

pub fn reconcile(input: &grid::Fixture) -> Result<SupportedResult, String> {
    // This validates every original and supplemental view before using any box.
    let naive = yolox_vehicle_consistency::reconcile(input)?;
    let original = coarse::yolox_tiles::rectangles(input.width, input.height)?;
    let prefix = original.len();
    let coarse_fixture = coarse::yolox_tiles::Fixture {
        width: input.width,
        height: input.height,
        views: original
            .into_iter()
            .zip(&input.views)
            .map(|(rectangle, row)| coarse::yolox_tiles::FixtureView {
                rectangle,
                detections: row.detections.clone(),
            })
            .collect(),
    };
    let coarse = coarse::yolox_vehicle_consistency::reconcile(&coarse_fixture)?;
    let mut stats = Stats::default();
    let mut proposals = Vec::new();
    for (view, row) in input.views.iter().enumerate().skip(prefix) {
        let r = row.rectangle;
        for (rank, detection) in row.detections.iter().enumerate() {
            stats.supplemental_input_boxes += 1;
            let [l, t, rr, b] = detection.bbox;
            if (r.x > 0 && l <= 1.)
                || (r.y > 0 && t <= 1.)
                || (r.x + r.width < input.width && rr >= r.width as f32 - 1.)
                || (r.y + r.height < input.height && b >= r.height as f32 - 1.)
            {
                stats.interior_edge_rejected += 1;
                continue;
            }
            let mut detection = detection.clone();
            detection.bbox = [
                l + r.x as f32,
                t + r.y as f32,
                rr + r.x as f32,
                b + r.y as f32,
            ];
            proposals.push(Candidate {
                detection,
                origin: grid::Origin { view, rank },
            });
        }
    }
    proposals.sort_by(|a, b| {
        b.detection
            .confidence
            .total_cmp(&a.detection.confidence)
            .then(a.detection.class_index.cmp(&b.detection.class_index))
            .then(a.detection.bbox[0].total_cmp(&b.detection.bbox[0]))
            .then(a.origin.view.cmp(&b.origin.view))
            .then(a.origin.rank.cmp(&b.origin.rank))
    });
    stats.proposals = proposals.len();
    let mut additions: Vec<Detection> = Vec::new();
    let mut added_origins = Vec::new();
    let mut decisions = Vec::new();
    for candidate in &proposals {
        let mut decision = Decision {
            origin: candidate.origin.clone(),
            witnesses: vec![],
            conflicting_vehicle_label: false,
            coarse_rank: None,
            added_rank: None,
            reason: Reason::InsufficientSupport,
        };
        for (rank, existing) in coarse.detections.iter().enumerate() {
            if overlaps(&candidate.detection, existing, &mut stats.iou_comparisons)? {
                decision.coarse_rank = Some(rank);
                break;
            }
        }
        if decision.coarse_rank.is_some() {
            decision.reason = Reason::CoarseOverlap;
            decisions.push(decision);
            continue;
        }
        let mut best: Vec<Option<&Candidate>> = vec![None; input.views.len()];
        for other in &proposals {
            let same = candidate.detection.class_index == other.detection.class_index;
            let vehicle = [2, 5, 7].contains(&candidate.detection.class_index)
                && [2, 5, 7].contains(&other.detection.class_index);
            if !(same || vehicle)
                || !overlaps(
                    &candidate.detection,
                    &other.detection,
                    &mut stats.iou_comparisons,
                )?
            {
                continue;
            }
            if !same {
                decision.conflicting_vehicle_label = true;
                continue;
            }
            let slot = &mut best[other.origin.view];
            if slot.is_none_or(|old| {
                other.detection.confidence > old.detection.confidence
                    || (other.detection.confidence == old.detection.confidence
                        && other.origin.rank < old.origin.rank)
            }) {
                *slot = Some(other);
            }
        }
        decision.witnesses = best
            .into_iter()
            .flatten()
            .map(|p| p.origin.clone())
            .collect();
        if decision.conflicting_vehicle_label {
            decision.reason = Reason::VehicleAmbiguity;
        } else if decision.witnesses.len() >= 2 {
            let mut suppressed = false;
            for other in &additions {
                if other.class_index == candidate.detection.class_index
                    && overlaps(&candidate.detection, other, &mut stats.iou_comparisons)?
                {
                    suppressed = true;
                    break;
                }
            }
            if suppressed {
                decision.reason = Reason::SuppressedAddition;
            } else if coarse.detections.len() + additions.len() >= 300 {
                decision.reason = Reason::OutputCap;
            } else {
                decision.reason = Reason::Added;
                decision.added_rank = Some(additions.len());
                additions.push(candidate.detection.clone());
                added_origins.push(candidate.origin.clone());
            }
        }
        decisions.push(decision);
    }
    stats.added = additions.len();
    let mut detections = coarse.detections.clone();
    detections.extend(additions);
    let mut detection_origins: Vec<_> = coarse
        .detection_origins
        .iter()
        .map(|o| grid::Origin {
            view: o.view,
            rank: o.rank,
        })
        .collect();
    detection_origins.extend(added_origins);
    Ok(SupportedResult {
        naive,
        coarse,
        detections,
        detection_origins,
        decisions,
        stats,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> grid::Fixture {
        grid::Fixture {
            width: 1280,
            height: 720,
            views: grid::rectangles(1280, 720)
                .unwrap()
                .into_iter()
                .map(|rectangle| grid::FixtureView {
                    rectangle,
                    detections: vec![],
                })
                .collect(),
        }
    }
    fn add(f: &mut grid::Fixture, view: usize, kind: usize, score: f32, box_global: [f32; 4]) {
        let r = f.views[view].rectangle;
        f.views[view].detections.push(Detection {
            class_index: kind,
            confidence: score,
            bbox: [
                box_global[0] - r.x as f32,
                box_global[1] - r.y as f32,
                box_global[2] - r.x as f32,
                box_global[3] - r.y as f32,
            ],
        });
    }
    #[test]
    fn two_views_add_one_hypothesis_and_one_view_cannot_supply_two_votes() {
        let mut f = fixture();
        let b = [300., 100., 350., 150.];
        add(&mut f, 4, 9, 0.8, b);
        add(&mut f, 4, 9, 0.7, b);
        assert!(reconcile(&f).unwrap().detections.is_empty());
        add(&mut f, 5, 9, 0.6, b);
        let result = reconcile(&f).unwrap();
        assert_eq!(result.stats.added, 1);
        assert_eq!(result.detections[0].confidence.to_bits(), 0.8_f32.to_bits());
        assert_eq!(result.decisions[0].witnesses.len(), 2);
    }
    #[test]
    fn coarse_measurements_survive_even_when_another_class_has_a_higher_score() {
        let mut f = fixture();
        let b = [300., 100., 350., 150.];
        add(&mut f, 0, 2, 0.6, b);
        add(&mut f, 4, 9, 0.99, b);
        add(&mut f, 5, 9, 0.98, b);
        let result = reconcile(&f).unwrap();
        assert_eq!(result.detections.len(), 1);
        assert_eq!(result.detections[0].class_index, 2);
        assert_eq!(result.detections[0].confidence.to_bits(), 0.6_f32.to_bits());
        assert!(
            result
                .decisions
                .iter()
                .all(|d| d.reason == Reason::CoarseOverlap)
        );
    }
    #[test]
    fn conflicting_vehicle_labels_cannot_add_a_hypothesis() {
        let mut f = fixture();
        let b = [300., 330., 350., 380.];
        add(&mut f, 4, 2, 0.9, b);
        add(&mut f, 5, 7, 0.8, b);
        add(&mut f, 8, 2, 0.7, b);
        let result = reconcile(&f).unwrap();
        assert!(result.detections.is_empty());
        assert!(
            result
                .decisions
                .iter()
                .all(|d| d.reason == Reason::VehicleAmbiguity)
        );
    }
    #[test]
    fn support_uses_the_same_proposal_without_transitive_chains() {
        let mut f = fixture();
        add(&mut f, 4, 0, 0.9, [300., 330., 350., 380.]);
        add(&mut f, 5, 0, 0.8, [310., 330., 360., 380.]);
        add(&mut f, 8, 0, 0.7, [322., 330., 372., 380.]);
        let result = reconcile(&f).unwrap();
        assert_eq!(result.decisions[0].witnesses.len(), 2);
        assert_eq!(result.decisions[1].witnesses.len(), 3);
    }
    #[test]
    fn overlapping_person_and_bicycle_labels_are_distinct_hypotheses() {
        let mut f = fixture();
        let b = [300., 100., 350., 150.];
        for view in [4, 5] {
            add(&mut f, view, 0, 0.9, b);
            add(&mut f, view, 1, 0.8, b);
        }
        let result = reconcile(&f).unwrap();
        assert_eq!(result.stats.added, 2);
    }
    #[test]
    fn full_coarse_output_cannot_be_displaced_by_supported_additions() {
        let mut f = fixture();
        for i in 0..300 {
            let x = (i % 10 * 10) as f32;
            let y = (i / 10 * 10) as f32;
            add(&mut f, 0, 9, 0.8, [x, y, x + 4., y + 4.]);
        }
        let b = [300., 330., 350., 380.];
        add(&mut f, 4, 0, 0.7, b);
        add(&mut f, 5, 0, 0.6, b);
        let result = reconcile(&f).unwrap();
        assert_eq!(result.detections.len(), 300);
        assert_eq!(result.stats.added, 0);
        assert!(
            result
                .decisions
                .iter()
                .all(|d| d.reason == Reason::OutputCap)
        );
    }
}
