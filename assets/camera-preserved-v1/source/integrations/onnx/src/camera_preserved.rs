//! Keep the complete anchor measurements; append only nonoverlapping proposals.
use rustdriving_camera_detect::{Detection, intersection_over_union};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(deny_unknown_fields)]
pub struct Origin {
    pub view: usize,
    pub rank: usize,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Input {
    pub width: u32,
    pub height: u32,
    pub anchor: Vec<Detection>,
    pub anchor_origins: Vec<Origin>,
    pub proposals: Vec<Detection>,
    pub proposal_origins: Vec<Origin>,
}
#[derive(Serialize)]
pub struct Lineage {
    pub stream: &'static str,
    pub source_rank: usize,
    pub origin: Origin,
}
#[derive(Debug, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Reason {
    AnchorOverlap,
    SuppressedAddition,
    OutputCap,
    Added,
}
#[derive(Serialize)]
pub struct Decision {
    pub proposal_rank: usize,
    pub anchor_rank: Option<usize>,
    pub addition_rank: Option<usize>,
    pub reason: Reason,
}
#[derive(Default, Serialize)]
pub struct Stats {
    pub anchor_boxes: usize,
    pub proposal_boxes: usize,
    pub added: usize,
    pub anchor_overlap: usize,
    pub suppressed_additions: usize,
    pub output_cap: usize,
    pub iou_comparisons: usize,
}
#[derive(Serialize)]
pub struct Result {
    pub detections: Vec<Detection>,
    pub lineage: Vec<Lineage>,
    pub decisions: Vec<Decision>,
    pub stats: Stats,
}

fn validate(input: &Input) -> std::result::Result<(), String> {
    if input.width == 0
        || input.height == 0
        || u64::from(input.width) * u64::from(input.height) > 16_000_000
    {
        return Err("invalid source-pixel image dimensions".into());
    }
    for (boxes, origins) in [
        (&input.anchor, &input.anchor_origins),
        (&input.proposals, &input.proposal_origins),
    ] {
        if boxes.len() > 300 || boxes.len() != origins.len() {
            return Err("at most300 measurements with complete origins per stream required".into());
        }
        let mut seen = BTreeSet::new();
        for (d, origin) in boxes.iter().zip(origins) {
            let [l, t, r, b] = d.bbox;
            if origin.view >= 16
                || origin.rank >= 300
                || !seen.insert(origin)
                || d.class_index >= 80
                || !d.confidence.is_finite()
                || !(0.3..=1.).contains(&d.confidence)
                || !d.bbox.iter().all(|v| v.is_finite())
                || l < 0.
                || t < 0.
                || r <= l
                || b <= t
                || r > input.width as f32
                || b > input.height as f32
            {
                return Err("invalid measurement or origin in a complete stream".into());
            }
        }
    }
    Ok(())
}
fn overlap(a: &Detection, b: &Detection, work: &mut usize) -> std::result::Result<bool, String> {
    if *work >= 180_000 {
        return Err("IoU comparison budget exceeded".into());
    }
    *work += 1;
    Ok(intersection_over_union(a.bbox, b.bbox) > 0.45)
}
pub fn combine(input: &Input) -> std::result::Result<Result, String> {
    validate(input)?;
    let mut stats = Stats {
        anchor_boxes: input.anchor.len(),
        proposal_boxes: input.proposals.len(),
        ..Stats::default()
    };
    let mut ranks: Vec<_> = (0..input.proposals.len()).collect();
    ranks.sort_by(|&a, &b| {
        let a_box = &input.proposals[a];
        let b_box = &input.proposals[b];
        b_box
            .confidence
            .total_cmp(&a_box.confidence)
            .then(a_box.class_index.cmp(&b_box.class_index))
            .then(a_box.bbox[0].total_cmp(&b_box.bbox[0]))
            .then(a.cmp(&b))
    });
    let mut additions: Vec<Detection> = vec![];
    let mut added_ranks = vec![];
    let mut decisions = vec![];
    for rank in ranks {
        let d = &input.proposals[rank];
        let mut decision = Decision {
            proposal_rank: rank,
            anchor_rank: None,
            addition_rank: None,
            reason: Reason::OutputCap,
        };
        if input.anchor.len() + additions.len() >= 300 {
            stats.output_cap += 1;
            decisions.push(decision);
            continue;
        }
        for (i, anchor) in input.anchor.iter().enumerate() {
            if overlap(d, anchor, &mut stats.iou_comparisons)? {
                decision.anchor_rank = Some(i);
                break;
            }
        }
        if decision.anchor_rank.is_some() {
            decision.reason = Reason::AnchorOverlap;
            stats.anchor_overlap += 1;
            decisions.push(decision);
            continue;
        }
        for (i, other) in additions.iter().enumerate() {
            if d.class_index == other.class_index && overlap(d, other, &mut stats.iou_comparisons)?
            {
                decision.addition_rank = Some(i);
                break;
            }
        }
        if decision.addition_rank.is_some() {
            decision.reason = Reason::SuppressedAddition;
            stats.suppressed_additions += 1;
        } else {
            decision.reason = Reason::Added;
            decision.addition_rank = Some(additions.len());
            additions.push(d.clone());
            added_ranks.push(rank);
        }
        decisions.push(decision);
    }
    stats.added = additions.len();
    let mut detections = input.anchor.clone();
    detections.extend(additions);
    let mut lineage: Vec<_> = input
        .anchor_origins
        .iter()
        .enumerate()
        .map(|(rank, o)| Lineage {
            stream: "anchor",
            source_rank: rank,
            origin: o.clone(),
        })
        .collect();
    lineage.extend(added_ranks.into_iter().map(|rank| Lineage {
        stream: "detail",
        source_rank: rank,
        origin: input.proposal_origins[rank].clone(),
    }));
    Ok(Result {
        detections,
        lineage,
        decisions,
        stats,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn d(class_index: usize, confidence: f32, bbox: [f32; 4]) -> Detection {
        Detection {
            class_index,
            confidence,
            bbox,
        }
    }
    fn input(anchor: Vec<Detection>, proposals: Vec<Detection>) -> Input {
        let origins = |n| (0..n).map(|rank| Origin { view: 0, rank }).collect();
        Input {
            width: 1280,
            height: 720,
            anchor_origins: origins(anchor.len()),
            proposal_origins: origins(proposals.len()),
            anchor,
            proposals,
        }
    }
    #[test]
    fn higher_confidence_other_class_never_replaces_an_anchor() {
        let i = input(
            vec![d(2, 0.4, [10., 20., 40., 60.])],
            vec![d(7, 0.99, [10., 20., 40., 60.])],
        );
        let r = combine(&i).unwrap();
        assert_eq!(r.detections.len(), 1);
        assert_eq!(r.detections[0].confidence.to_bits(), 0.4_f32.to_bits());
        assert_eq!(r.decisions[0].reason, Reason::AnchorOverlap);
    }
    #[test]
    fn anchors_keep_input_order_and_signed_zero_while_additions_keep_source_bits() {
        let i = input(
            vec![
                d(2, 0.3, [-0.0, 10., 30., 40.]),
                d(0, 0.8, [100., 100., 110., 150.]),
            ],
            vec![d(9, 0.9, [500., 100., 510., 120.])],
        );
        let r = combine(&i).unwrap();
        assert_eq!(r.detections[0].bbox[0].to_bits(), (-0.0_f32).to_bits());
        assert_eq!(r.detections[0].class_index, 2);
        assert_eq!(r.detections[2].confidence.to_bits(), 0.9_f32.to_bits());
        assert_eq!(r.lineage[2].stream, "detail");
    }
    #[test]
    fn only_additions_receive_classwise_nms_with_stable_rank_ties() {
        let i = input(
            vec![],
            vec![
                d(9, 0.8, [500., 100., 510., 120.]),
                d(9, 0.8, [500., 100., 510., 120.]),
                d(0, 0.7, [500., 100., 510., 120.]),
            ],
        );
        let r = combine(&i).unwrap();
        assert_eq!(r.detections.len(), 2);
        assert_eq!(r.lineage[0].source_rank, 0);
        assert_eq!(r.decisions[1].reason, Reason::SuppressedAddition);
    }
    #[test]
    fn output_cap_never_displaces_the_complete_anchor() {
        let anchor = (0..300)
            .map(|j| {
                d(
                    9,
                    0.5,
                    [
                        (j % 20) as f32 * 5.,
                        (j / 20) as f32 * 5.,
                        (j % 20) as f32 * 5. + 2.,
                        (j / 20) as f32 * 5. + 2.,
                    ],
                )
            })
            .collect();
        let r = combine(&input(anchor, vec![d(0, 1., [500., 100., 510., 120.])])).unwrap();
        assert_eq!(r.detections.len(), 300);
        assert_eq!(r.stats.output_cap, 1);
        assert_eq!(r.stats.iou_comparisons, 0);
    }
    #[test]
    fn malformed_measurements_and_lineage_fail_before_combining() {
        let mut i = input(vec![d(0, 0.8, [10., 10., 20., 20.])], vec![]);
        i.anchor_origins.clear();
        assert!(combine(&i).is_err());
        i.anchor_origins.push(Origin { view: 16, rank: 0 });
        assert!(combine(&i).is_err());
        i.anchor_origins[0].view = 0;
        i.anchor[0].confidence = f32::NAN;
        assert!(combine(&i).is_err());
    }
    #[test]
    fn duplicate_source_origins_are_rejected_even_for_valid_boxes() {
        let mut i = input(
            vec![
                d(0, 0.8, [10., 10., 20., 20.]),
                d(0, 0.7, [30., 30., 40., 40.]),
            ],
            vec![],
        );
        i.anchor_origins[1] = i.anchor_origins[0].clone();
        assert!(combine(&i).is_err());
    }
}
