//! Direct, one-to-one agreement between two measured supplemental streams.
//! Coordinates and confidence remain those of the wide stream; no model runs here.
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
    pub coarse: Vec<Detection>,
    pub coarse_origins: Vec<Origin>,
    pub wide: Vec<Detection>,
    pub wide_origins: Vec<Origin>,
    pub detail: Vec<Detection>,
    pub detail_origins: Vec<Origin>,
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
    NoAgreement,
    NonMutual,
    AnchorOverlap,
    SuppressedAddition,
    OutputCap,
    Added,
}
#[derive(Serialize)]
pub struct Decision {
    pub wide_rank: usize,
    pub detail_rank: Option<usize>,
    pub agreement_iou_bits: Option<u32>,
    pub reason: Reason,
}
#[derive(Default, Serialize)]
pub struct Stats {
    pub coarse_boxes: usize,
    pub wide_additions: usize,
    pub detail_additions: usize,
    pub added: usize,
    pub agreement_comparisons: usize,
    pub anchor_comparisons: usize,
    pub nms_comparisons: usize,
    pub iou_comparisons: usize,
}
#[derive(Serialize)]
pub struct Result {
    pub detections: Vec<Detection>,
    pub lineage: Vec<Lineage>,
    pub decisions: Vec<Decision>,
    pub stats: Stats,
}

fn same(a: &Detection, b: &Detection) -> bool {
    a.class_index == b.class_index
        && a.confidence.to_bits() == b.confidence.to_bits()
        && a.bbox.map(f32::to_bits) == b.bbox.map(f32::to_bits)
}
fn validate(input: &Input) -> std::result::Result<(), String> {
    if input.width == 0
        || input.height == 0
        || u64::from(input.width) * u64::from(input.height) > 16_000_000
    {
        return Err("invalid source-pixel image dimensions".into());
    }
    for (boxes, origins) in [
        (&input.coarse, &input.coarse_origins),
        (&input.wide, &input.wide_origins),
        (&input.detail, &input.detail_origins),
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
    for (boxes, origins) in [
        (&input.wide, &input.wide_origins),
        (&input.detail, &input.detail_origins),
    ] {
        if boxes.len() < input.coarse.len()
            || !input.coarse.iter().zip(boxes).all(|(a, b)| same(a, b))
            || origins[..input.coarse.len()] != input.coarse_origins
        {
            return Err(
                "both complete parents must retain the exact coarse prefix and origins".into(),
            );
        }
    }
    Ok(())
}
fn iou(a: &Detection, b: &Detection, work: &mut usize) -> std::result::Result<f32, String> {
    if *work >= 270_000 {
        return Err("IoU comparison budget exceeded".into());
    }
    *work += 1;
    Ok(intersection_over_union(a.bbox, b.bbox))
}
pub fn combine(input: &Input) -> std::result::Result<Result, String> {
    validate(input)?;
    let n = input.coarse.len();
    let left = &input.wide[n..];
    let right = &input.detail[n..];
    let mut stats = Stats {
        coarse_boxes: n,
        wide_additions: left.len(),
        detail_additions: right.len(),
        ..Stats::default()
    };
    let mut left_best: Vec<Option<(usize, f32)>> = vec![None; left.len()];
    let mut right_best: Vec<Option<(usize, f32)>> = vec![None; right.len()];
    for (i, a) in left.iter().enumerate() {
        for (j, b) in right.iter().enumerate() {
            if a.class_index != b.class_index {
                continue;
            }
            let value = iou(a, b, &mut stats.iou_comparisons)?;
            stats.agreement_comparisons += 1;
            if value > 0.45 {
                // Ascending original indices and strict improvement retain the lower tie.
                if left_best[i].is_none_or(|(_, old)| value > old) {
                    left_best[i] = Some((j, value));
                }
                if right_best[j].is_none_or(|(_, old)| value > old) {
                    right_best[j] = Some((i, value));
                }
            }
        }
    }
    let mut detections = input.coarse.clone();
    let mut lineage: Vec<_> = input
        .coarse_origins
        .iter()
        .enumerate()
        .map(|(i, o)| Lineage {
            stream: "coarse",
            source_rank: i,
            origin: o.clone(),
        })
        .collect();
    let mut decisions = Vec::with_capacity(left.len());
    for (i, d) in left.iter().enumerate() {
        let best = left_best[i];
        let mut decision = Decision {
            wide_rank: n + i,
            detail_rank: best.map(|(j, _)| n + j),
            agreement_iou_bits: best.map(|(_, value)| value.to_bits()),
            reason: Reason::NoAgreement,
        };
        if let Some((j, _)) = best {
            decision.reason = Reason::NonMutual;
            if right_best[j].map(|(rank, _)| rank) == Some(i) {
                decision.reason = Reason::OutputCap;
                if detections.len() < 300 {
                    let mut veto = false;
                    for anchor in &input.coarse {
                        stats.anchor_comparisons += 1;
                        if iou(d, anchor, &mut stats.iou_comparisons)? > 0.45 {
                            veto = true;
                            break;
                        }
                    }
                    decision.reason = Reason::AnchorOverlap;
                    if !veto {
                        for added in &detections[n..] {
                            if d.class_index == added.class_index {
                                stats.nms_comparisons += 1;
                                if iou(d, added, &mut stats.iou_comparisons)? > 0.45 {
                                    veto = true;
                                    break;
                                }
                            }
                        }
                        decision.reason = Reason::SuppressedAddition;
                        if !veto {
                            detections.push(d.clone());
                            lineage.push(Lineage {
                                stream: "wide",
                                source_rank: n + i,
                                origin: input.wide_origins[n + i].clone(),
                            });
                            stats.added += 1;
                            decision.reason = Reason::Added;
                        }
                    }
                }
            }
        }
        decisions.push(decision);
    }
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
    fn box_at(class_index: usize, x: f32) -> Detection {
        Detection {
            class_index,
            confidence: 0.8,
            bbox: [x, 10., x + 20., 30.],
        }
    }
    fn fixture(coarse: Vec<Detection>, extra: Vec<Detection>, witnesses: Vec<Detection>) -> Input {
        let origins = |n, view| (0..n).map(|rank| Origin { view, rank }).collect::<Vec<_>>();
        let mut wide = coarse.clone();
        wide.extend(extra);
        let mut detail = coarse.clone();
        detail.extend(witnesses);
        let coarse_origins = origins(coarse.len(), 0);
        let mut wide_origins = coarse_origins.clone();
        wide_origins.extend(origins(wide.len() - coarse.len(), 4));
        let mut detail_origins = coarse_origins.clone();
        detail_origins.extend(origins(detail.len() - coarse.len(), 4));
        Input {
            width: 1280,
            height: 720,
            coarse,
            coarse_origins,
            wide,
            wide_origins,
            detail,
            detail_origins,
        }
    }
    #[test]
    fn preserves_coarse_and_measured_wide_bits() {
        let mut anchor = box_at(0, -0.0);
        anchor.confidence = 0.4;
        let f = fixture(vec![anchor], vec![box_at(9, 100.)], vec![box_at(9, 101.)]);
        let r = combine(&f).unwrap();
        assert_eq!(r.detections.len(), 2);
        assert!(same(&r.detections[0], &f.coarse[0]));
        assert!(same(&r.detections[1], &f.wide[1]));
        assert_eq!(r.lineage[1].origin, f.wide_origins[1]);
    }
    #[test]
    fn one_witness_cannot_support_two_candidates_and_ties_use_first_rank() {
        let f = fixture(
            vec![],
            vec![box_at(2, 100.), box_at(2, 100.)],
            vec![box_at(2, 100.)],
        );
        let r = combine(&f).unwrap();
        assert_eq!(r.stats.added, 1);
        assert_eq!(r.decisions[0].reason, Reason::Added);
        assert_eq!(r.decisions[1].reason, Reason::NonMutual);
    }
    #[test]
    fn direct_mutual_overlap_is_required_instead_of_a_support_chain() {
        let f = fixture(
            vec![],
            vec![box_at(2, 100.), box_at(2, 102.)],
            vec![box_at(2, 102.)],
        );
        let r = combine(&f).unwrap();
        assert_eq!(r.decisions[0].reason, Reason::NonMutual);
        assert_eq!(r.lineage[0].source_rank, 1);
    }
    #[test]
    fn classes_do_not_share_witnesses() {
        let r = combine(&fixture(
            vec![],
            vec![box_at(2, 100.)],
            vec![box_at(7, 100.)],
        ))
        .unwrap();
        assert_eq!(r.stats.added, 0);
        assert_eq!(r.stats.iou_comparisons, 0);
    }
    #[test]
    fn different_class_anchor_veto_remains() {
        let r = combine(&fixture(
            vec![box_at(7, 100.)],
            vec![box_at(2, 100.)],
            vec![box_at(2, 100.)],
        ))
        .unwrap();
        assert_eq!(r.decisions[0].reason, Reason::AnchorOverlap);
        assert_eq!(r.detections.len(), 1);
    }
    #[test]
    fn overlap_at_threshold_is_not_agreement() {
        let a = Detection {
            class_index: 2,
            confidence: 0.8,
            bbox: [0., 0., 20., 20.],
        };
        let b = Detection {
            bbox: [0., 0., 9., 20.],
            ..a.clone()
        };
        assert_eq!(intersection_over_union(a.bbox, b.bbox), 0.45);
        let r = combine(&fixture(vec![], vec![a], vec![b])).unwrap();
        assert_eq!(r.stats.added, 0);
    }
    #[test]
    fn rejects_changed_prefix_even_when_only_zero_sign_differs() {
        let mut f = fixture(vec![box_at(2, -0.0)], vec![], vec![]);
        f.detail[0].bbox[0] = 0.;
        assert!(combine(&f).is_err());
        f.detail[0].bbox[0] = -0.0;
        f.detail_origins[0].view = 1;
        assert!(combine(&f).is_err());
    }
    #[test]
    fn complete_stream_budgets_and_invalid_measurements_fail() {
        let coarse = (0..300).map(|_| box_at(2, 100.)).collect();
        let mut f = fixture(coarse, vec![], vec![]);
        let r = combine(&f).unwrap();
        assert_eq!(r.detections.len(), 300);
        assert_eq!(r.stats.iou_comparisons, 0);
        f.wide.push(box_at(2, 100.));
        assert!(combine(&f).is_err());
        f.wide.pop();
        f.wide[0].confidence = f32::NAN;
        assert!(combine(&f).is_err());
    }
}
