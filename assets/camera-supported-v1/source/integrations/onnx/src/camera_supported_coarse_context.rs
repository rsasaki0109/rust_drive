//! Full-image context anchors vehicle class; crop votes never supply confidence.
use super::{
    yolox_tiles::{Fixture, Origin},
    yolox_vehicle_vote,
};
use rustdriving_camera_detect::Detection;
use serde::Serialize;
use sha2::{Digest, Sha256};

#[derive(Serialize)]
pub struct Selection {
    pub base_ranks: Vec<usize>,
    pub selected_base_rank: usize,
    pub class_index: usize,
    pub full_image_source_rank: usize,
}
#[derive(Serialize)]
pub struct ContextResult {
    pub voting_result: yolox_vehicle_vote::Result,
    pub detections: Vec<Detection>,
    pub detection_origins: Vec<Origin>,
    pub retained_base_ranks: Vec<usize>,
    pub selections: Vec<Selection>,
    pub ambiguous_without_full_context: Vec<Vec<usize>>,
}

pub fn reconcile(fixture: &Fixture) -> Result<ContextResult, String> {
    let voting_result = yolox_vehicle_vote::reconcile(fixture)?;
    let base = &voting_result.base_merged;
    let mut removed = vec![false; base.detections.len()];
    let mut selections = Vec::new();
    let mut ambiguous_without_full_context = Vec::new();
    for component in &voting_result.resolutions {
        let Some(full) = component.votes.iter().find(|vote| vote.view == 0) else {
            ambiguous_without_full_context.push(component.base_ranks.clone());
            continue;
        };
        let selected = *component
            .base_ranks
            .iter()
            .find(|&&i| base.detections[i].class_index == full.class_index)
            .ok_or("full-image class has no original component representative")?;
        for &index in &component.base_ranks {
            removed[index] = index != selected;
        }
        selections.push(Selection {
            base_ranks: component.base_ranks.clone(),
            selected_base_rank: selected,
            class_index: full.class_index,
            full_image_source_rank: full.source_rank,
        });
    }
    let retained_base_ranks: Vec<_> = (0..base.detections.len())
        .filter(|&i| !removed[i])
        .collect();
    Ok(ContextResult {
        detections: retained_base_ranks
            .iter()
            .map(|&i| base.detections[i].clone())
            .collect(),
        detection_origins: retained_base_ranks
            .iter()
            .map(|&i| base.detection_origins[i].clone())
            .collect(),
        voting_result,
        retained_base_ranks,
        selections,
        ambiguous_without_full_context,
    })
}

pub fn compiled_sources() -> std::collections::BTreeMap<&'static str, String> {
    let mut sources = yolox_vehicle_vote::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/yolox_vehicle_context.rs",
            include_bytes!("yolox_vehicle_context.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-detect-vehicle-context.rs",
            include_bytes!("bin/rustdriving-camera-detect-vehicle-context.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-vehicle-context.json",
            include_bytes!("../profile-vehicle-context.json").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

#[cfg(test)]
mod tests {
    use super::super::yolox_tiles::{FixtureView, rectangles};
    use super::*;
    fn fixture(full: bool) -> Fixture {
        let mut f = Fixture {
            width: 1280,
            height: 720,
            views: rectangles(1280, 720)
                .unwrap()
                .into_iter()
                .map(|rectangle| FixtureView {
                    rectangle,
                    detections: vec![],
                })
                .collect(),
        };
        for (view, class_index, confidence) in [(0, 2, 0.6), (1, 5, 0.8), (2, 5, 0.9)] {
            if view == 0 && !full {
                continue;
            }
            let offset = f.views[view].rectangle.x as f32;
            f.views[view].detections.push(Detection {
                class_index,
                confidence,
                bbox: [400. - offset, 100., 500. - offset, 200.],
            });
        }
        f
    }
    #[test]
    fn full_image_overrides_correlated_crop_majority_without_inflating_confidence() {
        let result = reconcile(&fixture(true)).unwrap();
        assert_eq!(result.voting_result.detections[0].class_index, 5);
        assert_eq!(result.detections.len(), 1);
        assert_eq!(result.detections[0].class_index, 2);
        assert_eq!(result.detections[0].confidence, 0.6);
        assert_eq!(result.detection_origins[0], Origin { view: 0, rank: 0 });
    }
    #[test]
    fn conflicting_hypotheses_remain_when_full_image_has_no_eligible_vote() {
        let mut f = fixture(false);
        f.views[1].detections[0].class_index = 2;
        let result = reconcile(&f).unwrap();
        assert_eq!(result.voting_result.detections.len(), 1);
        assert_eq!(result.detections.len(), 2);
        assert_eq!(result.ambiguous_without_full_context.len(), 1);
        assert!(result.selections.is_empty());
    }
}
