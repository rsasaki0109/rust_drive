//! Preserve every hypothesis unless full-image vehicle support is single-class.
use super::{
    yolox_tiles::{self, Fixture, FixtureView, Origin},
    yolox_vehicle_context,
};
use rustdriving_camera_detect::{Detection, intersection_over_union};
use serde::Serialize;
use sha2::{Digest, Sha256};

#[derive(Serialize)]
pub struct Support {
    pub source_rank: usize,
    pub class_index: usize,
    pub confidence: f32,
}
#[derive(Serialize)]
pub struct Decision {
    pub base_ranks: Vec<usize>,
    pub full_image_support: Vec<Support>,
    pub selected_base_rank: Option<usize>,
}
#[derive(Serialize)]
pub struct ConsistencyResult {
    pub context_result: yolox_vehicle_context::ContextResult,
    pub detections: Vec<Detection>,
    pub detection_origins: Vec<Origin>,
    pub retained_base_ranks: Vec<usize>,
    pub decisions: Vec<Decision>,
    pub comparison_work: usize,
}

pub fn reconcile(fixture: &Fixture) -> Result<ConsistencyResult, String> {
    let context_result = yolox_vehicle_context::reconcile(fixture)?;
    let voting = &context_result.voting_result;
    let base = &voting.base_merged;
    let mut work = voting.comparison_work;
    let single = Fixture {
        width: fixture.width,
        height: fixture.height,
        views: fixture
            .views
            .iter()
            .enumerate()
            .map(|(i, v)| FixtureView {
                rectangle: v.rectangle,
                detections: if i == 0 { v.detections.clone() } else { vec![] },
            })
            .collect(),
    };
    let full = yolox_tiles::merge(&single)?;
    work += full.merge.comparisons;
    let mut removed = vec![false; base.detections.len()];
    let mut decisions = Vec::new();
    for component in &voting.resolutions {
        let mut support = Vec::new();
        for (rank, candidate) in full.detections.iter().enumerate() {
            if !component
                .base_ranks
                .iter()
                .any(|&i| base.detections[i].class_index == candidate.class_index)
            {
                continue;
            }
            if component.base_ranks.iter().all(|&i| {
                work += 1;
                intersection_over_union(candidate.bbox, base.detections[i].bbox) > 0.45
            }) {
                support.push(Support {
                    source_rank: full.detection_origins[rank].rank,
                    class_index: candidate.class_index,
                    confidence: candidate.confidence,
                });
            }
        }
        let unique = support
            .first()
            .filter(|first| support.iter().all(|v| v.class_index == first.class_index));
        let selected = unique.and_then(|v| {
            component
                .base_ranks
                .iter()
                .copied()
                .find(|&i| base.detections[i].class_index == v.class_index)
        });
        if let Some(selected) = selected {
            for &i in &component.base_ranks {
                removed[i] = i != selected;
            }
        }
        decisions.push(Decision {
            base_ranks: component.base_ranks.clone(),
            full_image_support: support,
            selected_base_rank: selected,
        });
    }
    if work > 1_400_000 {
        return Err("bounded consistency comparison work exceeded".into());
    }
    let retained_base_ranks: Vec<_> = (0..base.detections.len())
        .filter(|&i| !removed[i])
        .collect();
    Ok(ConsistencyResult {
        detections: retained_base_ranks
            .iter()
            .map(|&i| base.detections[i].clone())
            .collect(),
        detection_origins: retained_base_ranks
            .iter()
            .map(|&i| base.detection_origins[i].clone())
            .collect(),
        context_result,
        retained_base_ranks,
        decisions,
        comparison_work: work,
    })
}

pub fn compiled_sources() -> std::collections::BTreeMap<&'static str, String> {
    let mut sources = yolox_vehicle_context::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/yolox_vehicle_consistency.rs",
            include_bytes!("yolox_vehicle_consistency.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-reconcile.rs",
            include_bytes!("bin/rustdriving-camera-reconcile.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-vehicle-consistency.json",
            include_bytes!("../profile-vehicle-consistency.json").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

pub fn producer_fixture(parent: &serde_json::Value) -> Result<Fixture, String> {
    let fail = || "invalid original tiled producer report".to_string();
    if parent["schema_version"] != 1
        || parent["algorithm"] != "pinned_yolox_s_full_plus_three_squares_v1"
        || parent["model_sha256"] != super::yolox_s::MODEL_SHA256
        || parent["compiled_sources"]
            != serde_json::to_value(yolox_tiles::compiled_sources()).map_err(|_| fail())?
    {
        return Err(fail());
    }
    let threshold = |v: &serde_json::Value, x: f32| {
        v.as_f64()
            .is_some_and(|n| n == f64::from(x) || n == if x == 0.3 { 0.3 } else { 0.45 })
    };
    if !threshold(&parent["score_threshold"], 0.3) || !threshold(&parent["nms_iou_threshold"], 0.45)
    {
        return Err(fail());
    }
    let width = parent["width"]
        .as_u64()
        .and_then(|v| u32::try_from(v).ok())
        .ok_or_else(fail)?;
    let height = parent["height"]
        .as_u64()
        .and_then(|v| u32::try_from(v).ok())
        .ok_or_else(fail)?;
    let image_sha = parent["source_image_sha256"].as_str().ok_or_else(fail)?;
    if image_sha.len() != 64 || !image_sha.bytes().all(|v| v.is_ascii_hexdigit()) {
        return Err(fail());
    }
    let rows = parent["views"].as_array().ok_or_else(fail)?;
    if rows.len() != yolox_tiles::rectangles(width, height)?.len() {
        return Err(fail());
    }
    let mut fixture = Fixture {
        width,
        height,
        views: Vec::new(),
    };
    for row in rows {
        let rectangle: yolox_tiles::Rectangle =
            serde_json::from_value(row["rectangle"].clone()).map_err(|_| fail())?;
        let p = &row["prediction"];
        let rgb = row["rgb_sha256"].as_str().ok_or_else(fail)?;
        if rgb.len() != 64 || !rgb.bytes().all(|v| v.is_ascii_hexdigit()) {
            return Err(fail());
        }
        if p["model_sha256"] != super::yolox_s::MODEL_SHA256
            || p["model_profile"] != "yolox-s-v1"
            || p["model_profile_sha256"]
                != super::yolox_s::compiled_sources()["integrations/onnx/profile-s.json"]
            || p["compiled_sources"]
                != serde_json::to_value(super::yolox_s::compiled_sources()).map_err(|_| fail())?
            || p["input_shape"] != serde_json::json!([1, 3, 640, 640])
            || p["output_shape"] != serde_json::json!([1, 8400, 85])
            || p["width"] != rectangle.width
            || p["height"] != rectangle.height
            || !threshold(&p["score_threshold"], 0.3)
            || !threshold(&p["nms_iou_threshold"], 0.45)
        {
            return Err(fail());
        }
        let detections: Vec<Detection> =
            serde_json::from_value(p["detections"].clone()).map_err(|_| fail())?;
        fixture.views.push(FixtureView {
            rectangle,
            detections,
        });
    }
    let base = yolox_tiles::merge(&fixture)?;
    let supplied: Vec<Detection> =
        serde_json::from_value(parent["detections"].clone()).map_err(|_| fail())?;
    if supplied.len() != base.detections.len()
        || supplied.iter().zip(&base.detections).any(|(a, b)| {
            a.class_index != b.class_index || a.confidence != b.confidence || a.bbox != b.bbox
        })
        || parent["detection_origins"]
            != serde_json::to_value(&base.detection_origins).map_err(|_| fail())?
        || parent["merge"] != serde_json::to_value(&base.merge).map_err(|_| fail())?
    {
        return Err(fail());
    }
    Ok(fixture)
}
