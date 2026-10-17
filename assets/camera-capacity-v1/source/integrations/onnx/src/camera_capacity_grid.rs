//! Complete original views plus a bounded overlapping 640px grid.
use super::yolox_s;
use image::RgbImage;
use rustdriving_camera_detect::{Detection, intersection_over_union};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Rectangle {
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
}

pub const MAX_VIEWS: usize = 16;
fn offsets(dimension: u32, side: u32) -> Vec<u32> {
    let gap = dimension - side;
    if gap == 0 {
        return vec![0];
    }
    let stride = (3 * side / 4).max(1);
    let steps = gap.div_ceil(stride);
    (0..=steps)
        .map(|i| ((u64::from(i) * u64::from(gap)) / u64::from(steps)) as u32)
        .collect()
}
pub fn rectangles(width: u32, height: u32) -> Result<Vec<Rectangle>, String> {
    let mut views: Vec<_> = super::original_tiles::rectangles(width, height)?
        .into_iter()
        .map(|r| Rectangle {
            x: r.x,
            y: r.y,
            width: r.width,
            height: r.height,
        })
        .collect();
    let side = width.min(height).min(640);
    let stride = (3 * side / 4).max(1);
    let nx = (width - side).div_ceil(stride) + 1;
    let ny = (height - side).div_ceil(stride) + 1;
    if u64::from(nx) * u64::from(ny) > MAX_VIEWS as u64 {
        return Err("complete grid exceeds sixteen-view budget".into());
    }
    for y in offsets(height, side) {
        for x in offsets(width, side) {
            let r = Rectangle {
                x,
                y,
                width: side,
                height: side,
            };
            if !views.contains(&r) {
                views.push(r);
            }
        }
    }
    if views.len() > MAX_VIEWS {
        return Err("complete combined inventory exceeds sixteen-view budget".into());
    }
    Ok(views)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FixtureView {
    pub rectangle: Rectangle,
    pub detections: Vec<Detection>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Fixture {
    pub width: u32,
    pub height: u32,
    pub views: Vec<FixtureView>,
}
#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct Origin {
    pub view: usize,
    pub rank: usize,
}
#[derive(Default, Serialize)]
pub struct MergeStats {
    pub input_boxes: usize,
    pub interior_edge_rejected: usize,
    pub suppressed: usize,
    pub comparisons: usize,
    pub output_cap_reached: bool,
}
#[derive(Serialize)]
pub struct Merged {
    pub detections: Vec<Detection>,
    pub detection_origins: Vec<Origin>,
    pub merge: MergeStats,
}

pub fn merge(input: &Fixture) -> Result<Merged, String> {
    let expected = rectangles(input.width, input.height)?;
    if input.views.len() != expected.len() {
        return Err("complete fixed view inventory required".into());
    }
    let mut candidates = Vec::new();
    let mut stats = MergeStats::default();
    for (view, (data, rectangle)) in input.views.iter().zip(expected).enumerate() {
        if data.rectangle != rectangle || data.detections.len() > 300 {
            return Err("fixed rectangles and bounded per-view detections required".into());
        }
        for (rank, detection) in data.detections.iter().enumerate() {
            let [l, t, r, b] = detection.bbox;
            if detection.class_index >= 80
                || !detection.confidence.is_finite()
                || !(0.3..=1.).contains(&detection.confidence)
                || !detection.bbox.iter().all(|v| v.is_finite())
                || l < 0.
                || t < 0.
                || r <= l
                || b <= t
                || r > rectangle.width as f32
                || b > rectangle.height as f32
            {
                return Err("invalid original-view detection".into());
            }
            stats.input_boxes += 1;
            if (rectangle.x > 0 && l <= 1.)
                || (rectangle.y > 0 && t <= 1.)
                || (rectangle.x + rectangle.width < input.width && r >= rectangle.width as f32 - 1.)
                || (rectangle.y + rectangle.height < input.height
                    && b >= rectangle.height as f32 - 1.)
            {
                stats.interior_edge_rejected += 1;
                continue;
            }
            let mut detection = detection.clone();
            detection.bbox = [
                l + rectangle.x as f32,
                t + rectangle.y as f32,
                r + rectangle.x as f32,
                b + rectangle.y as f32,
            ];
            candidates.push((detection, Origin { view, rank }));
        }
    }
    candidates.sort_by(|(a, ai), (b, bi)| {
        b.confidence
            .total_cmp(&a.confidence)
            .then(a.class_index.cmp(&b.class_index))
            .then(a.bbox[0].total_cmp(&b.bbox[0]))
            .then(ai.view.cmp(&bi.view))
            .then(ai.rank.cmp(&bi.rank))
    });
    let mut kept: Vec<Detection> = Vec::new();
    let mut origins = Vec::new();
    for (candidate, origin) in candidates {
        let suppressed = kept.iter().any(|other| {
            stats.comparisons += 1;
            other.class_index == candidate.class_index
                && intersection_over_union(other.bbox, candidate.bbox) > 0.45
        });
        if suppressed {
            stats.suppressed += 1;
            continue;
        }
        kept.push(candidate);
        origins.push(origin);
        if kept.len() == 300 {
            stats.output_cap_reached = true;
            break;
        }
    }
    Ok(Merged {
        detections: kept,
        detection_origins: origins,
        merge: stats,
    })
}

pub fn compiled_sources() -> std::collections::BTreeMap<&'static str, String> {
    super::compiled_sources()
}

#[derive(Serialize)]
pub struct View {
    pub rectangle: Rectangle,
    pub rgb_sha256: String,
    pub prediction: yolox_s::Report,
}
#[derive(Serialize)]
pub struct Report {
    pub schema_version: u32,
    pub algorithm: &'static str,
    pub compiled_sources: std::collections::BTreeMap<&'static str, String>,
    pub source_image_sha256: String,
    pub model_sha256: &'static str,
    pub width: u32,
    pub height: u32,
    pub score_threshold: f32,
    pub nms_iou_threshold: f32,
    pub views: Vec<View>,
    #[serde(flatten)]
    pub merged: Merged,
}

pub fn detect(
    detector: &yolox_s::Detector,
    image: &RgbImage,
    source_sha: String,
) -> Result<Report, String> {
    let mut views = Vec::new();
    let mut fixture = Fixture {
        width: image.width(),
        height: image.height(),
        views: Vec::new(),
    };
    for rectangle in rectangles(image.width(), image.height())? {
        let cropped = image::imageops::crop_imm(
            image,
            rectangle.x,
            rectangle.y,
            rectangle.width,
            rectangle.height,
        )
        .to_image();
        let prediction = detector.detect(&cropped, 0.3, 0.45)?;
        fixture.views.push(FixtureView {
            rectangle,
            detections: prediction.detections.clone(),
        });
        views.push(View {
            rectangle,
            rgb_sha256: format!("{:x}", Sha256::digest(cropped.as_raw())),
            prediction,
        });
    }
    let merged = merge(&fixture)?;
    Ok(Report {
        schema_version: 1,
        algorithm: "pinned_yolox_s_capacity_grid_serial_views_v1",
        compiled_sources: compiled_sources(),
        source_image_sha256: source_sha,
        model_sha256: yolox_s::MODEL_SHA256,
        width: image.width(),
        height: image.height(),
        score_threshold: 0.3,
        nms_iou_threshold: 0.45,
        views,
        merged,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fixed_views_cover_landscape_portrait_and_odd_gaps_without_duplicate_work() {
        let landscape = rectangles(1280, 720).unwrap();
        assert_eq!(
            landscape.iter().map(|r| r.x).collect::<Vec<_>>(),
            [0, 0, 280, 560, 0, 320, 640, 0, 320, 640]
        );
        assert_eq!(
            rectangles(720, 1281)
                .unwrap()
                .iter()
                .map(|r| r.y)
                .collect::<Vec<_>>(),
            [0, 0, 280, 561, 0, 0, 320, 320, 641, 641]
        );
        assert_eq!(rectangles(640, 640).unwrap().len(), 1);
        assert_eq!(rectangles(641, 640).unwrap().len(), 3);
        assert!(rectangles(0, 640).is_err());
        assert!(rectangles(5000, 5000).is_err());
    }
    #[test]
    fn interior_edges_reject_truncated_objects_and_global_nms_retains_class_identity() {
        let rects = rectangles(1280, 720).unwrap();
        let mut input = Fixture {
            width: 1280,
            height: 720,
            views: rects
                .iter()
                .map(|&rectangle| FixtureView {
                    rectangle,
                    detections: vec![],
                })
                .collect(),
        };
        input.views[0].detections.push(Detection {
            class_index: 2,
            confidence: 0.8,
            bbox: [300., 100., 400., 200.],
        });
        input.views[2].detections.push(Detection {
            class_index: 2,
            confidence: 0.8,
            bbox: [20., 100., 120., 200.],
        });
        input.views[2].detections.push(Detection {
            class_index: 0,
            confidence: 0.7,
            bbox: [20., 100., 120., 200.],
        });
        input.views[2].detections.push(Detection {
            class_index: 7,
            confidence: 0.9,
            bbox: [1., 10., 30., 30.],
        });
        let result = merge(&input).unwrap();
        assert_eq!(result.detections.len(), 2);
        assert_eq!(
            result.detection_origins,
            [Origin { view: 0, rank: 0 }, Origin { view: 2, rank: 1 }]
        );
        assert_eq!(
            (result.merge.interior_edge_rejected, result.merge.suppressed),
            (1, 1)
        );
    }
}

#[cfg(test)]
mod grid_tests {
    use super::*;
    #[test]
    fn all_accepted_axes_have_coverage_overlap_and_a_hard_work_bound() {
        for (w, h) in [
            (1280, 720),
            (720, 1280),
            (641, 640),
            (320, 240),
            (640, 640),
            (1, 1),
            (900, 900),
        ] {
            let views = rectangles(w, h).unwrap();
            assert!(views.len() <= MAX_VIEWS);
            let side = w.min(h).min(640);
            for (dimension, vertical) in [(w, false), (h, true)] {
                let positions = offsets(dimension, side);
                assert_eq!(positions[0], 0);
                assert_eq!(*positions.last().unwrap() + side, dimension);
                assert!(
                    positions
                        .windows(2)
                        .all(|p| p[1] - p[0] <= (3 * side / 4).max(1))
                );
                for value in positions {
                    assert!(views.iter().any(|r| if vertical {
                        r.y == value
                    } else {
                        r.x == value
                    }));
                }
            }
        }
        for (w, h) in [(2560, 1440), (1920, 1920), (16_000_000, 1)] {
            assert!(rectangles(w, h).is_err());
        }
    }
    #[test]
    fn missing_reordered_or_extra_views_never_form_a_complete_result() {
        let mut fixture = Fixture {
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
        assert_eq!(fixture.views.len(), 10);
        assert!(merge(&fixture).is_ok());
        fixture.views.swap(4, 5);
        assert!(merge(&fixture).is_err());
        fixture.views.swap(4, 5);
        fixture.views.pop();
        assert!(merge(&fixture).is_err());
    }
}
