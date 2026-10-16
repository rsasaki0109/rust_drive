//! Observe existing tract execution, with bitwise ordinary-forward verification.
use super::{camera_analysis, camera_repeat2, yolox_s, yolox_tiles, yolox_vehicle_consistency};
use image::{RgbImage, imageops::FilterType};
use rustdriving_camera_detect::Detection;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, io::Cursor, path::Path, time::Instant};
use tract_onnx::{prelude::*, tract_core::ops::matmul::optimized::OptMatMul};

type CpuPlan = SimplePlan<TypedFact, Box<dyn TypedOp>, TypedModel>;
const MAX_NODES: usize = 4096;

pub fn compiled_sources() -> BTreeMap<&'static str, String> {
    let mut sources = camera_analysis::compiled_sources();
    for (name, bytes) in [
        (
            "integrations/onnx/src/camera_profile.rs",
            include_bytes!("camera_profile.rs").as_slice(),
        ),
        (
            "integrations/onnx/src/bin/rustdriving-camera-profile.rs",
            include_bytes!("bin/rustdriving-camera-profile.rs").as_slice(),
        ),
        (
            "integrations/onnx/profile-operators.json",
            include_bytes!("../profile-operators.json").as_slice(),
        ),
        (
            "integrations/onnx/src/camera_repeat2.rs",
            include_bytes!("camera_repeat2.rs").as_slice(),
        ),
    ] {
        sources.insert(name, format!("{:x}", Sha256::digest(bytes)));
    }
    sources
}

#[derive(Clone, Serialize)]
struct Label {
    node_id: usize,
    node_name: String,
    operation: String,
    output_shapes: Vec<Vec<String>>,
    matrix_kernel_choices: Vec<String>,
}
#[derive(Serialize)]
struct Operator {
    #[serde(flatten)]
    label: Label,
    started_ns: u128,
    finished_ns: u128,
    elapsed_ns: u128,
}
#[derive(Serialize, Debug, PartialEq, Eq)]
struct Group {
    operation: String,
    calls: usize,
    elapsed_ns: u128,
}
#[derive(Serialize)]
struct Profile {
    forward_wall_ns: u128,
    operator_sum_ns: u128,
    unattributed_ns: u128,
    operators: Vec<Operator>,
    by_operation: Vec<Group>,
}

fn labels(plan: &CpuPlan) -> Result<Vec<Label>, String> {
    let nodes = plan.model().nodes();
    if nodes.is_empty() || nodes.len() > MAX_NODES {
        return Err("optimized graph exceeds node bound".into());
    }
    Ok(nodes
        .iter()
        .map(|node| Label {
            node_id: node.id,
            node_name: node.name.clone(),
            operation: node.op().name().into_owned(),
            output_shapes: node
                .outputs
                .iter()
                .map(|o| o.fact.shape.iter().map(ToString::to_string).collect())
                .collect(),
            matrix_kernel_choices: node
                .op_as::<OptMatMul>()
                .map(|op| op.mmm.iter().map(|m| m.name().to_owned()).collect())
                .unwrap_or_default(),
        })
        .collect())
}

fn summarize(
    labels: &[Label],
    samples: &[(usize, u128, u128)],
    wall_ns: u128,
) -> Result<Profile, String> {
    if labels.is_empty()
        || labels.len() > MAX_NODES
        || samples.is_empty()
        || samples.len() > labels.len()
    {
        return Err("bounded nonempty graph observations required".into());
    }
    let mut seen = vec![false; labels.len()];
    let mut previous_end = 0;
    let mut operators = Vec::new();
    let mut groups: BTreeMap<String, Group> = BTreeMap::new();
    let mut total = 0;
    for &(id, begin, end) in samples {
        let label = labels.get(id).ok_or("observed node outside graph")?;
        if label.node_id != id || seen[id] || begin < previous_end || end < begin || end > wall_ns {
            return Err("invalid serial operator observation".into());
        }
        seen[id] = true;
        previous_end = end;
        let elapsed = end - begin;
        total += elapsed;
        let group = groups
            .entry(label.operation.clone())
            .or_insert_with(|| Group {
                operation: label.operation.clone(),
                calls: 0,
                elapsed_ns: 0,
            });
        group.calls += 1;
        group.elapsed_ns += elapsed;
        operators.push(Operator {
            label: label.clone(),
            started_ns: begin,
            finished_ns: end,
            elapsed_ns: elapsed,
        });
    }
    let mut by_operation: Vec<_> = groups.into_values().collect();
    by_operation.sort_by(|a, b| {
        b.elapsed_ns
            .cmp(&a.elapsed_ns)
            .then(a.operation.cmp(&b.operation))
    });
    Ok(Profile {
        forward_wall_ns: wall_ns,
        operator_sum_ns: total,
        unattributed_ns: wall_ns - total,
        operators,
        by_operation,
    })
}

fn observed(
    plan: &CpuPlan,
    labels: &[Label],
    input: Tensor,
) -> Result<(TVec<TValue>, Profile), String> {
    let mut samples = Vec::with_capacity(labels.len());
    let started = Instant::now();
    let mut state = SimpleState::new(plan).map_err(|e| e.to_string())?;
    let outputs = state
        .run_plan_with_eval(tvec!(input.into()), |session, state, node, inputs| {
            let begin = started.elapsed().as_nanos();
            let result = tract_onnx::tract_core::plan::eval(session, state, node, inputs);
            let end = started.elapsed().as_nanos();
            samples.push((node.id, begin, end));
            result
        })
        .map_err(|e| format!("profiled inference: {e}"))?;
    drop(state);
    let wall_ns = started.elapsed().as_nanos();
    Ok((outputs, summarize(labels, &samples, wall_ns)?))
}

// Kept separate from the immutable detector source. Complete real-image output
// comparisons check this mirrored Triangle/BGR/f32 input path independently.
fn preprocess(image: &RgbImage) -> Result<(Tensor, f32), String> {
    let (w, h) = image.dimensions();
    if w == 0 || h == 0 || u64::from(w) * u64::from(h) > 16_000_000 {
        return Err("invalid bounded source image".into());
    }
    let side = yolox_s::INPUT_SIZE;
    let ratio = (side as f32 / w as f32).min(side as f32 / h as f32);
    let resized = image::imageops::resize(
        image,
        (w as f32 * ratio).floor().max(1.0) as u32,
        (h as f32 * ratio).floor().max(1.0) as u32,
        FilterType::Triangle,
    );
    let mut values = vec![114.0_f32; 3 * side * side];
    for (x, y, p) in resized.enumerate_pixels() {
        for c in 0..3 {
            values[c * side * side + y as usize * side + x as usize] = f32::from(p[2 - c]);
        }
    }
    Ok((
        Tensor::from_shape(&[1, 3, side, side], &values).map_err(|e| e.to_string())?,
        ratio,
    ))
}
fn values(outputs: &[TValue]) -> Result<&[f32], String> {
    if outputs.len() != 1 || outputs[0].shape() != [1, yolox_s::ROWS, yolox_s::CLASSES + 5] {
        return Err("unexpected pinned YOLOX output geometry".into());
    }
    outputs[0].as_slice::<f32>().map_err(|e| e.to_string())
}
fn float_hash(values: &[f32]) -> String {
    let mut hash = Sha256::new();
    for value in values {
        hash.update(value.to_bits().to_le_bytes());
    }
    format!("{:x}", hash.finalize())
}
fn same_bits(a: &[f32], b: &[f32]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(a, b)| a.to_bits() == b.to_bits())
}

#[derive(Serialize)]
struct View {
    rectangle: yolox_tiles::Rectangle,
    rgb_sha256: String,
    input_tensor_f32le_sha256: String,
    output_tensor_f32le_sha256: String,
    reference_output_tensor_f32le_sha256: String,
    output_values: usize,
    output_bitwise_equal: bool,
    profile: Profile,
    reference_forward_wall_ns: u128,
    detections: Vec<Detection>,
    reference_detections: Vec<Detection>,
}
#[derive(Serialize)]
pub struct Report {
    schema_version: u32,
    algorithm: &'static str,
    compiled_sources: BTreeMap<&'static str, String>,
    source_image_sha256: String,
    model_sha256: &'static str,
    width: u32,
    height: u32,
    score_threshold: f32,
    nms_iou_threshold: f32,
    execution: serde_json::Value,
    control_permission: bool,
    views: Vec<View>,
    merged: yolox_tiles::Merged,
    result: yolox_vehicle_consistency::ConsistencyResult,
    reference_merged: yolox_tiles::Merged,
    reference_result: yolox_vehicle_consistency::ConsistencyResult,
}

pub fn analyze(
    model: &Path,
    image: &RgbImage,
    source_sha: String,
    repeat2x: bool,
    reference_first: bool,
) -> Result<Report, String> {
    let bytes = yolox_s::verify_model(model).map_err(|e| format!("load model: {e}"))?;
    let onnx = tract_onnx::onnx();
    let proto = onnx
        .proto_model_for_read(&mut Cursor::new(bytes))
        .map_err(|e| e.to_string())?;
    let mut model = onnx
        .model_for_proto_model(&proto)
        .and_then(|m| {
            m.with_input_fact(
                0,
                f32::fact([1, 3, yolox_s::INPUT_SIZE, yolox_s::INPUT_SIZE]).into(),
            )
        })
        .and_then(|m| m.into_optimized())
        .map_err(|e| e.to_string())?;
    let reference_plan = if repeat2x {
        Some(model.clone().into_runnable().map_err(|e| e.to_string())?)
    } else {
        None
    };
    let rewritten = if repeat2x {
        camera_repeat2::rewrite(&mut model, &proto)?
    } else {
        0
    };
    let plan = model.into_runnable().map_err(|e| e.to_string())?;
    let reference_plan = reference_plan.as_ref().unwrap_or(&plan);
    let labels = labels(&plan)?;
    let mut views = Vec::new();
    for rectangle in yolox_tiles::rectangles(image.width(), image.height())? {
        let rgb = image::imageops::crop_imm(
            image,
            rectangle.x,
            rectangle.y,
            rectangle.width,
            rectangle.height,
        )
        .to_image();
        let (input, ratio) = preprocess(&rgb)?;
        let input_hash = float_hash(input.as_slice::<f32>().map_err(|e| e.to_string())?);
        let reference_run = |input: Tensor| -> Result<_, String> {
            let started = Instant::now();
            let result = reference_plan
                .run(tvec!(input.into()))
                .map_err(|e| e.to_string())?;
            Ok((result, started.elapsed().as_nanos()))
        };
        let (outputs, profile, reference, reference_wall) = if reference_first {
            let (reference, wall) = reference_run(input.clone())?;
            let (outputs, profile) = observed(&plan, &labels, input)?;
            (outputs, profile, reference, wall)
        } else {
            let (outputs, profile) = observed(&plan, &labels, input.clone())?;
            let (reference, wall) = reference_run(input)?;
            (outputs, profile, reference, wall)
        };
        let output_values = values(&outputs)?;
        let reference_values = values(&reference)?;
        if !same_bits(output_values, reference_values) {
            return Err("profiled plan changed raw model output bits".into());
        }
        let detections =
            yolox_s::decode(output_values, rgb.width(), rgb.height(), ratio, 0.3, 0.45)?;
        let reference_detections = yolox_s::decode(
            reference_values,
            rgb.width(),
            rgb.height(),
            ratio,
            0.3,
            0.45,
        )?;
        views.push(View {
            rectangle,
            rgb_sha256: format!("{:x}", Sha256::digest(rgb.as_raw())),
            input_tensor_f32le_sha256: input_hash,
            output_tensor_f32le_sha256: float_hash(output_values),
            reference_output_tensor_f32le_sha256: float_hash(reference_values),
            output_values: output_values.len(),
            output_bitwise_equal: true,
            profile,
            reference_forward_wall_ns: reference_wall,
            detections,
            reference_detections,
        });
    }
    let fixture = yolox_tiles::Fixture {
        width: image.width(),
        height: image.height(),
        views: views
            .iter()
            .map(|v| yolox_tiles::FixtureView {
                rectangle: v.rectangle,
                detections: v.detections.clone(),
            })
            .collect(),
    };
    let reference_fixture = yolox_tiles::Fixture {
        width: image.width(),
        height: image.height(),
        views: views
            .iter()
            .map(|v| yolox_tiles::FixtureView {
                rectangle: v.rectangle,
                detections: v.reference_detections.clone(),
            })
            .collect(),
    };
    let n = views.len();
    Ok(Report {
        schema_version: 2,
        algorithm: if repeat2x {
            "yolox_s_repeat2x_operator_profile_v2"
        } else {
            "yolox_s_cpu_operator_profile_v2"
        },
        compiled_sources: compiled_sources(),
        source_image_sha256: source_sha,
        model_sha256: yolox_s::MODEL_SHA256,
        width: image.width(),
        height: image.height(),
        score_threshold: 0.3,
        nms_iou_threshold: 0.45,
        execution: serde_json::json!({"model_loads":1,"model_plans":if repeat2x {2}else{1},"images_read":1,"graph_nodes":labels.len(),"profiled_forwards":n,
            "ordinary_reference_forwards":n,"model_forwards":n*2,"view_workers":1,"resizes_rewritten":rewritten,
            "order":if reference_first {"reference_then_profile_per_view"}else{"profile_then_reference_per_view"},
            "timing_purpose":"serial_operator_hotspots_not_deadline_measurement"}),
        control_permission: false,
        views,
        merged: yolox_tiles::merge(&fixture)?,
        result: yolox_vehicle_consistency::reconcile(&fixture)?,
        reference_merged: yolox_tiles::merge(&reference_fixture)?,
        reference_result: yolox_vehicle_consistency::reconcile(&reference_fixture)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn label(id: usize, kind: &str) -> Label {
        Label {
            node_id: id,
            node_name: format!("n{id}"),
            operation: kind.into(),
            output_shapes: vec![],
            matrix_kernel_choices: vec![],
        }
    }
    #[test]
    fn accounts_for_node_calls_and_unattributed_time() {
        let labels = [label(0, "MatMul"), label(1, "Pack"), label(2, "MatMul")];
        let p = summarize(&labels, &[(0, 2, 22), (1, 25, 35), (2, 40, 80)], 100).unwrap();
        assert_eq!((p.operator_sum_ns, p.unattributed_ns), (70, 30));
        assert_eq!(
            p.by_operation,
            vec![
                Group {
                    operation: "MatMul".into(),
                    calls: 2,
                    elapsed_ns: 60
                },
                Group {
                    operation: "Pack".into(),
                    calls: 1,
                    elapsed_ns: 10
                }
            ]
        );
    }
    #[test]
    fn rejects_impossible_or_duplicated_observations() {
        let labels = [label(0, "A"), label(1, "B")];
        for samples in [
            vec![(0, 1, 3), (0, 4, 5)],
            vec![(0, 1, 4), (1, 3, 5)],
            vec![(0, 4, 3)],
            vec![(2, 1, 3)],
            vec![(0, 0, 11)],
        ] {
            assert!(summarize(&labels, &samples, 10).is_err());
        }
    }
    #[test]
    fn observer_executes_the_actual_graph_and_preserves_bits() {
        let mut model = TypedModel::default();
        let source = model.add_source("input", f32::fact([3])).unwrap();
        let output = model
            .wire_node(
                "absolute",
                tract_onnx::tract_core::ops::math::abs(),
                &[source],
            )
            .unwrap();
        model.set_output_outlets(&output).unwrap();
        let plan = model.into_runnable().unwrap();
        let input = Tensor::from_shape(&[3], &[-2.0_f32, -0.0, 3.0]).unwrap();
        let (actual, p) = observed(&plan, &labels(&plan).unwrap(), input.clone()).unwrap();
        let expected = plan.run(tvec!(input.into())).unwrap();
        assert!(same_bits(
            actual[0].as_slice::<f32>().unwrap(),
            expected[0].as_slice::<f32>().unwrap()
        ));
        assert_eq!(actual[0].as_slice::<f32>().unwrap(), &[2.0, 0.0, 3.0]);
        assert!(p.operators.iter().any(|r| r.label.node_name == "absolute"));
    }
    #[test]
    fn raw_comparison_detects_signed_zero_and_distinct_nan_payloads() {
        assert!(!same_bits(&[0.0], &[-0.0]));
        assert!(!same_bits(
            &[f32::from_bits(0x7fc0_0001)],
            &[f32::from_bits(0x7fc0_0002)]
        ));
        assert!(!same_bits(&[1.0], &[]));
    }
    #[test]
    fn input_uses_bgr_planes_and_bottom_114_padding() {
        let image = RgbImage::from_pixel(416, 1, image::Rgb([1, 2, 3]));
        let (input, ratio) = preprocess(&image).unwrap();
        let v = input.as_slice::<f32>().unwrap();
        assert_eq!(ratio, 1.0);
        for (channel, value) in [3.0, 2.0, 1.0].into_iter().enumerate() {
            let start = channel * 416 * 416;
            assert!(v[start..start + 416].iter().all(|x| *x == value));
            assert!(
                v[start + 416..start + 416 * 416]
                    .iter()
                    .all(|x| *x == 114.0)
            );
        }
    }
}
