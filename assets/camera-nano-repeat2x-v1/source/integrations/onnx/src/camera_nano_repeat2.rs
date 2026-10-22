//! Narrow, validated lowering of this pinned model's two nearest 2x resizes.
use tract_onnx::{
    pb::NodeProto,
    prelude::*,
    tract_core::ops::{array::MultiBroadcastTo, change_axes::AxisOp},
};

fn semantics(node: &NodeProto) -> Result<usize, String> {
    let attr = |name: &str| {
        node.attribute
            .iter()
            .find(|a| a.name == name)
            .map(|a| a.s.as_slice())
    };
    if node.op_type != "Resize"
        || attr("mode") != Some(b"nearest".as_slice())
        || attr("coordinate_transformation_mode") != Some(b"asymmetric".as_slice())
        || attr("nearest_mode") != Some(b"floor".as_slice())
        || node.attribute.iter().any(|a| {
            !matches!(
                a.name.as_str(),
                "mode" | "coordinate_transformation_mode" | "nearest_mode" | "cubic_coeff_a"
            )
        })
        || node.input.len() < 3
        || node.input[0].is_empty()
        || node.input[2].is_empty()
        || node.input.iter().skip(3).any(|s| !s.is_empty())
    {
        return Err("only explicit asymmetric/floor/nearest scales are supported".into());
    }
    Ok(node.input[..2].iter().filter(|s| !s.is_empty()).count())
}

fn repeat2(
    model: &mut TypedModel,
    input: OutletId,
    name: &str,
    c: usize,
    h: usize,
    w: usize,
) -> TractResult<OutletId> {
    let x = model.wire_node(format!("{name}.height_axis"), AxisOp::Add(3), &[input])?[0];
    let x = model.wire_node(format!("{name}.width_axis"), AxisOp::Add(5), &[x])?[0];
    let x = model.wire_node(
        format!("{name}.repeat2"),
        MultiBroadcastTo {
            shape: tvec!(
                1.to_dim(),
                c.to_dim(),
                h.to_dim(),
                2.to_dim(),
                w.to_dim(),
                2.to_dim()
            )
            .into(),
        },
        &[x],
    )?[0];
    let x = model.wire_node(
        format!("{name}.merge_height"),
        AxisOp::Reshape(2, tvec!(h.to_dim(), 2.to_dim()), tvec!((h * 2).to_dim())),
        &[x],
    )?[0];
    Ok(model.wire_node(
        format!("{name}.merge_width"),
        AxisOp::Reshape(3, tvec!(w.to_dim(), 2.to_dim()), tvec!((w * 2).to_dim())),
        &[x],
    )?[0])
}

pub fn rewrite(
    model: &mut TypedModel,
    proto: &tract_onnx::pb::ModelProto,
) -> Result<usize, String> {
    let graph = proto.graph.as_ref().ok_or("missing original graph")?;
    let resizing: Vec<_> = graph
        .node
        .iter()
        .filter(|n| n.op_type == "Resize")
        .collect();
    if resizing.len() != 2 {
        return Err("exactly two pinned Resize nodes required".into());
    }
    for (name, c, h, w) in [("Resize_201", 128, 13, 13), ("Resize_226", 64, 26, 26)] {
        let source = resizing
            .iter()
            .find(|n| n.name == name)
            .ok_or("pinned Resize node missing")?;
        let scale_port = semantics(source)?;
        let id = model
            .nodes()
            .iter()
            .find(|n| n.name == name && n.op().name() == "Resize")
            .ok_or("optimized Resize node missing")?
            .id;
        let node = model.node(id);
        let input = model
            .outlet_fact(node.inputs[0])
            .map_err(|e| e.to_string())?;
        let scales = model
            .outlet_fact(*node.inputs.get(scale_port).ok_or("scale port missing")?)
            .map_err(|e| e.to_string())?;
        if input.datum_type != f32::datum_type()
            || input.shape.as_concrete() != Some([1, c, h, w].as_slice())
            || node.outputs.len() != 1
            || node.outputs[0].fact.shape.as_concrete() != Some([1, c, h * 2, w * 2].as_slice())
            || scales.konst.as_ref().and_then(|s| s.as_slice::<f32>().ok())
                != Some([1.0, 1.0, 2.0, 2.0].as_slice())
        {
            return Err("static f32 shape and exact [1,1,2,2] scales required".into());
        }
        if scale_port == 2 {
            let roi = model
                .outlet_fact(node.inputs[1])
                .map_err(|e| e.to_string())?;
            if roi.konst.as_ref().is_none_or(|v| v.len() != 0) {
                return Err("only absent or empty ROI supported".into());
            }
        }
        let mut patch = TypedModelPatch::default();
        let tap = patch
            .tap_model(model, node.inputs[0])
            .map_err(|e| e.to_string())?;
        let wire = repeat2(&mut patch.model, tap, name, c, h, w).map_err(|e| e.to_string())?;
        patch
            .shunt_outside(model, OutletId::new(id, 0), wire)
            .map_err(|e| e.to_string())?;
        patch.obliterate(id).map_err(|e| e.to_string())?;
        patch.apply(model).map_err(|e| e.to_string())?;
    }
    Ok(2)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn repeat_matches_independent_coordinate_oracle_for_all_float_bits() {
        let bits = [
            0,
            0x8000_0000,
            0x7fc0_0001,
            0x7fc0_0002,
            0x7f80_0000,
            0xff80_0000,
            1,
            0x7f7f_ffff,
        ];
        for (c, h, w) in [(1, 1, 1), (2, 2, 3), (3, 3, 5)] {
            let input: Vec<f32> = (0..c * h * w)
                .map(|i| f32::from_bits(bits[i % bits.len()]))
                .collect();
            let mut model = TypedModel::default();
            let source = model.add_source("input", f32::fact([1, c, h, w])).unwrap();
            let output = repeat2(&mut model, source, "nearest", c, h, w).unwrap();
            model.set_output_outlets(&[output]).unwrap();
            let plan = model.into_runnable().unwrap();
            let actual = plan
                .run(tvec!(
                    Tensor::from_shape(&[1, c, h, w], &input).unwrap().into()
                ))
                .unwrap();
            let actual = actual[0].as_slice::<f32>().unwrap();
            for channel in 0..c {
                for y in 0..h * 2 {
                    for x in 0..w * 2 {
                        let source = channel * h * w + (y / 2) * w + x / 2;
                        let target = channel * h * 2 * w * 2 + y * w * 2 + x;
                        assert_eq!(actual[target].to_bits(), input[source].to_bits());
                    }
                }
            }
        }
    }
    fn node() -> NodeProto {
        NodeProto {
            op_type: "Resize".into(),
            input: vec!["data".into(), "".into(), "scales".into()],
            attribute: [
                ("mode", "nearest"),
                ("coordinate_transformation_mode", "asymmetric"),
                ("nearest_mode", "floor"),
            ]
            .into_iter()
            .map(|(name, value)| tract_onnx::pb::AttributeProto {
                name: name.into(),
                s: value.as_bytes().to_vec(),
                r#type: 3,
                ..Default::default()
            })
            .collect(),
            ..Default::default()
        }
    }
    #[test]
    fn rejects_other_interpolation_coordinates_sizes_and_axes() {
        assert_eq!(semantics(&node()).unwrap(), 1);
        for (attr, value) in [
            ("mode", "linear"),
            ("coordinate_transformation_mode", "half_pixel"),
            ("nearest_mode", "ceil"),
        ] {
            let mut n = node();
            n.attribute.iter_mut().find(|a| a.name == attr).unwrap().s = value.as_bytes().to_vec();
            assert!(semantics(&n).is_err());
        }
        let mut n = node();
        n.input.push("sizes".into());
        assert!(semantics(&n).is_err());
        let mut n = node();
        n.attribute.push(tract_onnx::pb::AttributeProto {
            name: "axes".into(),
            ints: vec![2, 3],
            ..Default::default()
        });
        assert!(semantics(&n).is_err());
    }
    #[test]
    fn incomplete_original_graph_cannot_be_rewritten() {
        assert!(
            rewrite(
                &mut TypedModel::default(),
                &tract_onnx::pb::ModelProto::default()
            )
            .is_err()
        );
    }
}
