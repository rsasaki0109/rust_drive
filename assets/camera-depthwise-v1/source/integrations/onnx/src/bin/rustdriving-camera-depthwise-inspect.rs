//! No-forward inspection of the pinned Nano graph and depthwise operands.
use std::{env, io::Cursor, path::Path};
#[allow(dead_code)]
#[path = "../camera_depthwise.rs"]
mod camera_depthwise;
use tract_onnx::prelude::*;
use tract_onnx::tract_core::internal::anyhow;
fn main() -> TractResult<()> {
    let path = env::args()
        .nth(1)
        .ok_or_else(|| anyhow!("expected pinned Nano model"))?;
    let bytes =
        rustdriving_camera_detect::verify_model(Path::new(&path)).map_err(|e| anyhow!(e))?;
    let onnx = tract_onnx::onnx();
    let proto = onnx.proto_model_for_read(&mut Cursor::new(bytes))?;
    let model = onnx
        .model_for_proto_model(&proto)?
        .with_input_fact(0, f32::fact([1, 3, 416, 416]).into())?
        .into_optimized()?;
    let _ = camera_depthwise::bind(&model, &proto).map_err(|e| anyhow!(e))?;
    for node in model
        .nodes()
        .iter()
        .filter(|n| n.op().name() == "DepthWiseConv")
    {
        let original = proto
            .graph
            .as_ref()
            .unwrap()
            .node
            .iter()
            .find(|n| n.name == node.name);
        let inputs=node.inputs.iter().map(|i|{
            let f=model.outlet_fact(*i).unwrap();
            serde_json::json!({"shape":f.shape.iter().map(ToString::to_string).collect::<Vec<_>>(),"constant":f.konst.is_some(),"strides":f.konst.as_ref().map(|k|k.strides().to_vec())})
        }).collect::<Vec<_>>();
        println!(
            "{}",
            serde_json::json!({"id":node.id,"name":node.name,"inputs":inputs,
         "output":node.outputs[0].fact.shape.iter().map(ToString::to_string).collect::<Vec<_>>(),
         "original":original.map(|n|serde_json::json!({"op":n.op_type,"attributes":n.attribute.iter().map(|a|(a.name.clone(),serde_json::json!({"i":a.i,"ints":a.ints,"s":a.s,"type":a.r#type}))).collect::<std::collections::BTreeMap<_,_>>()}))})
        );
    }
    Ok(())
}
