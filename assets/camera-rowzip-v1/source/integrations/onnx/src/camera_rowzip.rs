//! Contiguous/strided slice traversal for the pinned safe depthwise rows.
use std::collections::BTreeMap;
use tract_onnx::{
    pb::ModelProto,
    prelude::*,
    tract_core::{internal::anyhow, plan},
};

pub type CpuPlan = SimplePlan<TypedFact, Box<dyn TypedOp>, TypedModel>;
#[derive(Clone, Debug)]
pub struct Spec {
    input: Vec<usize>,
    output: Vec<usize>,
    channels: usize,
    height: usize,
    width: usize,
    stride: usize,
}
pub type Bindings = BTreeMap<usize, Spec>;
const NAMES: [&str; 30] = [
    "Conv_44", "Conv_59", "Conv_70", "Conv_85", "Conv_95", "Conv_105", "Conv_116", "Conv_131",
    "Conv_141", "Conv_151", "Conv_162", "Conv_187", "Conv_212", "Conv_237", "Conv_315", "Conv_321",
    "Conv_302", "Conv_308", "Conv_247", "Conv_263", "Conv_348", "Conv_354", "Conv_335", "Conv_341",
    "Conv_273", "Conv_289", "Conv_381", "Conv_387", "Conv_368", "Conv_374",
];

// Fail closed on graph/schema changes; never infer convolution semantics from shapes alone.
pub fn bind(model: &TypedModel, proto: &ModelProto) -> Result<Bindings, String> {
    let graph = proto.graph.as_ref().ok_or("missing pinned graph")?;
    let nodes: Vec<_> = model
        .nodes()
        .iter()
        .filter(|n| n.op().name() == "DepthWiseConv")
        .collect();
    if nodes.len() != NAMES.len() {
        return Err("exactly thirty pinned depthwise nodes required".into());
    }
    let mut result = Bindings::new();
    for node in nodes {
        if !NAMES.contains(&node.name.as_str()) || node.inputs.len() != 3 || node.outputs.len() != 1
        {
            return Err("unexpected depthwise inventory or ports".into());
        }
        let source = graph
            .node
            .iter()
            .find(|n| n.name == node.name)
            .ok_or("original Conv missing")?;
        let ints = |name: &str| {
            source
                .attribute
                .iter()
                .find(|a| a.name == name && a.r#type == 7)
                .map(|a| a.ints.as_slice())
        };
        let group = source
            .attribute
            .iter()
            .find(|a| a.name == "group" && a.r#type == 2)
            .map(|a| a.i);
        if source.op_type != "Conv"
            || source.attribute.len() != 5
            || ints("kernel_shape") != Some([3, 3].as_slice())
            || ints("dilations") != Some([1, 1].as_slice())
            || ints("pads") != Some([1, 1, 1, 1].as_slice())
        {
            return Err("only explicit 3x3 unit-dilation pad-one convolution supported".into());
        }
        let stride = match ints("strides") {
            Some([1, 1]) => 1,
            Some([2, 2]) => 2,
            _ => return Err("only strides one or two supported".into()),
        };
        let facts: Vec<_> = node
            .inputs
            .iter()
            .map(|i| model.outlet_fact(*i).map_err(|e| e.to_string()))
            .collect::<Result<_, _>>()?;
        let shape = facts[0]
            .shape
            .as_concrete()
            .ok_or("static NCHW/CHW required")?;
        let chw = match shape {
            [1, c, h, w] | [c, h, w] => [*c, *h, *w],
            _ => return Err("single-image NCHW/CHW required".into()),
        };
        let [channels, height, width] = chw;
        if ![16, 32, 64, 128].contains(&channels)
            || ![13, 26, 52, 104, 208].contains(&height)
            || width != height
            || group != Some(channels as i64)
            || facts.iter().any(|f| f.datum_type != f32::datum_type())
            || node.outputs[0].fact.datum_type != f32::datum_type()
        {
            return Err("pinned finite f32 depthwise geometry required".into());
        }
        let output = node.outputs[0]
            .fact
            .shape
            .as_concrete()
            .ok_or("static output required")?;
        let mut expected = shape.to_vec();
        let len = expected.len();
        expected[len - 2] = height.div_ceil(stride);
        expected[len - 1] = width.div_ceil(stride);
        if output != expected
            || facts[1].shape.as_concrete() != Some([channels, 1, 9].as_slice())
            || facts[1]
                .konst
                .as_ref()
                .is_none_or(|k| k.strides() != [9, 9, 1])
            || facts[2]
                .shape
                .as_concrete()
                .map(|s| s.iter().product::<usize>())
                != Some(channels)
            || facts[2].konst.is_none()
        {
            return Err("original constant kernel/bias layout required".into());
        }
        for fact in &facts[1..] {
            if fact
                .konst
                .as_ref()
                .ok_or("constant required")?
                .as_slice::<f32>()
                .map_err(|e| e.to_string())?
                .iter()
                .any(|v| !v.is_finite())
            {
                return Err("finite constant parameters required".into());
            }
        }
        result.insert(
            node.id,
            Spec {
                input: shape.to_vec(),
                output: output.to_vec(),
                channels,
                height,
                width,
                stride,
            },
        );
    }
    if result.len() != NAMES.len() {
        return Err("duplicate depthwise binding".into());
    }
    Ok(result)
}

fn row<const STEP: usize>(
    input: &[f32],
    kernel: &[f32],
    bias: f32,
    height: usize,
    width: usize,
    output: &mut [f32],
) {
    let ow = width.div_ceil(STEP);
    for (y, dst) in output.chunks_exact_mut(ow).enumerate() {
        dst.fill(bias);
        // Interior x coordinates have three valid columns. Rows are independent.
        let left = 1usize;
        let right = ((width - 2) / STEP + 1).min(ow);
        for ky in 0..3 {
            let iy = y * STEP + ky;
            if iy == 0 || iy > height {
                continue;
            }
            let src = &input[(iy - 1) * width..iy * width];
            for kx in 0..3 {
                let k = kernel[ky * 3 + kx];
                if left < right {
                    let start = left * STEP + kx - 1;
                    let count = right - left;
                    let samples = &src[start..start + (count - 1) * STEP + 1];
                    for (value, &sample) in dst[left..right]
                        .iter_mut()
                        .zip(samples.iter().step_by(STEP))
                    {
                        *value += k * sample;
                    }
                }
                // Skip padded taps entirely, preserving the original addition order and signed zero.
                for x in [0, ow - 1] {
                    if x >= left && x < right || x == ow - 1 && x == 0 {
                        continue;
                    }
                    let ix = x * STEP + kx;
                    if ix > 0 && ix <= width {
                        dst[x] += k * src[ix - 1];
                    }
                }
            }
        }
    }
}
impl Spec {
    pub fn eval(&self, inputs: TVec<TValue>) -> TractResult<TVec<TValue>> {
        if inputs.len() != 3
            || inputs[0].shape() != self.input
            || inputs[1].shape() != [self.channels, 1, 9]
            || inputs[2].len() != self.channels
        {
            return Err(anyhow!("depthwise operands differ from bound graph"));
        }
        let input = inputs[0].as_slice::<f32>()?;
        let kernel = inputs[1].as_slice::<f32>()?;
        let bias = inputs[2].as_slice::<f32>()?;
        let volume = self.height * self.width;
        let plane = self.output[self.output.len() - 2] * self.output[self.output.len() - 1];
        let mut values = vec![0f32; self.channels * plane];
        for c in 0..self.channels {
            let image = &input[c * volume..(c + 1) * volume];
            let weights = &kernel[c * 9..(c + 1) * 9];
            let dst = &mut values[c * plane..(c + 1) * plane];
            if self.stride == 1 {
                row::<1>(image, weights, bias[c], self.height, self.width, dst);
            } else {
                row::<2>(image, weights, bias[c], self.height, self.width, dst);
            }
        }
        Ok(tvec!(
            Tensor::from_shape(&self.output, &values)?.into_tvalue()
        ))
    }
}
pub fn evaluate(plan: &CpuPlan, bindings: &Bindings, input: Tensor) -> TractResult<TVec<TValue>> {
    let mut calls = 0;
    let output = SimpleState::new(plan)?.run_plan_with_eval(
        tvec!(input.into()),
        |session, state, node, inputs| {
            if let Some(spec) = bindings.get(&node.id) {
                calls += 1;
                spec.eval(inputs)
            } else {
                plan::eval(session, state, node, inputs)
            }
        },
    )?;
    if calls != 30 {
        return Err(anyhow!("complete depthwise replacement execution required"));
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rows_match_independent_scalar_coordinates_for_both_strides_and_edges() {
        let mut seed = 17u32;
        let mut random = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed as i32 as f32) / 2147483648.
        };
        for size in [3usize, 4, 5, 13, 26, 52, 104, 208] {
            for stride in [1usize, 2] {
                let input: Vec<_> = (0..size * size).map(|_| random()).collect();
                let weights: Vec<_> = (0..9).map(|_| random()).collect();
                let bias = random();
                let n = size.div_ceil(stride);
                let mut actual = vec![0.; n * n];
                if stride == 1 {
                    row::<1>(&input, &weights, bias, size, size, &mut actual)
                } else {
                    row::<2>(&input, &weights, bias, size, size, &mut actual)
                }
                for y in 0..n {
                    for x in 0..n {
                        let mut sum = bias;
                        for ky in 0..3 {
                            for kx in 0..3 {
                                let iy = (y * stride + ky) as isize - 1;
                                let ix = (x * stride + kx) as isize - 1;
                                if iy >= 0 && ix >= 0 && iy < size as isize && ix < size as isize {
                                    sum += weights[ky * 3 + kx]
                                        * input[iy as usize * size + ix as usize];
                                }
                            }
                        }
                        assert_eq!(
                            sum.to_bits(),
                            actual[y * n + x].to_bits(),
                            "size={size} stride={stride} ({y},{x})"
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn signed_zero_and_subnormal_taps_do_not_add_padded_zeros() {
        for bit in [0, 0x80000000, 1, 0x80000001, 0x3f800001] {
            let input = vec![f32::from_bits(bit); 9];
            let weights = vec![-0.0; 9];
            let mut output = vec![0.; 9];
            row::<1>(&input, &weights, -0.0, 3, 3, &mut output);
            assert!(
                output
                    .iter()
                    .all(|v| v.to_bits() == 0x80000000 || bit == 0x80000000 || bit == 0x80000001)
            );
        }
    }
}

#[cfg(test)]
mod zip_contract_tests {
    use super::*;
    #[test]
    fn traversal_preserves_each_finite_float_bit_across_lane_boundaries() {
        let bits = [
            0, 0x80000000, 1, 0x80000001, 0x3f800001, 0xbf800001, 0x33800000, 0xb3800000,
        ];
        for size in [3usize, 4, 5, 7, 8, 9, 13, 26, 52, 104, 208] {
            for step in [1usize, 2] {
                let input: Vec<_> = (0..size * size)
                    .map(|i| f32::from_bits(bits[(i * 3) % bits.len()]))
                    .collect();
                let kernel: Vec<_> = (0..9)
                    .map(|i| f32::from_bits(bits[(i * 5 + 2) % bits.len()]))
                    .collect();
                let n = size.div_ceil(step);
                let mut output = vec![0.; n * n];
                if step == 1 {
                    row::<1>(&input, &kernel, -0.0, size, size, &mut output)
                } else {
                    row::<2>(&input, &kernel, -0.0, size, size, &mut output)
                }
                for y in 0..n {
                    for x in 0..n {
                        let mut expected = -0.0;
                        for ky in 0..3 {
                            for kx in 0..3 {
                                let iy = (y * step + ky) as isize - 1;
                                let ix = (x * step + kx) as isize - 1;
                                if iy >= 0 && ix >= 0 && iy < size as isize && ix < size as isize {
                                    expected += kernel[ky * 3 + kx]
                                        * input[iy as usize * size + ix as usize];
                                }
                            }
                        }
                        assert_eq!(expected.to_bits(), output[y * n + x].to_bits());
                    }
                }
            }
        }
    }
}
