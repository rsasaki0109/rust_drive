//! Label-free selection over measured XYZ components. No point-cloud evidence is erased.
use rustdriving_core::Vec3;
use rustdriving_perception::{ObjectAabb, ComponentSelectionConfig, select_components};
use serde::{Deserialize,Serialize};
use std::{collections::HashSet,fs::File,io::{Read,Write},path::Path};
const MAX_INPUT:usize=128*1024*1024;
#[derive(Clone,Copy,Deserialize,Serialize)]
#[serde(deny_unknown_fields)]
struct Common {
    range_breaks_m:[f64;2],max_center_height_gap_m:f64,
    radius_padding_m:f64,min_proxy_radius_m:f64,max_proxy_radius_m:f64,
    max_components:usize,max_return_indices:usize,max_candidate_work:usize,coordinate_bound_m:f64,
}
#[derive(Clone,Deserialize,Serialize)]
#[serde(deny_unknown_fields)]
struct Policy { id:String,min_points_by_range:[usize;3],overlap_threshold:Option<f64> }
impl Common {
    fn config(self,policy:&Policy)->ComponentSelectionConfig {
        ComponentSelectionConfig { min_points_by_range:policy.min_points_by_range,
            range_breaks_m:self.range_breaks_m,overlap_threshold:policy.overlap_threshold,
            max_center_height_gap_m:self.max_center_height_gap_m,radius_padding_m:self.radius_padding_m,
            min_proxy_radius_m:self.min_proxy_radius_m,max_proxy_radius_m:self.max_proxy_radius_m,
            max_components:self.max_components,max_return_indices:self.max_return_indices,
            max_candidate_work:self.max_candidate_work,coordinate_bound_m:self.coordinate_bound_m }
    }
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Candidate { min:Vec3,max:Vec3,point_count:usize,indices:Vec<usize> }
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Frame { id:String,data_point_count:usize,candidates:Vec<Candidate> }
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Input { schema_version:u32,common:Common,policies:Vec<Policy>,frames:Vec<Frame> }
#[derive(Serialize)]
struct Decision {
    id:String,kept_indices:Vec<usize>,low_support_indices:Vec<usize>,
    suppressed:Vec<[usize;2]>,candidate_work:usize,
}
#[derive(Serialize)]
struct FrameResult { id:String,decisions:Vec<Decision> }
#[derive(Serialize)]
struct Report { schema_version:u32,scope:&'static str,common:Common,policies:Vec<Policy>,frames:Vec<FrameResult> }
fn evaluate(input:Input)->Result<Report,String> {
    if input.schema_version!=1 || input.policies.is_empty() || input.policies.len()>32
        || input.frames.is_empty() || input.frames.len()>128 {
        return Err("bounded nonempty component-selection frames and policies required".into());
    }
    let mut ids=HashSet::new();
    for policy in &input.policies {
        if policy.id.is_empty() || policy.id.len()>40 || !ids.insert(&policy.id)
            || !policy.id.bytes().all(|c|c.is_ascii_alphanumeric()||c==b'-'||c==b'_') {
            return Err("invalid or duplicate policy identity".into());
        }
        input.common.config(policy).validate()?;
    }
    let mut ids=HashSet::new();let mut frames=Vec::new();
    for frame in input.frames {
        if frame.id.is_empty() || frame.id.len()>160 || !ids.insert(frame.id.clone())
            || frame.data_point_count>input.common.max_return_indices {
            return Err("invalid frame identity or return budget".into());
        }
        let mut objects=Vec::new();
        for c in frame.candidates {
            if c.indices.iter().any(|&i|i>=frame.data_point_count) {
                return Err("component ordinal outside measured frame".into());
            }
            objects.push(ObjectAabb { center:Vec3::new((c.min.x+c.max.x)*0.5,(c.min.y+c.max.y)*0.5,(c.min.z+c.max.z)*0.5),
                min:c.min,max:c.max,point_count:c.point_count,indices:c.indices });
        }
        let mut decisions=Vec::new();
        for policy in &input.policies {
            let selection=select_components(&objects,&input.common.config(policy))?;
            decisions.push(Decision { id:policy.id.clone(),kept_indices:selection.kept_indices,
                low_support_indices:selection.low_support_indices,
                suppressed:selection.suppressed.into_iter().map(|s|[s.candidate,s.representative]).collect(),
                candidate_work:selection.candidate_work });
        }
        frames.push(FrameResult { id:frame.id,decisions });
    }
    Ok(Report { schema_version:1,
        scope:"Measured-component hypothesis selection only. Withheld components and all source returns remain evidence. Support is not calibrated confidence; no semantic classification, free-space certificate, weather model or driving qualification.",
        common:input.common,policies:input.policies,frames })
}
fn main()->Result<(),Box<dyn std::error::Error>> {
    let args:Vec<_>=std::env::args().collect();
    if args.len()!=2 {return Err("usage: cadc_component_selection INPUT.json".into());}
    let file=File::open(Path::new(&args[1]))?;
    if !file.metadata()?.is_file(){return Err("regular input file required".into());}
    let mut bytes=Vec::new();file.take(MAX_INPUT as u64+1).read_to_end(&mut bytes)?;
    if bytes.len()>MAX_INPUT{return Err("selection input exceeds byte budget".into());}
    let report=evaluate(serde_json::from_slice(&bytes)?)?;
    let mut writer=std::io::BufWriter::new(std::io::stdout().lock());
    serde_json::to_writer(&mut writer,&report)?;writer.write_all(b"\n")?;Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    fn input()->Input {
        serde_json::from_str(r#"{
            "schema_version":1,
            "common":{"range_breaks_m":[15,30],"max_center_height_gap_m":0.5,
              "radius_padding_m":0.65,"min_proxy_radius_m":0.85,"max_proxy_radius_m":3,
              "max_components":10000,"max_return_indices":500000,"max_candidate_work":2000000,"coordinate_bound_m":10000000},
            "policies":[{"id":"raw","min_points_by_range":[3,3,3],"overlap_threshold":null}],
            "frames":[{"id":"empty","data_point_count":0,"candidates":[]}]
        }"#).unwrap()
    }
    #[test]
    fn empty_components_remain_empty_hypotheses_without_clearance_claim() {
        let report=evaluate(input()).unwrap();
        assert!(report.frames[0].decisions[0].kept_indices.is_empty());
    }
    #[test]
    fn policy_and_frame_bounds_are_checked_before_output() {
        let mut value=input();value.policies.push(value.policies[0].clone());
        assert!(evaluate(value).is_err());
        let mut value=input();value.frames[0].data_point_count=500001;
        assert!(evaluate(value).is_err());
        let mut value=input();value.frames[0].data_point_count=3;
        value.frames[0].candidates.push(Candidate { min:Vec3::default(),max:Vec3::default(),point_count:3,indices:vec![0,1,3] });
        assert!(evaluate(value).is_err());
    }
    #[test]
    fn label_metadata_is_not_a_selection_input() {
        assert!(serde_json::from_str::<Frame>(r#"{"id":"x","data_point_count":0,"candidates":[],"cuboids":[]}"#).is_err());
        assert!(serde_json::from_str::<Policy>(r#"{"id":"x","min_points_by_range":[3,3,3],"overlap_threshold":null,"weather":"snow"}"#).is_err());
    }
}
