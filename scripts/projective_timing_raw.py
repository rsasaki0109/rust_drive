"""Independent causal exposure warps from accepted depth poses, never pose labels."""
import numpy as np
from projective_depth_common import pose,require
from projective_motion_raw import log_body,exp_body


def warp(depth,image,interval):
    out=dict(depth_timestamp=depth,image_timestamp=image,
        previous_timestamp=interval[0][0] if interval else None,
        latest_timestamp=interval[1][0] if interval else None,
        status='unavailable',image_from_depth=None)
    if depth==image:
        out.update(status='coincident',image_from_depth=np.eye(4));return out
    if interval is None:return out
    (a,first),(b,last)=interval
    twist=log_body(np.linalg.inv(first)@last)
    if twist is None:return out
    scaled=twist*((depth-image)/(b-a));out['status']='increment_limit'
    if not np.isfinite(scaled).all() or np.linalg.norm(scaled[3:])>.15:return out
    result=exp_body(scaled)
    if np.isfinite(result).all() and np.linalg.norm(result[:3,3])<=.1:
        out.update(status='compensated',image_from_depth=result)
    return out


def compare_warp(actual,expected):
    require(actual.keys()==expected.keys(),'complete temporal warp provenance')
    for key in expected:
        if key=='image_from_depth':
            if expected[key] is None:require(actual[key] is None,'no fabricated time warp')
            else:np.testing.assert_allclose(pose(actual[key]),expected[key],atol=1e-12,rtol=0,err_msg='independent exposure time warp')
        else:require(actual[key]==expected[key],'causal exposure provenance '+key+': '+repr(actual[key])+' != '+repr(expected[key]))


class TimingHistory:
    def __init__(self):
        self.accepted=[];self.reference_depth=None;self.reference_interval=None

    def augment(self,context,stamp):
        source=warp(stamp,context['source_timestamp'],self.accepted if len(self.accepted)==2 else None)
        reference=None if context['reference_timestamp'] is None else warp(self.reference_depth,context['reference_timestamp'],self.reference_interval)
        context['timing']=dict(source=source,reference=reference)
        context['qualified']=context['target'] is not None and source['image_from_depth'] is not None and reference is not None and reference['image_from_depth'] is not None
        context['warps']=None if not context['qualified'] else (reference['image_from_depth'],source['image_from_depth'])
        return context

    def observe(self,update):
        if not update['accepted']:return
        stamp=update['timestamp']
        if self.reference_depth is None or update['keyframe_replaced']:
            self.reference_depth=stamp;self.reference_interval=None
        self.accepted=(self.accepted+[(stamp,pose(update['first_from_current']))])[-2:]
        if len(self.accepted)==2 and (self.reference_depth==stamp or (self.reference_interval is None and self.reference_depth==self.accepted[0][0])):
            self.reference_interval=list(self.accepted)
