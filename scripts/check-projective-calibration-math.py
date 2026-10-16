#!/usr/bin/env python3
import argparse
from pathlib import Path
import importlib.util,sys,json,types
import numpy as np
from projective_depth_common import ROOT,fresh,write
r=ROOT
p=argparse.ArgumentParser(description='Six-axis reference error and projected uncertainty finite-difference controls');p.add_argument('--output',type=Path,required=True);args=p.parse_args();out=fresh(args.output)
from projective_depth_common import exp_rotation
s=importlib.util.spec_from_file_location('cal',r/'scripts/check-projective-information-calibration.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
def p(t):return dict(rotation=t[:3,:3].tolist(),translation=t[:3,3].tolist())
key=np.eye(4);key[:3,:3]=exp_rotation(np.array([.1,-.15,.2]));key[:3,3]=[.4,-.3,.2]
t=np.eye(4);t[:3,:3]=exp_rotation(np.array([-.01,.02,-.03]));t[:3,3]=[.02,-.01,.03]
h=np.diag([1.,2.,3.,4.,5.,6.]);e=dict(h=h.tolist(),g=[0.]*6,count=200,rms_m=0.)
m.reference=lambda:types.SimpleNamespace(truth_at=lambda labels,stamp:[np.eye(4),key,key@t][int(stamp)])
for axis in range(6):
 delta=np.zeros(6);delta[axis]=.001;d=np.eye(4);d[:3,:3]=exp_rotation(delta[3:]);d[:3,3]=delta[:3];est=key@d@t
 rows=[]
 for i,g in enumerate([np.eye(4),key,est]):
  rows.append(dict(update=dict(timestamp=float(i),accepted=True,keyframe_replaced=i==1,first_from_current=p(g),rejection=None,
      iterations=[] if i==0 else [dict(equation=e,after=p(key if i==1 else d@t))],final_equation=None if i==0 else e)))
 got=m.samples(rows,[dict(depth_timestamp=float(i),source_index=i) for i in range(3)],None)[-1]
 expected=float(delta@h@delta/m.SIGMA_M**2);assert abs(got['nominal_nees']-expected)<1e-10
 j=np.zeros((3,6));rel=d@t
 for k in range(6):
  plus=np.eye(4);minus=np.eye(4)
  if k<3:plus[k,3]=1e-6;minus[k,3]=-1e-6
  else:
   w=np.zeros(3);w[k-3]=1e-6;plus[:3,:3]=exp_rotation(w);minus[:3,:3]=exp_rotation(-w)
  j[:,k]=((key@plus@rel)[:3,3]-(key@minus@rel)[:3,3])/2e-6
 cov=m.SIGMA_M**2*np.linalg.inv(h);radius=float(np.sqrt(m.CHI2_6_95*np.linalg.eigvalsh(j@cov@j.T)[-1]))
 assert abs(got['nominal_position_radius_m']-radius)<1e-10
write(out/'summary.json',dict(pass_calibration_math=True,pure_axes_checked=6,rotated_and_translated_reference=True,position_covariance_checked_with_finite_differences=True,new_heldout_data_read=False))
print('six-axis error transport and position covariance finite differences pass')
