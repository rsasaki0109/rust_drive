"""Independent accepted-pose prediction: quaternion eigensolve, quadrature and matrix exp."""
import numpy as np
from projective_depth_common import exp_rotation,pose,require

NODES,WEIGHTS=np.polynomial.legendre.leggauss(8)


def log_body(transform):
    r=transform[:3,:3]
    k=np.array([[r[0,0]-r[1,1]-r[2,2],r[0,1]+r[1,0],r[0,2]+r[2,0],r[2,1]-r[1,2]],
        [r[0,1]+r[1,0],r[1,1]-r[0,0]-r[2,2],r[1,2]+r[2,1],r[0,2]-r[2,0]],
        [r[0,2]+r[2,0],r[1,2]+r[2,1],r[2,2]-r[0,0]-r[1,1],r[1,0]-r[0,1]],
        [r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1],np.trace(r)]])
    _,vectors=np.linalg.eigh(k);q=vectors[:,-1]
    if q[3]<0:q=-q
    sine=np.linalg.norm(q[:3]);angle=2*np.arctan2(sine,q[3])
    if angle>np.pi-1e-6:return None
    w=q[:3]*(angle/sine if sine>1e-15 else 2.)
    # Integrate the left Jacobian, then solve, instead of Rust's closed inverse.
    jac=sum(weight*.5*exp_rotation(w*((node+1)*.5)) for node,weight in zip(NODES,WEIGHTS))
    return np.r_[np.linalg.solve(jac,transform[:3,3]),w]


def exp_body(twist):
    v=twist[:3];x,y,z=twist[3:];a=np.zeros((4,4));a[:3,:3]=[[0,-z,y],[z,0,-x],[-y,x,0]];a[:3,3]=v
    count=max(0,int(np.ceil(np.log2(max(1.,np.linalg.norm(a,ord=np.inf)*2)))))
    a=a/(2.**count);result=np.eye(4);term=np.eye(4)
    for index in range(1,25):term=term@a/index;result+=term
    for _ in range(count):result=result@result
    return result


def predict(history,stamp):
    previous,latest=(history[-2] if len(history)>1 else None),(history[-1] if history else None)
    result=dict(status='bootstrap',previous_timestamp=previous[0] if previous else None,latest_timestamp=latest[0] if latest else None,
        history_interval_s=None,horizon_s=stamp-latest[0] if latest else None,first_from_prediction=None,latest_from_prediction=None)
    if previous is None or latest is None:return result
    dt=latest[0]-previous[0];result['history_interval_s']=dt
    twist=log_body(np.linalg.inv(previous[1])@latest[1])
    if twist is None:result['status']='ambiguous_rotation';return result
    scaled=twist*((stamp-latest[0])/dt);result['status']='increment_limit'
    if not np.isfinite(scaled).all() or np.linalg.norm(scaled[3:])>.15:return result
    increment=exp_body(scaled)
    if not np.isfinite(increment).all() or np.linalg.norm(increment[:3,3])>.1:return result
    result.update(status='predicted',first_from_prediction=latest[1]@increment,latest_from_prediction=increment)
    return result


def compare(actual,expected):
    require(actual.keys()==expected.keys(),'complete causal prediction metadata')
    for key in ('status','previous_timestamp','latest_timestamp','history_interval_s','horizon_s'):
        require(actual[key]==expected[key],'accepted-history prediction '+key)
    for key in ('first_from_prediction','latest_from_prediction'):
        if expected[key] is None:require(actual[key] is None,'no invented predicted pose')
        else:np.testing.assert_allclose(pose(actual[key]),expected[key],atol=1e-10,rtol=0,err_msg='independent body-twist prediction')
