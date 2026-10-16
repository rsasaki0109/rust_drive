"""Independent fixed-measurement, frozen-weight nonlinear step reconstruction."""
import numpy as np
from projective_depth_common import exp_rotation,pose,require

SCALES=(1.,.5,.25,.125,.0625)


def sample(gray,camera,q):
    if np.any(q[:,2]<=.3):return None
    u=camera['fx']*q[:,0]/q[:,2]+camera['cx'];v=camera['fy']*q[:,1]/q[:,2]+camera['cy']
    u=np.where(abs(u-np.rint(u))<=1e-10,np.rint(u),u);v=np.where(abs(v-np.rint(v))<=1e-10,np.rint(v),v)
    if not (np.isfinite(u).all() and np.isfinite(v).all() and np.all((u>=0)&(u<gray.shape[1]-1)&(v>=0)&(v<gray.shape[0]-1))):return None
    x,y=np.floor(u).astype(np.int64),np.floor(v).astype(np.int64);dx,dy=u-x,v-y
    a,b,d,e=gray[y,x],gray[y,x+1],gray[y+1,x],gray[y+1,x+1]
    value=(1-dx)*(1-dy)*a+dx*(1-dy)*b+(1-dx)*dy*d+dx*dy*e
    gx=(1-dy)*(b-a)+dy*(e-d);gy=(1-dx)*(d-a)+dx*(e-b)
    return value,gx,gy


def witnesses(target,source,transform,scale,colors,photo):
    stride=2 if scale==1 else 1;c=target['camera']
    xyz=source['vertices'][::stride,::stride].reshape(-1,3);ns=source['normals'][::stride,::stride].reshape(-1,3)
    indices=np.arange(source['depth'].size).reshape(source['depth'].shape)[::stride,::stride].ravel()
    valid=np.sum(ns*ns,axis=1)>.25;xyz,ns,indices=xyz[valid],ns[valid],indices[valid]
    q=xyz@transform[:3,:3].T+transform[:3,3];ns=ns@transform[:3,:3].T
    valid=np.isfinite(q).all(axis=1)&(q[:,2]>.3);q,ns,indices=q[valid],ns[valid],indices[valid]
    u=np.rint(c['fx']*q[:,0]/q[:,2]+c['cx']);v=np.rint(c['fy']*q[:,1]/q[:,2]+c['cy'])
    valid=(u>=1)&(u<c['width']-1)&(v>=1)&(v<c['height']-1)
    q,ns,indices=q[valid],ns[valid],indices[valid];u,v=u[valid].astype(np.int64),v[valid].astype(np.int64)
    p=target['vertices'][v,u];n=target['normals'][v,u]
    valid=(np.sum(n*n,axis=1)>.25)&(np.linalg.norm(q-p,axis=1)<(.12 if scale==4 else .08))&(np.sum(ns*n,axis=1)>.8)
    q,p,n,indices=q[valid],p[valid],n[valid],indices[valid];r=np.einsum('ij,ij->i',n,q-p);w=np.minimum(1.,.01/np.maximum(abs(r),1e-15))
    image=None
    if photo is not None and photo['used']:
        require(colors is not None,'measured images for step witnesses')
        target_gray,source_gray=colors[0][scale],colors[1][scale]
        # Nearest geometry admission guarantees these coordinates are inside
        # the original bilinear image domain, before any candidate motion.
        value,gx,gy=sample(target_gray,c,q);color=source_gray.ravel()[indices]
        e=photo['illumination'];r=value-e['gain']*color-e['offset_gray']
        valid=(np.hypot(gx,gy)>=1)&(abs(r)<30);iw=(1-(r[valid]/30)**2)**2
        image=dict(query=q[valid],source=color[valid],weights=iw,gray=target_gray,scale=photo['scale_m2_per_gray2'])
    return dict(query=q,target=p,normal=n,weights=w,camera=c,image=image)


def cost(fixed,delta):
    rotation=exp_rotation(delta[3:]);translation=delta[:3]
    q=fixed['query']@rotation.T+translation
    if np.any(q[:,2]<=.3):return None
    residual=np.einsum('ij,ij->i',fixed['normal'],q-fixed['target'])
    value=.5*fixed['weights']@(residual*residual)
    image=fixed['image']
    if image is not None:
        warped=image['query']@rotation.T+translation;sampled=sample(image['gray'],fixed['camera'],warped)
        if sampled is None:return None
        values=sampled[0];sqrt=np.sqrt(image['weights']);design=np.column_stack([image['source'],np.ones(len(values))])
        gain_offset=np.linalg.lstsq(design*sqrt[:,None],values*sqrt,rcond=None)[0]
        residual=values-design@gain_offset
        value+=.5*image['scale']*(image['weights']@(residual*residual))
    require(np.isfinite(value),'finite fixed-support cost')
    return float(value)


def reconstruct(target,source,colors,step):
    fixed=witnesses(target,source,pose(step['before']),step['scale'],colors,step.get('photometric'))
    d=np.asarray(step['delta']);before=cost(fixed,np.zeros(6));trials=[];applied=0.
    for scale in SCALES:
        candidate=cost(fixed,d*scale);trials.append(candidate)
        if candidate is not None and candidate<=before+1e-12*(1+before):applied=scale;break
    return dict(geometric_count=len(fixed['query']),photometric_count=0 if fixed['image'] is None else len(fixed['image']['query']),before_cost_m2=before,trial_costs_m2=trials,applied_scale=applied)


def compare(actual,expected):
    require(actual.keys()==expected.keys(),'complete step report')
    for key in ('geometric_count','photometric_count','applied_scale'):require(actual[key]==expected[key],'raw step '+key)
    np.testing.assert_allclose(actual['before_cost_m2'],expected['before_cost_m2'],atol=1e-10,rtol=1e-9,err_msg='raw initial step cost')
    require(len(actual['trial_costs_m2'])==len(expected['trial_costs_m2']),'bounded, first successful trial')
    for actual_cost,expected_cost in zip(actual['trial_costs_m2'],expected['trial_costs_m2'],strict=True):
        if expected_cost is None:require(actual_cost is None,'no omitted trial image pixels')
        else:np.testing.assert_allclose(actual_cost,expected_cost,atol=1e-10,rtol=1e-9,err_msg='raw trial cost')
