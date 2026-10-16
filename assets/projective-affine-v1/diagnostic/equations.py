"""Independent measured-intensity equations for the fused-depth tracker."""
import numpy as np
from projective_depth_common import require


def gray_pyramid(raw):
    grids={1:raw.astype(float)}
    for scale in (2,4):
        a=grids[scale//2]
        grids[scale]=np.stack([a[dy::2,dx::2] for dy,dx in ((0,0),(0,1),(1,0),(1,1))]).mean(axis=0)
    return grids


def affine_system(jac,source,value):
    """Independent weighted least squares and orthogonal nuisance projection."""
    empty=dict(count=0,rms_gray=None,used=False,h=np.zeros((6,6)),g=np.zeros(6),illumination=None)
    if len(source)<200:return empty
    design=np.column_stack([source,np.ones(len(source))]);weights=np.ones(len(source))
    for iteration in range(4):
        total=weights.sum()
        if total<=0:return empty
        center=weights@source/total;variance=weights@((source-center)**2)/total
        if variance<25:return empty
        gain,offset=np.linalg.lstsq(design*np.sqrt(weights[:,None]),value*np.sqrt(weights),rcond=None)[0]
        residual=value-gain*source-offset
        weights=np.where(abs(residual)<30,(1-(residual/30)**2)**2,0.)
    valid=weights>0;count=int(valid.sum());total=weights.sum()
    if count<200 or not .5<=gain<=2 or abs(offset)>80 or total<=0:return empty
    center=weights@source/total;variance=weights@((source-center)**2)/total
    if variance<25:return empty
    # Remove the span of brightness gain/offset with NumPy's SVD least squares.
    # Rust instead accumulates a centered 2x2 Schur complement.
    weighted=jac*np.sqrt(weights[:,None]);nuisance=design*np.sqrt(weights[:,None])
    projected=weighted-nuisance@np.linalg.lstsq(nuisance,weighted,rcond=None)[0]
    return dict(count=count,rms_gray=float(np.sqrt(np.mean(residual[valid]**2))),used=True,
        h=projected.T@projected,g=projected.T@(np.sqrt(weights)*residual),
        illumination=dict(gain=float(gain),offset_gray=float(offset),candidate_count=len(source),source_variance_gray2=float(variance)))


def equation(target,source,target_gray,source_gray,transform,scale,enabled,affine=False):
    out=dict(count=0,rms_gray=None,used=False,scale_m2_per_gray2=.005**2/100,h=np.zeros((6,6)),g=np.zeros(6))
    if affine:out['illumination']=None
    if not enabled:return out
    stride=2 if scale==1 else 1
    xyz=source['vertices'][::stride,::stride].reshape(-1,3)
    normals=source['normals'][::stride,::stride].reshape(-1,3)
    color=source_gray[::stride,::stride].ravel()
    valid=np.sum(normals*normals,axis=1)>.25
    xyz,normals,color=xyz[valid],normals[valid],color[valid]
    q=xyz@transform[:3,:3].T+transform[:3,3];ns=normals@transform[:3,:3].T
    valid=np.isfinite(q).all(axis=1)&(q[:,2]>.3)
    q,ns,color=q[valid],ns[valid],color[valid]
    c=target['camera'];uf=c['fx']*q[:,0]/q[:,2]+c['cx'];vf=c['fy']*q[:,1]/q[:,2]+c['cy']
    u,v=np.rint(uf),np.rint(vf)
    valid=(u>=1)&(u<c['width']-1)&(v>=1)&(v<c['height']-1)
    q,ns,color,uf,vf=q[valid],ns[valid],color[valid],uf[valid],vf[valid]
    u,v=u[valid].astype(np.int64),v[valid].astype(np.int64)
    p=target['vertices'][v,u];n=target['normals'][v,u]
    valid=(np.sum(n*n,axis=1)>.25)&(np.linalg.norm(q-p,axis=1)<(.12 if scale==4 else .08))&(np.sum(ns*n,axis=1)>.8)
    q,color,uf,vf=q[valid],color[valid],uf[valid],vf[valid]
    uf=np.where(abs(uf-np.rint(uf))<=1e-10,np.rint(uf),uf)
    vf=np.where(abs(vf-np.rint(vf))<=1e-10,np.rint(vf),vf)
    valid=(uf>=0)&(uf<c['width']-1)&(vf>=0)&(vf<c['height']-1)
    q,color,uf,vf=q[valid],color[valid],uf[valid],vf[valid]
    u,v=np.floor(uf).astype(np.int64),np.floor(vf).astype(np.int64);dx,dy=uf-u,vf-v
    a,b,d,e=target_gray[v,u],target_gray[v,u+1],target_gray[v+1,u],target_gray[v+1,u+1]
    value=(1-dx)*(1-dy)*a+dx*(1-dy)*b+(1-dx)*dy*d+dx*dy*e
    gx=(1-dy)*(b-a)+dy*(e-d);gy=(1-dx)*(d-a)+dx*(e-b)
    r=value-color;valid=(np.hypot(gx,gy)>=1)&(True if affine else abs(r)<30)
    q,r,gx,gy,color,value=q[valid],r[valid],gx[valid],gy[valid],color[valid],value[valid];z=q[:,2]
    grad=np.column_stack([gx*c['fx']/z,gy*c['fy']/z,-(gx*c['fx']*q[:,0]+gy*c['fy']*q[:,1])/(z*z)])
    jac=np.column_stack([grad,np.cross(q,grad)]);w=(1-(r/30)**2)**2
    if affine:
        out.update(affine_system(jac,color,value))
        return out
    out.update(count=len(r),rms_gray=float(np.sqrt(np.mean(r*r))) if len(r) else None,used=len(r)>=200,
               h=jac.T@(w[:,None]*jac),g=jac.T@(w*r))
    return out


def compare(actual,expected):
    require(actual['count']==expected['count'] and actual['used']==expected['used']
            and actual['scale_m2_per_gray2']==expected['scale_m2_per_gray2'],'exact photometric support/usage/scale')
    if expected['rms_gray'] is None:require(actual['rms_gray'] is None,'no fabricated image residual')
    else:require(abs(actual['rms_gray']-expected['rms_gray'])<1e-9,'raw image RMS')
    if 'illumination' in expected:
        require('illumination' in actual,'explicit exposure model, including unavailable fits')
        a,e=actual['illumination'],expected['illumination']
        if e is None:require(a is None,'no fabricated exposure estimate')
        else:
            require(a is not None and a['candidate_count']==e['candidate_count'],'exposure fitting support')
            for key in ('gain','offset_gray','source_variance_gray2'):
                np.testing.assert_allclose(a[key],e[key],atol=1e-8,rtol=1e-10,err_msg='raw exposure '+key)
    np.testing.assert_allclose(actual['h'],expected['h'],atol=.001,rtol=1e-10,err_msg='raw photometric Hessian')
    np.testing.assert_allclose(actual['g'],expected['g'],atol=1e-5,rtol=1e-10,err_msg='raw photometric gradient')


def metadata(update,context):
    expected=dict(source_timestamp=context['source_timestamp'],reference_timestamp=context['reference_timestamp'],phase_qualified=context['qualified'],fresh=context['fresh'])
    require(update.get('photometric')==expected,'measured image reference/phase provenance')
