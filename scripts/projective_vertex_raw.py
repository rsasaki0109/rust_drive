"""Independent NumPy reconstruction for the projective XYZ model; no Rust fitter."""
import numpy as np

from projective_depth_common import pose,require
from projective_depth_raw import compare_equation,normal_equation,vertices


def pyramid(points,camera):
    grids={1:points.copy()}
    for scale in (2,4):
        old=grids[scale//2]
        block=np.stack([old[dy::2,dx::2] for dy,dx in ((0,0),(0,1),(1,0),(1,1))])
        z=block[:,:,:,2];lo,hi=z.min(axis=0),z.max(axis=0)
        valid=(lo>=.3)&(hi<=5)&(hi-lo<=.05+.01*lo)
        grids[scale]=np.where(valid[:,:,None],block.mean(axis=0),0.)
    levels={}
    for scale,xyz in grids.items():
        c=dict(width=camera['width']//scale,height=camera['height']//scale,
               fx=camera['fx']/scale,fy=camera['fy']/scale,
               cx=(camera['cx']+.5)/scale-.5,cy=(camera['cy']+.5)/scale-.5)
        depth=xyz[:,:,2];center=depth[1:-1,1:-1]
        valid=(center>.3)&(center<5)
        for other in (depth[1:-1,2:],depth[1:-1,:-2],depth[2:,1:-1],depth[:-2,1:-1]):
            valid&=(other>.3)&(abs(other-center)<.05+.01*center)
        tangent_x=xyz[1:-1,2:]-xyz[1:-1,:-2]
        tangent_y=xyz[2:,1:-1]-xyz[:-2,1:-1]
        normal=np.zeros_like(xyz);cross=np.cross(tangent_x,tangent_y)
        length=np.linalg.norm(cross,axis=-1);valid&=length>1e-10
        normal[1:-1,1:-1][valid]=cross[valid]/length[valid,None]
        levels[scale]=dict(vertices=xyz,normals=normal,depth=depth,camera=c)
    return levels


def raster(points,weights,transform,camera):
    flat=points.reshape(-1,3);support=weights.ravel()
    indices=np.flatnonzero((flat[:,2]>.3)&(flat[:,2]<5))
    query=flat[indices]@transform[:3,:3].T+transform[:3,3]
    stats=dict(source_valid=len(indices),projected=0,selected=0)
    front=np.isfinite(query).all(axis=1)&(query[:,2]>.3)&(query[:,2]<5)
    query,indices=query[front],indices[front]
    x=np.rint(camera['fx']*query[:,0]/query[:,2]+camera['cx'])
    y=np.rint(camera['fy']*query[:,1]/query[:,2]+camera['cy'])
    inside=(x>=0)&(x<camera['width'])&(y>=0)&(y<camera['height'])
    cells=y[inside].astype(np.int64)*camera['width']+x[inside].astype(np.int64)
    query,indices=query[inside],indices[inside];stats['projected']=len(indices)
    order=np.lexsort((indices,query[:,2],cells))
    ordered_cells=cells[order]
    first=np.ones(len(order),dtype=bool);first[1:]=ordered_cells[1:]!=ordered_cells[:-1]
    selected=order[first];stats['selected']=len(selected)
    output=np.zeros_like(flat);new_weights=np.zeros_like(support)
    output[cells[selected]]=query[selected];new_weights[cells[selected]]=support[indices[selected]]
    return output.reshape(points.shape),new_weights.reshape(weights.shape),stats


def fuse(model,weights,source,transform,camera):
    projected,present,measurement=raster(source,np.ones(weights.shape,dtype=np.uint8),transform,camera)
    old,new=model[:,:,2],projected[:,:,2]
    valid=present>0;filled=valid&(weights==0)
    averaged=valid&(weights>0)&(abs(new-old)<.03+.01*old)
    replaced=valid&(weights>0)&~averaged&(new<old-.03-.01*old)
    ignored=valid&~filled&~averaged&~replaced
    output=model.copy();support=weights.copy();w=np.minimum(16,weights[averaged]).astype(float)
    output[averaged]=(w[:,None]*model[averaged]+projected[averaged])/(w[:,None]+1)
    support[averaged]=np.minimum(16,weights[averaged]+1)
    output[filled|replaced]=projected[filled|replaced];support[filled|replaced]=1
    counts=dict(warp=None,measurement=measurement,filled=int(filled.sum()),averaged=int(averaged.sum()),
                replaced=int(replaced.sum()),ignored=int(ignored.sum()))
    return output,support,counts


def reconstruct(model,weights,raw,camera,update):
    source=pyramid(vertices(raw,camera),camera)
    if model is None:
        require(update['accepted'] and not update['iterations'],'vertex initialization')
        require(np.sum(np.sum(source[4]['normals']**2,axis=-1)>.25)>=200,'vertex surface support')
        return source[1]['vertices'].copy(),(raw>.3).astype(np.uint8),dict(iterations=0,final_equations=0,model_replaced=False)
    if model.ndim==2:model=vertices(model,camera)  # Analytic first-frame input.
    target=pyramid(model,camera)
    for step in update['iterations']:
        scale=step['scale']
        compare_equation(step['equation'],normal_equation(target[scale],source[scale],pose(step['before']),scale))
    if not update['accepted']:
        require(update['fusion'] is None and update['first_from_current'] is None,'vertex rejection is atomic')
        return model,weights,dict(iterations=len(update['iterations']),final_equations=0,model_replaced=False)
    relative=pose(update['iterations'][-1]['after'])
    compare_equation(update['final_equation'],normal_equation(target[1],source[1],relative,1))
    warp=None
    if update['keyframe_replaced']:
        model,weights,warp=raster(model,weights,np.linalg.inv(relative),camera)
        relative=np.eye(4)
    model,weights,counts=fuse(model,weights,source[1]['vertices'],relative,camera)
    counts['warp']=warp;require(counts==update['fusion'],'exact vertex fusion partitions')
    return model,weights,dict(iterations=len(update['iterations']),final_equations=1,model_replaced=update['keyframe_replaced'])


def controls():
    from projective_depth_common import exp_rotation
    from projective_depth_raw import raster as depth_raster
    c=dict(width=128,height=96,fx=110.,fy=110.,cx=63.5,cy=47.5)
    n=np.array([.3,-.2,1.]);n/=np.linalg.norm(n)
    rays=vertices(np.ones((96,128)),c);depth=2/np.einsum('...i,i->...',rays,n)
    xyz=vertices(depth,c);original=xyz.copy();weights=np.ones(depth.shape,dtype=np.uint8)
    legacy=depth.copy();legacy_weight=weights.copy();previous=np.eye(4)
    peak_vertex_rms=peak_depth_rms=0.
    for index in [*range(1,21),*range(19,-1,-1)]:
        view=np.eye(4);view[:3,:3]=exp_rotation(np.array([.001,-.0015,.0007])*index)
        view[:3,3]=np.array([.004,-.001,.001])*index
        transform=np.linalg.inv(view)@previous
        xyz,weights,_=raster(xyz,weights,transform,c)
        legacy,legacy_weight,_=depth_raster(legacy,legacy_weight,transform,c)
        camera_normal=view[:3,:3].T@n;offset=2-n@view[:3,3]
        a=np.einsum('...i,i->...',xyz,camera_normal)[weights>0]-offset
        b=np.einsum('...i,i->...',vertices(legacy,c),camera_normal)[legacy_weight>0]-offset
        peak_vertex_rms=max(peak_vertex_rms,float(np.sqrt(np.mean(a*a))))
        peak_depth_rms=max(peak_depth_rms,float(np.sqrt(np.mean(b*b))))
        previous=view
    valid=weights>0;old_valid=legacy_weight>0
    error=np.einsum('...i,i->...',xyz,n)[valid]-2
    old_error=np.einsum('...i,i->...',vertices(legacy,c),n)[old_valid]-2
    require(valid.sum()>5000 and old_valid.sum()>5000,'retained bounded round-trip support')
    require(np.max(abs(error))<1e-12 and peak_vertex_rms<1e-12 and peak_depth_rms>1e-4,'rigid plane preservation witness')
    for level in pyramid(original,c).values():
        p=level['vertices'];valid=(p[:,:,2]>.3)&(p[:,:,2]<5)
        np.testing.assert_allclose(np.einsum('...i,i->...',p,n)[valid],2,atol=2e-14,rtol=0)
    points=np.zeros_like(original);support=np.zeros(depth.shape,dtype=np.uint8)
    p=original[48,64]+[.005,0,0]
    points[0,0]=points[0,1]=p;support[0,:2]=[2,7]
    selected,w,stats=raster(points,support,np.eye(4),c)
    require(stats==dict(source_valid=2,projected=2,selected=1) and w[48,64]==2,'XYZ exact index tie')
    np.testing.assert_array_equal(selected[48,64],p)
    points[0,2]=vertices(np.ones(depth.shape),c)[48,64];support[0,2]=4
    selected,w,stats=raster(points,support,np.eye(4),c)
    require(w[48,64]==4 and stats['selected']==1,'XYZ nearest depth occlusion')
    model=np.zeros_like(original);support[:]=0;source=model.copy()
    old=vertices(np.full(depth.shape,2.),c)[48,64]
    new=vertices(np.full(depth.shape,2.01),c)[48,64]+[.005,-.002,0]
    model[48,64]=old;support[48,64]=16;source[48,64]=new
    fused,w,stats=fuse(model,support,source,np.eye(4),c)
    np.testing.assert_allclose(fused[48,64],old+(new-old)/17,atol=5e-16,rtol=0)
    require(w[48,64]==16 and stats['averaged']==1,'XYZ bounded fusion still updates')
    return dict(warps=40,retained_vertex_points=int((weights>0).sum()),
                retained_depth_points=int((legacy_weight>0).sum()),
                vertex_plane_max_error_m=float(np.max(abs(error))),
                vertex_plane_rms_m=float(np.sqrt(np.mean(error**2))),
                legacy_depth_plane_rms_m=float(np.sqrt(np.mean(old_error**2))),
                peak_vertex_plane_rms_m=peak_vertex_rms,peak_depth_plane_rms_m=peak_depth_rms,
                xyz_pyramids_preserve_plane=True,diagnostic_only_no_physical_accuracy_claim=True)
