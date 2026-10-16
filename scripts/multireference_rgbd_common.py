"""Independent checks for the optional raw-keyframe experiment, without fitter imports."""
import subprocess

import numpy as np
from projective_depth_common import Client, audit_iterations, pose, require
from projective_depth_raw import pyramid


class MultiClient(Client):
    def __init__(self, binary, camera, mode):
        self.process = subprocess.Popen([str(binary.absolute())],stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            require(self.send(dict(kind='Configure',camera=camera,mode=mode))
                    == dict(configured=True,schema_version=1),'versioned multi-reference configuration')
        except BaseException:
            self.abort()
            raise

    def observation(self, stamp, depth, image=None):
        return self.send(dict(kind='Frame',timestamp=stamp,depth_m=depth.ravel().tolist(),image=image))


def audit_equation(e, references, phases, current_phase, enabled):
    support,photo = e['reference_support'],e['reference_photo_support']
    require(len(support) == len(photo) == len(references) <= 5,'bounded reference partition')
    require(all(type(n) is int and 0 <= n <= 19200 for n in support),'bounded geometric samples')
    require(all(type(p) is int and (p == 0 or 200 <= p <= n) for n,p in zip(support,photo)),
            'qualified image support is a geometric subset')
    require(sum(support) == e['geometry']['count'] and sum(photo) == e['photometric_count'],
            'complete per-reference support accounting')
    for stamp,p in zip(references,photo):
        require(p == 0 or (enabled and abs(current_phase-phases[stamp]) <= .005),
                'repeated/phase-mismatched image cannot contribute')
    if not sum(photo):
        require(e['photometric_rms'] is None,'no fabricated image residual')
        np.testing.assert_allclose(e['h'],np.asarray(e['geometry']['h'])/.005**2,atol=1e-6,rtol=1e-12)
        np.testing.assert_allclose(e['g'],np.asarray(e['geometry']['g'])/.005**2,atol=1e-7,rtol=1e-12)
    else:
        require(0 <= e['photometric_rms'] < 30,'finite bounded image residual')
    require(e['geometry']['count'] >= 200 and 0 <= e['geometry']['rms_m'] <= 1,
            'supported geometric equation')


def audit_sequence(rows, frames, mode):
    require(len(rows) == len(frames) <= 200,'all acquisitions accounted for')
    keys = []
    poses,phases = {},{}
    latest = last_rgb = None
    lost = False
    checked = 0
    for row,frame in zip(rows,frames):
        u = row['update']
        stamp,rgb = frame['depth_timestamp'],frame.get('rgb_timestamp',0.)
        phase = rgb-stamp
        require(u['timestamp'] == stamp,'original acquisition timestamp')
        if latest is not None:
            lost |= stamp-latest > .2
        require(u['lost'] == lost and not (lost and u['accepted']),'terminal acquisition expiry')
        references = list(keys)
        if latest is not None and latest not in references:
            references.append(latest)
        require(u['reference_timestamps'] == (references if latest is not None and not lost else []),
                'first plus newest motion keys plus last accepted, without duplicates')
        enabled = mode == 'phase_gated_rgbd' and (last_rgb is None or rgb > last_rgb)
        if mode == 'phase_gated_rgbd':
            last_rgb = rgb
        require(u['photo_enabled'] == (enabled and latest is not None and not lost),
                'observed RGB freshness')
        for step in u['iterations']:
            audit_equation(step['equation'],references,phases,phase,enabled)
        # Reuse the separate NumPy solve/Rodrigues checker with the combined system.
        adapted = dict(u,accepted=False)
        adapted['iterations'] = [dict(s,equation=dict(s['equation'],
            count=s['equation']['geometry']['count'],rms_m=s['equation']['geometry']['rms_m']))
            for s in u['iterations']]
        checked += audit_iterations(adapted)
        if u['iterations']:
            np.testing.assert_allclose(pose(u['iterations'][0]['before']),poses[latest],atol=1e-11,rtol=0)
        if u['accepted']:
            root = pose(u['first_from_current'])
            if latest is None:
                require(not u['iterations'] and not u['keyframe_replaced'],'one cold initialization')
                np.testing.assert_allclose(root,np.eye(4),atol=0,rtol=0)
                keys = [stamp]
            else:
                require(bool(u['iterations']),'accepted motion requires fitting')
                np.testing.assert_allclose(root,pose(u['iterations'][-1]['after']),atol=1e-11,rtol=0)
                require(np.linalg.norm((np.linalg.inv(poses[latest])@root)[:3,3]) <= .5,
                        'last accepted displacement gate')
                e = u['final_equation']
                audit_equation(e,references,phases,phase,enabled)
                require(e['geometry']['rms_m'] <= .03,'returned-pose geometric gate')
                delta = np.linalg.inv(poses[keys[-1]])@root
                angle = np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))
                replace = np.linalg.norm(delta[:3,3]) > .1 or angle > .1
                require(u['keyframe_replaced'] == replace,'motion-selected keyframe threshold')
                if replace:
                    keys = [keys[0],*keys[1:][-2:],stamp]
            latest = stamp
            poses[stamp],phases[stamp] = root,phase
        else:
            require(u['first_from_current'] is None and u['final_equation'] is None
                    and not u['keyframe_replaced'],'rejection cannot commit')
        require(u['last_accepted_timestamp'] == latest,'accepted clock changes only on success')
    return checked


def raw_reference(depth, gray, camera, root, phase):
    images = None
    if gray is not None:
        images = {1:gray.astype(float)}
        for scale in (2,4):
            a = images[scale//2]
            images[scale] = sum(a[dy::2,dx::2] for dy,dx in ((0,0),(0,1),(1,0),(1,1)))/4
    return dict(surface=pyramid(depth,camera),gray=images,root=root,phase=phase)


def raw_equation(references, source, transform, scale, enabled):
    """Reconstruct raw correspondences and both Jacobians using vectorized NumPy."""
    stride = {4:1,2:2,1:4}[scale]
    layer = source['surface'][scale]
    src = layer['vertices'][::stride,::stride].reshape(-1,3)
    sn = layer['normals'][::stride,::stride].reshape(-1,3)
    keep = np.sum(sn*sn,axis=1) > .25
    src,sn = src[keep],sn[keep]
    intensity = source['gray'][scale][::stride,::stride].ravel()[keep] if source['gray'] else None
    q = src@transform[:3,:3].T+transform[:3,3]
    normals = sn@transform[:3,:3].T
    h,g,gh,gg = np.zeros((6,6)),np.zeros(6),np.zeros((6,6)),np.zeros(6)
    counts,photos = [],[]
    squared = psquared = 0.
    for ref in references:
        target = ref['surface'][scale]
        c = target['camera']
        local = (q-ref['root'][:3,3])@ref['root'][:3,:3]
        ids = np.flatnonzero(local[:,2] > .3)
        uf = c['fx']*local[ids,0]/local[ids,2]+c['cx']
        vf = c['fy']*local[ids,1]/local[ids,2]+c['cy']
        u,v = np.rint(uf),np.rint(vf)
        inside = (u >= 1)&(u < c['width']-1)&(v >= 1)&(v < c['height']-1)
        ids,uf,vf,u,v = ids[inside],uf[inside],vf[inside],u[inside].astype(int),v[inside].astype(int)
        n = target['normals'][v,u]
        nr = n@ref['root'][:3,:3].T
        ok = ((np.sum(n*n,axis=1) > .25)
              &(np.linalg.norm(local[ids]-target['vertices'][v,u],axis=1) < (.12 if scale == 4 else .08))
              &(np.sum(normals[ids]*nr,axis=1) > .8))
        ids,uf,vf,n,nr,u,v = ids[ok],uf[ok],vf[ok],n[ok],nr[ok],u[ok],v[ok]
        r = np.sum(n*(local[ids]-target['vertices'][v,u]),axis=1)
        j = np.column_stack([nr,np.cross(q[ids],nr)])
        w = np.minimum(1.,.01/np.maximum(abs(r),1e-15))
        eh,eg = j.T@(w[:,None]*j),j.T@(w*r)
        gh += eh; gg += eg; h += eh/.005**2; g += eg/.005**2
        squared += float(r@r); counts.append(len(r)); photos.append(0)
        if not enabled or abs(source['phase']-ref['phase']) > .005:
            continue
        image = ref['gray'][scale]
        # Fix the one-sided derivative convention at integer pixel knots.
        uf = np.where(abs(uf-np.rint(uf)) <= 1e-10,np.rint(uf),uf)
        vf = np.where(abs(vf-np.rint(vf)) <= 1e-10,np.rint(vf),vf)
        u,v = np.floor(uf).astype(int),np.floor(vf).astype(int)
        valid = (u >= 0)&(v >= 0)&(u < c['width']-1)&(v < c['height']-1)
        ids,uf,vf,u,v = ids[valid],uf[valid],vf[valid],u[valid],v[valid]
        dx,dy = uf-u,vf-v
        a,b,cc,d = image[v,u],image[v,u+1],image[v+1,u],image[v+1,u+1]
        residual = (1-dx)*(1-dy)*a+dx*(1-dy)*b+(1-dx)*dy*cc+dx*dy*d-intensity[ids]
        ix,iy = (1-dy)*(b-a)+dy*(d-cc),(1-dx)*(cc-a)+dx*(d-b)
        valid = (abs(residual) < 30)&(np.hypot(ix,iy) >= 1)
        if valid.sum() < 200:
            continue
        ids,residual,ix,iy = ids[valid],residual[valid],ix[valid],iy[valid]
        point = local[ids]; z = point[:,2]
        gradient = np.column_stack([ix*c['fx']/z,iy*c['fy']/z,
            -(ix*c['fx']*point[:,0]+iy*c['fy']*point[:,1])/(z*z)])@ref['root'][:3,:3].T
        j = np.column_stack([gradient,np.cross(q[ids],gradient)])
        w = (1-(residual/30)**2)**2
        h += j.T@(w[:,None]*j)/100; g += j.T@(w*residual)/100
        photos[-1] = len(residual); psquared += float(residual@residual)
    count,photo = sum(counts),sum(photos)
    return dict(geometry=dict(count=count,rms_m=np.sqrt(squared/count) if count else 0,h=gh,g=gg),
                h=h,g=g,reference_support=counts,reference_photo_support=photos,photometric_count=photo,
                photometric_rms=np.sqrt(psquared/photo) if photo else None)


def compare_raw(actual, expected):
    from projective_depth_raw import compare_equation
    compare_equation(actual['geometry'],expected['geometry'])
    for key in ('reference_support','reference_photo_support','photometric_count'):
        require(actual[key] == expected[key],'raw image/geometric support: '+key)
    for key,atol in (('h',1e-3),('g',1e-5)):
        np.testing.assert_allclose(actual[key],expected[key],atol=atol,rtol=1e-10)
    if expected['photometric_rms'] is None:
        require(actual['photometric_rms'] is None,'absent raw image support')
    else:
        np.testing.assert_allclose(actual['photometric_rms'],expected['photometric_rms'],atol=1e-9,rtol=0)
