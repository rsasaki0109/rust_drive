"""Evaluation transport and independent linear algebra for optional depth research."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CAMERA = dict(width=640,height=480,fx=525.,fy=525.,cx=319.5,cy=239.5)


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(body):
    return hashlib.sha256(body).hexdigest()


def write(path, value):
    if path.exists() or path.is_symlink():
        raise FileExistsError(str(path))
    descriptor, temporary = tempfile.mkstemp(prefix='.'+path.name+'.prepare-',dir=path.parent)
    try:
        with os.fdopen(descriptor,'w') as stream:
            json.dump(value,stream,indent=2,allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        # Publish only a complete JSON file; linking refuses an existing target
        # even when two writers pass the initial existence check concurrently.
        os.link(temporary,path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def fresh(path):
    path = path.absolute()
    require(not path.exists() and not path.is_symlink()
            and not any(p.is_symlink() for p in path.parents),'fresh real output directory required')
    path.mkdir(parents=True)
    return path


class Client:
    def __init__(self, binary, camera, *, configuration=None):
        self.process = subprocess.Popen([str(binary.absolute())],stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            require(self.send(dict(kind='Configure',camera=camera,**(configuration or {}))) == dict(configured=True,schema_version=1),
                    'versioned configuration acknowledgment')
        except BaseException:
            self.abort()
            raise

    def send(self, value):
        body = json.dumps(value,separators=(',',':'),allow_nan=False)+'\n'
        require(len(body.encode()) <= 8*1024*1024,'bounded input message')
        self.process.stdin.write(body)
        self.process.stdin.flush()
        response = self.process.stdout.readline(1024*1024+1)
        require(response.endswith('\n') and len(response) <= 1024*1024,'bounded complete solver response')
        return json.loads(response)

    def frame(self, stamp, depth, *, image=None):
        request=dict(kind='Frame',timestamp=stamp,depth_m=depth.ravel().tolist())
        if image is not None:request['image']=image
        return self.send(request)

    def finish(self):
        self.process.stdin.close()
        require(self.process.wait(timeout=30) == 0,'solver process completion')
        require(not self.process.stdout.read(1) and not self.process.stderr.read(1),'clean protocol footer')
        self.process.stdout.close()
        self.process.stderr.close()

    def abort(self):
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=30)
        for stream in (self.process.stdin,self.process.stdout,self.process.stderr):
            stream.close()


def pose(value):
    r = np.asarray(value['rotation'],dtype=float)
    t = np.asarray(value['translation'],dtype=float)
    require(r.shape == (3,3) and t.shape == (3,) and np.isfinite(r).all() and np.isfinite(t).all(),
            'finite pose layout')
    np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-8,rtol=0)
    require(abs(np.linalg.det(r)-1) <= 1e-8,'proper rotation')
    out = np.eye(4)
    out[:3,:3],out[:3,3] = r,t
    return out


def exp_rotation(vector):
    # Axis/angle outer-product form independent of Rust's skew-square expression.
    angle = np.linalg.norm(vector)
    if angle < 1e-14:
        return np.eye(3)
    a = vector/angle
    x,y,z = a
    cross = np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return np.cos(angle)*np.eye(3)+(1-np.cos(angle))*np.outer(a,a)+np.sin(angle)*cross


def audit_direction_support(equation):
    if 'direction_counts' in equation:
        counts=equation['direction_counts']
        require(type(counts) is list and len(counts)==2
                and all(type(n) is int and n>=200 for n in counts)
                and sum(counts)==equation['count'],'independent support in both projection directions')


def audit_iterations(update):
    steps = update['iterations']
    require(len(steps) <= 22,'bounded geometric iteration count')
    previous = None
    levels = []
    for index,step in enumerate(steps):
        level = step['scale']
        require(level in (4,2,1),'fixed pyramid level')
        levels.append(level)
        before,after = pose(step['before']),pose(step['after'])
        if previous is not None:
            np.testing.assert_allclose(before,previous,atol=1e-12,rtol=0)
        previous = after
        equation = step['equation']
        audit_direction_support(equation)
        h,g = np.asarray(equation['h']),np.asarray(equation['g'])
        require(h.shape == (6,6) and g.shape == (6,) and np.isfinite(h).all()
                and np.isfinite(g).all() and equation['count'] >= 200
                and 0 <= equation['rms_m'] <= 1.,'finite supported normal equations')
        np.testing.assert_allclose(h,h.T,atol=1e-9,rtol=1e-12)
        if 'photometric' in step:
            p=step['photometric'];ph,pg=np.asarray(p['h']),np.asarray(p['g'])
            require(type(p['count']) is int and 0<=p['count']<=equation['count']
                    and p['used']==(p['count']>=200) and p['scale_m2_per_gray2']==.005**2/100
                    and ph.shape==(6,6) and pg.shape==(6,) and np.isfinite(ph).all() and np.isfinite(pg).all(),
                    'bounded photometric contribution')
            require((p['rms_gray'] is None) if p['count']==0 else 0<=p['rms_gray']<30,'image residual disclosure')
            np.testing.assert_allclose(ph,ph.T,atol=1e-6,rtol=1e-12)
            if 'illumination' in p:
                illumination=p['illumination']
                require((illumination is not None)==p['used'],'exposure-qualified intensity support')
                if illumination is not None:
                    require(.5<=illumination['gain']<=2 and abs(illumination['offset_gray'])<=80
                            and 25<=illumination['source_variance_gray2']<=255**2
                            and type(illumination['candidate_count']) is int
                            and p['count']<=illumination['candidate_count']<=equation['count'],
                            'bounded measured exposure fit')
            if p['used']:h=h+p['scale_m2_per_gray2']*ph;g=g+p['scale_m2_per_gray2']*pg
        scale = np.sqrt(np.diag(h))
        a = h/scale[:,None]/scale[None,:]
        eigen = np.linalg.eigvalsh(a)
        condition = eigen[-1]/eigen[0]
        require(eigen[0] > 0 and condition <= 1e6,'scaled SPD admission')
        np.testing.assert_allclose(condition,step['scaled_condition'],atol=1e-7,rtol=1e-8)
        expected = np.linalg.solve(a,-g/scale)/scale
        d = np.asarray(step['delta'])
        np.testing.assert_allclose(d,expected,atol=1e-10,rtol=1e-8)
        require(np.linalg.norm(d[:3]) <= .1 and np.linalg.norm(d[3:]) <= .15,'bounded increments')
        if 'step_control' in step:
            control=step['step_control'];cost=control['before_cost_m2'];trials=control['trial_costs_m2']
            require(control['geometric_count']==equation['count'] and control['photometric_count']==step.get('photometric',{}).get('count',0)
                    and np.isfinite(cost) and cost>=0 and 1<=len(trials)<=5,'fixed-support step accounting')
            accepted=[]
            for i,value in enumerate(trials):
                require(value is None or (np.isfinite(value) and value>=0),'finite or unavailable trial')
                if value is not None and value<=cost+1e-12*(1+cost):accepted.append(i)
            require(accepted==[len(trials)-1] if accepted else len(trials)==5,'first passing step or bounded halt')
            factor=2.**(-(len(trials)-1)) if accepted else 0.
            require(control['applied_scale']==factor,'verified applied step scale')
            d=d*factor
            if factor==0 and index+1<len(steps):
                require(steps[index+1]['scale']<level,'halt ends this pyramid level')
        increment = np.eye(4)
        increment[:3,:3],increment[:3,3] = exp_rotation(d[3:]),d[:3]
        np.testing.assert_allclose(after,increment@before,atol=1e-11,rtol=0)
    require(levels == sorted(levels,reverse=True)
            and all(levels.count(s) <= cap for s,cap in ((4,10),(2,7),(1,5))), 'pyramid work caps')
    if update['accepted'] and steps:
        e = update['final_equation']
        audit_direction_support(e)
        require(e['count'] >= 200 and 0 <= e['rms_m'] <= .03,'returned-pose residual admission')
        fusion = update['fusion']
        stats = fusion['measurement']
        require(0 <= stats['selected'] <= stats['projected'] <= stats['source_valid'] <= 640*480,
                'raster count partition')
        require(stats['selected'] == sum(fusion[k] for k in ('filled','averaged','replaced','ignored')),
                'all selected depths retained in fusion accounting')
    return len(steps)


def audit_sequence(outcomes, timestamps):
    require(len(outcomes) == len(timestamps) and len(outcomes) <= 200,'complete bounded sequence')
    require(all(b > a for a,b in zip(timestamps,timestamps[1:])),'strict selected depth clocks')
    key_root,initial = np.eye(4),np.eye(4)
    last_accepted = None
    lost = False
    checked = 0
    history=[]
    for index,(row,stamp) in enumerate(zip(outcomes,timestamps)):
        update = row['update']
        require(update['timestamp'] == stamp,'unaltered acquisition identity')
        if last_accepted is not None:
            lost |= stamp-last_accepted > .2
        require(update['lost'] == lost and not (lost and update['accepted']),'accepted-pose expiry latch')
        checked += audit_iterations(update)
        steps = update['iterations']
        predicted_initial=initial
        if 'prediction' in update:
            from projective_motion_raw import predict,compare
            expected=predict(history,stamp);compare(update['prediction'],expected)
            if update['prediction']['latest_from_prediction'] is not None:
                predicted_initial=initial@pose(update['prediction']['latest_from_prediction'])
        if steps:
            np.testing.assert_allclose(pose(steps[0]['before']),predicted_initial,atol=1e-11,rtol=0)
        elif 'information_failure' in update:
            np.testing.assert_allclose(pose(update['information_failure']['pose']),predicted_initial,atol=1e-11,rtol=0)
        if update['accepted']:
            history.append((stamp,pose(update['first_from_current'])));history=history[-2:]
            last_accepted = stamp
            if index == 0:
                require(not steps and update['initialized'],'one cold initialization')
                np.testing.assert_allclose(pose(update['first_from_current']),np.eye(4),atol=0,rtol=0)
            else:
                require(bool(steps),'accepted update requires actual fitting')
                relative = pose(steps[-1]['after'])
                root = key_root@relative
                np.testing.assert_allclose(pose(update['first_from_current']),root,atol=1e-10,rtol=0)
                angle = np.arccos(np.clip((np.trace(relative[:3,:3])-1)/2,-1,1))
                replace = np.linalg.norm(relative[:3,3]) > .1 or angle > .1
                require(update['keyframe_replaced'] == replace,'fixed keyframe replacement rule')
                require((update['fusion']['warp'] is not None) == replace,'reference-view warp accounting')
                if replace:
                    key_root,initial = root,np.eye(4)
                else:
                    initial = relative
        else:
            require(update['first_from_current'] is None and update['fusion'] is None
                    and not update['keyframe_replaced'],'rejection cannot commit pose/model')
        require(update['last_accepted_timestamp'] == last_accepted,'rejection cannot renew accepted clock')
    return checked
