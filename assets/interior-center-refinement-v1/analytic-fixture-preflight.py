import importlib.util,sys
from pathlib import Path
import numpy as np
root=Path('/workspace/rust_drive');sys.path.insert(0,str(root/'scripts'))
spec=importlib.util.spec_from_file_location('interior_fixture_probe',root/'scripts/check-interior-center-refinement.py');check=importlib.util.module_from_spec(spec);spec.loader.exec_module(check)
k=check.kernel;audit=check.load('interior_fixture_raw','check-reciprocal-keyframes.py').auditor();identity=np.eye(4);camera=audit.rays.CAMERA
z=np.round(audit.rays.render(identity)/.002)*.002;truth=audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source=np.round(audit.rays.render(truth)/.002)*.002
tracker=k.fixed.base.Tracker(camera,'retain');tracker.step(1.,z);warm=tracker.step(1.03,source);initial=np.asarray(warm['revised_roots']);initial[1]=audit.increment(initial[1],np.array([.006,-.002,.001,.0008,-.0006,.0004]))
nodes=k.fixed.prepare([dict(timestamp=t,root=p.copy(),surface=k.fixed.base.pyramid(depth,camera)) for t,p,depth in zip((1.,1.03),initial,(z,source),strict=True)])
other=check.reference.prepare([dict(n,surface=audit.pyramid(depth,camera)) for n,depth in zip(nodes,(z,source),strict=True)])
q=k.refine(nodes,initial,None);systems=check.verify(q,other,initial,None,audit)
print(dict(rounds=q['association_rounds'],corrections=q['applied_iterations'],stop=q['termination'],selection_converged=q['selection_converged'],round_counts=[r['batch']['applied_iterations'] for r in q['rounds']],checked_systems=systems))
