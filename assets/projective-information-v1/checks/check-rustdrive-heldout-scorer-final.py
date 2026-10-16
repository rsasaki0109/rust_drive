from pathlib import Path
import json,importlib.util,shutil,sys,hashlib
r=Path('/workspace/rust_drive');sys.path.insert(0,str(r/'scripts'))
spec=importlib.util.spec_from_file_location('heldout',r/'scripts/projective-heldout.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
base=r/'artifacts/projective-information/scorer-regression-final';base.mkdir()
actual_freeze=json.loads((r/'artifacts/projective-information/heldout-rpy/design.json').read_text())
checks={}
for case,counts in [('tum-fr1-room-temporal',(64,80)),('tum-fr1-desk2-independent',(161,171))]:
 out=base/case;out.mkdir();m.BASE=out
 manifest=json.loads((r/'data'/case/'manifest.json').read_text());inventory={x['file']:x for x in manifest['files']}
 def read(name):
  body=(r/'data'/case/'raw'/name).read_bytes();assert hashlib.sha256(body).hexdigest()==inventory[name]['sha256'];return body
 m.inputs=lambda:(actual_freeze,manifest,read);m.frozen=lambda:actual_freeze
 for mode,source,raw in [('depth','default','assets/projective-depth-raw-audit-v1'),('timing','timing','assets/projective-timing-v1/raw')]:
  (out/mode).mkdir();shutil.copy2(r/'artifacts/projective-information'/source/case/'sensor-only.json',out/mode/'sensor-only.json');shutil.copy2(r/raw/case/'audit.json',out/mode/'raw-audit.json')
 assert m.score()==1
 for mode,count,source in [('depth',counts[0],'default'),('timing',counts[1],'timing')]:
  got=json.loads((out/mode/'results.json').read_text());old=json.loads((r/'artifacts/projective-information'/source/case/'results.json').read_text());assert got['summary']['root_accurate']==count
  for a,b in zip(got['frames'][1:],old['frames'][1:],strict=True):
   assert a['update']==b['update'] and a['accurate']==b['accurate']
   assert abs(a['position_error_m']-b['position_error_m'])<1e-9 and abs(a['rotation_error_rad']-b['rotation_error_rad'])<2e-7
  checks[case+'/'+mode]=dict(score_matches=True,updates=179,accurate=count)
(base/'summary.json').write_text(json.dumps(dict(pass_scorer_regression=True,known_updates_checked=716,checks=checks,new_heldout_data_read=False),indent=2)+'\n')
