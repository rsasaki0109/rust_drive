"""One-time source-bound compatibility replay; no numeric reference reads."""
import argparse,gzip,io,json,sys
from pathlib import Path
import numpy as np
from PIL import Image

r=Path.cwd();sys.path.insert(0,str(r/'scripts'))
from projective_depth_common import CAMERA,Client,fresh,require,sha,write
p=argparse.ArgumentParser();p.add_argument('--mode',choices=('depth','rgbd','timing'),required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args();out=fresh(args.output)
binary=r/'integrations/projective-depth/target/release/rustdriving-projective-depth';binary_hash=sha(binary.read_bytes())
sources=[Path(__file__).absolute(),r/'scripts/projective_depth_common.py',*sorted((r/'integrations/projective-depth/src').rglob('*.rs'))]
pins={str(p.relative_to(r)):sha(p.read_bytes()) for p in sources}
configuration={} if args.mode=='depth' else dict(photometric='keyframe')
if args.mode=='timing':configuration.update(motion_model='constant_velocity',timing='previous_motion')
write(out/'freeze.json',dict(source_sha256=pins,binary_sha256=binary_hash,configuration=configuration,numeric_reference_poses_decoded=False))
cases=['tum-fr1-room-temporal','tum-fr1-desk2-independent']
if args.mode!='rgbd':cases.append('tum-fr1-rpy-heldout')
checks={}
for case in cases:
 if case.endswith('heldout'):
  packet=r/'assets/projective-information-v1';folder=packet/'heldout/rpy'/args.mode
 elif args.mode=='depth':packet=r/'assets/projective-depth-rust-v1';folder=packet/case
 elif args.mode=='rgbd':packet=r/'assets/projective-affine-v1';folder=packet/'recorded'/case
 else:packet=r/'assets/projective-timing-v1';folder=packet/'recorded'/case
 path=folder/'sensor-only.json.gz';body=path.read_bytes();binding=json.loads((packet/'files.json').read_text())[str(path.relative_to(packet))]
 require(len(body)==binding['bytes'] and sha(body)==binding['sha256'],'historical packet digest')
 expected=json.loads(gzip.decompress(body));require(expected['ground_truth_parsed'] is False,'pre-label baseline')
 base=r/'data'/case;manifest_body=(base/'manifest.json').read_bytes();manifest=json.loads(manifest_body)
 inventory={i['file']:i for i in manifest['files']};client=Client(binary,CAMERA,configuration=configuration);rows=[]
 def image(name,color=False):
  data=(base/'raw'/name).read_bytes();pin=inventory[name]
  require(len(data)==pin['bytes'] and sha(data)==pin['sha256'],'qualified sensor bytes')
  im=Image.open(io.BytesIO(data));require(im.size==(640,480),'original sensor dimensions')
  if color:
   require(im.mode=='RGB','measured RGB');a=np.asarray(im,dtype=np.uint32)
   return ((77*a[:,:,0]+150*a[:,:,1]+29*a[:,:,2])>>8).astype(np.uint8)
  require(im.mode in ('I;16','I;16B','I'),'measured axial depth');return np.asarray(im,dtype=float)/5000.
 try:
  for frame in manifest['frames']:
   rgb=None if args.mode=='depth' else dict(timestamp=frame['rgb_timestamp'],gray=image(frame['rgb_file'],True).ravel().tolist())
   u=client.frame(frame['depth_timestamp'],image(frame['depth_file']),image=rgb);rows.append(dict(source_index=frame['source_index'],update=u))
  client.finish()
 except BaseException:client.abort();raise
 require(len(rows)==len(expected['frames'])==180,'complete comparison')
 if rows!=expected['frames']:
  write(out/(case+'-first-disagreement.json'),dict(actual=rows,expected_ledger_sha256=sha(body)))
  raise ValueError('sensor replay differs: '+case)
 checks[case]=dict(identical_outcomes=180,manifest_sha256=sha(manifest_body),baseline_file=str(path.relative_to(r)),baseline_sha256=sha(body))
 write(out/(case+'.json'),checks[case]);print(args.mode,case,'180 outcomes exactly identical',flush=True)
require(sha(binary.read_bytes())==binary_hash and all(sha((r/n).read_bytes())==h for n,h in pins.items()),'binary and sources unchanged throughout replay')
write(out/'summary.json',dict(pass_exact_compatibility=True,cases=checks,identical_outcomes=180*len(checks),numeric_reference_poses_decoded=False))
