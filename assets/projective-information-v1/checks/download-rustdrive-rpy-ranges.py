from pathlib import Path
import concurrent.futures,subprocess,re,json,hashlib
base=Path('/workspace/rust_drive/artifacts/projective-information/heldout-rpy');total=410268381
url='https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_rpy.tgz';etag='"187432dd-4ae2a1aa93980"'
parts=[(start,min(total,start+64*1024**2)-1) for start in range(0,total,64*1024**2)]
def fetch(item):
 start,end=item;path=base/('range-'+str(start));header=path.with_suffix('.headers')
 assert not path.exists() and not header.exists()
 result=subprocess.run(['curl','--fail','--silent','--show-error','--connect-timeout','20','--max-time','120','--proto','=https','--range',str(start)+'-'+str(end),'--header','If-Range: '+etag,'--max-filesize',str(end-start+1),'--dump-header',str(header),'--output',str(path),url],capture_output=True,text=True)
 assert result.returncode==0,(start,result.stderr)
 text=header.read_text();statuses=re.findall(r'^HTTP/[^ ]+ (\d+)',text,re.M)
 assert statuses[-1]=='206' and re.search(r'^content-range: bytes '+str(start)+'-'+str(end)+'/'+str(total)+r'\s*$',text,re.M|re.I),text
 assert etag in text and path.stat().st_size==end-start+1
 return path
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:paths=list(pool.map(fetch,parts))
h=hashlib.sha256();out=base/'archive.tgz'
with out.open('xb') as stream:
 for path in paths:
  with path.open('rb') as source:
   for chunk in iter(lambda:source.read(1024*1024),b''):stream.write(chunk);h.update(chunk)
assert out.stat().st_size==total
(base/'range-transfer.json').write_text(json.dumps(dict(url=url,etag=etag,ranges=parts,bytes=total,sha256=h.hexdigest(),verified_content_ranges=True),indent=2)+'\n')
print('verified complete ranged transfer',total,h.hexdigest(),flush=True)
