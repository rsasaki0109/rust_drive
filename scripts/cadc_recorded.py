"""Bounded CADC acquisition and independent geometric primitives.

Only selected point members and annotation JSON are retained locally. Archive
members are never extracted to archive-supplied paths. No credentials are used.
"""
import hashlib
import io
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import struct
import stat
import tarfile
import tempfile
import urllib.request
from urllib.parse import urlsplit
import zipfile

MAX_MEMBER=8*1024*1024
MAX_METADATA=16*1024*1024
MAX_RANGE_TOTAL=128*1024*1024
HOST='wiselab.uwaterloo.ca'
def digest(data):return hashlib.sha256(data).hexdigest()
def require(value,message):
    if not value:raise ValueError(message)
def safe_url(url):
    p=urlsplit(url)
    require(p.scheme=='https' and p.netloc==HOST and not p.query and not p.fragment,'non-public CADC URL')
    require(p.path.startswith(('/cadc/','/cadc-clear/')),'unexpected CADC path')
    return url
def head(record,opener=urllib.request.urlopen):
    safe_url(record['url'])
    req=urllib.request.Request(record['url'],method='HEAD',headers={'Accept-Encoding':'identity'})
    with opener(req,timeout=40) as r:
        require(r.status==200 and r.geturl()==record['url'],'unexpected archive HEAD')
        require(int(r.headers['Content-Length'])==record['size'] and r.headers.get('ETag')==record['etag'],'archive version changed')
def get_small(url,cap=MAX_METADATA,opener=urllib.request.urlopen):
    safe_url(url)
    with opener(urllib.request.Request(url,headers={'Accept-Encoding':'identity'}),timeout=60) as r:
        require(r.status==200 and r.geturl()==url,'metadata redirect or status differs')
        data=r.read(cap+1)
    require(len(data)<=cap,'metadata byte budget exceeded')
    return data

class RangeIO(io.RawIOBase):
    """Seekable remote ZIP input, strictly requiring bounded identity ranges."""
    def __init__(self,record,opener=urllib.request.urlopen):
        self.record=record;self.opener=opener;self.position=0;self.network_bytes=0
        self.cache=b'';self.cache_at=0;safe_url(record['url'])
    def readable(self):return True
    def seekable(self):return True
    def tell(self):return self.position
    def seek(self,offset,whence=0):
        require(whence in (0,1,2),'invalid seek mode')
        target=offset+(0 if whence==0 else self.position if whence==1 else self.record['size'])
        require(0<=target<=self.record['size'],'remote seek outside archive')
        self.position=target;return target
    def read(self,size=-1):
        remaining=self.record['size']-self.position
        if size<0:size=remaining
        size=min(size,remaining)
        require(size<=MAX_METADATA,'range read budget exceeded')
        if not size:return b''
        start=self.position;end=start+size
        if not(self.cache_at<=start and end<=self.cache_at+len(self.cache)):
            stop=min(self.record['size'],max(end,start+65536))
            require(self.network_bytes+stop-start<=MAX_RANGE_TOTAL,'total ZIP transfer budget exceeded')
            headers={'Accept-Encoding':'identity','Range':'bytes='+str(start)+'-'+str(stop-1),'If-Match':self.record['etag']}
            with self.opener(urllib.request.Request(self.record['url'],headers=headers),timeout=90) as r:
                require(r.status==206 and r.geturl()==self.record['url'],'server did not honor bounded range')
                require(r.headers.get('Content-Range')=='bytes '+str(start)+'-'+str(stop-1)+'/'+str(self.record['size']),'range identity differs')
                require(r.headers.get('ETag')==self.record['etag'],'archive ETag differs')
                data=r.read(stop-start+1)
            require(len(data)==stop-start,'range byte count differs')
            self.network_bytes+=len(data);self.cache_at=start;self.cache=data
        out=self.cache[start-self.cache_at:end-self.cache_at]
        self.position=end;return out

class CountedStream:
    def __init__(self,source,cap):self.source=source;self.cap=cap;self.count=0;self.hasher=hashlib.sha256()
    def read(self,n=-1):
        require(0<=n<=MAX_METADATA,'unbounded compressed read')
        data=self.source.read(min(n,self.cap-self.count+1));self.count+=len(data);self.hasher.update(data)
        require(self.count<=self.cap,'compressed transfer budget exceeded')
        return data
    def readable(self):return True
    def close(self):self.source.close()

def point_number(name):
    path=PurePosixPath(name)
    require(not path.is_absolute() and all(p not in ('.','..') for p in path.parts),'unsafe archive member name')
    if len(path.parts)==4 and path.parts[:3]==('labeled','lidar_points','data') and re.fullmatch(r'\d{10}\.bin',path.name):
        return int(path.stem)
    return None

def select_tar(source,wanted):
    """Read a decompressed tar stream; never extract filenames or links."""
    result={}
    with tarfile.open(fileobj=source,mode='r|') as archive:
        for i,item in enumerate(archive):
            require(i<50000,'archive member count exceeded')
            index=point_number(item.name)
            if index not in wanted:continue
            require(item.isfile() and not item.issym() and not item.islnk(),'point member is not a regular file')
            require(index not in result and 0<item.size<=MAX_MEMBER and item.size%16==0,'invalid/duplicate point member')
            stream=archive.extractfile(item);require(stream is not None,'point member missing')
            data=stream.read(MAX_MEMBER+1)
            require(len(data)==item.size,'point payload length differs')
            result[index]=data
    require(set(result)==set(wanted),'selected point frames missing')
    return result

def zip_points(record,wanted,opener=urllib.request.urlopen):
    stream=RangeIO(record,opener);out={}
    with zipfile.ZipFile(stream) as archive:
        infos=archive.infolist();require(len(infos)<=50000,'ZIP member count exceeded')
        for item in infos:
            index=point_number(item.filename)
            if index not in wanted:continue
            require(index not in out and 0<item.file_size<=MAX_MEMBER and item.file_size%16==0,'invalid/duplicate ZIP point member')
            require(not item.flag_bits&1 and item.compress_size<=MAX_MEMBER,'encrypted or oversized point member')
            require(not stat.S_ISLNK(item.external_attr>>16),'ZIP point member is a symlink')
            require(item.compress_type in (zipfile.ZIP_STORED,zipfile.ZIP_DEFLATED),'unsupported point codec')
            out[index]=archive.read(item)
    require(set(out)==set(wanted),'selected ZIP point frames missing')
    return out,{'transferred_bytes':stream.network_bytes,'archive_sha256':None,'integrity':'pinned HTTP ranges and ZIP member CRC32; selected payload SHA256'}

def zstd_points(record,wanted):
    import zstandard
    headers={'Accept-Encoding':'identity','If-Match':record['etag']}
    with urllib.request.urlopen(urllib.request.Request(record['url'],headers=headers),timeout=120) as response:
        require(response.status==200 and response.geturl()==record['url'],'unexpected zstd response')
        require(response.headers.get('ETag')==record['etag'] and int(response.headers['Content-Length'])==record['size'],'zstd archive changed')
        counted=CountedStream(response,record['size'])
        # python-zstandard 0.25 forwards this value to ZSTD_DCtx_setMaxWindowSize in bytes.
        with zstandard.ZstdDecompressor(max_window_size=256*1024*1024).stream_reader(counted,read_size=131072,read_across_frames=True) as decoded:
            bounded=CountedStream(decoded,8*1024*1024*1024)
            result=select_tar(bounded,wanted)
            while bounded.read(131072):pass
        require(counted.count==record['size'],'compressed archive truncated')
        return result,{'transferred_bytes':counted.count,'archive_sha256':counted.hasher.hexdigest(),'integrity':'pinned complete zstd archive SHA256; selected payload SHA256'}

def acquire(protocol_path,output,pair=None):
    protocol_path=Path(protocol_path);output=Path(output).absolute()
    require(not output.exists(),'dataset destination already exists')
    protocol_bytes=protocol_path.read_bytes();protocol=json.loads(protocol_bytes)
    require(protocol['schema_version']==1 and len(protocol['sequences'])==4,'unsupported frozen protocol')
    wanted=protocol['frames'];require(wanted==[0,10,20,30,40,50,60,70],'frame protocol changed')
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.cadc-staging-',dir=output.parent) as tmp:
        stage=Path(tmp);(stage/'points').mkdir();(stage/'annotations').mkdir()
        sensor={'profile':protocol['profile'],'frames':[]};records=[]
        require(pair in (None,0,1),'invalid route pair')
        for record in protocol['sequences']:
            if pair is not None and record['pair']!=pair:continue
            head(record)
            annotation=get_small(record['annotation_url'])
            require(digest(annotation)==record['annotation_sha256'],'annotation hash differs')
            labels=json.loads(annotation)
            require(isinstance(labels,list) and len(labels)>max(wanted),'annotation frame protocol differs')
            name=record['domain']+'-'+str(record['pair'])
            label_path='annotations/'+name+'.json';(stage/label_path).write_bytes(annotation)
            selected,transfer=zip_points(record,wanted) if record['url'].endswith('.zip') else zstd_points(record,wanted)
            for frame in wanted:
                data=selected[frame];require(len(data)<=MAX_MEMBER and len(data)%16==0,'invalid XYZI member')
                point_path='points/'+name+'-'+str(frame).zfill(10)+'.bin'
                (stage/point_path).write_bytes(data)
                identity=name+'-'+str(frame).zfill(10)
                sensor['frames'].append({'id':identity,'path':point_path,'stamp':frame/10})
                records.append({'id':identity,'domain':record['domain'],'pair':record['pair'],
                    'sequence':record['sequence'],'frame':frame,'point_path':point_path,'point_sha256':digest(data),
                    'point_bytes':len(data),'annotation_path':label_path,'annotation_sha256':digest(annotation),
                    'archive_url':record['url'],'archive_bytes':record['size'],'archive_etag':record['etag'],**transfer})
            print('CADC_FETCH',json.dumps({'domain':record['domain'],'sequence':record['sequence'],
                'point_frames':len(selected),**transfer,'archive_bytes':record['size']}),flush=True)
        (stage/'sensor.json').write_text(json.dumps(sensor,separators=(',',':'))+'\n')
        (stage/'receipt.json').write_text(json.dumps({'protocol_sha256':digest(protocol_bytes),
            'license_url':protocol['license_url'],'dataset_url':protocol['dataset_url'],
            'frames':records,'sensor_sha256':digest((stage/'sensor.json').read_bytes())},indent=2)+'\n')
        require(not output.exists(),'dataset destination appeared during acquisition')
        os.rename(stage,output)
    return output

def project(data,profile):
    require(data and len(data)<=MAX_MEMBER and len(data)%16==0,'invalid XYZI bytes')
    bins={};invalid=0;slab=0
    for index,(x,y,z,intensity) in enumerate(struct.iter_unpack('<ffff',data)):
        if not all(math.isfinite(v) for v in (x,y,z)):invalid+=1;continue
        radius=math.hypot(x,y)
        if not(profile['z_min']<=z<=profile['z_max'] and profile['min_range']<=radius<=profile['max_range']):continue
        slab+=1
        angle=math.atan2(y,x)%math.tau
        bin_id=min(profile['bins']-1,math.floor(angle/math.tau*profile['bins']))
        if bin_id not in bins or radius<bins[bin_id][0]:bins[bin_id]=(radius,index,(x,y))
    ordered=[bins[k] for k in sorted(bins)]
    return {'rows':[v[1] for v in ordered],'points':[v[2] for v in ordered],
        'total_rows':len(data)//16,'invalid_xyz_rows':invalid,'slab_rows':slab}

def reference_detect(points):
    """Exhaustive directed BFS and QR circle fit, independent of Rust's grid/normal-equation solve."""
    import numpy as np
    pts=np.array(points,dtype=np.float64).reshape((-1,2));visited=set();output=[]
    for seed in range(len(pts)):
        if seed in visited:continue
        visited.add(seed);queue=[seed];cursor=0
        while cursor<len(queue):
            index=queue[cursor];cursor+=1
            tolerance=min(.85,.28+math.hypot(*pts[index])*.015)
            for j in range(len(pts)):
                if j not in visited and math.hypot(*(pts[index]-pts[j]))<tolerance:
                    visited.add(j);queue.append(j)
        if len(queue)<3:continue
        cluster=pts[queue];origin=np.array([math.fsum(cluster[:,0])/len(queue),math.fsum(cluster[:,1])/len(queue)])
        extent=max(math.hypot(*(p-origin)) for p in cluster)
        center=origin;radius=min(3,max(.85,extent+.65))
        if len(queue)>=5:
            q=cluster-origin
            matrix=np.column_stack((2*q[:,0],2*q[:,1],np.ones(len(q))))
            solved,_,rank,_=np.linalg.lstsq(matrix,(q*q).sum(axis=1),rcond=None)
            r2=solved[2]+solved[0]**2+solved[1]**2
            if rank==3 and r2>0:
                r=math.sqrt(r2);candidate=origin+solved[:2]
                if .4<=r<=2.5 and max(abs(math.hypot(*(p-candidate))-r) for p in cluster)<=.12:
                    center=candidate;radius=r+.12
        output.append({'center':{'x':float(center[0]),'y':float(center[1])},'radius':float(radius)})
    return output

def polygon_area(points):
    if len(points)<3:return 0.0
    return abs(math.fsum(a[0]*b[1]-a[1]*b[0] for a,b in zip(points,points[1:]+points[:1])))/2
def box_polygon(box):
    x,y=box['position']['x'],box['position']['y']
    length,width=box['dimensions']['y'],box['dimensions']['x']
    c,s=math.cos(box['yaw']),math.sin(box['yaw'])
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in
        [(-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)]]
def proxy_iou(detection,box,vertices=64):
    center=detection['center'];r=detection['radius']
    polygon=[(center['x']+r*math.cos(math.tau*i/vertices),center['y']+r*math.sin(math.tau*i/vertices)) for i in range(vertices)]
    disc_area=polygon_area(polygon);rectangle=box_polygon(box)
    def side(a,b,p):return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    for a,b in zip(rectangle,rectangle[1:]+rectangle[:1]):
        old=polygon;polygon=[]
        if not old:break
        for previous,current in zip(old[-1:]+old[:-1],old):
            p,q=side(a,b,previous),side(a,b,current)
            if (p>=0)!=(q>=0):
                t=p/(p-q);polygon.append((previous[0]+t*(current[0]-previous[0]),previous[1]+t*(current[1]-previous[1])))
            if q>=0:polygon.append(current)
    intersection=polygon_area(polygon)
    union=disc_area+box['dimensions']['x']*box['dimensions']['y']-intersection
    return intersection/union if union>0 else 0.0
def maximum_matches(adjacency):
    owner={}
    def augment(box,seen):
        for detection in adjacency[box]:
            if detection in seen:continue
            seen.add(detection)
            if detection not in owner or augment(owner[detection],seen):
                owner[detection]=box;return True
        return False
    for box in range(len(adjacency)):augment(box,set())
    return sorted((box,detection) for detection,box in owner.items())
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--protocol',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pair',type=int,choices=(0,1))
    args=p.parse_args();acquire(args.protocol,args.output,args.pair)
