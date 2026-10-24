#!/usr/bin/env python3
import io
import itertools
import json
import math
from pathlib import Path
import random
import struct
import tarfile
import tempfile
import unittest
import zipfile
import cadc_recorded as cadc

PROFILE={'z_min':-.9,'z_max':-.3,'min_range':2,'max_range':45,'bins':360}
def raw(rows):return b''.join(struct.pack('<ffff',*row) for row in rows)
def box(length=4,width=2,x=0,y=0,yaw=0):
    return {'position':{'x':x,'y':y,'z':-.6},'dimensions':{'x':width,'y':length,'z':1.5},'yaw':yaw}
def disc(x=0,y=0,r=.5):return {'center':{'x':x,'y':y},'radius':r}
class Response(io.BytesIO):
    def __init__(self,data,url,status,headers):
        super().__init__(data);self.url=url;self.status=status;self.headers=headers;self.read_called=False
    def geturl(self):return self.url
    def read(self,n=-1):self.read_called=True;return super().read(n)
def range_source(payload,status=206,etag='"v1"'):
    record={'url':'https://wiselab.uwaterloo.ca/cadc/d/s/labeled.zip','size':len(payload),'etag':'"v1"'}
    responses=[]
    def opener(request,timeout):
        a,b=map(int,request.get_header('Range').removeprefix('bytes=').split('-'))
        r=Response(payload[a:b+1],record['url'],status,
            {'ETag':etag,'Content-Range':'bytes '+str(a)+'-'+str(b)+'/'+str(len(payload))})
        responses.append(r);return r
    return record,opener,responses

class RecordedTests(unittest.TestCase):
    def test_nearest_return_keeps_measured_angle_and_first_equal_range(self):
        source=raw([(4,0,-.6,1),(3,.01,-.6,float('nan')),(3,.01,-.6,2),(0,4,-.6,1)])
        out=cadc.project(source,PROFILE)
        self.assertEqual(out['rows'],[1,3]);self.assertEqual(out['points'][0][1],struct.unpack('<f',struct.pack('<f',.01))[0])
    def test_projection_counts_invalid_and_empty_slab(self):
        out=cadc.project(raw([(3,0,2,1),(float('nan'),0,-.6,1),(0,0,-.6,1)]),PROFILE)
        self.assertEqual(out['invalid_xyz_rows'],1);self.assertEqual(out['points'],[])
    def test_projection_rejects_truncated_or_unbounded_rows(self):
        for data in (b'',b'x'*17,b'x'*(cadc.MAX_MEMBER+16)):
            with self.assertRaises(ValueError):cadc.project(data,PROFILE)
    def test_cadc_width_length_convention_and_rotation(self):
        polygon=cadc.box_polygon(box(yaw=math.pi/2))
        self.assertAlmostEqual(max(abs(p[0]) for p in polygon),1)
        self.assertAlmostEqual(max(abs(p[1]) for p in polygon),2)
    def test_iou_fully_contained_circle_and_disjoint(self):
        area=64/2*.5**2*math.sin(math.tau/64)
        self.assertAlmostEqual(cadc.proxy_iou(disc(),box()),area/8,places=12)
        self.assertEqual(cadc.proxy_iou(disc(x=20),box()),0)
    def test_clipping_half_circle_and_rigid_rotation(self):
        full=box(length=20,width=20,x=10)
        area=64/2*.5**2*math.sin(math.tau/64)
        self.assertAlmostEqual(cadc.proxy_iou(disc(),full),(area/2)/(400+area/2),places=12)
        self.assertAlmostEqual(cadc.proxy_iou(disc(x=1),box()),cadc.proxy_iou(disc(y=1),box(yaw=math.pi/2)),places=12)
    def test_matching_reassigns_a_greedy_trap(self):
        self.assertEqual(len(cadc.maximum_matches([[0,1],[0]])),2)
    def test_matching_matches_exhaustive_cardinality(self):
        rng=random.Random(93)
        for _ in range(100):
            n=rng.randrange(1,5);m=rng.randrange(1,5)
            graph=[[j for j in range(m) if rng.random()<.5] for _ in range(n)]
            expected=max(sum(j>=0 for j in assignment)
                for assignment in itertools.product(range(-1,m),repeat=n)
                if len([j for j in assignment if j>=0])==len(set(j for j in assignment if j>=0))
                and all(j<0 or j in graph[i] for i,j in enumerate(assignment)))
            self.assertEqual(len(cadc.maximum_matches(graph)),expected)
    def test_remote_zip_range_reader_extracts_only_selected_payload(self):
        buffer=io.BytesIO();point=raw([(3,0,-.6,1)])
        with zipfile.ZipFile(buffer,'w',compression=zipfile.ZIP_DEFLATED) as z:
            z.writestr('labeled/image_00/data/0000000000.png',b'irrelevant')
            z.writestr('labeled/lidar_points/data/0000000000.bin',point)
        record,opener,_=range_source(buffer.getvalue())
        selected,receipt=cadc.zip_points(record,[0],opener)
        self.assertEqual(selected,{0:point});self.assertLessEqual(receipt['transferred_bytes'],len(buffer.getvalue())*5)
    def test_ignored_range_is_rejected_before_payload_read(self):
        record,opener,responses=range_source(b'x'*100,status=200)
        with self.assertRaises(ValueError):cadc.RangeIO(record,opener).read(10)
        self.assertFalse(responses[0].read_called)
    def test_wrong_etag_and_unbounded_range_are_rejected(self):
        record,opener,_=range_source(b'x'*100,etag='"changed"')
        with self.assertRaises(ValueError):cadc.RangeIO(record,opener).read(10)
        stream=cadc.RangeIO({'url':record['url'],'size':cadc.MAX_METADATA+2,'etag':'"v1"'},opener)
        with self.assertRaises(ValueError):stream.read()
        with self.assertRaises(ValueError):stream.seek(-1)
        with self.assertRaises(ValueError):cadc.safe_url('https://evil.example/cadc/a')
    def test_tar_selection_does_not_extract_archive_paths(self):
        data=io.BytesIO();point=raw([(3,0,-.6,1)])
        with tarfile.open(fileobj=data,mode='w') as tar:
            item=tarfile.TarInfo('labeled/lidar_points/data/0000000000.bin');item.size=len(point);tar.addfile(item,io.BytesIO(point))
        data.seek(0);self.assertEqual(cadc.select_tar(data,[0]),{0:point})
    def test_tar_symlinks_and_traversal_are_rejected(self):
        for name in ('../outside.bin','labeled/lidar_points/data/0000000000.bin'):
            data=io.BytesIO()
            with tarfile.open(fileobj=data,mode='w') as tar:
                item=tarfile.TarInfo(name);item.type=tarfile.SYMTYPE;item.linkname='/outside';tar.addfile(item)
            data.seek(0)
            with self.assertRaises(ValueError):cadc.select_tar(data,[0])
    def test_existing_data_destination_is_preserved_before_network(self):
        with tempfile.TemporaryDirectory() as tmp:
            destination=Path(tmp)/'existing';destination.mkdir();sentinel=destination/'keep';sentinel.write_text('unchanged')
            with self.assertRaises(ValueError):cadc.acquire(Path(tmp)/'absent-protocol.json',destination)
            self.assertEqual(sentinel.read_text(),'unchanged')
if __name__=='__main__':unittest.main()
