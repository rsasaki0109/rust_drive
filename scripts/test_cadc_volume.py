#!/usr/bin/env python3
import importlib.util
import itertools
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
import numpy as np
from cadc_volume_acquire import acquire
from cadc_volume_reference import crop,ground_reference,xyz_reference
ROOT=Path(__file__).resolve().parent.parent
PROTOCOL=json.loads((ROOT/'integrations/cadc/protocol-volume-v1.json').read_text())
def raw(rows):return b''.join(struct.pack('<ffff',*row) for row in rows)
class VolumeTests(unittest.TestCase):
    def test_crop_keeps_original_xyz_without_azimuth_thinning(self):
        result=crop(raw([(3,0,.5,float('nan')),(3.1,0,1.5,1),(1,0,0,1),(4,0,4,1),(float('nan'),0,0,1)]),PROTOCOL['volume'])
        self.assertEqual(result['rows'],[0,1]);self.assertEqual(result['invalid_xyz_rows'],1)
        self.assertEqual(result['points'][:,2].tolist(),[.5,1.5])
    def test_crop_rejects_incomplete_bytes(self):
        for value in (b'',b'x'*17):
            with self.assertRaises(ValueError):crop(value,PROTOCOL['volume'])
    def test_morphology_separates_analytic_ground_roof_and_low_outlier(self):
        points=np.array([(x,y,4 if 8<=x<=12 and 8<=y<=12 else 0) for x in range(21) for y in range(21)],dtype=float)
        roof=np.flatnonzero(points[:,2]>0).tolist()
        points=np.vstack((points,[[3,3,-10],[30,30,-10],[5.1,5.1,1]]))
        ground,diagnostics=ground_reference(points,PROTOCOL['terrain'])
        self.assertTrue(diagnostics['confident']);self.assertGreater(len(ground),350)
        self.assertTrue(all(i not in ground for i in roof))
        self.assertTrue(all(i not in ground for i in range(len(points)-3,len(points))))
    def test_empty_sparse_and_unsupported_ground_cannot_remove_points(self):
        for points in ([],[[5,0,-1.5]],[[0,0,0],[3,0,0],[0,3,0]]):
            indices,diagnostics=ground_reference(points,PROTOCOL['terrain'])
            self.assertEqual(indices,[]);self.assertFalse(diagnostics['confident'])
    def test_morphology_order_and_integer_translation_preserve_labels(self):
        points=np.array([(x,y,.1*x-.05*y) for x in range(17) for y in range(17)])
        points=np.vstack((points,[[5.1,5.1,2],[7,7,-20]]))
        before,_=ground_reference(points,PROTOCOL['terrain'])
        shifted=points[::-1]+[500000,5000000,100]
        after,_=ground_reference(shifted,PROTOCOL['terrain'])
        self.assertEqual(before,sorted(len(points)-1-i for i in after))
    def test_ground_bounds_and_nonfinite_input_fail_without_partial_output(self):
        cfg={**PROTOCOL['terrain'],'max_candidate_work':1}
        with self.assertRaises(ValueError):ground_reference([[0,0,0]],cfg)
        with self.assertRaises(ValueError):ground_reference([[0,0,float('nan')]],PROTOCOL['terrain'])
    def test_xyz_connectivity_matches_exhaustive_pair_graph(self):
        rng=np.random.default_rng(19)
        for _ in range(15):
            points=rng.normal(0,.5,(25,3));parents=list(range(len(points)))
            def root(i):
                while parents[i]!=i:i=parents[i]
                return i
            for i,j in itertools.combinations(range(len(points)),2):
                if math.fsum(float(v*v) for v in points[i]-points[j])<=.6**2:parents[root(i)]=root(j)
            expected={}
            for i in range(len(points)):expected.setdefault(root(i),[]).append(i)
            components=sorted([v for v in expected.values() if len(v)>=3],key=lambda v:v[0])
            noise=sorted(i for v in expected.values() if len(v)<3 for i in v)
            actual,actual_noise=xyz_reference(points,list(range(len(points))),PROTOCOL['clustering'],PROTOCOL['proxy'])
            self.assertEqual([r['indices'] for r in actual],components);self.assertEqual(actual_noise,noise)
    def test_xyz_preserves_vertical_separation_and_measured_bounds(self):
        points=np.array([[5,0,.5],[5.1,0,.5],[5.2,0,.5],[5,0,2.5],[5.1,0,2.5],[5.2,0,2.5]])
        objects,noise=xyz_reference(points,list(range(6)),PROTOCOL['clustering'],PROTOCOL['proxy'])
        self.assertEqual([o['indices'] for o in objects],[[0,1,2],[3,4,5]])
        self.assertEqual(noise,[])
        flattened=points.copy();flattened[:,2]=0
        xy,_=xyz_reference(flattened,list(range(6)),PROTOCOL['clustering'],PROTOCOL['proxy'])
        self.assertEqual(len(xy),1)
        self.assertEqual(objects[0]['min']['z'],.5);self.assertEqual(objects[1]['max']['z'],2.5)
    def test_existing_acquisition_destination_is_preserved_before_network(self):
        with tempfile.TemporaryDirectory() as folder:
            target=Path(folder)/'existing';target.mkdir();(target/'keep').write_text('data')
            with self.assertRaises(ValueError):acquire(Path(folder)/'absent.json',target)
            self.assertEqual((target/'keep').read_text(),'data')
    def test_previous_32_frame_evidence_remains_frozen(self):
        spec=importlib.util.spec_from_file_location('volume_audit',ROOT/'scripts/check-cadc-volume.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        previous=module.frozen_previous(PROTOCOL)
        self.assertEqual(len(previous),32)
        identities={d+'-'+str(p)+'-'+str(f).zfill(10) for p in PROTOCOL['partitions']['previous']
                    for d in ('clear','winter') for f in PROTOCOL['frames']}
        self.assertEqual(set(previous),identities)
if __name__=='__main__':unittest.main()
