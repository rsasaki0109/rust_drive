#!/usr/bin/env python3
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import subprocess
import tempfile
import unittest
from cadc_selection_reference import proxy,circle_iou_integral,select_reference,choose_policy
ROOT=Path(__file__).resolve().parent.parent
PROTOCOL=json.loads((ROOT/'integrations/cadc/protocol-selection-v1.json').read_text())
COMMON=PROTOCOL['selection_common']
def component(x=5,y=0,z=0,n=8,first=0,half=.1):
    return {'min':{'x':x-half,'y':y-half,'z':z-.1},'max':{'x':x+half,'y':y+half,'z':z+.1},
            'point_count':n,'indices':list(range(first,first+n))}
def load_audit():
    spec=importlib.util.spec_from_file_location('selection_audit',ROOT/'scripts/check-cadc-selection.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
def synthetic_summary(policies):
    result={}
    for domain in ('clear','winter'):
        result[domain]={'raw':{'matched':100,'outputs':500,'rangeBins':[{'matched':20}]}}
        for policy,matched,far,outputs in policies:
            result[domain][policy]={'matched':matched,'outputs':outputs,'rangeBins':[{'matched':far}]}
    return result
class SelectionTests(unittest.TestCase):
    def test_lens_integral_has_known_equal_circle_and_containment_area(self):
        a=({'x':0,'y':0,'z':0},1.);b=({'x':1,'y':0,'z':0},1.)
        area=2*math.pi/3-math.sqrt(3)/2
        self.assertAlmostEqual(circle_iou_integral(a,b),area/(2*math.pi-area),places=9)
        self.assertEqual(circle_iou_integral(a,({'x':0,'y':0,'z':0},.5)),.25)
        self.assertEqual(circle_iou_integral(a,({'x':3,'y':0,'z':0},1.)),0.)
    def test_range_support_and_height_gates_keep_distinct_measurements(self):
        points=[component(n=6),component(x=35,n=3,first=6),component(x=35,z=3,n=8,first=9)]
        p={'min_points_by_range':[8,5,3],'overlap_threshold':.5}
        result=select_reference(points,COMMON,p)
        self.assertEqual(result['kept_indices'],[1,2]);self.assertEqual(result['low_support_indices'],[0])
    def test_support_rank_tie_is_independent_of_input_order(self):
        values=[component(x=5.99,n=8,first=10),component(x=6.01,n=8,first=0)]
        policy={'min_points_by_range':[3,3,3],'overlap_threshold':.5}
        first=select_reference(values,COMMON,policy);second=select_reference(values[::-1],COMMON,policy)
        self.assertEqual(values[first['kept_indices'][0]]['indices'],values[::-1][second['kept_indices'][0]]['indices'])
    def test_choice_optimizes_only_after_all_retention_constraints_pass(self):
        ids=['p3-3-3-off','p5-5-5-off','p8-8-8-off','p12-12-12-off']
        p={**PROTOCOL,'policies':[v for v in PROTOCOL['policies'] if v['id'] in ids]}
        summary=synthetic_summary([(ids[0],100,20,500),(ids[1],96,20,200),(ids[2],94,20,100),(ids[3],99,18,50)])
        choice=choose_policy(summary,p)
        self.assertEqual(choice['id'],ids[1]);self.assertEqual(choice['objectiveOutputs'],400)
    def test_undefined_far_matches_do_not_qualify_as_perfect_retention(self):
        identity='p3-3-3-off';p={**PROTOCOL,'policies':[v for v in PROTOCOL['policies'] if v['id']==identity]}
        summary=synthetic_summary([(identity,100,0,500)])
        for domain in summary:summary[domain]['raw']['rangeBins'][0]['matched']=0
        with self.assertRaises(ValueError):choose_policy(summary,p)
    def test_cached_overlap_and_subset_matching_match_legacy_naive_scoring(self):
        from cadc_recorded import proxy_iou,maximum_matches
        module=load_audit();rng=random.Random(23)
        for _ in range(15):
            detections=[{'proxy':{'center':{'x':rng.uniform(2,12),'y':rng.uniform(-3,3)},'radius':rng.uniform(.85,2)}} for i in range(8)]
            boxes=[{'position':{'x':rng.uniform(3,12),'y':rng.uniform(-3,3),'z':-.5},
                    'dimensions':{'x':2.,'y':4.,'z':1.5},'yaw':rng.uniform(-math.pi,math.pi)} for i in range(5)]
            graph=module.overlap_graph(detections,boxes,PROTOCOL);selected=[0,2,5,7]
            score=module.score_subset(selected,graph,boxes,list(range(len(boxes))))
            naive=[[i for i in selected if proxy_iou(detections[i]['proxy'],box)>=.1] for box in boxes]
            self.assertEqual(score['matched'],len(maximum_matches(naive)))
            self.assertEqual(sum(score['context'].values()),len(selected))
    def test_known_64_frame_reference_and_new_partition_are_complete(self):
        previous,manifest=load_audit().previous_frames(PROTOCOL)
        self.assertEqual(len(previous),64);self.assertEqual(manifest['sourceCommit'],PROTOCOL['raw_source'])
        self.assertEqual(set(PROTOCOL['study_partitions']['development']),{0,1,2,3})
        self.assertEqual(set(PROTOCOL['study_partitions']['validation']),{4,5})
        self.assertEqual(PROTOCOL['frames'],list(range(0,80,10)))
    @unittest.skipUnless(os.environ.get('CADC_SELECTION_BINARY'),'native example path required')
    def test_native_grid_matches_all_pairs_integral_reference_for_all_policies(self):
        rng=random.Random(31);components=[];next_index=0
        for i in range(45):
            n=rng.randrange(3,21);components.append(component(x=rng.uniform(-5,40),y=rng.uniform(-3,3),
                z=rng.choice([0.,.2,1.,3.]),n=n,first=next_index,half=rng.uniform(.05,1.6)));next_index+=n
        components.extend([component(x=5.99,n=9,first=next_index),component(x=6.01,n=12,first=next_index+9)])
        next_index+=21
        value={'schema_version':1,'common':COMMON,'policies':PROTOCOL['policies'],
               'frames':[{'id':'fixture','data_point_count':next_index,'candidates':components}]}
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'input.json';path.write_text(json.dumps(value))
            result=subprocess.run([os.environ['CADC_SELECTION_BINARY'],str(path)],capture_output=True,text=True,check=True)
        report=json.loads(result.stdout)
        self.assertEqual(report['common'],COMMON)
        for decision,policy in zip(report['frames'][0]['decisions'],PROTOCOL['policies']):
            expected=select_reference(components,COMMON,policy)
            for key,value in expected.items():self.assertEqual(decision[key],value,policy['id']+' '+key)
    @unittest.skipUnless(os.environ.get('CADC_SELECTION_BINARY'),'native example path required')
    def test_native_input_rejects_labels_without_partial_stdout(self):
        value={'schema_version':1,'common':COMMON,'policies':PROTOCOL['policies'][:1],
            'frames':[{'id':'fixture','data_point_count':0,'candidates':[],'cuboids':[]}]}
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'input.json';path.write_text(json.dumps(value))
            result=subprocess.run([os.environ['CADC_SELECTION_BINARY'],str(path)],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0);self.assertEqual(result.stdout,'')
if __name__=='__main__':unittest.main()
