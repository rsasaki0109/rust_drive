"""Check full output-bit claims, producer separation and unchanged clock guards."""
import collections
import copy
import gzip
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
import unittest

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.insert(0,str(HERE))
PACKET=ROOT/'assets/camera-nano-repeat2x-v1'
spec=importlib.util.spec_from_file_location('nano_repeat_tests',HERE/'camera_nano_repeat_check.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)


def read(name):
    path=PACKET/name
    if path.exists():return json.loads(path.read_text())
    return json.loads(gzip.decompress(Path(str(path)+'.gz').read_bytes()))


def adjacent(value):
    bits=struct.unpack('<I',struct.pack('<f',value))[0]
    return struct.unpack('<f',struct.pack('<I',bits+1))[0]


class NanoRepeatBoundaries(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.real_profile=read('validation/profiles/tum-0-first.json')
        cls.reference=read('validation/references/tum-0.json')
        cls.real_image=read('validation/images/tum-0-first.json')
        cls.saved=json.loads((ROOT/'assets/camera-sequence-v1/recorded/exports/first-0.json').read_text())
        cls.source=json.loads((ROOT/'assets/camera-sequence-v1/sequence.json').read_text())
        cls.inspection=read('validation/optimized-inspection.json')
        cls.real_fifo=read('validation/fifo/optimized-first.json')
        cls.real_latest=read('validation/latest/optimized-first.json')
        cls.references=[read(f'validation/references/tum-{i}.json') for i in range(8)]

    def profile(self,value):c.audit_profile(value,self.reference,c.expected(),True,False)
    def image(self,value):c.check_image(value,self.saved,self.reference)
    def fifo(self,value):return c.clock.audit_fifo(value,self.source,self.inspection,c.freshness,c.tracking)
    def latest(self,value):return c.clock.audit_latest(value,self.source,self.inspection,self.references,c.expected,c.check_image)

    def test_actual_profiles_parallel_outputs_and_both_clocks_pass(self):
        self.profile(self.__class__.real_profile)
        self.image(self.__class__.real_image)
        self.fifo(self.__class__.real_fifo)
        self.latest(self.__class__.real_latest)
        c.check_sources()

    def test_changed_raw_output_hash_and_false_equality_flag_rejected(self):
        for key,value in [('output_tensor_f32le_sha256','0'*64),('output_bitwise_equal',False)]:
            bad=copy.deepcopy(self.__class__.real_profile);bad['views'][0][key]=value
            with self.assertRaises(ValueError):self.profile(bad)

    def test_wrong_S_output_geometry_cannot_reuse_the_Nano_audit(self):
        bad=copy.deepcopy(self.__class__.real_profile);bad['views'][0]['output_values']=714000
        with self.assertRaises(ValueError):self.profile(bad)

    def test_one_ulp_threshold_relaxation_is_rejected(self):
        bad=copy.deepcopy(self.__class__.real_profile);bad['score_threshold']=adjacent(.3)
        with self.assertRaises(ValueError):self.profile(bad)

    def test_generic_Resize_cannot_be_hidden_by_valid_clock_totals(self):
        bad=copy.deepcopy(self.__class__.real_profile);p=bad['views'][0]['profile']
        row=next(r for r in p['operators'] if r['operation']=='MultiBroadcastTo' and r['node_name'].endswith('.repeat2'))
        row['operation']='Resize'
        calls=collections.Counter();times=collections.Counter()
        for r in p['operators']:calls[r['operation']]+=1;times[r['operation']]+=r['elapsed_ns']
        p['by_operation']=[dict(operation=n,calls=calls[n],elapsed_ns=times[n]) for n in sorted(times,key=lambda n:(-times[n],n))]
        with self.assertRaises(ValueError):self.profile(bad)

    def test_one_ulp_operator_clock_change_is_rejected(self):
        bad=copy.deepcopy(self.__class__.real_profile);bad['views'][0]['profile']['operators'][0]['finished_ns']+=1
        with self.assertRaises(ValueError):self.profile(bad)

    def test_one_ulp_parallel_measurement_change_is_rejected(self):
        bad=copy.deepcopy(self.__class__.real_image);d=bad['base_prediction']['views'][0]['prediction']['detections'][0]
        d['confidence']=adjacent(d['confidence'])
        with self.assertRaises(ValueError):self.image(bad)

    def test_complete_source_closure_and_producer_are_required(self):
        for field,value in [('compiled_sources',{}),('algorithm','yolox_nano_parallel_image_analysis_v1'),('control_permission',True)]:
            bad=copy.deepcopy(self.__class__.real_image);bad[field]=value
            with self.assertRaises(ValueError):self.image(bad)

    def test_one_ulp_FIFO_clock_change_is_rejected(self):
        bad=copy.deepcopy(self.__class__.real_fifo);t=bad['events'][0]['measured']['timing']
        t['sample_submitted_s']=math.nextafter(t['sample_submitted_s'],math.inf)
        with self.assertRaises(ValueError):self.fifo(bad)

    def test_original_recovery_policy_cannot_be_relaxed(self):
        bad=copy.deepcopy(self.__class__.real_fifo);bad['policy']['recovery_samples']=1
        with self.assertRaises(ValueError):self.fifo(bad)

    def test_false_intermediate_ready_is_rejected_by_the_independent_guard(self):
        bad=copy.deepcopy(self.__class__.real_fifo)
        bad['events'][0]['measured']['guard_events'][0]['output']['ready']=True
        with self.assertRaises(ValueError):self.fifo(bad)

    def test_latest_drop_and_future_source_cannot_be_hidden(self):
        bad=copy.deepcopy(self.__class__.real_latest)
        row=next(r for r in bad['events'] if r['superseded_sources']);row['superseded_sources'].pop()
        with self.assertRaises(ValueError):self.latest(bad)
        bad=copy.deepcopy(self.__class__.real_latest)
        row=next(r for r in bad['events'] if r['superseded_sources']);row['selection_checked_s']=row['source']['release_offset_s']-.00001
        with self.assertRaises(ValueError):self.latest(bad)

    def test_dropped_sources_never_count_as_available(self):
        bad=copy.deepcopy(self.__class__.real_latest)
        bad['summary']['available_at_dispatched_checks']+=bad['summary']['superseded_images']
        with self.assertRaises(ValueError):self.latest(bad)


if __name__=='__main__':unittest.main()
