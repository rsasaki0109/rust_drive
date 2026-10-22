"""Corrupt frozen raw, timing and producer records; execute no model."""
import copy
import gzip
import json
import math
from pathlib import Path
import struct
import unittest

import camera_rowzip_check as c
PACKET=c.ROOT/'assets/camera-rowzip-v1'
RAW=c.ROOT/'artifacts/camera-rowzip-v1'


def read(name):
    root=PACKET if PACKET.exists() else RAW
    p=root/name
    return json.loads(p.read_bytes() if p.exists() else gzip.decompress(Path(str(p)+'.gz').read_bytes()))


class RowzipAudit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile=read('validation/profiles/tum-0-optimized.json')
        cls.serial=read('validation/references/tum-0.json')
        cls.image=read('validation/images/tum-0.json')
        cls.saved=c.load(c.ROOT/'assets/camera-sequence-v1/recorded/exports/first-0.json')
        cls.supervised=read('validation/supervised/optimized-first.json')
        cls.inspection=read('validation/optimized-inspection.json')
        cls.source=c.load(c.ROOT/'assets/camera-sequence-v1/sequence.json')

    def reject_profile(self,mutate):
        r=copy.deepcopy(self.profile);mutate(r)
        with self.assertRaises((ValueError,AssertionError,KeyError)):
            c.audit_profile(r,self.serial,c.expected(),True,False)

    def reject_supervised(self,mutate):
        r=copy.deepcopy(self.supervised);mutate(r)
        with self.assertRaises((ValueError,AssertionError,KeyError)):
            c.audit_supervised(r,self.source,self.inspection)

    def test_original_raw_and_measured_traces_are_auditable(self):
        c.audit_profile(self.profile,self.serial,c.expected(),True,False)
        c.check_image(self.image,self.saved,self.serial)
        c.audit_supervised(self.supervised,self.source,self.inspection)

    def test_rejects_replacement_count_or_execution_order_changes(self):
        self.reject_profile(lambda r:r['execution'].__setitem__('depthwise_nodes_replaced',29))
        self.reject_profile(lambda r:r['execution'].__setitem__('order','reference_then_profile_per_view'))

    def test_rejects_raw_hash_or_false_bit_equality(self):
        self.reject_profile(lambda r:r['views'][0].__setitem__('output_tensor_f32le_sha256','0'*64))
        self.reject_profile(lambda r:r['views'][0].__setitem__('output_bitwise_equal',False))

    def test_rejects_wrong_output_shape(self):
        self.reject_profile(lambda r:r['views'][0].__setitem__('output_values',714000))

    def test_rejects_forged_operator_or_group_arithmetic(self):
        self.reject_profile(lambda r:r['views'][0]['profile']['by_operation'][0].__setitem__('elapsed_ns',0))
        self.reject_profile(lambda r:r['views'][0]['profile']['operators'][0].__setitem__('elapsed_ns',0))

    def test_rejects_hidden_original_depthwise_fallback(self):
        def mutate(r):
            p=r['views'][0]['profile']
            for row in p['operators']:
                if row['operation']=='SafeRowDepthWise3x3':row['operation']='DepthWiseConv'
            for group in p['by_operation']:
                if group['operation']=='SafeRowDepthWise3x3':group['operation']='DepthWiseConv'
        self.reject_profile(mutate)

    def test_rejects_one_f32_ulp_in_a_current_detection(self):
        r=copy.deepcopy(self.image)
        p=next(v['prediction'] for v in r['base_prediction']['views'] if v['prediction']['detections'])
        d=p['detections'][0];bits=struct.unpack('<I',struct.pack('<f',d['confidence']))[0]
        d['confidence']=struct.unpack('<f',struct.pack('<I',bits+1))[0]
        with self.assertRaises((ValueError,AssertionError)):
            c.check_image(r,self.saved,self.serial)

    def test_rejects_renewed_source_acquisition_by_one_float_ulp(self):
        def mutate(r):
            row=next(x['change'] for x in r['trace'] if x['change']['kind']=='guard' and x['change']['event']['input']['kind']=='sample')
            data=row['event']['input'];data['acquired_s']=math.nextafter(data['acquired_s'],math.inf)
        self.reject_supervised(mutate)

    def test_rejects_future_dispatch_and_unknown_active_image(self):
        self.reject_supervised(lambda r:next(x for x in r['trace'] if x['change']['kind']=='started').__setitem__('active_source_index',31))
        self.reject_supervised(lambda r:next(x['change'] for x in r['trace'] if x['change']['kind']=='source').__setitem__('event_index',7))

    def test_rejects_policy_or_producer_substitution(self):
        self.reject_supervised(lambda r:r['policy'].__setitem__('recovery_samples',1))
        self.reject_supervised(lambda r:r.__setitem__('compiled_sources',{}))
        self.reject_supervised(lambda r:r.__setitem__('algorithm','yolox_nano_supervised_replay_v1'))

    def test_rejects_forged_readiness_and_coverage(self):
        self.reject_supervised(lambda r:next(x['change'] for x in r['trace'] if x['change']['kind']=='guard')['output'].__setitem__('ready',True))
        self.reject_supervised(lambda r:r['summary'].__setitem__('available_result_images',8))

    def test_rejects_missing_final_expiry_poll(self):
        self.reject_supervised(lambda r:r['trace'].pop())

    def test_rejects_variant_or_profile_substitution(self):
        for key,value in [('producer_variant','old-indexed-rows'),('variant_profile_sha256','0'*64)]:
            r=copy.deepcopy(self.image);r[key]=value
            with self.assertRaises(ValueError):c.variant(r)

    def test_variant_source_receipt_cannot_omit_the_new_kernel(self):
        r=copy.deepcopy(self.image);r['compiled_sources'].pop('integrations/onnx/src/camera_rowzip.rs')
        with self.assertRaises(ValueError):c.variant(r)


if __name__=='__main__':unittest.main()
