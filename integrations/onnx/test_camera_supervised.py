"""Reject corrupted scheduling, provenance and supervision claims without inference."""
import copy
import gzip
import json
import math
from pathlib import Path
import unittest

import camera_supervised_check as c

PACKET=c.ROOT/'assets/camera-supervised-v1'
RAW=c.ROOT/'artifacts/camera-supervised-v1'


def read(name):
    if PACKET.exists():
        path=PACKET/name
        if path.exists():return json.loads(path.read_text())
        return json.loads(gzip.decompress(Path(str(path)+'.gz').read_bytes()))
    return c.load(RAW/name)


class SupervisionAudit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source=c.load(c.ROOT/'assets/camera-sequence-v1/sequence.json')
        cls.report=read('validation/supervised-first.json')
        cls.inspection=read('validation/supervised-inspection.json')
        cls.fault=read('validation/fault-supervised-first.json')
        cls.fault_source=read('validation/fault-sequence.json')
        cls.fault_inspection=read('validation/fault-supervised-inspection.json')

    def reject(self, mutate, fault=False):
        report=copy.deepcopy(self.fault if fault else self.report)
        mutate(report)
        with self.assertRaises((ValueError,AssertionError,KeyError,StopIteration)):
            c.audit(report,self.fault_source if fault else self.source,self.fault_inspection if fault else self.inspection)

    def test_original_reports_are_auditable_without_inference(self):
        c.audit(self.report,self.source,self.inspection)
        c.audit(self.fault,self.fault_source,self.fault_inspection)

    def test_rejects_one_ulp_clock_renewal(self):
        def mutate(r):
            row=next(x for x in r['trace'] if x['change']['kind']=='guard' and x['change']['event']['input']['kind']=='sample')
            data=row['change']['event']['input']; data['acquired_s']=math.nextafter(data['acquired_s'],math.inf)
        self.reject(mutate)

    def test_rejects_false_in_flight_identity(self):
        self.reject(lambda r:next(x for x in r['trace'] if x['change']['kind']=='started').__setitem__('active_source_index',31))

    def test_rejects_unknown_or_future_source(self):
        self.reject(lambda r:next(x for x in r['trace'] if x['change']['kind']=='source')['change'].__setitem__('event_index',7))

    def test_rejects_dropped_image_disappearance(self):
        self.reject(lambda r:r['trace'].remove(next(x for x in r['trace'] if x['change']['kind']=='superseded')))

    def test_rejects_model_source_or_policy_substitution(self):
        for mutate in (lambda r:r.__setitem__('model_sha256','0'*64),lambda r:r['compiled_sources'].pop(next(iter(r['compiled_sources']))),lambda r:r['policy'].__setitem__('max_age_s',.3)):
            self.reject(mutate)

    def test_rejects_fault_kind_suppression_during_inference(self):
        def mutate(r):
            row=next(x for x in r['trace'] if x['change']['kind']=='guard' and x['change']['reason']=='source_barrier' and x['change']['event']['input']['kind']=='failed')
            row['change']['event']['input']['kind']='tick'
        self.reject(mutate,True)

    def test_rejects_worker_finish_before_dispatch(self):
        self.reject(lambda r:next(x for x in r['trace'] if x['change']['kind']=='finished')['change'].__setitem__('worker_started_s',0.))

    def test_rejects_forged_ready_result(self):
        self.reject(lambda r:next(x for x in r['trace'] if x['change']['kind']=='guard')['change']['output'].__setitem__('ready',True))

    def test_rejects_missing_post_tracking_or_final_check(self):
        for reason in ('post_tracking_poll','final_poll'):
            self.reject(lambda r,reason=reason:r['trace'].remove(next(x for x in r['trace'] if x['change']['kind']=='guard' and x['change']['reason']==reason)))

    def test_rejects_inflated_summary_or_forward_counts(self):
        self.reject(lambda r:r['summary'].__setitem__('available_result_images',8))
        self.reject(lambda r:r.__setitem__('model_forwards',r['model_forwards']+1))

    def test_supervision_overlap_uses_worker_finish_clock(self):
        r=copy.deepcopy(self.fault)
        metrics=c.supervision_metrics(r)
        self.assertEqual(len(metrics['fault_observations']),1)
        self.assertFalse(metrics['hard_real_time_guarantee'])
        completion=next(x['change'] for x in r['trace'] if x['change']['kind']=='finished')
        completion['worker_finished_s']=0.
        self.assertFalse(c.supervision_metrics(r)['fault_observations'][0]['delivered_during_actual_inference'])


if __name__=='__main__':unittest.main()
