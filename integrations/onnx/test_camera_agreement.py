"""Independent fixture receipts, witness uniqueness and measurement preservation."""
import copy
from pathlib import Path
import tempfile
import unittest
import camera_agreement_check as c

class AgreementChecks(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.path=Path(self.tmp.name)/'fixture.json'
        self.f=c.fixture([c.box(0,.4,(-0.,10.,20.,30.))],[c.box(9,.8,(100.,100.,120.,120.))],[c.box(9,.7,(101.,100.,121.,120.))]);c.write(self.path,self.f)
        self.r=dict(schema_version=1,algorithm='mutual_scale_agreement_fixture_v1',compiled_sources=c.expected(),profile_sha256=c.digest(c.HERE/'profile-agreement.json'),source_fixture_sha256=c.digest(self.path),model_read=False,images_read=False,model_forwards=0,control_permission=False,result=c.oracle(self.f))
    def reject(self,change):
        change(self.r)
        with self.assertRaises((ValueError,AssertionError,KeyError)):c.check(self.r,self.path)
    def test_complete_receipt_passes(self):c.check(self.r,self.path)
    def test_source_identity_is_required(self):self.reject(lambda r:r.__setitem__('compiled_sources',{}))
    def test_fixture_bytes_are_required(self):self.reject(lambda r:r.__setitem__('source_fixture_sha256','a'*64))
    def test_cannot_claim_inference(self):self.reject(lambda r:r.__setitem__('model_forwards',1))
    def test_measured_confidence_cannot_change(self):self.reject(lambda r:r['result']['detections'][1].__setitem__('confidence',.9))
    def test_measured_coordinates_cannot_change(self):self.reject(lambda r:r['result']['detections'][1]['bbox'].__setitem__(0,102.))
    def test_coarse_signed_zero_is_preserved(self):self.reject(lambda r:r['result']['detections'][0]['bbox'].__setitem__(0,0.))
    def test_lineage_cannot_change(self):self.reject(lambda r:r['result']['lineage'][0].__setitem__('stream','wide'))
    def test_witness_rank_cannot_change(self):self.reject(lambda r:r['result']['decisions'][0].__setitem__('detail_rank',0))
    def test_iou_bits_cannot_change(self):self.reject(lambda r:r['result']['decisions'][0].__setitem__('agreement_iou_bits',0))
    def test_decision_cannot_change(self):self.reject(lambda r:r['result']['decisions'][0].__setitem__('reason','non_mutual'))
    def test_work_cannot_change(self):self.reject(lambda r:r['result']['stats'].__setitem__('iou_comparisons',0))
    def test_one_witness_does_not_support_two_equal_candidates(self):
        r=c.oracle(c.fixture([], [c.box(),c.box()],[c.box()]));self.assertEqual(r['stats']['added'],1);self.assertEqual(r['decisions'][1]['reason'],'non_mutual')
    def test_coarse_prefix_cannot_supply_supplemental_agreement(self):
        r=c.oracle(c.fixture([c.box(2)],[c.box(2)],[]));self.assertEqual(r['decisions'][0]['reason'],'no_agreement')
    def test_duplicate_origins_are_rejected(self):
        f=copy.deepcopy(self.f);f['wide_origins'][1]=f['wide_origins'][0]
        with self.assertRaises(ValueError):c.oracle(f)
    def test_parent_measurement_mutation_cannot_change_coarse_fixture(self):
        f=c.fixture([c.box()]);f['detail'][0]['confidence']=.9
        self.assertEqual(f['coarse'][0]['confidence'],.8)
        with self.assertRaises(ValueError):c.oracle(f)

if __name__=='__main__':unittest.main()
