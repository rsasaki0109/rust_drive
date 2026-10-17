"""Independent prefix, decision, signed-zero and receipt corruption checks."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
import camera_preserved_check as c


class PreservedChecks(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.path=Path(self.temp.name)/'fixture.json'
        self.f=c.fixture([c.box(2,.4)],[c.box(7,.99),c.box(9,.8,(500.,100.,510.,120.))]);c.write(self.path,self.f)
        self.r=dict(schema_version=1,algorithm='baseline_preserving_camera_fixture_v1',compiled_sources=c.expected(),profile_sha256=c.digest(c.HERE/'profile-preserved.json'),source_fixture_sha256=c.digest(self.path),model_read=False,images_read=False,model_forwards=0,control_permission=False,result=c.oracle(self.f))
    def audit(self):c.check(self.r,self.path)
    def rejected(self,change):
        change(self.r)
        with self.assertRaises((ValueError,AssertionError,KeyError)):self.audit()
    def test_complete_result_is_valid(self):self.audit()
    def test_higher_score_other_class_never_changes_anchor(self):
        r=self.r['result'];self.assertEqual(r['detections'][0]['class_index'],2);self.assertEqual(r['decisions'][0]['reason'],'anchor_overlap');self.assertEqual(r['stats']['added'],1)
    def test_source_identity_is_required(self):self.rejected(lambda r:r.__setitem__('compiled_sources',{}))
    def test_model_execution_cannot_be_claimed(self):self.rejected(lambda r:r.__setitem__('model_forwards',1))
    def test_wrong_fixture_bytes_are_rejected(self):self.rejected(lambda r:r.__setitem__('source_fixture_sha256','a'*64))
    def test_rescored_anchor_is_rejected(self):self.rejected(lambda r:r['result']['detections'][0].__setitem__('confidence',.99))
    def test_forged_lineage_is_rejected(self):self.rejected(lambda r:r['result']['lineage'][0].__setitem__('stream','detail'))
    def test_forged_decision_is_rejected(self):self.rejected(lambda r:r['result']['decisions'][0].__setitem__('reason','added'))
    def test_forged_work_is_rejected(self):self.rejected(lambda r:r['result']['stats'].__setitem__('iou_comparisons',0))
    def test_signed_zero_is_part_of_proposal_order_and_preserved_measurement(self):
        f=c.fixture([], [c.box(9,.8,(0.,10.,30.,40.)),c.box(9,.8,(-0.,10.,30.,40.))]);r=c.oracle(f)
        self.assertEqual(r['lineage'][0]['source_rank'],1);self.assertEqual(c.total_key(r['detections'][0]['bbox'][0]),c.total_key(-0.0))
    def test_full_anchor_leaves_no_room_for_any_proposal(self):
        f=c.fixture([c.box(9,.5,(float(i%20*5),float(i//20*5),float(i%20*5+2),float(i//20*5+2))) for i in range(300)],[c.box()]);r=c.oracle(f)
        self.assertEqual(len(r['detections']),300);self.assertEqual(r['stats']['output_cap'],1);self.assertEqual(r['stats']['iou_comparisons'],0)
    def test_parent_streams_are_the_original_frozen_producers(self):
        old=c.load(c.HERE/'profile-detail.json');identity=old['viewed_ids'][0]
        self.assertEqual(c.parent(identity,'first','baseline'),c.detail.parent(identity,'first'))
        self.assertIn('camera-detail-v1/validation/detail/first',str(c.parent(identity,'first','detail')))


if __name__=='__main__':unittest.main()
