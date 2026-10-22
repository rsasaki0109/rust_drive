"""Model-free support decisions, immutable coarse results and receipt mutations."""
import copy
import inspect
import unittest
import camera_supported_check as c
import test_camera_grid as grid_tests


class SupportedChecks(unittest.TestCase):
    def setUp(self):
        old=grid_tests.GridChecks();old.setUp();self.row=old.row;self.pixels=old.pixel
        source=old.report
        source['compiled_sources']=c.expected();source['base_prediction']['compiled_sources']=c.expected()
        for view in source['base_prediction']['views']:view['prediction']['compiled_sources']=c.expected()
        self.source=source
        self.report=dict(schema_version=1,algorithm='nano_supported_grid_image_v1',compiled_sources=c.expected(),profile_sha256=c.digest(c.HERE/'profile-supported.json'),source_image_sha256=self.row['image_sha256'],source_analysis=source,control_permission=False,result=c.supported_oracle(c.make_fixture(source)))
    def audit(self):c.check_image(self.report,self.row,self.pixels)
    def rejected(self,change):
        change(self.report)
        with self.assertRaises((ValueError,AssertionError,KeyError)):self.audit()
    def test_complete_empty_image_report_is_valid(self):self.audit()
    def test_source_closure_is_required(self):self.rejected(lambda r:r.__setitem__('compiled_sources',{}))
    def test_missing_view_is_rejected(self):self.rejected(lambda r:r['source_analysis']['base_prediction']['views'].pop())
    def test_worker_limit_is_bound(self):self.rejected(lambda r:r['source_analysis']['execution'].__setitem__('view_workers',12))
    def test_image_identity_is_required(self):self.rejected(lambda r:r.__setitem__('source_image_sha256','e'*64))
    def test_forged_support_work_is_rejected(self):self.rejected(lambda r:r['result']['stats'].__setitem__('iou_comparisons',1))
    def test_undeclared_coarse_replacement_is_rejected(self):self.rejected(lambda r:r['result']['detections'].append(dict(class_index=2,confidence=.9,bbox=[1.,1.,5.,5.])))
    def test_forged_original_graph_is_rejected(self):self.rejected(lambda r:r['source_analysis'].__setitem__('original_graph_reference',True))
    def test_original_grid_audit_body_is_preserved_with_explicit_source_parameter(self):
        old=inspect.getsource(c.grid.audit_image).replace('def audit_image(actual,row,pixels,grid=True,reference=False):\n    sources=expected() if grid else prior.expected()','def audit_source(actual,row,pixels,pins,reference=False):\n    grid=True\n    sources=pins')
        self.assertEqual(inspect.getsource(c.audit_source),old)
    def test_one_view_cannot_provide_two_votes(self):
        f=c.grid.fixture();d=dict(class_index=9,confidence=.8,bbox=[300.,100.,350.,150.])
        f['views'][4]['detections']=[d,dict(d,confidence=.7)]
        result=c.supported_oracle(f);self.assertEqual(result['stats']['added'],0)
        self.assertTrue(all(len(x['witnesses'])==1 for x in result['decisions']))
    def test_correlated_support_adds_one_box_without_changing_its_measurements(self):
        f=c.grid.fixture();f['views'][4]['detections']=[dict(class_index=9,confidence=.8,bbox=[300.,100.,350.,150.])]
        f['views'][5]['detections']=[dict(class_index=9,confidence=.7,bbox=[12.,100.,62.,150.])]
        result=c.supported_oracle(f);self.assertEqual(result['stats']['added'],1);self.assertEqual(result['detections'][0]['bbox'],[300.,100.,350.,150.])
        changed=copy.deepcopy(result);changed['decisions'][0]['witnesses'].pop()
        with self.assertRaises((ValueError,AssertionError)):c.prior.compare(result,changed,'witness identity')


if __name__=='__main__':unittest.main()
