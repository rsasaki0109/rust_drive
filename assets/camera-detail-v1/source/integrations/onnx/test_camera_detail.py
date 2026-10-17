"""Model-free S/Nano identity, complete views and unchanged support policy."""
import copy
import inspect
import unittest
import numpy as np
import camera_detail_check as c


class DetailChecks(unittest.TestCase):
    def setUp(self):
        self.row=dict(id='fixture',image_sha256='a'*64)
        self.pixels=dict(views=[dict(rectangle=r,rgb_sha256='b'*64) for r in c.rectangles(1280,720)])
        f=c.fixture();views=[]
        for pixel in self.pixels['views']:
            r=pixel['rectangle'];p=dict(schema_version=1,algorithm='pinned_s_detail_repeat2x_triangle_v1',model_profile='yolox-s-detail-v1',compiled_sources=c.expected(),model_profile_sha256=c.digest(c.HERE/'profile-detail.json'),input_shape=[1,3,640,640],output_shape=[1,8400,85],model_sha256=c.MODEL_SHA,source_image_sha256=None,width=r['width'],height=r['height'],score_threshold=.3,nms_iou_threshold=.45,model_load_seconds=.1,inference_seconds=.1,detections=[])
            views.append(dict(rectangle=r,rgb_sha256=pixel['rgb_sha256'],prediction=p))
        base=dict(schema_version=1,algorithm='pinned_yolox_s_detail_grid_fixed_views_v1',compiled_sources=c.expected(),source_image_sha256=self.row['image_sha256'],model_sha256=c.MODEL_SHA,width=1280,height=720,score_threshold=.3,nms_iou_threshold=.45,views=views,**c.merge(f))
        source=dict(schema_version=1,algorithm='yolox_s_detail_grid_image_analysis_v1',compiled_sources=c.expected(),source_image_sha256=self.row['image_sha256'],original_graph_reference=False,profile_sha256=c.digest(c.HERE/'profile-detail.json'),control_permission=False,base_prediction=base,result=c.oracle(f),execution=dict(model_read=True,images_read=1,model_forwards=12,inference_run=True,view_workers=4,shared_model_plans=1,analysis_wall_s=.5))
        self.report=dict(schema_version=1,algorithm='s_detail_grid_image_v1',compiled_sources=c.expected(),profile_sha256=c.digest(c.HERE/'profile-detail.json'),source_image_sha256=self.row['image_sha256'],source_analysis=source,control_permission=False,result=c.supported_oracle(f))
    def audit(self):c.check_image(self.report,self.row,self.pixels)
    def rejected(self,change):
        change(self.report)
        with self.assertRaises((ValueError,AssertionError,KeyError)):self.audit()
    def test_complete_empty_measurements_are_valid(self):self.audit()
    def test_missing_supplemental_view_is_rejected(self):self.rejected(lambda r:r['source_analysis']['base_prediction']['views'].pop())
    def test_source_closure_is_required(self):self.rejected(lambda r:r.__setitem__('compiled_sources',{}))
    def test_nano_model_cannot_be_admitted_as_s(self):self.rejected(lambda r:r['source_analysis']['base_prediction']['views'][4]['prediction'].__setitem__('model_sha256',c.prior.MODEL_SHA))
    def test_wrong_model_dimensions_are_rejected(self):self.rejected(lambda r:r['source_analysis']['base_prediction']['views'][4]['prediction'].__setitem__('input_shape',[1,3,416,416]))
    def test_wrong_crop_pixels_are_rejected(self):self.rejected(lambda r:r['source_analysis']['base_prediction']['views'][4].__setitem__('rgb_sha256','c'*64))
    def test_threshold_is_bound_to_every_view(self):self.rejected(lambda r:r['source_analysis']['base_prediction']['views'][8]['prediction'].__setitem__('score_threshold',.2))
    def test_incomplete_forward_count_is_rejected(self):self.rejected(lambda r:r['source_analysis']['execution'].__setitem__('model_forwards',4))
    def test_worker_budget_is_bound(self):self.rejected(lambda r:r['source_analysis']['execution'].__setitem__('view_workers',10))
    def test_original_graph_identity_is_bound(self):self.rejected(lambda r:r['source_analysis'].__setitem__('original_graph_reference',True))
    def test_forged_merge_work_is_rejected(self):self.rejected(lambda r:r['source_analysis']['base_prediction']['merge'].__setitem__('comparisons',1))
    def test_forged_support_work_is_rejected(self):self.rejected(lambda r:r['result']['stats'].__setitem__('iou_comparisons',1))
    def test_undeclared_result_box_is_rejected(self):self.rejected(lambda r:r['result']['detections'].append(dict(class_index=2,confidence=.8,bbox=[10.,10.,20.,20.])))
    def test_supplemental_grid_covers_source_and_preserves_original_prefix(self):
        rects=c.rectangles(1280,720);self.assertEqual(rects[:4],c.prior.nano.tiles.rectangles(1280,720));self.assertEqual(len(rects),12)
        covered=np.zeros((720,1280),dtype=bool)
        for r in rects[4:]:covered[r['y']:r['y']+r['height'],r['x']:r['x']+r['width']]=True
        self.assertTrue(covered.all())
        for w,h in ((0,10),(2560,1440),(1920,1920),(16000000,1)):
            with self.assertRaises(ValueError):c.rectangles(w,h)
    def test_source_auditor_changes_only_its_explicit_variant_contract(self):
        original=inspect.getsource(c.baseline.check_source).replace('capacity','detail').replace('len(views)==10','len(views)==12').replace("len(pixels['views'])==10","len(pixels['views'])==12").replace('ten view','twelve view').replace('ten forwards','twelve forwards').replace('model_forwards=10','model_forwards=12')
        self.assertEqual(inspect.getsource(c.check_source),original)
        self.assertIs(c.supported_oracle,c.baseline.previous.supported_oracle)
    def test_repeated_measurements_do_not_accept_different_class_or_coordinate_bits(self):
        source=self.report['source_analysis'];altered=copy.deepcopy(source)
        altered['base_prediction']['views'][9]['prediction']['detections']=[dict(class_index=2,confidence=.9,bbox=[10.,10.,20.,20.])]
        with self.assertRaises((ValueError,AssertionError)):c.grid.same_views(source,altered)


if __name__=='__main__':unittest.main()
