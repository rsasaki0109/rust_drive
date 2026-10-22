"""Model-free coverage, receipt corruption and complete-result regressions."""
import copy
import unittest
import numpy as np
import camera_grid_check as c


class GridChecks(unittest.TestCase):
    def setUp(self):
        self.row=dict(id='fixture',image_sha256='a'*64)
        self.pixel=dict(views=[dict(rectangle=r,rgb_sha256='b'*64) for r in c.rectangles(1280,720)])
        f=c.fixture();views=[]
        for v in self.pixel['views']:
            r=v['rectangle'];p=dict(schema_version=1,algorithm='pinned_nano_grid_depthwise_triangle_v1',model_profile='yolox-nano-grid-v1',compiled_sources=c.expected(),model_profile_sha256=c.digest(c.HERE/'profile-grid.json'),input_shape=[1,3,416,416],output_shape=[1,3549,85],model_sha256=c.prior.MODEL_SHA,source_image_sha256=None,width=r['width'],height=r['height'],score_threshold=.3,nms_iou_threshold=.45,model_load_seconds=.1,inference_seconds=.1,detections=[])
            views.append(dict(rectangle=r,rgb_sha256=v['rgb_sha256'],prediction=p))
        base=dict(schema_version=1,algorithm='pinned_yolox_nano_grid_fixed_views_v1',compiled_sources=c.expected(),source_image_sha256=self.row['image_sha256'],model_sha256=c.prior.MODEL_SHA,width=1280,height=720,score_threshold=.3,nms_iou_threshold=.45,views=views,**c.merge(f))
        self.report=dict(schema_version=1,algorithm='yolox_nano_grid_image_analysis_v1',compiled_sources=c.expected(),source_image_sha256=self.row['image_sha256'],original_graph_reference=False,profile_sha256=c.digest(c.HERE/'profile-grid.json'),control_permission=False,base_prediction=base,result=c.oracle(f),execution=dict(model_read=True,images_read=1,model_forwards=12,inference_run=True,view_workers=4,shared_model_plans=1,analysis_wall_s=.5))

    def audit(self):c.audit_image(self.report,self.row,self.pixel)
    def rejected(self,change):
        change(self.report)
        with self.assertRaises((ValueError,AssertionError,KeyError)):self.audit()

    def test_complete_empty_measurements_are_valid(self):
        self.audit();c.same_views(self.report,copy.deepcopy(self.report))

    def test_small_tiles_cover_all_pixels_and_preserve_original_prefix(self):
        rects=c.rectangles(1280,720)
        self.assertEqual(rects[:4],c.prior.nano.tiles.rectangles(1280,720));self.assertEqual(len(rects),12)
        covered=np.zeros((720,1280),dtype=bool)
        for r in rects[4:]:covered[r['y']:r['y']+r['height'],r['x']:r['x']+r['width']]=True
        self.assertTrue(covered.all())

    def test_excessive_work_or_empty_geometry_is_rejected(self):
        for w,h in ((0,10),(1920,1080),(1280,1280),(16000000,1)):
            with self.assertRaises(ValueError):c.rectangles(w,h)

    def test_missing_supplemental_view_is_rejected(self):self.rejected(lambda r:r['base_prediction']['views'].pop())
    def test_wrong_original_pixels_are_rejected(self):self.rejected(lambda r:r['base_prediction']['views'][4].__setitem__('rgb_sha256','c'*64))
    def test_source_closure_is_required(self):self.rejected(lambda r:r.__setitem__('compiled_sources',{}))
    def test_threshold_is_bound_to_every_view(self):self.rejected(lambda r:r['base_prediction']['views'][5]['prediction'].__setitem__('score_threshold',.2))
    def test_worker_budget_is_verified(self):self.rejected(lambda r:r['execution'].__setitem__('view_workers',12))
    def test_complete_forward_count_is_verified(self):self.rejected(lambda r:r['execution'].__setitem__('model_forwards',4))
    def test_reference_graph_cannot_be_misidentified(self):self.rejected(lambda r:r.__setitem__('original_graph_reference',True))
    def test_forged_global_merge_is_rejected(self):self.rejected(lambda r:r['base_prediction']['merge'].__setitem__('comparisons',1))
    def test_nonfinite_observation_is_rejected(self):self.rejected(lambda r:r['base_prediction']['views'][0]['prediction'].__setitem__('inference_seconds',float('nan')))

    def test_current_per_view_disagreement_is_rejected(self):
        altered=copy.deepcopy(self.report)
        altered['base_prediction']['views'][9]['prediction']['detections']=[dict(class_index=2,confidence=.9,bbox=[1.,1.,10.,10.])]
        with self.assertRaises((ValueError,AssertionError)):c.same_views(self.report,altered)


if __name__=='__main__':unittest.main()
