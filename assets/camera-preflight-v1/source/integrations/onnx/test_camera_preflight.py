"""Reject bad original data and changed bytes before any model callback."""
import copy
import hashlib
import inspect
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
from PIL import Image
import camera_preflight as p
import camera_preflight_check as runner


class PreflightChecks(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        self.design=runner.load(runner.HERE/'profile-preflight.json')
        self.a=dict(name='fixture',attributes={},frames=[dict(timestamp=10000,objects=[dict(id=1,category='car',attributes={'occluded':True,'truncated':True},box2d=dict(x1=10.,y1=20.,x2=30.,y2=40.))])])
        self.image=self.root/'fixture.jpg';Image.new('RGB',(1280,720),(21,34,55)).save(self.image,format='JPEG')
        self.label=self.root/'fixture.json';self.save(self.a)
    def save(self,a):
        self.label.write_text(json.dumps(a));self.row=dict(id='fixture',partition='fresh_sample',path=self.image,annotation=self.label,image_sha256=p.sha(self.image.read_bytes()),annotation_sha256=p.sha(self.label.read_bytes()))
    def rejected(self,change,code):
        a=copy.deepcopy(self.a);change(a);self.save(a);r,t=p.inspect([self.row],self.design)
        self.assertFalse(r['accepted']);self.assertIsNone(t);self.assertIn(code,[e['code'] for e in r['inputs'][0]['issues']])
        calls=[]
        with self.assertRaises(p.Rejected):p.guarded_call(t,self.row,lambda:calls.append(True))
        self.assertEqual(calls,[])
    def test_valid_original_allows_callback_without_changing_boxes(self):
        before=self.label.read_bytes();r,t=p.inspect([self.row],self.design);calls=[];p.guarded_call(t,self.row,lambda:calls.append(1))
        self.assertTrue(r['accepted']);self.assertEqual(calls,[1]);self.assertEqual(self.label.read_bytes(),before);self.assertEqual(r['inputs'][0]['annotation']['mapped_box_count'],1)
    def test_zero_height_blocks_callback(self):self.rejected(lambda a:a['frames'][0]['objects'][0]['box2d'].__setitem__('y2',20.),'box_bounds')
    def test_zero_width_blocks_callback(self):self.rejected(lambda a:a['frames'][0]['objects'][0]['box2d'].__setitem__('x2',10.),'box_bounds')
    def test_out_of_frame_box_is_not_clipped(self):self.rejected(lambda a:a['frames'][0]['objects'][0]['box2d'].__setitem__('x1',-1.),'box_bounds')
    def test_duplicate_reference_id_blocks_callback(self):self.rejected(lambda a:a['frames'][0]['objects'].append(copy.deepcopy(a['frames'][0]['objects'][0])),'reference_identity')
    def test_unknown_box_category_blocks_callback(self):self.rejected(lambda a:a['frames'][0]['objects'][0].__setitem__('category','unknown'),'category_scope')
    def test_wrong_identity_blocks_callback(self):self.rejected(lambda a:a.__setitem__('name','other'),'frame_identity')
    def test_boolean_timestamp_is_not_an_integer_timestamp(self):self.rejected(lambda a:a['frames'][0].__setitem__('timestamp',True),'frame_timestamp')
    def test_nonfinite_json_blocks_callback(self):self.rejected(lambda a:a['frames'][0]['objects'][0]['box2d'].__setitem__('x1',float('nan')),'nonfinite_json')
    def test_duplicate_json_keys_are_rejected(self):
        with self.assertRaises(p.Rejected):p.strict_json(b'{"name":"a","name":"b"}')
    def test_unscored_polygons_keep_original_scorer_scope(self):
        a=copy.deepcopy(self.a);a['frames'][0]['objects'].append(dict(id=2,category='lane',poly2d=[]));self.save(a);r,t=p.inspect([self.row],self.design)
        self.assertTrue(r['accepted']);self.assertEqual(r['inputs'][0]['annotation']['box_count'],1)
    def test_traffic_sign_boxes_are_validated_despite_unscored_category(self):
        self.rejected(lambda a:a['frames'][0]['objects'][0].update(category='traffic sign',box2d=dict(x1=0,y1=0,x2=0,y2=20)),'box_bounds')
    def test_truncated_jpeg_blocks_callback(self):
        self.image.write_bytes(self.image.read_bytes()[:100]);self.row['image_sha256']=p.sha(self.image.read_bytes());r,t=p.inspect([self.row],self.design);self.assertFalse(r['accepted']);self.assertIsNone(t)
    def test_source_hash_mismatch_blocks_callback(self):
        self.row['annotation_sha256']='a'*64;r,t=p.inspect([self.row],self.design);self.assertFalse(r['accepted']);self.assertEqual(r['inputs'][0]['issues'][0]['code'],'source_hash')
    def test_changed_annotation_after_pass_never_reaches_callback(self):
        r,t=p.inspect([self.row],self.design);self.label.write_bytes(self.label.read_bytes()+b' ');calls=[]
        with self.assertRaises(p.Rejected):p.guarded_call(t,self.row,lambda:calls.append(1))
        self.assertEqual(calls,[])
    def test_symlink_after_pass_never_reaches_callback(self):
        r,t=p.inspect([self.row],self.design);target=self.root/'copy.jpg';target.write_bytes(self.image.read_bytes());self.image.unlink();self.image.symlink_to(target)
        with self.assertRaises(p.Rejected):p.guarded_call(t,self.row,lambda:self.fail('model called'))
    def test_actual_controls_stop_before_any_native_subprocess(self):
        a=copy.deepcopy(self.a);a['frames'][0]['objects'][0]['box2d']['x2']=10.;self.save(a)
        args=SimpleNamespace(data=self.root,output=self.root/'rejected')
        with patch.object(runner,'records',return_value=[self.row]),patch.object(runner,'native') as native:
            with self.assertRaises(ValueError):runner.controls(args)
            native.assert_not_called()
        self.assertFalse((args.output/'inputs').exists());self.assertFalse(runner.load(args.output/'preflight.json')['accepted'])
    def test_original_auditor_only_changes_explicit_protocol_and_parent_namespace(self):
        original=inspect.getsource(runner.old.audit_saved).replace("digest(HERE/'profile-preserved.json')","digest(HERE/'profile-preflight.json')").replace('parents(out,row,mode,control)','old.parents(out,row,mode,control)')
        self.assertEqual(inspect.getsource(runner.audit_saved),original)


if __name__=='__main__':unittest.main()
