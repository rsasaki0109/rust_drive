#!/usr/bin/env python3
"""Bounded original-BDD integrity validation and an in-memory model-call gate."""
from dataclasses import dataclass
import hashlib
import io
import json
import math
from pathlib import Path
import stat
from PIL import Image

LIMIT=4*1024*1024
class Rejected(ValueError):
    def __init__(self,code,location,detail):
        super().__init__(f'{code} at {location}: {detail}');self.issue=dict(code=code,location=location,detail=detail)
def need(condition,code,location,detail):
    if not condition:raise Rejected(code,location,detail)
def sha(body):return hashlib.sha256(body).hexdigest()
def bounded_read(path):
    p=Path(path);s=p.lstat();need(stat.S_ISREG(s.st_mode),'file_type',str(p),'regular non-symlink input required')
    need(0<s.st_size<=LIMIT,'file_size',str(p),'input must be positive and at most4MiB')
    with p.open('rb') as stream:body=stream.read(LIMIT+1)
    need(len(body)==s.st_size and len(body)<=LIMIT,'file_changed',str(p),'size changed while reading')
    return body
def strict_json(body):
    def pairs(rows):
        result={}
        for k,v in rows:
            need(k not in result,'duplicate_json_key',k,'duplicate keys are not accepted');result[k]=v
        return result
    def constant(value):raise Rejected('nonfinite_json','$',value)
    try:return json.loads(body,object_pairs_hook=pairs,parse_constant=constant)
    except (json.JSONDecodeError,UnicodeDecodeError) as e:raise Rejected('json_syntax','$',str(e)) from e
def annotation(body,identity,design):
    a=strict_json(body);need(type(a) is dict,'annotation_schema','$','object required')
    need(a.get('name')==identity,'frame_identity','name','annotation name must equal fixed image ID')
    frames=a.get('frames');need(type(frames) is list and len(frames)==1,'frame_count','frames','exactly one legacy frame required')
    frame=frames[0];need(type(frame) is dict and type(frame.get('timestamp')) is int and frame['timestamp']==10000,'frame_timestamp','frames[0]','exact legacy timestamp10000 required')
    need(type(a.get('attributes')) is dict,'annotation_attributes','attributes','original attributes object required')
    objects=frame.get('objects');need(type(objects) is list and len(objects)<=4096,'object_inventory','objects','bounded original objects required')
    seen=set();boxes=mapped=unscored=0
    for i,label in enumerate(objects):
        loc=f'objects[{i}]';need(type(label) is dict,'object_schema',loc,'object required')
        identity=label.get('id');need(type(identity) is int and identity>=0 and identity not in seen,'reference_identity',loc,'unique nonnegative integer ID required');seen.add(identity)
        if 'box2d' not in label:continue
        category=label.get('category');need(type(category) is str and category in (design['class_mapping']|design['unscored_category_reason']),'category_scope',loc,'unchanged original category scope required')
        need(type(label.get('attributes')) is dict,'object_attributes',loc,'original attributes object required')
        box=label['box2d'];need(type(box) is dict and all(k in box for k in ('x1','y1','x2','y2')),'box_schema',loc,'complete source-pixel box required')
        values=[box[k] for k in ('x1','y1','x2','y2')]
        need(all(type(v) in (int,float) and math.isfinite(v) for v in values),'box_number',loc,'finite numeric coordinates required')
        l,t,r,b=values
        need(0<=l<r<=1280 and 0<=t<b<=720,'box_bounds',loc,f'nondegenerate original box inside1280x720 required: {values}')
        boxes+=1
        if category in design['class_mapping']:mapped+=1
        else:unscored+=1
    return dict(bytes=len(body),sha256=sha(body),object_count=len(objects),box_count=boxes,mapped_box_count=mapped,unscored_box_count=unscored,original_reference_ids=sorted(seen))
def image(body):
    try:
        with Image.open(io.BytesIO(body)) as im:
            need(im.format=='JPEG' and im.size==(1280,720),'image_geometry','image','original1280x720 JPEG required')
            need(im.mode=='RGB','image_mode','image','original RGB image required')
            im.load()
            return dict(bytes=len(body),sha256=sha(body),format=im.format,width=im.width,height=im.height,mode=im.mode,fully_decoded=True)
    except Rejected:raise
    except (OSError,ValueError,Image.DecompressionBombError) as e:raise Rejected('image_decode','image',str(e)) from e

@dataclass(frozen=True)
class _Ticket:
    pins:tuple
    def verify(self,row):
        pin=next((p for p in self.pins if p[0]==row['id']),None);need(pin is not None,'ticket_identity',row['id'],'fixed accepted row required')
        for key,path,expected in (('path',pin[1],pin[2]),('annotation',pin[3],pin[4])):
            need(str(row[key])==path,'ticket_path',key,'accepted source path changed')
            need(sha(bounded_read(path))==expected,'ticket_changed',key,'accepted original bytes changed before model execution')
    def verify_all(self,rows):
        need(len(rows)==len(self.pins),'ticket_inventory','$','complete accepted inventory required')
        for row in rows:self.verify(row)

def inspect(rows,design):
    results=[];pins=[];seen=set()
    for row in rows:
        entry=dict(id=row['id'],partition=row['partition'],image_sha256=row['image_sha256'],annotation_sha256=row['annotation_sha256'],image=None,annotation=None,issues=[])
        if row['id'] in seen:entry['issues'].append(dict(code='duplicate_image_id',location='id',detail='fixed inputs must be unique'))
        seen.add(row['id'])
        for key,expected,validator in (('annotation',row['annotation_sha256'],lambda b:annotation(b,row['id'],design)),('path',row['image_sha256'],image)):
            try:
                body=bounded_read(row[key]);need(sha(body)==expected,'source_hash',key,'fixed original SHA256 required')
                entry['annotation' if key=='annotation' else 'image']=validator(body)
            except Rejected as error:entry['issues'].append(error.issue)
            except OSError as error:entry['issues'].append(dict(code='file_io',location=key,detail=str(error)))
        results.append(entry);pins.append((row['id'],str(row['path']),row['image_sha256'],str(row['annotation']),row['annotation_sha256']))
    accepted=bool(rows) and all(not r['issues'] for r in results)
    receipt=dict(schema_version=1,algorithm='bounded_original_bdd_preflight_v1',validator_sha256=sha(Path(__file__).read_bytes()),accepted=accepted,row_count=len(rows),inputs=results,numeric_labels_validated=True,labels_used_for_inference=False,images_or_labels_changed=False,model_forwards=0,control_permission=False)
    return receipt,_Ticket(tuple(pins)) if accepted else None

def guarded_call(ticket,row,callback,*args):
    need(isinstance(ticket,_Ticket),'preflight_rejected','$','no model callback without an accepted in-memory ticket')
    ticket.verify(row)
    return callback(*args)
