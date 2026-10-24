#!/usr/bin/env python3
"""Acquire frozen CADC volume-comparison inputs using the audited bounded readers."""
import argparse
import json
import os
from pathlib import Path
import tempfile
from cadc_recorded import require,digest,head,get_small,zip_points,zstd_points,MAX_MEMBER

def acquire(protocol_path,output,pair=None):
    protocol_path=Path(protocol_path);output=Path(output).absolute()
    require(not output.exists(),'dataset destination already exists')
    protocol_bytes=protocol_path.read_bytes();protocol=json.loads(protocol_bytes)
    require(protocol['schema_version']==1 and len(protocol['sequences'])==8,'unsupported frozen protocol')
    wanted=protocol['frames'];require(wanted==list(range(0,80,10)),'frozen annotated frame cadence changed')
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.cadc-staging-',dir=output.parent) as tmp:
        stage=Path(tmp);(stage/'points').mkdir();(stage/'annotations').mkdir()
        sensor={'profile':protocol['profile'],'frames':[]};records=[]
        require(pair in (None,0,1,2,3),'invalid route pair')
        for record in protocol['sequences']:
            if pair is not None and record['pair']!=pair:continue
            head(record)
            annotation=get_small(record['annotation_url'])
            require(digest(annotation)==record['annotation_sha256'],'annotation hash differs')
            labels=json.loads(annotation)
            require(isinstance(labels,list) and len(labels)>max(wanted),'annotation frame protocol differs')
            name=record['domain']+'-'+str(record['pair'])
            label_path='annotations/'+name+'.json';(stage/label_path).write_bytes(annotation)
            selected,transfer=zip_points(record,wanted) if record['url'].endswith('.zip') else zstd_points(record,wanted)
            if record['domain']=='clear' and record['sequence'] in protocol['archive_sha256']:
                require(transfer['archive_sha256']==protocol['archive_sha256'][record['sequence']],'pinned full archive SHA256 differs')
            for frame in wanted:
                data=selected[frame];require(len(data)<=MAX_MEMBER and len(data)%16==0,'invalid XYZI member')
                point_path='points/'+name+'-'+str(frame).zfill(10)+'.bin'
                (stage/point_path).write_bytes(data)
                identity=name+'-'+str(frame).zfill(10)
                sensor['frames'].append({'id':identity,'path':point_path,'stamp':frame/10})
                records.append({'id':identity,'domain':record['domain'],'pair':record['pair'],
                    'sequence':record['sequence'],'frame':frame,'point_path':point_path,'point_sha256':digest(data),
                    'point_bytes':len(data),'annotation_path':label_path,'annotation_sha256':digest(annotation),
                    'archive_url':record['url'],'archive_bytes':record['size'],'archive_etag':record['etag'],**transfer})
            print('CADC_FETCH',json.dumps({'domain':record['domain'],'sequence':record['sequence'],
                'point_frames':len(selected),**transfer,'archive_bytes':record['size']}),flush=True)
        (stage/'sensor.json').write_text(json.dumps(sensor,separators=(',',':'))+'\n')
        (stage/'receipt.json').write_text(json.dumps({'protocol_sha256':digest(protocol_bytes),
            'license_url':protocol['license_url'],'dataset_url':protocol['dataset_url'],
            'frames':records,'sensor_sha256':digest((stage/'sensor.json').read_bytes())},indent=2)+'\n')
        require(not output.exists(),'dataset destination appeared during acquisition')
        os.rename(stage,output)
    return output

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--protocol',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pair',type=int,choices=(0,1,2,3))
    args=p.parse_args();acquire(args.protocol,args.output,args.pair)
