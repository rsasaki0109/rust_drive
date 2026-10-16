#!/usr/bin/env python3
"""Independent immutable-map raw equations, state, analytic motion and score audit."""
import argparse
import copy
import importlib.util
import io
import json
from pathlib import Path

import numpy as np

from projective_depth_common import (ROOT,Client,audit_iterations,exp_rotation,
                                     fresh,pose,require,sha,write)
from projective_depth_raw import pyramid,normal_equation,compare_equation
from projective_information_raw import audit as information_audit


def load(path):
    require(path.stat().st_size<=64*1024*1024,'bounded JSON evidence')
    return json.loads(path.read_bytes())


def check_update(update,target,source,prior):
    # Reuse only independent numerical trace checking; its fusion acceptance
    # check belongs to the rolling mapper and is inapplicable to an immutable map.
    steps=audit_iterations(dict(update,accepted=False))
    information_audit(dict(update,fusion=None))
    systems=0
    if update['iterations']:
        np.testing.assert_allclose(pose(update['iterations'][0]['before']),prior,atol=1e-12,rtol=0)
    for step in update['iterations']:
        scale=step['scale']
        expected=normal_equation(target[scale],source[scale],pose(step['before']),scale)
        compare_equation(step['equation'],expected);systems+=1
    if update['accepted']:
        require(update['rejection'] is None and update['iterations'],'qualified self-fit or map fit')
        transform=pose(update['first_from_current'])
        np.testing.assert_allclose(transform,pose(update['iterations'][-1]['after']),atol=1e-12,rtol=0)
        equation=normal_equation(target[1],source[1],transform,1)
        compare_equation(update['final_equation'],equation);systems+=1
        require(equation['count']>=200 and equation['rms_m']<=.03
                and np.linalg.norm(transform[:3,3])<=.5,'unchanged returned-pose admission')
    else:
        require(update['first_from_current'] is None and update['final_equation'] is None,
                'rejected fitting cannot issue a pose')
    failure=update['information_failure']
    if failure is not None:
        scale=failure['scale']
        compare_equation(failure['equation'],normal_equation(target[scale],source[scale],pose(failure['pose']),scale))
        systems+=1
    return steps,systems


def controls(binary,output):
    spec=importlib.util.spec_from_file_location('independent_fixed_scene',ROOT/'scripts/check-projective-depth.py')
    scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    camera=scene.CAMERA;rows=[]
    for axis in range(6):
        transform=np.eye(4)
        if axis<3:transform[axis,3]=.02
        else:
            vector=np.zeros(3);vector[axis-3]=.02;transform[:3,:3]=exp_rotation(vector)
        client=Client(binary,camera)
        first=scene.render(np.eye(4));current=scene.render(transform)
        try:
            initial=client.frame(1.,first);result=client.frame(1.03,current)
            require(initial['accepted'] and result['accepted'],'analytic map and pose')
            a,b=check_update(initial,pyramid(first,camera),pyramid(first,camera),np.eye(4))
            c,d=check_update(result,pyramid(first,camera),pyramid(current,camera),np.eye(4))
            estimate=pose(result['first_from_current'])
            position=float(np.linalg.norm(estimate[:3,3]-transform[:3,3]))
            rotation=float(np.arccos(np.clip((np.trace(transform[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
            require(position<=.004 and rotation<=.004,'six-axis optical direction/metres')
            duplicate=client.frame(1.03,current)
            rejected=client.frame(1.1,np.zeros_like(current))
            expired=client.frame(1.231,np.zeros_like(current))
            later=client.frame(1.24,current)
            require(not duplicate['accepted'] and not rejected['accepted']
                    and rejected['last_accepted_timestamp']==1.03,'rejected acquisitions cannot renew pose')
            require(expired['lost'] and later['lost'] and not later['accepted']
                    and later['map_timestamp']==1.,'immutable map and expiry latch')
            client.finish()
        except BaseException:
            client.abort();raise
        mutations={
            'matrix':lambda u:u['iterations'][0]['equation']['h'][0].__setitem__(0,u['iterations'][0]['equation']['h'][0][0]+1),
            'gradient':lambda u:u['iterations'][0]['equation']['g'].__setitem__(0,u['iterations'][0]['equation']['g'][0]+1),
            'prior':lambda u:u['iterations'][0]['before']['translation'].__setitem__(0,.1),
            'pose':lambda u:u['first_from_current']['translation'].__setitem__(0,.2),
            'count':lambda u:u['final_equation'].__setitem__('count',u['final_equation']['count']+1),
        }
        rejected_mutants=[]
        for name,mutate in mutations.items():
            mutant=copy.deepcopy(result);mutate(mutant)
            try:check_update(mutant,pyramid(first,camera),pyramid(current,camera),np.eye(4))
            except (ValueError,AssertionError):rejected_mutants.append(name)
            else:raise ValueError('undetected report corruption: '+name)
        row=dict(axis=axis,position_error_m=position,rotation_error_rad=rotation,
                 checked_steps=a+c,raw_systems=b+d,corruptions_rejected=rejected_mutants)
        write(output/f'axis-{axis}.json',dict(summary=row,initial=initial,result=result,
              duplicate=duplicate,rejected=rejected,expired=expired,later=later))
        rows.append(row)
    return dict(schema_version=1,six_axis_controls=rows,pass_controls=True)


def recorded(reports,output):
    spec=importlib.util.spec_from_file_location('fixed_map_inputs',ROOT/'scripts/evaluate-projective-fixed-map.py')
    inputs=importlib.util.module_from_spec(spec);spec.loader.exec_module(inputs)
    reference_spec=importlib.util.spec_from_file_location('fixed_map_mocap',ROOT/'scripts/check-opencv-rgbd.py')
    reference=importlib.util.module_from_spec(reference_spec);reference_spec.loader.exec_module(reference)
    summaries=[]
    for case in inputs.CASES:
        directory=reports/case;freeze=load(directory/'freeze.json');document=load(directory/'results.json')
        for name,pin in freeze['source_sha256'].items():require(sha((ROOT/name).read_bytes())==pin,'source frozen: '+name)
        base,manifest,inventory=inputs.metadata(case)
        require(sha((base/'manifest.json').read_bytes())==freeze['manifest_sha256'] and inventory==freeze['inputs'],'original input inventory')
        rows=document['frames'];sensor=load(directory/'sensor-only.json')
        require(len(rows)==len(sensor['frames'])==180 and sensor['ground_truth_parsed'] is False,'complete pre-label operational record')
        journal=load(reports/'before-reference-parsing.json')
        require(journal['ground_truth_parsed'] is False
                and journal['sensor_report_sha256'][case]==sha((directory/'sensor-only.json').read_bytes()),'native-before-reference journal')
        target=None;prior=np.eye(4);map_time=last=None;lost=False;systems=steps=0;accurate=accepted=0;errors=[]
        labels=np.loadtxt(io.BytesIO(inputs.verified(base,inventory,'groundtruth.txt')),comments='#');origin=None
        for index,(row,frame,native) in enumerate(zip(rows,manifest['frames'],sensor['frames'])):
            update=row['update'];stamp=frame['depth_timestamp']
            require(row['source_index']==frame['source_index'] and update==native['update']
                    and update['timestamp']==stamp,'actual native acquisition/output unchanged')
            if last is not None:lost|=stamp-last>.2
            require(update['lost']==lost and not(lost and update['accepted']),'independent age/loss')
            source=pyramid(inputs.decode(base,inventory,frame),freeze['camera'])
            a,b=check_update(update,target if target is not None else source,source,prior)
            steps+=a;systems+=b
            if update['accepted']:
                if target is None:
                    target=source;map_time=stamp;origin=np.linalg.inv(reference.truth_at(labels,stamp))
                prior=pose(update['first_from_current']);last=stamp
                truth=origin@reference.truth_at(labels,stamp)
                distance=float(np.linalg.norm(prior[:3,3]-truth[:3,3]))
                relative=truth[:3,:3].T@prior[:3,:3]
                sine=np.linalg.norm([relative[2,1]-relative[1,2],relative[0,2]-relative[2,0],relative[1,0]-relative[0,1]])/2
                angle=float(np.arctan2(sine,(np.trace(relative)-1)/2))
                np.testing.assert_allclose([row['position_error_m'],row['rotation_error_rad']],
                                           [distance,angle],atol=3e-8,rtol=0)
                require(row['accurate']==(distance<=.1 and angle<=.1),'independent root accuracy classification')
                if index:accepted+=1;accurate+=row['accurate'];errors.append((distance,angle))
            require(update['map_timestamp']==map_time and update['last_accepted_timestamp']==last
                    and update['initialized']==(target is not None),'fixed origin and genuine accepted clock')
        summary=dict(dataset=case,raw_systems=systems,checked_steps=steps,accepted=accepted,root_accurate=accurate,lost=lost)
        for key in ('accepted','root_accurate','lost'):require(summary[key]==document['summary'][key],'independent score summary')
        require(document['summary']['total_updates']==179 and document['summary']['complete_accuracy_pass']==(accurate==179 and not lost),'complete denominator/accuracy gate')
        write(output/(case+'.json'),summary);summaries.append(summary)
        print(json.dumps(summary),flush=True)
    return dict(schema_version=1,cases=summaries,raw_systems=sum(s['raw_systems'] for s in summaries),
                integrity_passed=True,quality_passed=all(s['root_accurate']==179 and not s['lost'] for s in summaries))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary',type=Path)
    parser.add_argument('--reports',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();output=fresh(args.output)
    if args.reports is None:
        require(args.binary is not None,'native binary required for controls')
        result=controls(args.binary,output)
    else:result=recorded(args.reports,output)
    write(output/'summary.json',result)
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
