"""Independent measured-clock audit, copied unchanged except Nano producer identity."""
import copy
import math
import struct

ORIGINAL_FUNCTION_SHA256 = 'ae999a2f23b2abf6e6b87dc4b9f2527fcc1d657f7d7955e3fc8d8ffca8a984cf'


def require(condition,message):
    if not condition:raise ValueError(message)


def audit_measured(report,source,inspection,freshness,tracking):
    require(report['algorithm']=='yolox_nano_measured_replay_v1' and report['clock']=='std::time::Instant_serial_replay','unchanged measured controller')
    require(report['policy']==freshness.POLICY and report['control_permission'] is False,'unchanged age/recovery policy')
    require(len(report['events'])==len(source['events'])==8,'all original scheduled images retained')
    origin=float(source['events'][0]['timestamp_s'])
    previous=None
    events,outputs=[],[]
    counts=dict(image_events=0,inferred_images=0,skipped_images=0,inferred_but_late_images=0,late_at_check_images=0,available_at_check_images=0)
    ages=[]
    for index,(original,row,slot) in enumerate(zip(source['events'],report['events'],inspection['slots'])):
        require(row['source']==slot and slot['event_index']==index and slot['source_timestamp_text']==original['timestamp_s'],'source identity and order')
        source_s=float(original['timestamp_s'])-origin
        require(slot['release_offset_s']==source_s,'original acquisition intervals')
        measured=row['measured']
        timing=measured['timing']
        start,submit,check=[timing[k] for k in ('processing_started_s','sample_submitted_s','checked_after_tracking_s')]
        require(all(math.isfinite(v) for v in (start,submit,check)) and start>=source_s and submit-start>=1e-6 and check-submit>=1e-6,'actual host checkpoint order')
        if previous is not None:
            require(start-previous['now_s']>=1e-6,'serial frame clock advances')
        require(timing['read_analysis_and_probe_wall_s']==submit-start and timing['sample_guard_wall_s']==check-submit,'measured stage arithmetic')
        require(timing['source_age_at_start_s']==start-source_s and timing['source_age_at_check_s']==check-source_s
                and timing['acquisition_deadline_s']==source_s+.2,'independent source age/deadline')
        missed=check>source_s+.2
        require(timing['deadline_missed_at_check']==missed,'same inclusive age gate')
        entries=measured['guard_events']
        require(len(entries)==2 and entries[0]['event']['now_s']==submit and entries[1]['event']==dict(now_s=check,input=dict(kind='tick')),'post-tracking guard check')
        data=entries[0]['event']['input']
        require(data['kind']=='sample' and data['acquired_s']==source_s and data['image_sha256']==original['input']['sha256'],'image identity and acquisition enter guard')
        accepted=previous['last_accepted_acquired_s'] if previous else None
        barrier=previous['fault_epoch_s'] if previous else None
        armed=accepted is not None and (barrier is None or accepted>barrier)
        if armed and start>accepted+.2:
            barrier=accepted+.2
        decision=('before_fault' if barrier is not None and source_s<=barrier else
                  'out_of_order' if accepted is not None and source_s-accepted<1e-6 else
                  'stale_sample' if start>source_s+.2 else 'accepted')
        require(measured['precheck_decision']==decision,'independent admission at processing start')
        inferred=measured['analysis'] is not None
        require(inferred==(decision=='accepted'),'no inference for excluded work')
        if inferred:
            analysis=measured['analysis']
            f32=lambda v:struct.unpack('<f',struct.pack('<f',v))[0]
            boxes=[dict(class_index=d['class_index'],confidence=f32(d['confidence']),bbox=list(map(f32,d['bbox']))) for d in analysis['result']['detections']]
            require(data['detections']==boxes,'unchanged original f32 measurements')
            require(timing['read_analysis_and_probe_wall_s']>=analysis['execution']['analysis_wall_s'],'measured frame covers full parallel analyzer')
        else:
            require(data['detections']==[],'excluded image does not invent boxes')
        for entry in entries:
            event=copy.deepcopy(entry['event'])
            if event['input']['kind']=='sample':
                event['input']['kind']='detections'
            events.append(event)
            outputs.append(entry['output'])
        counts['image_events']+=1
        counts['inferred_images']+=inferred
        counts['skipped_images']+=not inferred
        counts['inferred_but_late_images']+=inferred and missed
        counts['late_at_check_images']+=missed
        counts['available_at_check_images']+=entries[1]['output']['ready']
        ages.append(check-source_s)
        previous=entries[-1]['output']
    require(report['summary']==counts|dict(maximum_source_age_at_check_s=max(ages),deadline_gate_pass=counts['late_at_check_images']==0),'independent deadline counts')
    require(report['model_loads']==1 and report['preflight_image_reads']==8 and report['inference_image_reads']==counts['inferred_images']
            and report['model_forwards']==counts['inferred_images']*4,'one plan and actual forward/read counts')
    require(report['startup_before_schedule_wall_s']>=report['manifest_and_preflight_wall_s']+report['model_load_wall_s']
            and report['schedule_wall_s']>=previous['now_s'],'startup/schedule accounting')
    manifest=dict(schema_version=1,camera_id=report['camera_id'],width=report['width'],height=report['height'],
        config=report['config'],policy=report['policy'],events=events)
    backend,snapshots=freshness.check_contract(manifest,events,outputs,tracking)
    return manifest,backend,snapshots
