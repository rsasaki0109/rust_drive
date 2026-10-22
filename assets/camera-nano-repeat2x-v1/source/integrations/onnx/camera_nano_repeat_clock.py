"""Original quantitative FIFO/latest audits, with distinct new producer identities."""
import copy
import importlib.util
import math
from pathlib import Path
import struct
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location('nano_repeat_clock_support',HERE/'camera_nano_check.py')
nano = importlib.util.module_from_spec(spec);spec.loader.exec_module(nano)
freshness, tracking = nano.freshness, nano.tracking
require, load = nano.require, nano.load

def audit_fifo(report,source,inspection,freshness,tracking):
    require(report['algorithm']=='yolox_nano_repeat2x_measured_replay_v1' and report['clock']=='std::time::Instant_serial_replay','unchanged measured controller')
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

def audit_latest(report, source, inspection, references=None, expected=None, check_image=None):
    require(report['schema_version'] == 1 and report['algorithm'] == 'yolox_nano_repeat2x_latest_ready_replay_v1'
            and report['clock'] == 'std::time::Instant_latest_ready_replay', 'distinct latest-ready identity')
    require(report['compiled_sources'] == expected() and report['model_sha256'] == nano.MODEL_SHA, 'complete latest-ready source/model receipt')
    require(report['policy'] == freshness.POLICY and report['control_permission'] is False, 'original age/recovery policy')
    require(report['source_sequence_sha256'] == inspection['source_sequence_sha256'] and inspection['compiled_sources'] == expected(), 'source inspection binding')
    require(inspection['algorithm'] == 'yolox_nano_repeat2x_schedule_validation_v1'
            and inspection['policy'] == freshness.POLICY and inspection['control_permission'] is False, 'inspection identity and policy')
    require(not inspection['model_read'] and not inspection['inference_run'], 'complete preflight without model')
    for field in ('camera_id', 'width', 'height'):
        require(report[field] == inspection[field] == source[field], 'original camera identity')
    require(report['config'] == inspection['config'], 'original tracking configuration')
    origin = float(source['events'][0]['timestamp_s']); slots = []
    for index, event in enumerate(source['events']):
        value = float(event['timestamp_s']); offset = value - origin
        require(math.isfinite(value) and 0 <= offset <= 2 and (not slots or value - slots[-1]['source_timestamp_s'] >= 1e-6), 'original bounded acquisition clock')
        slots.append(dict(event_index=index, source_timestamp_text=event['timestamp_s'], source_timestamp_s=value,
            release_offset_s=offset, source_image_sha256=event['input'].get('sha256')))
    require(report['source_slots'] == inspection['slots'] == slots and inspection['images_read'] == sum(s['source_image_sha256'] is not None for s in slots), 'complete original source inventory')
    next_index = 0; previous = None; events = []; outputs = []
    counts = dict(source_images=sum(s['source_image_sha256'] is not None for s in slots), dispatched_images=0,
        superseded_images=0, inferred_images=0, inference_excluded_images=0, inferred_but_late_images=0,
        late_dispatched_images=0, available_at_dispatched_checks=0)
    for row in report['events']:
        selected_at = row['selection_checked_s']
        require(type(selected_at) in (int,float) and math.isfinite(selected_at) and 0 <= selected_at <= 1e12, 'observed finite selection clock')
        if previous is not None:require(selected_at - previous['now_s'] >= 1e-6, 'selection follows previous processing')
        due = [i for i in range(next_index,len(slots)) if slots[i]['release_offset_s'] <= selected_at]
        require(due and due[0] == next_index, 'selection only after original release')
        barriers = [i for i in due if source['events'][i]['input']['kind'] != 'image']
        index = min(barriers) if barriers else max(due)
        require(row['source'] == slots[index] and row['superseded_sources'] == slots[next_index:index], 'independent latest-ready choice and exact discarded identities')
        require(all(source['events'][i]['input']['kind'] == 'image' for i in range(next_index,index)), 'no missing/failure event discarded')
        counts['superseded_images'] += index - next_index
        original = source['events'][index]; offset = slots[index]['release_offset_s']
        measured = row['measured']; timing = measured['timing']
        start, submit, check = [timing[k] for k in ('processing_started_s','sample_submitted_s','checked_after_tracking_s')]
        require(all(type(v) in (int,float) and math.isfinite(v) and v <= 1e12 for v in (start,submit,check))
                and start - selected_at >= 1e-6 and submit - start >= 1e-6 and check - submit >= 1e-6, 'actual ordered host checkpoints')
        require(timing['read_analysis_and_probe_wall_s'] == submit-start and timing['sample_guard_wall_s'] == check-submit, 'exact measured stage arithmetic')
        entries = measured['guard_events']
        require(len(entries) == 2 and entries[0]['event']['now_s'] == submit and entries[1]['event'] == dict(now_s=check,input=dict(kind='tick')), 'complete sample and post-tracking check')
        data = entries[0]['event']['input']; inferred = measured['analysis'] is not None
        if original['input']['kind'] == 'image':
            counts['dispatched_images'] += 1
            require(data['kind'] == 'sample' and data['acquired_s'] == offset and data['image_sha256'] == original['input']['sha256'], 'unchanged acquired time and image hash')
            require(timing['source_age_at_start_s'] == start-offset and timing['source_age_at_check_s'] == check-offset
                    and timing['acquisition_deadline_s'] == offset+.2, 'exact source ages and unchanged deadline')
            missed = check > offset+.2
            require(timing['deadline_missed_at_check'] is missed, 'same inclusive deadline')
            accepted = previous['last_accepted_acquired_s'] if previous else None
            barrier = previous['fault_epoch_s'] if previous else None
            armed = accepted is not None and (barrier is None or accepted > barrier)
            if armed and start > accepted+.2:barrier = accepted+.2
            decision = ('before_fault' if barrier is not None and offset <= barrier else
                        'out_of_order' if accepted is not None and offset-accepted < 1e-6 else
                        'stale_sample' if start > offset+.2 else 'accepted')
            require(measured['precheck_decision'] == decision and inferred == (decision == 'accepted'), 'independent original inference admission')
            if inferred:
                analysis = measured['analysis']
                saved = load(ROOT / f'assets/camera-sequence-v1/recorded/exports/first-{index}.json')
                check_image(analysis, saved, None if references is None else references[index])
                f32 = lambda v:struct.unpack('<f',struct.pack('<f',v))[0]
                boxes = [dict(class_index=d['class_index'],confidence=f32(d['confidence']),bbox=list(map(f32,d['bbox']))) for d in analysis['result']['detections']]
                require(data['detections'] == boxes, 'unchanged f32 detections enter the original guard')
                require(submit-start >= analysis['execution']['analysis_wall_s'], 'processing covers full image analysis')
            else:require(data['detections'] == [], 'excluded image invents no measurements')
            counts['inferred_images'] += inferred; counts['inference_excluded_images'] += not inferred
            counts['inferred_but_late_images'] += inferred and missed; counts['late_dispatched_images'] += missed
            counts['available_at_dispatched_checks'] += entries[1]['output']['ready']
        else:
            require(data == dict(kind='tick' if original['input']['kind'] == 'missing' else 'failed'), 'non-image barrier delivered unchanged')
            require(measured['precheck_decision'] is None and not inferred, 'barrier does no inference or sample recovery')
            require(all(timing[k] is None for k in ('source_age_at_start_s','source_age_at_check_s','acquisition_deadline_s','deadline_missed_at_check')), 'no invented barrier source age')
        for entry in entries:
            event = copy.deepcopy(entry['event'])
            if event['input']['kind'] == 'sample':event['input']['kind'] = 'detections'
            events.append(event); outputs.append(entry['output'])
        previous = entries[-1]['output']; next_index = index+1
    require(next_index == len(slots), 'every original event accounted once')
    counts.update(dispatched_deadline_pass=None if not counts['dispatched_images'] else counts['late_dispatched_images'] == 0,
        all_source_images_inferred=counts['source_images'] > 0 and counts['inferred_images'] == counts['source_images'],
        all_source_results_available=counts['source_images'] > 0 and counts['available_at_dispatched_checks'] == counts['source_images'])
    require(report['summary'] == counts and counts['dispatched_images'] + counts['superseded_images'] == counts['source_images'], 'independent complete coverage and deadline denominators')
    require(report['model_loads'] == int(counts['source_images'] > 0) and report['preflight_image_reads'] == counts['source_images']
            and report['inference_image_reads'] == counts['inferred_images'] and report['model_forwards'] == counts['inferred_images']*4, 'actual inference and model reads')
    require(report['startup_before_schedule_wall_s'] >= report['manifest_and_preflight_wall_s'] + report['model_load_wall_s']
            and report['schedule_wall_s'] >= previous['now_s'], 'startup and measured schedule accounting')
    manifest = dict(schema_version=1,camera_id=report['camera_id'],width=report['width'],height=report['height'],config=report['config'],policy=report['policy'],events=events)
    freshness.check_contract(manifest, events, outputs, tracking)
    return manifest
