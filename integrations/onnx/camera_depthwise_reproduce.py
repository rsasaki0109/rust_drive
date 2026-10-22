#!/usr/bin/env python3
"""Run the explicit 16-forward pilot before the frozen main comparison."""
import argparse
import json
from pathlib import Path
import camera_depthwise_check as c


def audit_pilot(folder):
    profile=c.load(folder/'pilot.json');serial=c.load(folder/'pilot-serial.json');image=c.load(folder/'pilot-image.json')
    saved=c.load(c.ROOT/'assets/camera-sequence-v1/recorded/exports/first-0.json')
    c.nano.check_image(serial,saved,parallel=False)
    c.audit_profile(profile,serial,c.expected(),True,False)
    c.check_image(image,saved,serial)
    a=sum(v['profile']['forward_wall_ns'] for v in profile['views']);b=sum(v['reference_forward_wall_ns'] for v in profile['views'])
    result=dict(pass_checks=True,actual_model_forwards=16,output_bits_exact=True,serial_optimized_ns=a,serial_resize_only_ns=b,reduction_fraction=1-a/b)
    c.require(result==c.load(folder/'pilot-summary.json'),'exact pilot counters and results')
    return result


def pilot(args,folder):
    c.fresh(folder)
    receipt,_=c.native(args.binary,['--executable-receipt'],folder/'executable.json')
    c.require(receipt['compiled_sources']==c.expected() and receipt['executable_sha256']==c.digest(args.binary),'actual pilot executable')
    c.write(folder/'pilot-before.json',dict(executable_receipt=receipt,model_sha256=c.MODEL_SHA,
        order=['profile','original-serial','production-parallel'],planned_model_forwards=16,diagnostic_only=True,
        checker_sha256=c.digest(c.__file__),orchestrator_sha256=c.digest(__file__)))
    source=c.load(c.ROOT/'assets/camera-sequence-v1/sequence.json')
    image=c.ROOT/'assets/camera-sequence-v1'/source['events'][0]['input']['path']
    c.native(args.binary,['--model',args.model,'--profile-image',image],folder/'pilot.json')
    c.native(args.serial_binary,['--model',args.model,'--serial-image',image],folder/'pilot-serial.json')
    c.native(args.binary,['--model',args.model,'--image',image],folder/'pilot-image.json')
    p=c.load(folder/'pilot.json')
    a=sum(v['profile']['forward_wall_ns'] for v in p['views']);b=sum(v['reference_forward_wall_ns'] for v in p['views'])
    c.write(folder/'pilot-summary.json',dict(pass_checks=True,actual_model_forwards=16,output_bits_exact=True,
        serial_optimized_ns=a,serial_resize_only_ns=b,reduction_fraction=1-a/b))
    audit_pilot(folder)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--audit-pilot',type=Path,help='Audit an existing pilot only; executes no model')
    parser.add_argument('--binary',type=Path,default=c.HERE/'target/release/rustdriving-camera-depthwise')
    parser.add_argument('--baseline',type=Path,default=c.HERE/'target/release/rustdriving-camera-nano-repeat2x')
    parser.add_argument('--supervised',type=Path,default=c.HERE/'target/release/rustdriving-camera-supervised')
    parser.add_argument('--serial-binary',type=Path,default=c.HERE/'target/release/rustdriving-camera-nano')
    parser.add_argument('--guard',type=Path,default=c.ROOT/'integrations/camera-tracking/target/release/rustdriving-camera-guard')
    parser.add_argument('--model',type=Path,default=c.ROOT/'artifacts/camera-model/yolox_nano.onnx')
    parser.add_argument('--s-model',type=Path,default=c.ROOT/'artifacts/camera-model-s/yolox_s.onnx')
    args=parser.parse_args()
    if args.audit_pilot:
        c.require(args.output is None,'choose only an audit or a fresh reproduction')
        print(json.dumps(dict(audit_pilot(args.audit_pilot),model_execution_this_check=False)));return
    c.require(args.output is not None,'--output NEW is required')
    output=c.fresh(args.output)
    controls=argparse.Namespace(**vars(args));controls.output=output/'controls';c.controls(controls)
    pilot(args,output/'pilot')
    main_args=argparse.Namespace(**vars(args));main_args.controls=output/'controls';main_args.output=output/'validation';c.run(main_args)
    result=c.load(output/'validation/summary.json')
    c.require(result['prior_diagnostic_model_forwards']==16 and result['actual_all_model_forwards']==result['actual_main_model_forwards']+16,'current reproduction includes exactly its own pilot')


if __name__=='__main__':main()
