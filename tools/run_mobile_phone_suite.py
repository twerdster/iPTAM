#!/usr/bin/env python3
"""Sequential recorded-input mobile iPhone trials, using the frozen input plan."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from run_host_validation import sha, write_json
from storage import check_output_storage
from phone_config import bundle_identifier

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--app',type=Path,required=True)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--device',required=True,help='Explicit devicectl device identifier')
    p.add_argument('--expected-udid',help='Optional independent device identity check')
    p.add_argument('--bundle',default=bundle_identifier('mobile'),help='Installed mobile app bundle identifier')
    p.add_argument('--expected-plan-sha256',help='Reject any input-plan change from this SHA256')
    p.add_argument('--profile',choices=['paper','auto','diagnostics','serial','coarse','partial'],default='paper')
    p.add_argument('--cases',nargs='+')
    p.add_argument('--repeat-cases',nargs='*',default=['lounge_new','euroc_cam1'])
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--skip-copy',action='store_true',help='Only for already verified dataset caches in this app')
    p.add_argument('--reuse-cache-cases',nargs='*',default=[],help='Reuse only these previously verified per-bundle dataset caches')
    args=p.parse_args()
    plan_sha=sha(args.plan)
    if args.expected_plan_sha256 and plan_sha!=args.expected_plan_sha256:raise ValueError('Input plan hash changed')
    if args.repeats<1:p.error('repeats must be positive')
    check_output_storage(args.output);args.output.mkdir(parents=True,exist_ok=False)
    plan=json.loads(args.plan.read_text())
    cases=[c for c in plan['cases'] if not args.cases or c['name'] in args.cases]
    if args.cases and set(args.cases)!={c['name'] for c in cases}:raise ValueError('Unknown case')
    outcomes=[]
    stop_file=args.output/'STOP_AFTER_TRIAL'
    for case in cases:
        dataset=Path(case['dataset'])
        if sha(dataset/'manifest.json')!=case['manifest_sha256']:raise ValueError('Frozen manifest changed')
        for repeat in range(args.repeats if case['name'] in args.repeat_cases else 1):
            if stop_file.exists():
                write_json(args.output/'suite-stopped.json',dict(reason='Explicit stop marker; previous trial cleaned up normally',finished_trials=len(outcomes)))
                return
            folder=args.output/case['name']/f'rep{repeat+1:02d}'
            command=[sys.executable,str(Path(__file__).with_name('run_phone_replay.py')),
                     '--dataset',str(dataset),'--output',str(folder),'--app',str(args.app.resolve()),
                     '--device',args.device,'--bundle',args.bundle,'--source-paced','--mobile-profile',args.profile,
                     '--init-first',str(case['init_first']),'--init-second',str(case['init_second']),
                     '--timeout','1200']
            if args.expected_udid:command += ['--expected-udid',args.expected_udid]
            if repeat or args.skip_copy or case['name'] in args.reuse_cache_cases:command.append('--skip-copy')
            log=args.output/f"{case['name']}-rep{repeat+1:02d}.log"
            print(f"Starting {args.profile}/{case['name']}/rep{repeat+1:02d}",flush=True)
            with log.open('w') as stream:
                process=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT)
            outcome=dict(case=case['name'],repeat=repeat+1,profile=args.profile,argv=command,exit_code=process.returncode,log=str(log))
            if (folder/'run/summary.json').exists():outcome['summary']=json.loads((folder/'run/summary.json').read_text())
            outcomes.append(outcome);write_json(args.output/'outcomes.json',outcomes)
            if process.returncode:raise RuntimeError(f"Trial failed; preserve {folder} and inspect before continuing")
            print(f"Completed {args.profile}/{case['name']}/rep{repeat+1:02d}",flush=True)
    write_json(args.output/'suite-complete.json',dict(profile=args.profile,plan_sha256=plan_sha,completed_trials=len(outcomes),pipeline_complete=True,
               tracking_stability='separate outcome; inspect per-recording coverage and accuracy'))

if __name__=='__main__':main()
