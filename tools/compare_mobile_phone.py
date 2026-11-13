#!/usr/bin/env python3
"""Compare a fresh mobile suite to saved frozen current/upstream phone controls."""
import argparse
from pathlib import Path
import shutil

from compare_upstream_phone import (DEFAULT_PLAN, analyze_run, digest, discover,
    markdown, matched_timestamp_diagnostics, read_json, write_json)
from storage import check_output_storage

ENGINES = ('upstream', 'current', 'mobile')

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline-root', type=Path, required=True)
    p.add_argument('--mobile-root', type=Path, required=True, help='Contains DATASET/repNN saved phone outputs')
    p.add_argument('--input-plan', type=Path, default=DEFAULT_PLAN)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--automatic-system-comparison', action='store_true', help='Explicitly compare different initialization policies as complete systems')
    args = p.parse_args()
    check_output_storage(args.output)
    args.output.mkdir(parents=True, exist_ok=False)
    plan = read_json(args.input_plan);cases = plan['cases']
    shutil.copyfile(args.input_plan, args.output/'input-plan.json')
    runs = [(r, args.baseline_root.resolve()) for r in discover(args.baseline_root.resolve(), cases)]
    for case in cases:
        base = args.mobile_root.resolve()/case['name']
        if base.is_dir():
            runs.extend((r, args.mobile_root.resolve()) for r in sorted(base.iterdir()) if r.is_dir() and (r/'requested-manifest.json').exists())
    report = dict(engines=ENGINES, baseline_root=str(args.baseline_root.resolve()),mobile_root=str(args.mobile_root.resolve()),
                  helper_sha256=digest(Path(__file__)), input_plan_sha256=digest(args.input_plan),
                  automatic_system_comparison=args.automatic_system_comparison,
                  policy=('Source-paced complete-system comparison; frozen same images/K/clocks/predeclared start; mobile initialization policy recorded separately; '
                          'no fixed-pair tracking isolation for automatic or unknown policies; references only after replay'
                          if args.automatic_system_comparison else
                          'Frozen same-image/K/timestamp/fixed-pair phone controls; only confirmed fixed mobile initialization is eligible; references only after replay'),
                  trials=[],groups={e:{c['name']:[] for c in cases} for e in ENGINES}, missing=[])
    for run,root in runs:
        r = analyze_run(run,root,cases,args.output)
        report['trials'].append(r)
        if r['engine'] in report['groups'] and r['dataset'] in report['groups'][r['engine']]:
            report['groups'][r['engine']][r['dataset']].append(r)
    eligibility = 'eligible_for_source_paced_system_comparison' if args.automatic_system_comparison else 'eligible_for_paired_source_paced_comparison'
    report['matched_timestamp_diagnostics']=matched_timestamp_diagnostics(report['groups'],cases,args.output,ENGINES,eligibility)
    write_json(args.output/'report.json',report)
    (args.output/'report.md').write_text(markdown(report))
    print(args.output/'report.md')

if __name__ == '__main__':
    main()
