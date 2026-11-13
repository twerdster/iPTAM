#!/usr/bin/env python3
"""Freeze a recording bootstrap from pixels/K; never read reference poses."""
import argparse
import json
from pathlib import Path
import cv2

from prepare_datasets import digest
from select_visual_initialization import inspect_pair
from storage import check_output_storage


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=['essential','planar'],default='essential')
    args=p.parse_args();check_output_storage(args.output)
    if args.output.exists():raise FileExistsError(args.output)
    cv2.setNumThreads(0);cv2.ocl.setUseOpenCL(False)
    manifest_path=args.dataset/'manifest.json';manifest=json.loads(manifest_path.read_text())
    starts=[0,15,30,60,120,240,360,480,600,720];steps=[5,10,15,20,30]
    pairs=[(a,a+d) for a in starts for d in steps if a+d+80<len(manifest['frames'])]
    inspected=[inspect_pair(args.dataset,manifest,a,b) for a,b in pairs]
    for r in inspected:
        r['planar_policy_passed']=bool(r.get('persistent_tracks',0)>=100 and r.get('homography_inliers',0)>=100 and
            r.get('essential_cheirality_inliers',0)>=50 and 10<=r.get('median_displacement_px',0)<=50 and
            .5<=r.get('median_rotation_corrected_parallax_deg',0)<=8)
    passed=[r for r in inspected if r['passed' if args.mode=='essential' else 'planar_policy_passed']]
    selected=None
    if passed:
        earliest=min(r['first'] for r in passed)
        selected=max((r for r in passed if r['first']==earliest),key=lambda r:(
            r['homography_inliers'] if args.mode=='planar' else r['essential_cheirality_inliers'],
            r['essential_cheirality_inliers'] if args.mode=='planar' else r['homography_inliers'],
            -r['relative_rotation_deg'],-r['second']))
    report=dict(dataset=manifest.get('name'),manifest_sha256=digest(manifest_path),
        reference_files_read=False,selection_mode=args.mode,standard_essential_passes=sum(r['passed'] for r in inspected),
        planar_policy='>=100 LK tracks,>=100 homography inliers,>=50 cheirality inliers,10..50px displacement,0.5..8deg corrected parallax; earliest first, then mostH/mostE/leastrotation/earliestsecond',
        selection_policy='Fixed candidate grid; same GFTT/LK/essential thresholds; require80followup frames; earliest passing first frame, then most cheirality inliers, most homography inliers, least rotation, earliest second',
        starts=starts,steps=steps,inspected=inspected,selected=selected,
        selector_sha256=digest(Path(__file__)),feature_selector_sha256=digest(Path(inspect_pair.__code__.co_filename)))
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(inspected=len(inspected),mode=args.mode,passed=len(passed),selected=selected),indent=2))


if __name__=='__main__':main()
