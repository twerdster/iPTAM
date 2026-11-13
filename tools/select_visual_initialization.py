#!/usr/bin/env python3
"""Inspect initialization pairs using images/intrinsics, never reference poses.

Lucas-Kanade forward/back checks, essential-matrix cheirality, and
rotation-corrected ray parallax identify candidates worth testing with PTAM.
The actual PTAM initialization and held-out tracking still determine viability.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from storage import check_output_storage


def inspect_pair(root, manifest, first_index, second_index):
    images = [cv2.imread(str(root/manifest["frames"][i]["path"]),0) for i in (first_index,second_index)]
    a = cv2.goodFeaturesToTrack(images[0],600,.01,8)
    base = dict(first=first_index,second=second_index,corners=0 if a is None else len(a))
    if a is None or len(a)<12:
        return dict(**base,passed=False)
    b,ok,_ = cv2.calcOpticalFlowPyrLK(images[0],images[1],a,None,winSize=(21,21),maxLevel=3)
    back,ok_back,_ = cv2.calcOpticalFlowPyrLK(images[1],images[0],b,None,winSize=(21,21),maxLevel=3)
    keep=ok[:,0].astype(bool)&ok_back[:,0].astype(bool)&(np.linalg.norm(back-a,axis=2)[:,0]<.5)
    a,b=a[keep,0],b[keep,0]
    base.update(persistent_tracks=len(a),median_displacement_px=float(np.median(np.linalg.norm(b-a,axis=1))) if len(a) else 0)
    if len(a)<30:
        return dict(**base,passed=False)
    k=np.array([[manifest["fx"],0,manifest["cx"]],[0,manifest["fy"],manifest["cy"]],[0,0,1]])
    cv2.setRNGSeed(42)
    essential,inlier=cv2.findEssentialMat(a,b,k,method=cv2.RANSAC,prob=.999,threshold=1)
    if essential is None:
        return dict(**base,passed=False)
    count,r,translation,mask=cv2.recoverPose(essential[:3],a,b,k,mask=inlier)
    good=mask[:,0]!=0
    aa=np.c_[a[good],np.ones(good.sum())]@np.linalg.inv(k).T
    bb=np.c_[b[good],np.ones(good.sum())]@np.linalg.inv(k).T@r
    if len(aa):
        cosine=np.sum(aa*bb,axis=1)/np.linalg.norm(aa,axis=1)/np.linalg.norm(bb,axis=1)
        parallax=float(np.median(np.rad2deg(np.arccos(np.clip(cosine,-1,1)))))
    else:
        parallax=0
    _,hmask=cv2.findHomography(a,b,cv2.RANSAC,2)
    base.update(essential_cheirality_inliers=int(count),essential_inlier_fraction=count/len(a),
                median_rotation_corrected_parallax_deg=parallax,
                homography_inliers=int(hmask.sum()) if hmask is not None else 0,
                relative_rotation_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(r)-1)/2,-1,1)))))
    # Thresholds are fixed before testing tracking or reading any reference.
    base["passed"]=bool(len(a)>=100 and count>=50 and count/len(a)>=.6 and .5<=parallax<=8)
    return base


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--datasets-root",type=Path,default=Path("data/prepared"))
    parser.add_argument("--datasets",nargs="+",default=["euroc_cam0","euroc_cam1"])
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--minimum-followup-frames",type=int,default=80)
    parser.add_argument("--all-adjacent",action="store_true",help="Inspect every adjacent and one-skipped pair; retain the same geometry/followup rules")
    args=parser.parse_args()
    check_output_storage(args.output)
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    all_results={};selected={}
    for name in args.datasets:
        root=args.datasets_root/name
        manifest=json.loads((root/"manifest.json").read_text())
        if len(manifest["frames"]) < 500:
            starts=[0,5,10,12,14,16,18,20,30,40,50,60,80,100,120,140,160,180,200]
            steps=[1,2,3,5,8,10,15]
            buckets=[(0,20),(20,60),(60,120),(120,500)]
        else:
            starts=[0,800,860,870,880,890,900,920,940,1000,1200,1600,2000,2400,2800,3200]
            steps=[5,10,15,20,30,40]
            buckets=[(0,800),(800,950),(950,1800),(1800,3700)]
        results=[]
        pairs=([(first,first+step) for first in range(len(manifest["frames"])-1) for step in (1,2)
                if first+step<len(manifest["frames"])] if args.all_adjacent else
               [(first,first+step) for first in starts for step in steps if first+step<len(manifest["frames"])])
        for first,second in pairs:
            results.append(inspect_pair(root,manifest,first,second))
        accepted=[r for r in results if r["passed"] and
                  r["second"]+args.minimum_followup_frames<len(manifest["frames"])]
        rank=lambda r:(r["essential_cheirality_inliers"],r["homography_inliers"],-r["relative_rotation_deg"])
        # Sample several trajectory regions instead of favoring an early
        # stationary period. Preserve the entire inspected candidate table.
        chosen=[]
        for low,high in buckets:
            group=sorted([r for r in accepted if low<=r["first"]<high],key=rank,reverse=True)[:2]
            chosen.extend([r["first"],r["second"]] for r in group)
        all_results[name]=dict(inspected=results,selected_pairs=chosen,reference_used=False)
        selected[name]=chosen
        print(name,'selected',chosen,'passed',len(accepted),'of',len(results),flush=True)
    (args.output/"visual_analysis.json").write_text(json.dumps(dict(
        reference_files_read=False,opencv=cv2.__version__,numpy=np.__version__,
        policy="GFTT600; LK forward/back<0.5px; >=100tracks,>=50cheirality inliers,inlier fraction>=0.6,median parallax0.5..8deg; top2 per fixed time bucket",
        minimum_followup_frames=args.minimum_followup_frames,all_adjacent=args.all_adjacent,
        datasets=all_results),indent=2)+'\n')
    (args.output/"candidates.json").write_text(json.dumps(selected,indent=2)+'\n')


if __name__=="__main__":
    main()
