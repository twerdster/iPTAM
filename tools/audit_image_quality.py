#!/usr/bin/env python3
"""Measure per-frame texture/flow without changing inputs or reading truth poses."""
import argparse
import csv
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from storage import check_output_storage


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest",type=Path,required=True)
    parser.add_argument("--trajectory",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    check_output_storage(args.output)
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    manifest=json.loads(args.manifest.read_text())
    with args.trajectory.open() as stream:
        outputs=list(csv.DictReader(stream))
    states={float(r["timestamp"]):r for r in outputs}
    h,w=manifest["height"],manifest["width"]
    mask=np.zeros((h,w),np.uint8);mask[10:-10,10:-10]=255
    if manifest.get("valid_mask"):
        mask &= cv2.imread(str(args.manifest.parent/manifest["valid_mask"]),0)
    detector=cv2.FastFeatureDetector_create(20,True,cv2.FAST_FEATURE_DETECTOR_TYPE_9_16)
    rows=[];previous=None;previous_points=None
    for i,frame in enumerate(manifest["frames"]):
        image=cv2.imread(str(args.manifest.parent/frame["path"]),0)
        corners=cv2.goodFeaturesToTrack(image,5000,.01,5,mask=mask)
        points=np.empty((0,1,2),np.float32) if corners is None else corners
        bins=np.zeros((3,4),int)
        for x,y in points[:,0]:
            bins[min(int(y*3/h),2),min(int(x*4/w),3)]+=1
        flow_count=0;flow_displacement=None
        if previous is not None and previous_points is not None and len(previous_points):
            pred,ok,_=cv2.calcOpticalFlowPyrLK(previous,image,previous_points,None,winSize=(21,21),maxLevel=3)
            back,ok2,_=cv2.calcOpticalFlowPyrLK(image,previous,pred,None,winSize=(21,21),maxLevel=3)
            keep=ok[:,0].astype(bool)&ok2[:,0].astype(bool)&(np.linalg.norm(back-previous_points,axis=2)[:,0]<.5)
            flow_count=int(keep.sum())
            if flow_count:
                flow_displacement=float(np.median(np.linalg.norm(pred[keep]-previous_points[keep],axis=2)))
        state=min(states.values(),key=lambda r:abs(float(r["timestamp"])-frame["timestamp"])) if states else None
        good=bool(state and abs(float(state["timestamp"])-frame["timestamp"])<1e-6 and
                  int(state["quality"])==2 and int(state["found"])>=10 and int(state["points"])>0)
        values=image[mask>0]
        rows.append(dict(frame=i,timestamp=frame["timestamp"],strict_good=good,
                         mean=float(values.mean()),std=float(values.std()),
                         p05_p95_range=float(np.percentile(values,95)-np.percentile(values,5)),
                         laplacian_variance=float(cv2.Laplacian(image,cv2.CV_64F)[mask>0].var()),
                         shi_tomasi_corners=len(points),fast9_corners=len(detector.detect(image,mask)),
                         textured_grid_fraction=float(np.mean(bins>=5)),
                         previous_frame_persistent_tracks=flow_count,median_flow_px=flow_displacement))
        previous=image;previous_points=cv2.goodFeaturesToTrack(image,600,.01,8,mask=mask)
    with (args.output/"per_frame.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    summaries={}
    for name,subset in [("good",[r for r in rows if r["strict_good"]]),
                        ("historical_lost_73_199",[r for r in rows if 73<=r["frame"]<=199])]:
        summaries[name]=dict(frames=len(subset),**{key:float(np.median([r[key] for r in subset])) if subset else None
            for key in ["std","laplacian_variance","shi_tomasi_corners","fast9_corners","textured_grid_fraction","previous_frame_persistent_tracks"]})
    report=dict(reference_poses_read=False,frame_count=len(rows),metrics=summaries,
                methods="border10, GFTT quality0.01 minDist5 max5000, FAST9 threshold20 nonmax; consecutive LK600 forward/back<0.5px",
                interpretation="Feature counts/flow are texture indicators, not proof of observability or a cause of estimator failure. FAST9 differs from PTAM FAST10.")
    (args.output/"summary.json").write_text(json.dumps(report,indent=2)+'\n')
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    t=[r["timestamp"] for r in rows]
    for ax,key,label in zip(axes,["shi_tomasi_corners","textured_grid_fraction","previous_frame_persistent_tracks"],
                            ["GFTT corners","textured 4×3 tiles","persistent LK tracks"]):
        ax.plot(t,[r[key] for r in rows],lw=1);ax.set_ylabel(label);ax.grid(alpha=.2)
        for r in rows:
            if not r["strict_good"]:
                ax.axvspan(r["timestamp"],r["timestamp"]+.1,color="#D55E00",alpha=.05,lw=0)
    axes[-1].set_xlabel("source time [s]")
    fig.suptitle("Historical raw lounge: texture/flow indicators; red background = no strict-good output")
    fig.tight_layout(rect=(0,0,1,.96));fig.savefig(args.output/"quality_timeline.png",dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(12,7))
    for ax,index in zip(axes.flat,[33,73,120,170,200,225]):
        f=manifest["frames"][index];image=cv2.imread(str(args.manifest.parent/f["path"]),0)
        ax.imshow(image,cmap="gray",vmin=0,vmax=255);ax.set_title(f"frame{index}: {rows[index]['shi_tomasi_corners']} corners; good={rows[index]['strict_good']}")
        ax.axis("off")
    fig.suptitle("Unmodified raw image samples across tracking loss and recovery")
    fig.tight_layout(rect=(0,0,1,.96));fig.savefig(args.output/"samples.png",dpi=140);plt.close(fig)
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
