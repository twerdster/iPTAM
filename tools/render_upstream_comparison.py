#!/usr/bin/env python3
"""Render matched saved phone runs, never control an app or rerun an estimator.

Requires a compare_upstream_phone.py report. Default selections are fixed rep01
of new pinhole lounge and EuRoC right, with full inputs and visible lost gaps.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import subprocess

import cv2
import numpy as np

from compare_upstream_phone import digest, quality_good
from storage import check_output_storage


WIDTH, HEIGHT = 1600, 1100
COLORS = dict(GOOD=(85, 210, 95), WEAK=(45, 185, 245), LOST=(85, 80, 230),
              INITIALIZING=(150, 145, 140), UNPROCESSED=(105, 105, 105))
FG, MUTED = (236, 238, 241), (165, 173, 185)


def text(canvas, value, point, scale=.55, color=FG, thickness=1):
    cv2.putText(canvas, str(value), point, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def initialization_caption(selected, system_comparison):
    """Describe actual selected policies, rather than just a CLI mode flag."""
    mobile = [trial for trial in selected if trial.get('engine') == 'mobile']
    unknown = any(not trial.get('initialization_policy_known', False) for trial in mobile)
    automatic = any(trial.get('initialization_policy_difference', False) for trial in mobile)
    if not system_comparison and (automatic or unknown):
        raise ValueError('Automatic or unknown mobile initialization requires explicit system-comparison mode')
    if unknown:
        return 'same pixels/K/clocks; mobile initialization policy unverified'
    if automatic:
        return 'same pixels/K/clocks; automatic mobile second view, fixed control pair'
    return 'same prepared pixels/K/clocks/fixed pair'


class Side:
    def __init__(self, info, manifest):
        self.info = info
        self.run = Path(info['run'])
        for path, expected in info['artifacts'].items():
            assert digest(Path(path)) == expected, f'Artifact changed since evaluation: {path}'
        self.rows_path = self.run / 'run/frames.jsonl'
        self.rows = [json.loads(line) for line in self.rows_path.read_text().splitlines() if line.strip()]
        requested = json.loads((self.run / 'requested-manifest.json').read_text())
        for key in ['width', 'height', 'fx', 'fy', 'cx', 'cy', 'lens_model', 'frames', 'init_first', 'init_second']:
            assert requested.get(key) == manifest.get(key), f'Matched input differs: {key}'
        self.times = np.asarray([r['timestamp'] for r in requested['frames']])
        assert len(self.rows) == len(self.times)
        assert np.allclose([r['timestamp'] for r in self.rows], self.times, atol=1e-6, rtol=0)
        self.states = []
        self.good = []
        for row in self.rows:
            supported = (row['initialized'] and row['finite'] and row['map_points'] > 0 and row['found'] >= 10)
            good = supported and quality_good(row['quality'])
            self.good.append(good)
            self.states.append('GOOD' if good else 'INITIALIZING' if not row['initialized'] else
                               'WEAK' if supported and row['quality'] == 1 else 'LOST')
        self.good = np.asarray(self.good)
        assert int(self.good.sum()) == info['common_good_frames']
        fit = info['common_evaluation'].get('sim3')
        self.positions = None
        if fit:
            p = np.asarray([r['world_from_camera_t_qxyzw'][:3] for r in self.rows])
            self.positions = fit['scale'] * (p @ np.asarray(fit['rotation']).T) + fit['translation']
        self.provenance = json.loads((self.run / 'provenance.json').read_text())
        self.summary = json.loads((self.run / 'run/summary.json').read_text())


class Pair:
    def __init__(self, report, case, repeat, engines=('upstream','current'), system_comparison=False):
        self.case = case
        self.engines = engines
        self.system_comparison = system_comparison
        groups = report['groups']
        selected = []
        for engine in engines:
            choices = [r for r in groups[engine][case] if Path(r['run']).name == repeat]
            eligibility = 'eligible_for_source_paced_system_comparison' if system_comparison else 'eligible_for_paired_source_paced_comparison'
            if len(choices) != 1 or not choices[0].get(eligibility):
                raise ValueError(f'{engine}/{case}/{repeat} must be one completed verified source-paced trial')
            selected.append(choices[0])
        self.initialization_caption = initialization_caption(selected, system_comparison)
        self.manifest_path = Path(selected[0]['run']) / 'requested-manifest.json'
        self.manifest = json.loads(self.manifest_path.read_text())
        assert self.manifest.get('lens_model', 'pinhole') == 'pinhole'
        assert [self.manifest['width'], self.manifest['height']] == [640, 480]
        self.sides = [Side(info, self.manifest) for info in selected]
        self.source = Path(self.sides[0].provenance['source'])
        assert all(self.source == Path(side.provenance['source']) for side in self.sides)
        self.times = np.asarray([r['timestamp'] for r in self.manifest['frames']])
        self.span = float(self.times[-1] - self.times[0])
        self.reference_path = self.source / 'reference.json'
        for side in self.sides:
            saved_reference=Path(side.info['evaluation_directory'])/'common/reference.json'
            assert digest(saved_reference)==digest(self.reference_path)
        reference = json.loads(self.reference_path.read_text())
        self.reference = np.asarray([np.asarray(p['matrix'])[:3, 3] for p in reference['poses']])
        self.axes = np.sort(np.argsort(np.ptp(self.reference, axis=0))[-2:])
        all_points = [self.reference]
        all_points.extend(s.positions[s.good] for s in self.sides if s.positions is not None)
        projected = np.concatenate(all_points)[:, self.axes]
        self.center = (projected.max(axis=0) + projected.min(axis=0)) / 2
        self.scale = 165 / max(float(np.ptp(projected, axis=0).max()), .1)
        self.ref_pixels = self.project(self.reference)
        for side in self.sides:
            side.pixels = self.project(side.positions) if side.positions is not None else None
        self.last_index, self.image = -1, None
        pixel_hash = __import__('hashlib').sha256()
        for f in self.manifest['frames']:
            image = cv2.imread(str(self.source / f['path']), cv2.IMREAD_GRAYSCALE)
            if image is None or image.shape != (480, 640):
                raise ValueError('Missing or wrong-sized matched source image')
            pixel_hash.update(image.tobytes())
        for side in self.sides:
            assert side.summary['rectified_pixels_concatenated_sha256'] == pixel_hash.hexdigest()
        self.receipt = dict(dataset=case, repeat=repeat, system_comparison=system_comparison,
            initialization_caption=self.initialization_caption, source=str(self.source), source_frames=len(self.times),
            source_span_seconds=self.span, source_pixels_sha256=pixel_hash.hexdigest(),
            source_K={k:self.manifest[k] for k in ['fx','fy','cx','cy']}, lens_model='pinhole',
            reference_role=reference['role'], reference_sha256=digest(self.reference_path),
            sampling='Inclusive first/last exposure; last image at or before source time, no image interpolation',
            trajectory='Saved common-rule evaluation Sim3, only adjacent source exposures both common GOOD connected',
            trials=[dict(engine=s.info['engine'], run=str(s.run), app=s.provenance['app'],
                frames_jsonl_sha256=digest(s.rows_path), summary_sha256=digest(s.run/'run/summary.json'),
                requested_manifest_sha256=digest(s.run/'requested-manifest.json'),
                raw_good=s.info['raw_quality_good_frames'], common_good=s.info['common_good_frames'],
                strict_good=s.info['strict_good_frames'], strict_available=s.info['strict_support_available'],
                common_evaluation=s.info['common_evaluation'], processing_time_scope=s.info['processing_time_scope']) for s in self.sides])
        self.shared = next((d for d in report.get('matched_timestamp_diagnostics', [])
                            if d['dataset'] == case and d['repeat'] == repeat and d['status'] == 'computed'), None)
        self.receipt['shared_accepted_time_diagnostic'] = self.shared

    def project(self, positions):
        return np.rint((positions[:,self.axes] - self.center) * [self.scale,-self.scale] + [400,895]).astype(np.int32)

    def frame(self, ordinal, count, fps):
        source_time = self.times[0] + ordinal / (count - 1) * self.span
        index = int(np.clip(np.searchsorted(self.times, source_time, side='right')-1, 0, len(self.times)-1))
        if ordinal == count - 1:
            index = len(self.times)-1
        if index != self.last_index:
            gray = cv2.imread(str(self.source / self.manifest['frames'][index]['path']), 0)
            self.image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            self.last_index = index
        canvas = np.full((HEIGHT, WIDTH, 3), (20,24,31), np.uint8)
        title = 'New lounge | common pinhole inputs' if self.case == 'lounge_new' else 'EuRoC MH01 right | common pinhole inputs'
        text(canvas, title, (32,35), .85, FG, 2)
        speed = self.span / ((count-1)/fps)
        text(canvas, f'Saved physical-iPhone logs; offline visualization, not screen capture | source-time {speed:.2f}x, not phone FPS', (32,65), .56, MUTED)
        for n, side in enumerate(self.sides):
            x = n*800
            row, state = side.rows[index], side.states[index]
            engine_title = {'upstream':'UPSTREAM PTAM | original asynchronous mapper',
                            'current':'CURRENT iPTAM | serial mapper',
                            'mobile':'MOBILE VARIANT | paper-inspired modern core'}[side.info['engine']]
            if side.info['engine']=='mobile' and side.info.get('initialization_policy_difference'):
                engine_title='MOBILE VARIANT | automatic planar initialization'
            if side.info['engine']=='mobile' and not side.info.get('initialization_policy_known', False):
                engine_title='MOBILE VARIANT | initialization policy unverified'
            text(canvas, engine_title, (x+35,102), .65, FG, 2)
            cv2.rectangle(canvas, (x+35,113), (x+765,150), COLORS[state], -1)
            text(canvas, state + ' (common patch rule)' if state == 'GOOD' else state, (x+48,140), .65, (16,20,26), 2)
            canvas[168:648, x+80:x+720] = self.image
            text(canvas, f'Frame {index+1}/{len(self.times)} | source {self.times[index]-self.times[0]:.2f}s | raw quality {row["quality"]}', (x+35,677), .55)
            support = str(row['pose_inliers']) if side.info['strict_support_available'] else 'unknown'
            lag = f'{row["schedule_lag_ms"]:.1f}ms' if row.get('schedule_lag_ms') is not None else 'unknown'
            text(canvas, f'Matches {row["found"]} | robust support {support} | processing {row["processing_ms"]:.1f}ms | lag {lag}', (x+35,703), .49)
            scope = {'upstream':'Tracker+export; background mapper CPU excluded',
                     'current':'Tracker+serial mapper+export in one call',
                     'mobile':'Foreground call; extra idle mapping logged separately'}[side.info['engine']]
            text(canvas, scope, (x+35,727), .46, MUTED)
            e=side.info['common_evaluation'];rms=e.get('position_rms_m')
            rms_text=f'{rms:.4f} m' if rms is not None else 'no fit'
            motion=e.get('reference_motion_coverage')
            motion_text=f'{100*motion:.2f}%' if motion is not None else 'unknown'
            text(canvas, f'Primary RMS {rms_text} | motion covered {motion_text}', (x+35,767), .71, FG, 2)
            shared_rms = self.shared['engines'][side.info['engine']]['evaluation'].get('position_rms_m') if self.shared else None
            if self.shared is not None:
                accepted, associated = self.shared['intersection_frames'], self.shared['reference_associated_intersection_frames']
                note = f' ({associated} ref)' if associated != accepted else ''
                score = f'{shared_rms:.4f} m' if shared_rms is not None else 'no fit'
                text(canvas, f'Shared accepted times {accepted}/{len(self.times)}{note} | RMS {score} | separate fit', (x+35,793), .47, MUTED)
            else:
                text(canvas, 'Offline Sim3 on GOOD subset; more GOOD alone is not an accuracy win', (x+35,793), .46, MUTED)
            cv2.rectangle(canvas, (x+35,807), (x+765,984), (29,34,42), -1)
            ref = self.ref_pixels + [x,0]
            cv2.polylines(canvas, [ref], False, (114,120,132), 1, cv2.LINE_AA)
            if side.pixels is not None:
                for j in range(1,index+1):
                    if side.good[j-1] and side.good[j]:
                        cv2.line(canvas, tuple(side.pixels[j-1]+[x,0]), tuple(side.pixels[j]+[x,0]), COLORS['GOOD'], 2, cv2.LINE_AA)
                if side.good[index]:
                    cv2.circle(canvas, tuple(side.pixels[index]+[x,0]), 4, COLORS['GOOD'], -1)
            else:
                text(canvas, 'No fitted supported trajectory', (x+230,904), .55, COLORS['LOST'])
            text(canvas, f'{"".join("XYZ"[a] for a in self.axes)}; full reference gray; GOOD fragments green; gaps unjoined', (x+45,825), .42, MUTED)
            strict=str(side.info['strict_good_frames']) if side.info['strict_support_available'] else 'unknown'
            text(canvas, f'Raw GOOD {side.info["raw_quality_good_frames"]} | common {side.info["common_good_frames"]}/{len(self.times)} | strict {strict}', (x+35,1011), .58)
            for j,status in enumerate(side.states):
                a=int(x+35+(self.times[j]-self.times[0])/self.span*730)
                end=self.times[j+1] if j+1<len(self.times) else self.times[-1]
                b=int(x+35+(end-self.times[0])/self.span*730)
                cv2.rectangle(canvas,(a,1028),(max(a,b),1042),COLORS[status],-1)
            cursor=int(x+35+(self.times[index]-self.times[0])/self.span*730)
            cv2.line(canvas,(cursor,1024),(cursor,1046),FG,2)
        reference_label='Simulation truth' if self.case=='lounge_new' else 'EuRoC Leica/IMU reference'
        text(canvas, 'GOOD green | weak amber | lost red | initializing gray | upstream final-pose geometric support is unavailable', (32,1070), .48, MUTED)
        text(canvas, f'{reference_label}; Sim3 fits GOOD subset only | {self.initialization_caption} | no feature overlays', (32,1093), .46, MUTED)
        return canvas


def encode(pair, path, fps, duration):
    count=max(2,round(fps*duration))
    command=['ffmpeg','-hide_banner','-loglevel','warning','-y','-f','rawvideo','-pix_fmt','bgr24',
        '-s',f'{WIDTH}x{HEIGHT}','-r',str(fps),'-i','pipe:0','-an','-c:v','libx264','-threads','2',
        '-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    with path.with_suffix('.ffmpeg.log').open('w') as log:
        process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
        try:
            for i in range(count):
                process.stdin.write(pair.frame(i,count,fps).tobytes())
            process.stdin.close()
            if process.wait()!=0:
                raise RuntimeError(f'ffmpeg failed for {path}')
        except BaseException:
            process.kill();process.wait();raise
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(path)]))
    stream=probe['streams'][0]
    assert stream['codec_name']=='h264' and stream['pix_fmt']=='yuv420p' and int(stream['nb_frames'])==count
    assert [stream['width'],stream['height']]==[WIDTH,HEIGHT]
    return dict(path=str(path),sha256=digest(path),bytes=path.stat().st_size,frames=count,
        output_duration_seconds=float(probe['format']['duration']),fps=fps,ffmpeg_command=command,
        source_time_speed=pair.span/((count-1)/fps),first_source_index=0,last_source_index=len(pair.times)-1)


def main():
    global WIDTH
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--datasets',nargs='+',choices=['lounge_new','euroc_cam1'],default=['lounge_new','euroc_cam1'])
    p.add_argument('--repeat',default='rep01')
    p.add_argument('--engines',nargs='+',choices=['upstream','current','mobile'],default=['upstream','current'])
    p.add_argument('--system-comparison',action='store_true',help='Explicitly show differing initialization policies')
    p.add_argument('--fps',type=int,default=20)
    args=p.parse_args()
    if not 1<=args.fps<=60:
        p.error('fps must be in1..60')
    if len(args.engines)<2 or len(set(args.engines))!=len(args.engines):
        p.error('Select two or three distinct engines')
    WIDTH=800*len(args.engines)
    check_output_storage(args.output)
    if args.output.exists():raise FileExistsError(args.output)
    report=json.loads(args.report.read_text())
    cv2.setNumThreads(0);cv2.ocl.setUseOpenCL(False)
    if args.system_comparison and not report.get('automatic_system_comparison'):
        p.error('System comparison requires an explicitly evaluated system-comparison report')
    pairs=[Pair(report,name,args.repeat,args.engines,args.system_comparison) for name in args.datasets]
    args.output.mkdir(parents=True)
    shutil.copyfile(__file__,args.output/Path(__file__).name)
    receipt=dict(description='Offline side-by-side saved physical-phone comparison; not screen capture or measured phone FPS',
        report=str(args.report),report_sha256=digest(args.report),script_sha256=digest(Path(__file__)),
        repeat=args.repeat,engines=args.engines,fps=args.fps,cases=[],outputs=[])
    for pair in pairs:
        duration=min(30.,math.ceil((pair.span+float(np.median(np.diff(pair.times))))*args.fps)/args.fps)
        print(f'Rendering {pair.case} {args.repeat}: {len(pair.times)} shared images, {duration:.2f}s',flush=True)
        result=encode(pair,args.output/f'{pair.case}_{"_vs_".join(args.engines)}.mp4',args.fps,duration)
        receipt['cases'].append(pair.receipt);receipt['outputs'].append(result)
        sample=pair.frame(round(result['frames']/2),result['frames'],args.fps)
        cv2.imwrite(str(args.output/f'{pair.case}_sample.jpg'),sample)
        (args.output/'provenance.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
    print(args.output,flush=True)


if __name__=='__main__':main()
