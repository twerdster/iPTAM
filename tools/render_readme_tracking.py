#!/usr/bin/env python3
"""Make a compact GIF from an evaluated, saved mobile replay (never run SLAM).

Example, with paths relative to your own evidence directory::

    python tools/render_readme_tracking.py --run results/lounge_new/rep01 \
        --report results/comparison/report.json --dataset lounge_new \
        --output docs/assets/lounge-tracking.gif

Requires Pillow 10.1 or newer and NumPy. The run's provenance locates the prepared images;
use --source if those images have moved. No source paths or device identifiers
are copied into the GIF or its public receipt. Animation timing follows source
exposures; it is not a screen recording or a processing-speed measurement.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageSequence

BG = '#101b2a'
PANEL = '#172638'
FG = '#edf4fb'
MUTED = '#a3b3c7'
GREEN = '#53dfb2'
AMBER = '#ffc56c'
RED = '#f3828e'
GRAY = '#65768b'
W, H = 640, 438
IX, IY, IW, IH = 12, 62, 424, 318
PX = 448


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def text(draw, xy, value, size=14, fill=FG):
    draw.text(xy, str(value), font=ImageFont.load_default(size=size), fill=fill)


def good(row):
    return (row.get('quality') == 2 and row.get('initialized') and row.get('finite')
            and row.get('map_points', 0) > 0 and row.get('found', 0) >= 10
            and row.get('pose_inliers', 0) >= 10)


def state(row):
    if not row.get('initialized'):
        return 'Initializing', GRAY
    if good(row):
        return 'Tracking', GREEN
    if row.get('quality') == 1:
        return 'Weak', AMBER
    return 'Lost', RED


def load(args):
    report = json.loads(args.report.read_text())
    candidates = [r for r in report['groups']['mobile'][args.dataset]
                  if Path(r['run']).resolve() == args.run.resolve()]
    if len(candidates) != 1:
        raise ValueError('Expected exactly one evaluated mobile trial matching --run')
    info = candidates[0]
    if not info.get('eligible_for_source_paced_system_comparison'):
        raise ValueError('Trial must be complete, verified, source-paced and eligible')
    for path, expected in info['artifacts'].items():
        if sha(path) != expected:
            raise ValueError('Saved evaluation artifact changed')
    manifest = json.loads((args.run / 'requested-manifest.json').read_text())
    rows_path = args.run / 'run/frames.jsonl'
    rows = [json.loads(line) for line in rows_path.read_text().splitlines() if line.strip()]
    summary = json.loads((args.run / 'run/summary.json').read_text())
    provenance = json.loads((args.run / 'provenance.json').read_text())
    source = args.source or Path(provenance['source'])
    times = np.array([f['timestamp'] for f in manifest['frames']])
    if len(rows) != len(times) or not np.allclose(times, [r['timestamp'] for r in rows], atol=1e-6, rtol=0):
        raise ValueError('Frame count or source exposure timestamps do not match')
    masks = np.array([good(r) for r in rows], dtype=bool)
    if masks.sum() != info['strict_good_frames']:
        raise ValueError('Strict tracking mask disagrees with saved evaluation')
    pixel_hash = hashlib.sha256()
    images = []
    for frame in manifest['frames']:
        image = Image.open(source / frame['path']).convert('L')
        if image.size != (manifest['width'], manifest['height']):
            raise ValueError('Prepared source image dimensions differ')
        pixel_hash.update(image.tobytes())
        images.append(image.resize((IW, IH), Image.Resampling.LANCZOS))
    if pixel_hash.hexdigest() != summary['rectified_pixels_concatenated_sha256']:
        raise ValueError('Prepared pixels differ from pixels processed by phone')
    reference_path = source / 'reference.json'
    saved_reference = Path(info['evaluation_directory']) / 'strict/reference.json'
    if sha(reference_path) != sha(saved_reference):
        raise ValueError('Evaluation reference changed')
    reference = json.loads(reference_path.read_text())
    poses = np.array([np.array(p['matrix'])[:3, 3] for p in reference['poses']])
    if (len(poses) != len(rows) or not np.allclose(
            times, [p['timestamp'] for p in reference['poses']], atol=1e-6, rtol=0)):
        raise ValueError('This small renderer requires an exposure-aligned reference')
    fit = info['strict_evaluation']['sim3']
    estimated = np.array([r['world_from_camera_t_qxyzw'][:3] for r in rows])
    estimated = fit['scale'] * (estimated @ np.array(fit['rotation']).T) + fit['translation']
    axes = np.sort(np.argsort(np.ptp(poses, axis=0))[-2:])
    all_points = np.concatenate([poses, estimated[masks]])[:, axes]
    center = (all_points.min(0) + all_points.max(0)) / 2
    scale = 142 / max(float(np.ptp(all_points, axis=0).max()), .1)

    def project(points):
        p = (points[:, axes] - center) * scale
        return np.column_stack([p[:, 0] + 538, 222 - p[:, 1]])

    receipt = {
        'dataset': args.dataset,
        'description': 'Simulated lounge; physical-phone automatic replay',
        'visualization': 'Offline source frames plus saved camera trajectory; no invented feature overlays',
        'alignment': 'Offline strict-support Sim3 fit; reference never fed to estimator',
        'trajectory_connections': 'Only adjacent source frames with two accepted poses; weak/lost gaps remain open',
        'timing': 'Source exposure timing; not a screen recording or a measured processing frame rate',
        'source_frames': len(rows),
        'accepted_frames': int(masks.sum()),
        'first_accepted_frame': int(np.flatnonzero(masks)[0]),
        'weak_frame_indices': [i for i,r in enumerate(rows) if r['quality'] == 1],
        'source_pixels_sha256': pixel_hash.hexdigest(),
        'frames_jsonl_sha256': sha(rows_path),
        'requested_manifest_sha256': sha(args.run / 'requested-manifest.json'),
        'benchmark_executable_sha256': provenance['app']['executable_sha256'],
        'strict_position_rms_m': info['strict_evaluation']['position_rms_m'],
        'reference_motion_coverage': info['strict_evaluation']['reference_motion_coverage'],
    }
    return rows, masks, times, images, project(poses), project(estimated), receipt


def render(rows, masks, times, images, ref, estimated):
    frames = []
    for index, row in enumerate(rows):
        canvas = Image.new('RGB', (W, H), BG)
        draw = ImageDraw.Draw(canvas)
        text(draw, (12, 10), 'Simulated lounge · phone replay', 20)
        text(draw, (12, 37), 'Mobile PTAM  /  automatic initialization', 13, MUTED)
        label, color = state(row)
        draw.rounded_rectangle((525, 12, 628, 39), radius=8, fill=color)
        text(draw, (538, 17), label, 14, BG)
        canvas.paste(images[index].convert('RGB'), (IX, IY))
        draw.rounded_rectangle((PX, IY, 628, 380), radius=10, fill=PANEL)
        text(draw, (460, 74), 'Camera path', 17)
        text(draw, (460, 100), 'offline Sim3 fit', 12, MUTED)
        # Reference is displayed for context; estimator output is never invented.
        draw.line([tuple(p) for p in ref], fill=GRAY, width=2)
        for j in range(1, index + 1):
            if masks[j - 1] and masks[j]:
                draw.line([tuple(estimated[j - 1]), tuple(estimated[j])], fill=GREEN, width=3)
        if masks[index]:
            x, y = estimated[index]
            draw.ellipse((x-4,y-4,x+4,y+4),fill=FG,outline=GREEN,width=2)
        draw.line((461, 328, 478, 328), fill=GRAY, width=2)
        text(draw, (485, 320), 'Reference', 12, MUTED)
        draw.line((461, 352, 478, 352), fill=GREEN, width=3)
        text(draw, (485, 344), 'Accepted pose', 12)
        text(draw, (12, 388), f'{times[index]-times[0]:.1f}s  ·  {index+1}/{len(rows)} frames', 13)
        text(draw, (262, 388), f'{int(masks.sum())}/{len(rows)} accepted', 13, GREEN)
        text(draw, (460, 388), f'{row.get("pose_inliers", 0)} pose inliers', 13, MUTED)
        for j, other in enumerate(rows):
            _, c = state(other)
            x0 = 12 + j * 616 / len(rows)
            x1 = 12 + (j+1) * 616 / len(rows)
            draw.rectangle((x0, 413, x1, 420), fill=c)
        x = 12 + (index + .5) * 616 / len(rows)
        draw.line((x, 409, x, 424), fill=FG, width=2)
        frames.append(canvas)
    return frames


def palette(gray_levels):
    colors = [(round(i * 255 / (gray_levels - 1)),)*3 for i in range(gray_levels)]
    # Fixed palette keeps source luminance compact and labels/crucial states clear.
    for value in [BG, PANEL, FG, MUTED, GREEN, AMBER, RED, GRAY]:
        colors.append(tuple(bytes.fromhex(value[1:])))
    raw = [v for rgb in colors for v in rgb]
    raw += [0] * (768 - len(raw))
    p = Image.new('P', (1,1)); p.putpalette(raw)
    return p


def save_gif(frames, times, output, max_bytes):
    milliseconds = np.diff(times) * 1000
    durations = [max(10, round(float(d) / 10) * 10) for d in milliseconds]
    durations.append(durations[-1])
    output.parent.mkdir(parents=True, exist_ok=True)
    # Keep all 100 source exposures, including the single weak frame.
    for levels in [48, 40, 32, 24, 20, 16, 12, 8]:
        p = palette(levels)
        indexed = [f.quantize(palette=p, dither=Image.Dither.NONE) for f in frames]
        indexed[0].save(output, save_all=True, append_images=indexed[1:], loop=0,
                        duration=durations, optimize=True, disposal=1)
        if output.stat().st_size < max_bytes:
            with Image.open(output) as decoded:
                actual_duration = sum(f.info.get('duration', 0) for f in ImageSequence.Iterator(decoded))
                if decoded.n_frames != len(frames) or actual_duration != sum(durations):
                    raise ValueError('GIF frame count or duration verification failed')
            return levels, sum(durations)
    raise ValueError('GIF does not fit requested byte budget; reduce renderer dimensions')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--dataset', choices=['lounge_new'], required=True)
    p.add_argument('--source', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--max-bytes', type=int, default=2_000_000)
    args = p.parse_args()
    if args.max_bytes <= 0:
        p.error('--max-bytes must be positive')
    rows, masks, times, images, ref, estimated, receipt = load(args)
    frames = render(rows, masks, times, images, ref, estimated)
    levels, duration = save_gif(frames, times, args.output, args.max_bytes)
    receipt.update(gif_sha256=sha(args.output), gif_bytes=args.output.stat().st_size,
                   gif_frames=len(frames), gif_size=[W,H], gif_duration_ms=duration,
                   source_gray_levels=levels)
    args.output.with_suffix('.json').write_text(json.dumps(receipt, indent=2) + '\n')
    # A local QA artifact, intentionally omitted from the public receipt.
    sheet = Image.new('RGB', (W*2,H*3), BG)
    for n,index in enumerate([0,17,35,60,61,99]):
        sheet.paste(frames[index], ((n%2)*W, (n//2)*H))
    sheet.save(args.output.with_suffix('.contact.png'))
    print(json.dumps({'gif_bytes':receipt['gif_bytes'], 'frames':len(frames),
                      'duration_ms':duration, 'source_gray_levels':levels}))


if __name__ == '__main__':
    main()
