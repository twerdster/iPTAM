#!/usr/bin/env python3
"""Replay one frozen, image-selected recording; evaluate reference afterward.

When nested under a resource monitor the replay intentionally inherits this
process group, so the monitor accounts for the actual estimator, too.
"""
import argparse
import json
import platform
import re
import resource
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np

from evaluate_trajectory import evaluate
from prepare_datasets import digest
from run_host_validation import summarize, trial_failed, write_json
from storage import check_output_storage


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--binary-sha256', required=True)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--selection', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--timeout', type=float, default=180.)
    args = p.parse_args()
    if not np.isfinite(args.timeout) or args.timeout <= 0:
        p.error('timeout must be finite and positive')
    check_output_storage(args.output)
    if args.output.exists():
        raise FileExistsError(args.output)
    if digest(args.binary) != args.binary_sha256:
        raise ValueError('Frozen replay binary hash mismatch')
    manifest_path = args.dataset / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    selection = json.loads(args.selection.read_text())
    if selection['manifest_sha256'] != digest(manifest_path) or selection['reference_files_read']:
        raise ValueError('Selection must use this exact manifest and images only')
    selected = selection['selected']
    if selected is None:
        raise ValueError('No image-only bootstrap was selected')
    first, second = selected['first'], selected['second']
    limit = len(manifest['frames'])
    if not 0 <= first < second < limit:
        raise ValueError('Selected pair is outside the recording')
    lens = manifest.get('lens_model', 'pinhole')
    if lens not in {'pinhole', 'equidistant'}:
        raise ValueError('Unsupported optical model')
    args.output.mkdir(parents=True)
    shutil.copyfile(args.selection, args.output / 'selection.json')
    requested = dict(manifest, init_first=first, init_second=second)
    write_json(args.output / 'requested-manifest.json', requested)
    command = [str(args.binary.resolve()), '--frames', str((args.dataset / 'frames.csv').resolve()),
               '--fx', str(manifest['fx']), '--fy', str(manifest['fy']),
               '--cx', str(manifest['cx']), '--cy', str(manifest['cy']), '--lens-model', lens,
               '--init-first', str(first), '--init-second', str(second), '--max-frames', str(limit),
               '--output', str((args.output / 'trajectory.csv').resolve())]
    write_json(args.output / 'command.json', dict(argv=command, binary_sha256=args.binary_sha256,
               manifest_sha256=digest(manifest_path), selection_sha256=digest(args.selection),
               wrapper_sha256=digest(Path(__file__)), platform=platform.platform(),
               reference_available_to_estimator=False, reference_read_after_replay=True,
               inherits_resource_monitor_process_group=True, timeout_seconds=args.timeout))
    start = time.monotonic()
    timed_out, numerical_abort = False, None
    offset, tail = 0, b''
    with (args.output / 'stdout.log').open('w') as out, (args.output / 'stderr.log').open('w') as err:
        # No new process group: an outer monitor must see the estimator's RSS/CPU.
        process = subprocess.Popen(command, stdout=out, stderr=err)
        while True:
            try:
                code = process.wait(timeout=.2)
                break
            except subprocess.TimeoutExpired:
                with (args.output / 'stderr.log').open('rb') as log:
                    log.seek(offset)
                    addition = log.read(1024 * 1024)
                    offset = log.tell()
                check = tail + addition
                tail = check[-128:]
                if b'NANNERY!' in check or b'tried to give me a nan' in check:
                    numerical_abort = 'Legacy internal NaN diagnostic; owned estimator stopped'
                timed_out = time.monotonic() - start > args.timeout
                if numerical_abort or timed_out:
                    process.terminate()
                    try:
                        code = process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        code = process.wait(timeout=3)
                    break
    stdout = (args.output / 'stdout.log').read_text()
    stderr = (args.output / 'stderr.log').read_text()
    actual = re.findall(r'\blens_model=(pinhole|equidistant)\b', stdout)
    if 'NANNERY!' in stderr or 'tried to give me a nan' in stderr:
        numerical_abort = numerical_abort or 'Legacy internal NaN diagnostic before exit'
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    result = dict(exit_code=code, timed_out=timed_out, numerical_abort=numerical_abort,
                  camera_protocol_failure=None if actual and all(x == lens for x in actual) else 'Lens model not confirmed',
                  requested_lens_model=lens, actual_lens_models=actual,
                  wall_seconds=time.monotonic() - start,
                  peak_rss_bytes=int(usage.ru_maxrss * (1 if platform.system() == 'Darwin' else 1024)),
                  init_first=first, init_second=second, max_frames=limit,
                  **summarize(args.output / 'trajectory.csv', second, limit))
    result['initialization_failed'] = code == 2 and result['rows'] == limit and bool(re.search(r'\bmap_good=0\b', stdout))
    result['pipeline_complete'] = not trial_failed(result) and result['rows'] == limit
    write_json(args.output / 'result.json', result)
    # References are first opened here; they cannot influence replay or selection.
    if result['pipeline_complete']:
        shutil.copyfile(args.dataset / 'reference.json', args.output / 'reference.json')
        write_json(args.output / 'evaluation.json', evaluate(args.output / 'requested-manifest.json', args.output / 'trajectory.csv'))
    print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    if not result['pipeline_complete']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
