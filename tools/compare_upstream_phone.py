#!/usr/bin/env python3
"""Evaluate saved upstream/current phone runs without controlling either app.

Usage: .venv/bin/python tools/compare_upstream_phone.py --root SUITE --output FRESH
Expected layout: SUITE/{upstream,current}/DATASET/repNN/. A single saved phone
run is also accepted for validation. Output is a fresh directory containing JSON,
Markdown and private evaluator inputs. Original recordings and logs are read only.
"""
import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

from evaluate_trajectory import evaluate, is_rigid, read_estimates
from storage import check_output_storage
from phone_config import engine_from_bundle


DEFAULT_PLAN = Path('data/prepared/input-plan.json')
ENGINES = ('upstream', 'current')
COLUMNS = ['timestamp', 'tx', 'ty', 'tz'] + [f'r{i}{j}' for i in range(3) for j in range(3)] + [
    'quality', 'found', 'points', 'elapsed_ms', 'pose_inliers', 'map_id',
    'solver_failures', 'solver_retained_stops']


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_json(path, optional=False):
    if optional and not path.exists():
        return {}
    return json.loads(path.read_text())


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def distribution(values):
    values = np.asarray([float(v) for v in values if v is not None], dtype=float)
    usable = values[np.isfinite(values)]
    return dict(samples=len(values), finite_samples=len(usable), nonfinite_samples=int((~np.isfinite(values)).sum()),
                minimum=float(usable.min()) if len(usable) else None,
                mean=float(usable.mean()) if len(usable) else None,
                median=float(np.median(usable)) if len(usable) else None,
                p95=float(np.percentile(usable, 95)) if len(usable) else None,
                maximum=float(usable.max()) if len(usable) else None)


def quality_good(value):
    return str(value).lower() in {'2', 'good'}


def logged_pose(row):
    value = row.get('world_from_camera_t_qxyzw')
    if value is None and row.get('finite') is False:
        # The app intentionally omits an invalid inverse. Preserve failure as
        # an undefined transform, never identity, prediction or interpolation.
        return np.full(7, np.nan)
    if value is None:
        raise ValueError('A purported finite saved frame omits its optical-camera pose')
    pose = np.asarray(value, dtype=float)
    if pose.shape != (7,):
        raise ValueError('Saved camera pose is not translation + quaternion xyzw')
    return pose


def discover(root, cases):
    if (root / 'requested-manifest.json').exists():
        return [root]
    found = []
    for engine in ENGINES:
        for case in cases:
            base = root / engine / case['name']
            if not base.is_dir():
                continue
            if (base / 'requested-manifest.json').exists():
                found.append(base)
            else:
                children = [p for p in sorted(base.iterdir()) if p.is_dir() and not p.name.startswith('.')]
                found.extend(children or [base])
    return found


def identify(run, root, cases, provenance, manifest, summary):
    parts = run.relative_to(root).parts
    engine = parts[0] if parts and parts[0] in (*ENGINES, 'mobile') else None
    if engine is None:
        variant = str(summary.get('engine_variant', provenance.get('engine', ''))).lower()
        if 'upstream' in variant:
            engine = 'upstream'
        elif 'mobile' in variant:
            engine = 'mobile'
        elif any(v in variant for v in ('current', 'modern', 'serial')):
            engine = 'current'
        else:
            engine = engine_from_bundle(provenance.get('app', {}).get('bundle_identifier'))
            if engine is None and 'serial' in str(summary.get('estimator', '')).lower():
                engine = 'current'
    case = next((c for c in cases if len(parts) > 1 and parts[1] == c['name']), None)
    if case is None:
        source = provenance.get('source')
        case = next((c for c in cases if source and Path(c['dataset']) == Path(source)), None)
    if case is None:
        name = manifest.get('name', '')
        case = next((c for c in cases if c['name'] == name), None)
    return engine or 'unknown', case


def json_rows_to_csv(rows, target):
    """Convert saved optical-camera poses using the existing harness convention."""
    with target.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            pose = logged_pose(row)
            rotation = np.full((3, 3), np.nan)
            if np.isfinite(pose).all() and abs(np.linalg.norm(pose[3:]) - 1) <= 2e-3:
                rotation = Rotation.from_quat(pose[3:]).as_matrix()
            values = dict(timestamp=row['timestamp'], tx=pose[0], ty=pose[1], tz=pose[2],
                quality=row['quality'], found=row['found'], points=row['map_points'],
                elapsed_ms=row.get('processing_ms', ''), pose_inliers=row.get('pose_inliers', ''),
                map_id=row.get('map_id', '0'), solver_failures=row.get('solver_failures', 0),
                solver_retained_stops=row.get('solver_retained_stops', row.get('retained_solves', 0)))
            values.update({f'r{i}{j}': rotation[i, j] for i in range(3) for j in range(3)})
            writer.writerow(values)


def exposure_indices(rows, times):
    indices = []
    for row in rows:
        t = row['timestamp']
        if not np.isfinite(t):
            indices.append(None)
            continue
        j = int(np.argmin(abs(times - t)))
        indices.append(j if abs(times[j] - t) <= 1e-6 else None)
    return indices


def gaps(indices, total):
    if not indices:
        return dict(first_good_frame=None, last_good_frame=None, largest_gap_frames=None,
                    gaps_after_first_good=[], tracking_established=False)
    present = set(indices)
    missing = [i for i in range(min(present), total) if i not in present]
    segments = np.split(missing, np.flatnonzero(np.diff(missing) > 1) + 1) if missing else []
    intervals = [dict(first_frame=int(s[0]), last_frame=int(s[-1]), frames=len(s)) for s in segments]
    return dict(first_good_frame=min(present), last_good_frame=max(present),
                largest_gap_frames=max((s['frames'] for s in intervals), default=0),
                gaps_after_first_good=intervals, tracking_established=True)


def normalized_csv(rows, raw_rows, indices, target, strict):
    with target.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        for i, row in enumerate(rows):
            raw = raw_rows[i] if raw_rows else {}
            accepted_state = (indices[i] is not None and raw.get('initialized', True) and raw.get('finite', True))
            values = dict(timestamp=row['timestamp'], tx=row['matrix'][0, 3], ty=row['matrix'][1, 3],
                tz=row['matrix'][2, 3], quality=row['quality'] if accepted_state else '0',
                found=row['found'], points=row['points'], elapsed_ms=row['elapsed_ms'],
                pose_inliers=row['pose_inliers'] if strict and row['pose_inliers'] is not None else '',
                map_id=row['map_id'], solver_failures=row['solver_failures'],
                solver_retained_stops=row['retained_solver_stops'])
            values.update({f'r{a}{b}': row['matrix'][a, b] for a in range(3) for b in range(3)})
            writer.writerow(values)


def evaluate_saved(manifest, reference, rows, raw_rows, indices, folder, strict, solver_available):
    folder.mkdir()
    write_json(folder / 'manifest.json', manifest)
    shutil.copyfile(reference, folder / 'reference.json')
    normalized_csv(rows, raw_rows, indices, folder / 'trajectory.csv', strict)
    result = evaluate(folder / 'manifest.json', folder / 'trajectory.csv')
    result['comparison_policy'] = ('final-pose robust support >=10 in addition to patch-supported native GOOD'
                                   if strict else 'native GOOD, initialized nonempty map, found>=10, finite rigid pose; final-pose support deliberately not required')
    result['pose_support_omitted_for_common_rule'] = not strict
    result['solver_diagnostics_available'] = solver_available
    if not solver_available:
        result['handled_solver_failures'] = None
        result['retained_solver_stops'] = None
    write_json(folder / 'evaluation.json', result)
    return result


def mobile_initialization_policy(summary, provenance):
    """Confirm the executed config; other declarations may contradict it.

    A missing config is unknown, never an implicit fixed-pair control. Profile
    and human-readable policy metadata are cross-checks, not substitutes for
    the executed setting.
    """
    config = summary.get('mobile_config')
    value = config.get('automatic_initialization') if isinstance(config, dict) else None
    confirmed = type(value) in (bool, int) and value in (0, 1)
    declarations = {'config': bool(value)} if confirmed else {}
    profile = provenance.get('mobile_profile', summary.get('mobile_profile'))
    if isinstance(profile, str):
        profile = profile.strip().lower()
        if profile == 'auto':
            declarations['profile'] = True
        elif profile in ('paper', 'diagnostics', 'serial', 'coarse', 'partial'):
            declarations['profile'] = False
    label = summary.get('initialization_policy')
    if isinstance(label, str):
        label = label.lower()
        if 'automatic' in label:
            declarations['summary'] = True
        elif 'fixed' in label:
            declarations['summary'] = False
    conflict = len(set(declarations.values())) > 1
    state = ('conflicting' if conflict else
             'automatic' if confirmed and value else
             'fixed' if confirmed else 'unknown')
    return state, declarations


def analyze_run(run, root, cases, output):
    provenance = read_json(run / 'provenance.json', optional=True)
    manifest = read_json(run / 'requested-manifest.json', optional=True)
    summary_path = run / 'run/summary.json'
    if not summary_path.exists():
        summary_path = run / 'partial-run/summary.json'
    if not summary_path.exists():
        summary_path = run / 'latest-summary.json'
    summary = read_json(summary_path, optional=True)
    engine, case = identify(run, root, cases, provenance, manifest, summary)
    report = dict(run=str(run), engine=engine, dataset=case['name'] if case else 'unknown',
                  engine_variant=summary.get('engine_variant'), state='incomplete', warnings=[], input_errors=[],
                  strict_evaluation=None, strict_good_frames=None, strict_support_available=False)
    report['artifacts'] = {str(p): digest(p) for p in [run / 'provenance.json',
        run / 'requested-manifest.json', summary_path, run / 'validation.json', run / 'failure.json',
        run / 'invalidated.json'] if p.exists()}
    report['app'] = provenance.get('app')
    declared_engine = summary.get('engine_variant')
    if declared_engine in (*ENGINES, 'mobile') and declared_engine != engine:
        report['input_errors'].append(f'Folder engine {engine} conflicts with declared engine {declared_engine}')
    bundle_engine = engine_from_bundle(provenance.get('app', {}).get('bundle_identifier'))
    if bundle_engine is not None and bundle_engine != engine:
        report['input_errors'].append(f'Folder engine {engine} conflicts with app bundle for {bundle_engine}')
    report['summary_status'] = summary.get('status')
    report['recorded_failure'] = read_json(run / 'failure.json', optional=True)
    report['invalidated'] = read_json(run / 'invalidated.json', optional=True)
    if case is None or not manifest:
        report['warnings'].append('Missing requested manifest or dataset identification; no metrics fabricated')
        return report
    source = Path(case['dataset'])
    original = read_json(source / 'manifest.json')
    if digest(source / 'manifest.json') != case['manifest_sha256']:
        report['input_errors'].append('Frozen dataset manifest hash changed')
    if any(manifest.get(k, 'pinhole' if k == 'lens_model' else None) != original.get(k, 'pinhole' if k == 'lens_model' else None)
           for k in ['width', 'height', 'fx', 'fy', 'cx', 'cy', 'lens_model', 'frames']):
        report['input_errors'].append('Requested images/K/model/exposure clocks differ from the common input')
    if [manifest.get('init_first'), manifest.get('init_second')] != [case['init_first'], case['init_second']]:
        report['input_errors'].append('Initialization indices differ from the frozen pair')
    policy_state, policy_declarations = (mobile_initialization_policy(summary, provenance)
                                         if engine == 'mobile' else ('fixed', {}))
    automatic_policy = policy_state == 'automatic'
    if policy_state == 'conflicting':
        report['input_errors'].append('Conflicting mobile initialization policy declarations: ' +
            ', '.join(f'{key}={"automatic" if value else "fixed"}' for key, value in policy_declarations.items()))
    elif policy_state == 'unknown':
        report['warnings'].append('Mobile initialization config is unavailable or malformed; policy is unknown, system comparison only, never fixed-pair isolation')
    if automatic_policy:
        report['warnings'].append('Automatic initialization is a separate policy; fixed-pair tracking isolation is unavailable')
    manifest_hash = digest(run / 'requested-manifest.json')
    for claimed in [provenance.get('manifest_sha256'), summary.get('manifest_sha256')]:
        if claimed and claimed != manifest_hash:
            report['input_errors'].append('Recorded requested-manifest hash disagrees with file')
    pixel_verified = summary.get('rectified_pixels_concatenated_sha256') == case['pixel_sha256']
    declared_complete = summary.get('status') == 'completed' and summary.get('frames') == len(manifest['frames'])
    partial_saved = summary_path.parent.name == 'partial-run'
    if not pixel_verified:
        if partial_saved or not declared_complete:
            report['warnings'].append('Partial/in-progress run has no verified complete-input pixel hash; this is not evidence of input corruption')
        else:
            report['input_errors'].append('Complete phone pixel hash is missing or differs from common input')
    reference = run / 'reference.json'
    if not reference.exists():
        reference = source / 'reference.json'
    if digest(reference) != case['reference_sha256']:
        raise ValueError('Evaluation reference hash differs from the frozen input plan')
    raw_path = run / 'run/frames.jsonl'
    if not raw_path.exists():
        raw_path = run / 'partial-run/frames.jsonl'
    partial_saved = partial_saved or raw_path.parent.name == 'partial-run' and raw_path.exists()
    raw_rows = [json.loads(line) for line in raw_path.read_text().splitlines() if line.strip()] if raw_path.exists() else []
    trajectory = run / 'trajectory.csv'
    private = output / 'evaluated' / f'{engine}_{case["name"]}_{hashlib.sha256(str(run).encode()).hexdigest()[:12]}'
    private.mkdir(parents=True)
    report['evaluation_directory'] = str(private)
    if not trajectory.exists():
        if not raw_rows:
            report['warnings'].append('No saved frame rows; no metrics fabricated')
            return report
        trajectory = private / 'converted-source.csv'
        json_rows_to_csv(raw_rows, trajectory)
    rows = read_estimates(trajectory)
    report['artifacts'][str(trajectory)] = digest(trajectory)
    if raw_path.exists():
        report['artifacts'][str(raw_path)] = digest(raw_path)
    if raw_rows and (len(raw_rows) != len(rows) or any(abs(a['timestamp'] - b['timestamp']) > 1e-6
                                                      for a, b in zip(raw_rows, rows))):
        raise ValueError('CSV and saved phone JSON rows do not agree in count/time')
    for raw, row in zip(raw_rows, rows):
        if (str(raw['quality']).lower() != str(row['quality']).lower() or
                int(raw['found']) != row['found'] or int(raw['map_points']) != row['points'] or
                raw.get('pose_inliers') != row['pose_inliers']):
            raise ValueError('CSV and saved phone JSON quality/support fields disagree')
        pose = logged_pose(raw)
        if np.isfinite(pose).all() and abs(np.linalg.norm(pose[3:]) - 1) <= 2e-3:
            if (not np.allclose(row['matrix'][:3, 3], pose[:3], atol=1e-8, rtol=0) or
                    not np.allclose(row['matrix'][:3, :3], Rotation.from_quat(pose[3:]).as_matrix(), atol=1e-8, rtol=0)):
                raise ValueError('CSV transform differs from the saved phone optical-camera pose')
    times = np.asarray([r['timestamp'] for r in manifest['frames']])
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0) or not len(times):
        raise ValueError('Requested exposure clocks are not finite and increasing')
    indices = exposure_indices(rows, times)
    present = [i for i in indices if i is not None]
    if len(present) != len(rows) or len(set(present)) != len(present) or any(b <= a for a, b in zip(present, present[1:])):
        report['input_errors'].append('Saved outputs include unmatched, duplicate or out-of-order exposure clocks')
    solver_available = summary.get('solver_diagnostics_available', summary.get('solver_events_available'))
    if solver_available is None:
        solver_available = bool(raw_rows) and all('solver_failures' in r for r in raw_rows)
    solver_available = bool(solver_available)
    common = evaluate_saved(manifest, reference, rows, raw_rows, indices, private / 'common', False, solver_available)
    raw_good = [i for i, r in enumerate(rows) if quality_good(r['quality'])]
    common_indices = [j for i, (r, j) in enumerate(zip(rows, indices)) if j is not None and quality_good(r['quality']) and
        r['points'] > 0 and r['found'] >= 10 and np.isfinite(r['timestamp']) and is_rigid(r['matrix']) and
        (not raw_rows or raw_rows[i].get('initialized', True) and raw_rows[i].get('finite', True))]
    assert len(set(common_indices)) == common['valid_output_frames']
    support_declared = summary.get('pose_support_diagnostics_available')
    support_available = support_declared is not False and bool(rows) and all(r['pose_inliers'] is not None for r in rows)
    if support_available:
        strict = evaluate_saved(manifest, reference, rows, raw_rows, indices, private / 'strict', True, solver_available)
        report.update(strict_evaluation=strict, strict_good_frames=strict['valid_output_frames'], strict_support_available=True)
    else:
        report['warnings'].append('Final-pose robust support unavailable; strict GOOD/accuracy are unknown, not zero')
    initialized = [indices[i] for i, r in enumerate(rows) if indices[i] is not None and
                   (raw_rows[i].get('initialized', r['points'] > 0) if raw_rows else r['points'] > 0)]
    processing = distribution([r['elapsed_ms'] for r in rows])
    lag = distribution([r.get('schedule_lag_ms') for r in raw_rows])
    delivery = distribution([r.get('delivery_wall_seconds') for r in raw_rows])
    lag_values = [r['schedule_lag_ms'] for r in raw_rows if r.get('schedule_lag_ms') is not None and np.isfinite(r['schedule_lag_ms'])]
    processing_scope = summary.get('processing_time_scope') or (
        'tracker call only; concurrent upstream mapper cost excluded' if engine == 'upstream' else
        'tracker plus serial mapping stage in the current port; not equivalent to upstream tracker-only timing')
    wall = summary.get('replay_wall_seconds')
    wall_source = 'summary.replay_wall_seconds' if wall is not None else None
    if wall is None and summary.get('started') and summary.get('ended'):
        wall = (datetime.fromisoformat(summary['ended'].replace('Z', '+00:00')) -
                datetime.fromisoformat(summary['started'].replace('Z', '+00:00'))).total_seconds()
        wall_source = 'summary ended-started; includes whatever lifecycle those timestamps cover'
    complete = len(present) == len(times) and set(present) == set(range(len(times))) and summary.get('status') == 'completed'
    failed = summary.get('status') in {'failed', 'error'} or bool(report['recorded_failure'])
    quality_counts = {}
    for row in rows:
        q = str(row['quality']).lower()
        quality_counts[q] = quality_counts.get(q, 0) + 1
    report.update(state='invalidated' if report['invalidated'] else 'failed' if failed else 'completed' if complete else 'incomplete',
        input_frames=len(times), output_rows=len(rows), missing_output_frames=len(times)-len(set(present)),
        raw_quality_counts=quality_counts, raw_quality_good_frames=len(raw_good),
        common_good_frames=common['valid_output_frames'], common_evaluation=common,
        common_tracking=gaps(common_indices, len(times)), first_initialized_frame=min(initialized) if initialized else None,
        observed_initialized=bool(initialized), never_initialized=not bool(initialized) if complete else None,
        common_input_verified=not report['input_errors'] and pixel_verified,
        full_input_pixel_hash_verified=pixel_verified, partial_artifacts=partial_saved,
        source_paced=summary.get('source_paced'), processing_ms=processing, processing_time_scope=processing_scope,
        replay_wall_seconds=wall, replay_wall_seconds_source=wall_source,
        delivery_wall_seconds=delivery, schedule_lag_ms=lag,
        lag_exceeds_1ms_frames=sum(v > 1 for v in lag_values) if lag_values else None,
        lag_exceeds_50ms_frames=sum(v > 50 for v in lag_values) if lag_values else None,
        summary_max_schedule_lag_ms=summary.get('max_schedule_lag_ms'),
        source_span_seconds=float(times[-1]-times[0]),
        thermal={k: v for k, v in summary.items() if 'thermal' in k or k == 'low_power_mode'},
        solver_diagnostics_available=solver_available,
        solver_numerical_rejections=common['handled_solver_failures'], solver_retained_stops=common['retained_solver_stops'],
        map_count_scope='approximate original asynchronous size snapshots' if engine == 'upstream' else 'serial-worker map counts',
        motion_model_uses_timestamps=summary.get('motion_model_uses_timestamps'),
        replay_timing=summary.get('replay_timing'), visual_acceptance=summary.get('visual_acceptance'),
        mobile_config=summary.get('mobile_config'), mobile_profile=provenance.get('mobile_profile'),
        initialization_policy=summary.get('initialization_policy'),
        initialization_policy_difference=automatic_policy,
        initialization_policy_state=policy_state,
        initialization_policy_known=policy_state in ('fixed', 'automatic'),
        initialization_policy_declarations=policy_declarations,
        memory={k:summary[k] for k in ('peak_resident_bytes','peak_phys_footprint_bytes') if k in summary},
        mobile_mapping=summary.get('mobile_mapping'), rolling_shutter_status=summary.get('rolling_shutter_status'),
        eligible_for_paired_source_paced_comparison=bool(engine in (*ENGINES, 'mobile') and complete and pixel_verified and not report['input_errors'] and
            not failed and policy_state == 'fixed' and not report['invalidated'] and summary.get('source_paced') is True),
        eligible_for_source_paced_system_comparison=bool(engine in (*ENGINES, 'mobile') and complete and pixel_verified and not report['input_errors'] and
            not failed and not report['invalidated'] and summary.get('source_paced') is True))
    if summary.get('source_paced') is not True:
        report['warnings'].append('Source-paced delivery is not confirmed; timing is not a matched scheduling comparison')
    if not solver_available:
        report['warnings'].append('Solver events unavailable; rejection/retained counts are unknown')
    if not complete:
        report['warnings'].append('Incomplete run: full manifest denominator retained; processed-subset metrics are partial')
    report['artifacts_unchanged_after_read'] = all(Path(p).exists() and digest(Path(p)) == h for p, h in report['artifacts'].items())
    if not report['artifacts_unchanged_after_read']:
        report['eligible_for_paired_source_paced_comparison'] = False
        report['eligible_for_source_paced_system_comparison'] = False
        report['warnings'].append('An input artifact changed during analysis; treat this as an in-progress snapshot')
    return report


def matched_timestamp_diagnostics(groups, cases, output, engines=ENGINES,
                                  eligibility_field='eligible_for_paired_source_paced_comparison'):
    """Fit predeclared rep01 runs at the same quality-selected exposure indices.

    This never chooses an initializer, window or repeat, and never changes the
    primary common CSV. Reference positions are first read after intersection.
    """
    diagnostics = []
    for case in cases:
        item = dict(dataset=case['name'], repeat='rep01', status='unavailable',
                    eligibility_field=eligibility_field,
                    policy='Intersection of logged common GOOD exposure indices, no reference selection; independent Sim3 for each engine at exactly the same timestamps; primary coverage is unchanged')
        chosen = {}
        for engine in engines:
            trials = [r for r in groups[engine][case['name']] if Path(r['run']).name == 'rep01']
            if len(trials) == 1 and trials[0].get(eligibility_field):
                chosen[engine] = trials[0]
        if len(chosen) != len(engines):
            item['reason'] = 'Requires all predeclared rep01 trials with complete, verified common inputs and source pacing'
            diagnostics.append(item)
            continue
        if any(trial.get('initialization_policy_difference') for trial in chosen.values()):
            item['policy'] += '; complete-system comparison includes different initialization policies'
        if any(trial.get('initialization_policy_state') == 'unknown' for trial in chosen.values()):
            item['policy'] += '; mobile initialization policy is unverified'
        inputs = {}
        accepted = {}
        for engine, trial in chosen.items():
            base = Path(trial['evaluation_directory']) / 'common'
            manifest = read_json(base / 'manifest.json')
            times = np.asarray([r['timestamp'] for r in manifest['frames']])
            rows = read_estimates(base / 'trajectory.csv')
            indices = exposure_indices(rows, times)
            accepted[engine] = {i for r, i in zip(rows, indices) if i is not None and quality_good(r['quality']) and
                r['found'] >= 10 and r['points'] > 0 and np.isfinite(r['timestamp']) and is_rigid(r['matrix'])}
            inputs[engine] = dict(base=base, manifest=manifest, rows=rows, indices=indices,
                                  primary_csv_sha256=digest(base / 'trajectory.csv'))
        assert all(v['manifest']['frames'] == inputs[engines[0]]['manifest']['frames'] for v in inputs.values())
        intersection = sorted(set.intersection(*(accepted[e] for e in engines)))
        shared = set(intersection)
        # Reference is evaluation-only; the accepted exposure set is already fixed.
        reference_path = inputs[engines[0]]['base'] / 'reference.json'
        assert all(digest(reference_path) == digest(v['base'] / 'reference.json') for v in inputs.values())
        reference = read_json(reference_path)
        rt = np.asarray([r['timestamp'] for r in reference['poses']])
        positions = np.asarray([np.asarray(r['matrix'])[:3, 3] for r in reference['poses']])
        times = np.asarray([r['timestamp'] for r in inputs[engines[0]]['manifest']['frames']])
        matched = []
        for i in intersection:
            j = int(np.argmin(abs(rt - times[i]))) if len(rt) else None
            if j is not None and abs(rt[j] - times[i]) <= 1e-6:
                matched.append((i, j))
        length = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()) if len(positions) > 1 else 0.
        covered = float(sum(np.linalg.norm(positions[b[1]] - positions[a[1]]) for a, b in zip(matched, matched[1:]) if b[0]-a[0] == 1))
        item.update(status='computed', input_frames=len(times), intersection_frames=len(intersection),
            intersection_frame_indices=intersection, intersection_input_coverage=len(intersection)/len(times),
            reference_associated_intersection_frames=len(matched), reference_role=reference['role'],
            full_reference_path_length_m=length, shared_adjacent_reference_path_length_m=covered,
            shared_reference_motion_coverage=covered/length if length > 1e-8 else None, engines={})
        folder = output / 'matched_timestamps' / case['name']
        folder.mkdir(parents=True)
        for engine, source in inputs.items():
            rows = [dict(row, quality=row['quality'] if i in shared else '0')
                    for row, i in zip(source['rows'], source['indices'])]
            metrics = evaluate_saved(source['manifest'], source['base']/'reference.json', rows, [],
                source['indices'], folder/engine, False, chosen[engine]['solver_diagnostics_available'])
            assert metrics['valid_output_frames'] == len(intersection)
            assert metrics['matched_reference_frames'] == len(matched)
            assert digest(source['base']/'trajectory.csv') == source['primary_csv_sha256']
            item['engines'][engine] = dict(run=chosen[engine]['run'], evaluation=metrics,
                                          primary_csv_sha256=source['primary_csv_sha256'])
        diagnostics.append(item)
    return diagnostics


def markdown(report):
    engines = report.get('engines', ENGINES)
    lines = ['# Saved phone comparison', '',
        'Common GOOD means native quality GOOD, initialized nonempty map, at least ten raw matches and a finite rigid pose. '
        'It does not establish equal geometric quality: the current/mobile engines have an internal final-pose support gate, while upstream retains its original decisions.', '',
        'Strict GOOD additionally requires at least ten measured final-pose robust contributors. '
        'Unavailable upstream diagnostics are shown as unknown, not zero. References are evaluation-only and monocular scale is fitted.', '',
        '| Engine | Recording / trial | Status | Raw GOOD | Common GOOD / input | Strict GOOD | Common Sim3 RMS | Motion coverage |',
        '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |']
    if report.get('automatic_system_comparison'):
        lines[2:2] = ['Complete-system comparison uses each trial\'s recorded initialization policy. '
                      'Automatic or unknown mobile policies do not qualify for fixed-pair tracking isolation; '
                      'coverage and accuracy reflect initialization and subsequent tracking/mapping together.', '']
    for engine in engines:
        for dataset, group in report['groups'][engine].items():
            if not group:
                lines.append(f'| {engine} | {dataset} | missing | — | — | unknown | — | — |')
            for r in group:
                e = r.get('common_evaluation', {})
                rms = f"{e['position_rms_m']:.6g} m" if e.get('position_rms_m') is not None else 'no fit'
                motion = f"{100*e['reference_motion_coverage']:.2f}%" if e.get('reference_motion_coverage') is not None else 'unknown'
                lines.append(f"| {engine} | {dataset} / {Path(r['run']).name} | {r['state']} | {r.get('raw_quality_good_frames','—')} | "
                    f"{r.get('common_good_frames','—')}/{r.get('input_frames','—')} | {r['strict_good_frames'] if r['strict_good_frames'] is not None else 'unknown'} | {rms} | {motion} |")
    lines += ['', '| Engine / recording / trial | Mean processing | Replay wall | Max lag | Max thermal | Fixed-pair eligible | System eligible |',
              '| --- | ---: | ---: | ---: | --- | --- | --- |']
    for r in report['trials']:
        mean, lag = r.get('processing_ms', {}).get('mean'), r.get('schedule_lag_ms', {}).get('maximum')
        lines.append(f"| {r['engine']} / {r['dataset']} / {Path(r['run']).name} | " +
            (f'{mean:.3f} ms' if mean is not None else 'unknown') + ' | ' +
            (f"{r['replay_wall_seconds']:.3f} s" if r.get('replay_wall_seconds') is not None else 'unknown') + ' | ' +
            (f'{lag:.3f} ms' if lag is not None else 'unknown') +
            f" | {r.get('thermal',{}).get('max_thermal_state','unknown')} | {r.get('eligible_for_paired_source_paced_comparison',False)} | "
            f"{r.get('eligible_for_source_paced_system_comparison',False)} |")
    lines += ['', 'Upstream processing measures its tracker call while its mapper runs concurrently. '
              'The current port includes its serial mapping stage. These means are not equal measures of total estimator cost; '
              'source-paced completion, lag and thermal state must be considered together.', '']
    lines += ['## Shared accepted-time diagnostic', '',
        'For predeclared rep01 only, the engines fit their own Sim3 on exactly the same intersection of common accepted exposure times. '
        'The intersection uses logged quality/support only, never reference error. This does not assess unobserved tails or replace the full-coverage primary results.', '',
        '| Recording | Shared / full input | Shared reference motion | ' + ' | '.join(e + ' shared-time RMS' for e in engines) + ' |',
        '| --- | ---: | ---: | ' + ' | '.join('---:' for e in engines) + ' |']
    for item in report.get('matched_timestamp_diagnostics', []):
        if item['status'] != 'computed':
            lines.append(f"| {item['dataset']} | unavailable | — | " + ' | '.join('—' for e in engines) + ' |')
            continue
        motion = item['shared_reference_motion_coverage']
        scores = []
        for engine in engines:
            score = item['engines'][engine]['evaluation'].get('position_rms_m')
            scores.append(f'{score:.6g} m' if score is not None else 'no fit')
        lines.append(f"| {item['dataset']} | {item['intersection_frames']}/{item['input_frames']} | " +
            (f'{100*motion:.2f}%' if motion is not None else 'unknown') + ' | ' + ' | '.join(scores) + ' |')
    lines.append('')
    for r in report['trials']:
        notes = r.get('input_errors', []) + r.get('warnings', [])
        if r.get('error'):
            notes.append(r['error'])
        if notes:
            lines += [f"- {r['engine']} / {r['dataset']} / {Path(r['run']).name}: " + '; '.join(notes)]
    return '\n'.join(lines) + '\n'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True, help='Fresh report directory; existing paths are never overwritten')
    p.add_argument('--input-plan', type=Path, default=DEFAULT_PLAN)
    p.add_argument('--completed-only', action='store_true', help='Interim view: evaluate only completed, exported trials; list pending paths separately')
    args = p.parse_args()
    root = args.root.resolve()
    if not root.is_dir():
        p.error('--root must be an existing saved-run directory')
    check_output_storage(args.output)
    if args.output.exists():
        raise FileExistsError(args.output)
    plan = read_json(args.input_plan)
    cases = plan['cases']
    runs = discover(root, cases)
    args.output.mkdir(parents=True)
    shutil.copyfile(args.input_plan, args.output / 'input-plan.json')
    report = dict(root=str(root), helper_sha256=digest(Path(__file__)), input_plan_sha256=digest(args.input_plan),
                  evaluator_sha256=digest(Path(__file__).with_name('evaluate_trajectory.py')),
                  policy='Read-only saved runs; common patch-supported native GOOD for both; strict robust pose support only when measured; no reference initialization; no missing-frame interpolation',
                  trials=[], groups={e: {c['name']: [] for c in cases} for e in ENGINES}, missing=[], pending=[],
                  completed_only=args.completed_only)
    for run in runs:
        if args.completed_only:
            saved_summary = read_json(run / 'run/summary.json', optional=True)
            exported = read_json(run / 'validation.json', optional=True)
            if saved_summary.get('status') != 'completed' or exported.get('pipeline_validation') != 'passed':
                report['pending'].append(dict(run=str(run), summary_status=saved_summary.get('status'),
                                              reason='Not a completed exported trial in this interim view'))
                continue
        try:
            result = analyze_run(run, root, cases, args.output)
        except Exception as error:
            parts = run.relative_to(root).parts
            result = dict(run=str(run), engine=parts[0] if parts and parts[0] in ENGINES else 'unknown',
                          dataset=parts[1] if len(parts) > 1 else 'unknown', state='unreadable',
                          error=f'{type(error).__name__}: {error}', strict_good_frames=None)
        report['trials'].append(result)
        if result['engine'] in report['groups'] and result['dataset'] in report['groups'][result['engine']]:
            report['groups'][result['engine']][result['dataset']].append(result)
    for engine in ENGINES:
        for name, trials in report['groups'][engine].items():
            if not trials:
                report['missing'].append(dict(engine=engine, dataset=name, state='missing'))
    state_counts = {}
    for trial in report['trials']:
        state_counts[trial['state']] = state_counts.get(trial['state'], 0) + 1
    report['totals'] = dict(recorded_trials=len(report['trials']), state_counts=state_counts,
        planned_input_rows_in_recorded_trials=sum(r.get('input_frames', 0) for r in report['trials']),
        saved_output_rows=sum(r.get('output_rows', 0) for r in report['trials']),
        nonfinite_pose_rows=sum(r.get('common_evaluation', {}).get('nonfinite_rows_all_states', 0) for r in report['trials']),
        trials_with_frame_metrics=sum('output_rows' in r for r in report['trials']),
        missing_engine_dataset_pairs=len(report['missing']))
    report['matched_timestamp_diagnostics'] = matched_timestamp_diagnostics(report['groups'], cases, args.output)
    write_json(args.output / 'report.json', report)
    (args.output / 'report.md').write_text(markdown(report))
    print(json.dumps(dict(report=str(args.output / 'report.md'), saved_trials=len(report['trials']),
                          missing_engine_dataset_pairs=len(report['missing'])), indent=2))


if __name__ == '__main__':
    main()
