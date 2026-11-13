"""Independent pose truth and partial failure regressions for saved-run analysis."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from compare_upstream_phone import analyze_run, digest, matched_timestamp_diagnostics, mobile_initialization_policy
from render_upstream_comparison import initialization_caption
from phone_config import bundle_identifier


class UpstreamComparisonTest(unittest.TestCase):
    def fixture(self, root, partial, actual_engine='upstream', folder_engine='upstream'):
        source = root / 'source'
        source.mkdir(exist_ok=True)
        frames = [dict(timestamp=i/10, path=f'images/{i:06d}.pgm') for i in range(9)]
        manifest = dict(name='fixture', width=640, height=480, fx=400, fy=400,
                        cx=319.5, cy=239.5, lens_model='pinhole', init_first=0, init_second=1, frames=frames)
        (source/'manifest.json').write_text(json.dumps(manifest))
        points = np.array([[0,0,0], [1,0,0], [1,1,0], [2,1,0], [2,2,0],
                           [3,2,0], [3,3,0], [4,3,0], [4,4,0]], float)
        # Independent reference: p_ref = 2.5 * Rz(90deg) * p + [3,-4,2].
        rotation = np.array([[0,-1,0], [1,0,0], [0,0,1.]])
        poses = []
        for i, point in enumerate(points):
            matrix = np.eye(4)
            matrix[:3,:3] = rotation
            matrix[:3,3] = 2.5 * rotation @ point + [3,-4,2]
            poses.append(dict(timestamp=i/10, matrix=matrix.tolist()))
        (source/'reference.json').write_text(json.dumps(dict(role='independent synthetic fixture only', poses=poses)))
        case = dict(name='fixture', dataset=str(source), manifest_sha256=digest(source/'manifest.json'),
                    reference_sha256=digest(source/'reference.json'), pixel_sha256='f'*64, init_first=0, init_second=1)
        run = root/'suite'/folder_engine/'fixture/rep01'
        saved = run/('partial-run' if partial else 'run')
        saved.mkdir(parents=True)
        requested = run/'requested-manifest.json'
        requested.write_text(json.dumps(manifest))
        rows = []
        for i in range(7 if partial else 9):
            row = dict(timestamp=i/10, initialized=i>0, finite=True, quality=2,
                found=20 if i>0 else 0, map_points=30 if i>0 else 0,
                world_from_camera_t_qxyzw=[*points[i],0,0,0,1], processing_ms=5,
                delivery_wall_seconds=i*.1+.002, schedule_lag_ms=2)
            if partial and i==6:
                row.update(finite=False, quality=0, camera_from_world_3x4=[None]*12)
                del row['world_from_camera_t_qxyzw']
            rows.append(row)
        (saved/'frames.jsonl').write_text('\n'.join(json.dumps(row) for row in rows)+'\n')
        summary = dict(status='failed' if partial else 'completed', frames=len(rows), source_paced=True,
            engine_variant=actual_engine, manifest_sha256=digest(requested),
            pose_support_diagnostics_available=False, solver_diagnostics_available=False,
            replay_wall_seconds=.9, max_schedule_lag_ms=2, processing_time_scope='tracker only')
        if not partial:
            summary['rectified_pixels_concatenated_sha256'] = case['pixel_sha256']
        if actual_engine == 'mobile':
            summary['mobile_config'] = {'automatic_initialization': False}
        (saved/'summary.json').write_text(json.dumps(summary))
        (run/'provenance.json').write_text(json.dumps(dict(source=str(source), manifest_sha256=digest(requested),
            app=dict(executable_sha256='a'*64, bundle_identifier=bundle_identifier(actual_engine)))))
        return run, case

    def test_partial_nonfinite_frame_omits_inverse_and_keeps_valid_prefix(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, case = self.fixture(root, partial=True)
            paths = list((root/'source').glob('*.json')) + list((run/'partial-run').glob('*'))
            before = {p:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
            result = analyze_run(run, root/'suite', [case], root/'analysis')
            self.assertEqual(result['state'], 'failed')
            self.assertTrue(result['partial_artifacts'])
            self.assertEqual(result['input_frames'], 9)
            self.assertEqual(result['output_rows'], 7)
            self.assertEqual(result['missing_output_frames'], 2)
            self.assertEqual(result['common_good_frames'], 5)
            self.assertIsNone(result['strict_good_frames'])
            self.assertIsNone(result['solver_numerical_rejections'])
            self.assertEqual(result['input_errors'], [])
            self.assertFalse(result['full_input_pixel_hash_verified'])
            self.assertFalse(result['eligible_for_paired_source_paced_comparison'])
            evaluation = result['common_evaluation']
            self.assertEqual(evaluation['nonfinite_rows_all_states'], 1)
            self.assertEqual(evaluation['matched_reference_frames'], 5)
            self.assertAlmostEqual(evaluation['position_rms_m'], 0, places=12)
            self.assertEqual(result['common_tracking']['largest_gap_frames'], 3)
            self.assertEqual(before, {p:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})

    def test_misfiled_engine_cannot_enter_paired_comparison(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run, case = self.fixture(root, partial=False, actual_engine='current')
            result = analyze_run(run, root/'suite', [case], root/'analysis')
            self.assertEqual(result['state'], 'completed')
            self.assertFalse(result['common_input_verified'])
            self.assertFalse(result['eligible_for_paired_source_paced_comparison'])
            self.assertEqual(len(result['input_errors']), 2)
            self.assertTrue(any('declared engine' in error for error in result['input_errors']))
            self.assertTrue(any('app bundle' in error for error in result['input_errors']))

    def test_shared_timestamp_fit_excludes_quality_unobserved_tail_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            upstream, case = self.fixture(root, partial=False)
            current, _ = self.fixture(root, partial=False, actual_engine='current', folder_engine='current')
            path = upstream/'run/frames.jsonl'
            original = [json.loads(line) for line in path.read_text().splitlines()]
            for row in original[5:]:
                row['quality'] = 0
            path.write_text('\n'.join(json.dumps(row) for row in original)+'\n')
            path = current/'run/frames.jsonl'
            modern = [json.loads(line) for line in path.read_text().splitlines()]
            for row in modern[5:]:
                row['world_from_camera_t_qxyzw'][0] += 50
                row['world_from_camera_t_qxyzw'][1] += 20
            path.write_text('\n'.join(json.dumps(row) for row in modern)+'\n')
            a = analyze_run(upstream, root/'suite', [case], root/'analysis')
            b = analyze_run(current, root/'suite', [case], root/'analysis')
            before = b['common_evaluation']['position_rms_m']
            self.assertGreater(before, .1)
            self.assertEqual((a['common_good_frames'], b['common_good_frames']), (4, 8))
            groups = {'upstream': {'fixture':[a]}, 'current': {'fixture':[b]}}
            result = matched_timestamp_diagnostics(groups, [case], root/'analysis')[0]
            self.assertEqual(result['intersection_frame_indices'], [1,2,3,4])
            self.assertEqual(result['reference_associated_intersection_frames'], 4)
            self.assertAlmostEqual(result['shared_reference_motion_coverage'], 3/8)
            for engine in ['upstream', 'current']:
                self.assertAlmostEqual(result['engines'][engine]['evaluation']['position_rms_m'], 0, places=12)
            self.assertEqual(b['common_evaluation']['position_rms_m'], before)
            self.assertEqual((a['common_good_frames'], b['common_good_frames']), (4, 8))

    def test_three_way_intersection_and_mobile_policy_are_explicit(self):
        self.assertEqual(mobile_initialization_policy({'mobile_config':{'automatic_initialization':0}}, {})[0],'fixed')
        self.assertEqual(mobile_initialization_policy({'mobile_config':{'automatic_initialization':1}}, {})[0],'automatic')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory);groups = {}
            for engine in ('upstream','current','mobile'):
                run,case = self.fixture(root,partial=False,actual_engine=engine,folder_engine=engine)
                if engine == 'mobile':
                    path = run/'run/frames.jsonl'
                    rows = [json.loads(line) for line in path.read_text().splitlines()]
                    rows[-1]['quality'] = 0
                    path.write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
                item = analyze_run(run,root/'suite',[case],root/'analysis')
                self.assertEqual(item['engine'],engine)
                self.assertTrue(item['eligible_for_paired_source_paced_comparison'])
                groups[engine] = {'fixture':[item]}
            shared = matched_timestamp_diagnostics(groups,[case],root/'three_way',tuple(groups))[0]
            self.assertEqual(shared['intersection_frame_indices'],list(range(1,8)))
            self.assertEqual(set(shared['engines']),set(groups))
            summary_path = run/'run/summary.json'
            summary = json.loads(summary_path.read_text())
            summary['mobile_config'] = {'automatic_initialization':True}
            summary_path.write_text(json.dumps(summary))
            auto = analyze_run(run,root/'suite',[case],root/'auto_analysis')
            self.assertFalse(auto['eligible_for_paired_source_paced_comparison'])
            self.assertTrue(auto['eligible_for_source_paced_system_comparison'])
            self.assertTrue(auto['initialization_policy_difference'])
            self.assertTrue(any('separate policy' in e for e in auto['warnings']))
            groups['mobile']['fixture'] = [auto]
            fixed = matched_timestamp_diagnostics(groups,[case],root/'auto_fixed',tuple(groups))[0]
            self.assertEqual(fixed['status'],'unavailable')
            system = matched_timestamp_diagnostics(groups,[case],root/'auto_system',tuple(groups),
                'eligible_for_source_paced_system_comparison')[0]
            self.assertEqual(system['intersection_frame_indices'],list(range(1,8)))
            self.assertIn('different initialization policies',system['policy'])

    def test_changed_artifact_clears_both_comparison_modes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run,case = self.fixture(root,False,'mobile','mobile')
            summary_path = run/'run/summary.json'
            summary_calls = 0
            def changing_digest(path):
                nonlocal summary_calls
                if Path(path) == summary_path:
                    summary_calls += 1
                    if summary_calls > 1:
                        return '0'*64
                return digest(path)
            with mock.patch('compare_upstream_phone.digest',side_effect=changing_digest):
                result = analyze_run(run,root/'suite',[case],root/'analysis')
            self.assertGreaterEqual(summary_calls,2)
            self.assertEqual(result['state'],'completed')
            self.assertFalse(result['input_errors'])
            self.assertFalse(result['artifacts_unchanged_after_read'])
            self.assertFalse(result['eligible_for_paired_source_paced_comparison'])
            self.assertFalse(result['eligible_for_source_paced_system_comparison'])

    def test_unknown_and_conflicting_mobile_policies_never_isolate_fixed_pair(self):
        cases = [
            ('missing',None,None,None,False),
            ('malformed',{'automatic_initialization':'false'},None,None,False),
            ('profile_without_config',None,'auto',None,False),
            ('profile_conflict',{'automatic_initialization':False},'auto',None,True),
            ('summary_conflict',{'automatic_initialization':True},None,'predeclared fixed image pair',True),
            ('automatic_summary_conflict',{'automatic_initialization':False},None,
             'one predeclared start; image-only automatic second view',True),
        ]
        for name,config,profile,label,conflict in cases:
            with self.subTest(name=name),tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                run,case = self.fixture(root,False,'mobile','mobile')
                path = run/'run/summary.json';summary=json.loads(path.read_text())
                if config is None:
                    summary.pop('mobile_config',None)
                else:
                    summary['mobile_config']=config
                if label is not None:
                    summary['initialization_policy']=label
                path.write_text(json.dumps(summary))
                if profile is not None:
                    path=run/'provenance.json';provenance=json.loads(path.read_text())
                    provenance['mobile_profile']=profile;path.write_text(json.dumps(provenance))
                result=analyze_run(run,root/'suite',[case],root/'analysis')
                self.assertFalse(result['eligible_for_paired_source_paced_comparison'])
                self.assertFalse(result['initialization_policy_known'])
                self.assertEqual(result['eligible_for_source_paced_system_comparison'],not conflict)
                if conflict:
                    self.assertTrue(any('Conflicting mobile initialization' in e for e in result['input_errors']))
                else:
                    self.assertEqual(result['initialization_policy_state'],'unknown')

    def test_renderer_labels_only_the_selected_actual_policies(self):
        controls=[{'engine':'upstream'},{'engine':'current'}]
        automatic={'engine':'mobile','initialization_policy_difference':True,'initialization_policy_known':True}
        fixed={'engine':'mobile','initialization_policy_difference':False,'initialization_policy_known':True}
        unknown={'engine':'mobile','initialization_policy_difference':False,'initialization_policy_known':False}
        self.assertNotIn('automatic',initialization_caption(controls,True))
        self.assertNotIn('automatic',initialization_caption(controls+[fixed],True))
        self.assertIn('automatic mobile second view',initialization_caption(controls+[automatic],True))
        self.assertIn('unverified',initialization_caption(controls+[unknown],True))
        with self.assertRaises(ValueError):
            initialization_caption(controls+[automatic],False)
        with self.assertRaises(ValueError):
            initialization_caption(controls+[unknown],False)


if __name__=='__main__':
    unittest.main()
