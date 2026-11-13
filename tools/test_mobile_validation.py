import json
from pathlib import Path
import tempfile
import unittest

from compare_upstream_phone import analyze_run, json_rows_to_csv
from evaluate_trajectory import evaluate
from run_host_validation import summarize
from run_mobile_validation import apply_auto_denominator
import test_upstream_comparison as fixtures

class MobileValidationTest(unittest.TestCase):
    def test_auto_counts_early_tracking_without_fixed_second_cutoff(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            run,case=fixtures.UpstreamComparisonTest().fixture(root,False,'mobile','mobile')
            rows=[json.loads(line) for line in (run/'run/frames.jsonl').read_text().splitlines()]
            csv=root/'trajectory.csv';json_rows_to_csv(rows,csv)
            result=summarize(csv,second=5,limit=9)
            self.assertEqual(result['visual_valid_frames'],3)
            result['evaluation']=evaluate(root/'source/manifest.json',csv)
            apply_auto_denominator(result,9)
            self.assertEqual(result['visual_valid_frames'],8)
            self.assertEqual(result['eligible_frames'],9)
            self.assertAlmostEqual(result['visual_valid_fraction'],8/9)
            self.assertEqual(result['fixed_pair_summary_unused']['visual_valid_frames'],3)

    def test_completed_summary_with_validation_failure_is_ineligible(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            run,case=fixtures.UpstreamComparisonTest().fixture(root,False,'mobile','mobile')
            (run/'failure.json').write_text(json.dumps({'error':'validation failed'}))
            r=analyze_run(run,root/'suite',[case],root/'analysis')
            self.assertEqual(r['state'],'failed')
            self.assertFalse(r['eligible_for_paired_source_paced_comparison'])
            self.assertFalse(r['eligible_for_source_paced_system_comparison'])

if __name__=='__main__':unittest.main()
