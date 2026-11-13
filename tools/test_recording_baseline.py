"""Check a real bounded recording child and complete initialization rejection."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

from prepare_datasets import digest


class RecordingBaselineTest(unittest.TestCase):
    def setup_trial(self, root, body):
        binary = root / 'fake_replay.py'
        binary.write_text(f'#!{sys.executable}\n' + body)
        binary.chmod(0o755)
        data = root / 'data'
        data.mkdir()
        (data / 'manifest.json').write_text(json.dumps(dict(
            fx=400, fy=400, cx=319.5, cy=239.5, lens_model='pinhole',
            frames=[dict(timestamp=i*.1, path=f'{i}.pgm') for i in range(2)])))
        (data / 'frames.csv').write_text('timestamp,path\n')
        selection = root / 'selection.json'
        selection.write_text(json.dumps(dict(manifest_sha256=digest(data / 'manifest.json'),
            reference_files_read=False, selected=dict(first=0, second=1))))
        return [sys.executable, str(Path(__file__).with_name('run_recording_baseline.py')),
                '--binary', str(binary), '--binary-sha256', digest(binary), '--dataset', str(data),
                '--selection', str(selection), '--output', str(root / 'result'), '--timeout', '2']

    def test_nan_child_stopped_in_monitored_group_without_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command = self.setup_trial(root, "import os,sys,time\n"
                "sys.stderr.write(f'pgid={os.getpgrp()} NANNERY!\\n');sys.stderr.flush()\n"
                "time.sleep(30)\n")
            # No reference file: failure must not open it or trigger evaluation.
            start = time.monotonic()
            run = subprocess.run(command, capture_output=True, text=True, timeout=5)
            self.assertLess(time.monotonic()-start, 5)
            self.assertEqual(run.returncode, 1, run.stderr)
            result = json.loads((root / 'result/result.json').read_text())
            self.assertIsNotNone(result['numerical_abort'])
            self.assertFalse(result['pipeline_complete'])
            self.assertIn(f'pgid={os.getpgrp()}', (root / 'result/stderr.log').read_text())
            self.assertFalse((root / 'result/evaluation.json').exists())

    def test_complete_uninitialized_recording_is_retained_and_evaluated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            command = self.setup_trial(root, "import csv,sys\n"
                "with open(sys.argv[sys.argv.index('--output')+1],'w') as out:\n"
                " w=csv.writer(out); w.writerow('timestamp,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22,quality,found,attempted,points,keyframes,elapsed_ms'.split(','))\n"
                " for i in range(2): w.writerow([i*.1,0,0,0,1,0,0,0,1,0,0,0,1,-1,0,0,0,0,0])\n"
                "print('map_good=0 lens_model=pinhole');raise SystemExit(2)\n")
            (root / 'data/reference.json').write_text(json.dumps(dict(role='test reference', poses=[])))
            run = subprocess.run(command, capture_output=True, text=True, timeout=5)
            self.assertEqual(run.returncode, 0, run.stderr)
            result = json.loads((root / 'result/result.json').read_text())
            self.assertTrue(result['initialization_failed'])
            self.assertTrue(result['pipeline_complete'])
            evaluation = json.loads((root / 'result/evaluation.json').read_text())
            self.assertEqual(evaluation['valid_output_frames'], 0)
            self.assertIsNotNone(evaluation['failure'])


if __name__ == '__main__':
    unittest.main()
