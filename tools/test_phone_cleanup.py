"""Failure evidence must never prevent termination of the owned test process."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from run_phone_replay import cleanup_owned_trial


class PhoneCleanupTests(unittest.TestCase):
    def test_copy_timeout_still_copies_other_evidence_and_terminates_owned_pid(self):
        calls = []

        def ctl(*parts, **kwargs):
            calls.append(parts)
            if parts[:2] == ('copy', 'from') and parts[3].endswith('summary.json'):
                raise subprocess.TimeoutExpired('devicectl', 135)

        with tempfile.TemporaryDirectory() as folder:
            cleanup_owned_trial(ctl, 12345, {'run_id': 'test-run'}, Path(folder), [])
        self.assertEqual(len(calls), 3)
        self.assertTrue(calls[1][3].endswith('frames.jsonl'))
        self.assertEqual(calls[-1], ('process', 'terminate', '--pid', '12345'))

    def test_invalid_run_id_still_terminates_without_copying_outside_run(self):
        calls = []
        with tempfile.TemporaryDirectory() as folder:
            cleanup_owned_trial(lambda *p, **k: calls.append(p), 42,
                                {'run_id': '../other'}, Path(folder), [])
        self.assertEqual(calls, [('process', 'terminate', '--pid', '42')])


if __name__ == '__main__':
    unittest.main()
