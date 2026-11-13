"""Exercise the live numerical-failure watchdog with an actual child process."""
from argparse import Namespace
import json
from pathlib import Path
import sys
import subprocess
import tempfile
import time
import unittest

from run_host_validation import run_trial, sha, trial_failed


class HostValidationTest(unittest.TestCase):
    def test_native_model_requires_actual_runtime_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);binary=root/"old_replay.py"
            binary.write_text(f"#!{sys.executable}\nimport sys\nfrom pathlib import Path\n"+
                              "Path(sys.argv[sys.argv.index('--output')+1]).write_text('timestamp\\n')\n"+
                              "print('map_good=1')\n")
            binary.chmod(0o755);dataset=root/"data";dataset.mkdir()
            (dataset/"manifest.json").write_text(json.dumps(dict(fx=253,fy=253,cx=319.5,cy=239.5,lens_model="equidistant")))
            (dataset/"frames.csv").write_text("timestamp,path\n")
            result=run_trial(Namespace(binary=binary,timeout=10),dataset,0,1,10,root/"trial",sha(binary))
            self.assertIsNotNone(result["camera_protocol_failure"])
            self.assertEqual(result["requested_lens_model"],"equidistant")

    def test_complete_uninitialized_run_is_a_tracking_outcome(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); binary=root/"uninitialized.py"
            binary.write_text(f"#!{sys.executable}\nimport csv,sys\n"+
                "with open(sys.argv[sys.argv.index('--output')+1],'w') as out:\n"+
                " w=csv.writer(out); w.writerow('timestamp,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22,quality,found,attempted,points,keyframes,elapsed_ms'.split(','))\n"+
                " for i in range(10): w.writerow([i*.1,0,0,0,1,0,0,0,1,0,0,0,1,-1,0,0,0,0,0])\n"+
                "print('map_good=0 lens_model=pinhole'); raise SystemExit(2)\n")
            binary.chmod(0o755);dataset=root/"data";dataset.mkdir()
            (dataset/"manifest.json").write_text(json.dumps(dict(fx=400,fy=400,cx=319.5,cy=239.5)))
            (dataset/"frames.csv").write_text("timestamp,path\n")
            result=run_trial(Namespace(binary=binary,timeout=10),dataset,0,1,10,root/"trial",sha(binary))
            self.assertTrue(result["initialization_failed"])
            self.assertFalse(trial_failed(result))
            self.assertEqual(result["rows"],10)
            self.assertEqual(result["visual_valid_frames"],0)

    def test_failed_child_does_not_launch_further_trials(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); binary=root/"failing_replay.py"; calls=root/"calls"
            binary.write_text(f"#!{sys.executable}\nfrom pathlib import Path\n"+
                              f"with Path({str(calls)!r}).open('a') as s: s.write('called\\n')\n"+
                              "raise SystemExit(7)\n")
            binary.chmod(0o755); dataset=root/"data/lounge_new"; dataset.mkdir(parents=True)
            (dataset/"manifest.json").write_text(json.dumps(dict(
                fx=400,fy=400,cx=319.5,cy=239.5,init_first=0,init_second=1,
                frames=[dict(timestamp=i*.1,path=f"{i}.pgm") for i in range(10)])))
            (dataset/"frames.csv").write_text("timestamp,path\n")
            process=subprocess.run([sys.executable,str(Path(__file__).with_name("run_host_validation.py")),
                "--binary",str(binary),"--datasets-root",str(root/"data"),"--datasets","lounge_new",
                "--skip-sweep","--repeats","3","--output",str(root/"suite")],capture_output=True,text=True)
            self.assertNotEqual(process.returncode,0)
            self.assertEqual(calls.read_text().splitlines(),["called"])
            failure=json.loads((root/"suite/failure.json").read_text())
            self.assertEqual(failure["exit_code"],7)
            self.assertFalse((root/"suite/lounge_new/repeat_01").exists())

    def test_nan_loop_is_stopped_and_evidence_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root/"fake_replay.py"
            binary.write_text(f"#!{sys.executable}\n" +
                "import sys,time\nfrom pathlib import Path\n" +
                "p=Path(sys.argv[sys.argv.index('--output')+1])\n" +
                "p.write_text('timestamp,tx,ty,tz,r00,r01,r02,r10,r11,r12,r20,r21,r22,quality,found,attempted,points,keyframes,elapsed_ms\\n')\n" +
                "sys.stderr.write('NANNERY!\\n');sys.stderr.flush()\ntime.sleep(30)\n")
            binary.chmod(0o755)
            dataset = root/"data"; dataset.mkdir()
            (dataset/"manifest.json").write_text(json.dumps(dict(fx=400,fy=400,cx=319.5,cy=239.5)))
            (dataset/"frames.csv").write_text("timestamp,path\n")
            args = Namespace(binary=binary,timeout=10)
            start=time.monotonic()
            result=run_trial(args,dataset,0,1,10,root/"trial",sha(binary))
            self.assertLess(time.monotonic()-start,5)
            self.assertIsNotNone(result["numerical_abort"])
            self.assertFalse(result["timed_out"])
            self.assertGreater(result["internal_nan_messages"],0)
            self.assertNotEqual(result["exit_code"],0)
            self.assertTrue((root/"trial/stderr.log").exists())


if __name__ == "__main__":
    unittest.main()
