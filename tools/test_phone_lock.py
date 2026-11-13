"""Verify that a second phone controller cannot clobber an active trial."""
from pathlib import Path
import subprocess
import sys
import unittest
import uuid


class PhoneLockTest(unittest.TestCase):
    def test_device_lock_excludes_other_process_and_releases_on_exit(self):
        device = "test-" + uuid.uuid4().hex
        prelude = "from run_phone_replay import acquire_device_lock; acquire_device_lock(" + repr(device) + ");"
        holder = subprocess.Popen(
            [sys.executable, "-c", prelude + "print('locked',flush=True);input()"],
            cwd=Path(__file__).parent, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True)
        try:
            self.assertEqual(holder.stdout.readline().strip(), "locked")
            blocked = subprocess.run([sys.executable, "-c", prelude], cwd=Path(__file__).parent,
                                     capture_output=True, text=True, timeout=5)
            self.assertNotEqual(blocked.returncode, 0)
            self.assertIn("Another iPTAM test controls this device", blocked.stderr)
        finally:
            holder.communicate("release\n", timeout=5)
        released = subprocess.run([sys.executable, "-c", prelude], cwd=Path(__file__).parent,
                                  capture_output=True, text=True, timeout=5)
        self.assertEqual(released.returncode, 0, released.stderr)


if __name__ == "__main__":
    unittest.main()
