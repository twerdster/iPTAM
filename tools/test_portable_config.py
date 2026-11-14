"""Portable device selection, app identity and storage safety contracts."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from phone_config import engine_from_bundle, verify_expected_udid
import storage
import run_mobile_phone_suite


class PortableConfigTest(unittest.TestCase):
    def test_bundle_suffixes_accept_custom_and_historical_namespaces(self):
        for namespace in ("org.example.iptam", "legacy.archive", "com.example.research"):
            for suffix, engine in (("iPTAM", "current"), ("PTAMUpstream", "upstream"), ("PTAMMobile", "mobile")):
                self.assertEqual(engine_from_bundle(namespace + "." + suffix), engine)
        self.assertIsNone(engine_from_bundle("com.example.unrelated"))
        self.assertIsNone(engine_from_bundle(None))
        result = subprocess.run([sys.executable, "-c",
            "from phone_config import bundle_identifier; print(bundle_identifier('mobile'))"],
            cwd=Path(__file__).parent, env=dict(os.environ, IPTAM_BUNDLE_PREFIX="com.example.replay"),
            capture_output=True, text=True, check=True)
        self.assertEqual(result.stdout.strip(), "com.example.replay.PTAMMobile")

    def test_optional_udid_check_never_selects_an_implicit_device(self):
        verify_expected_udid({"udid": "fixture-device"}, None)
        verify_expected_udid({"udid": "fixture-device"}, "fixture-device")
        with self.assertRaisesRegex(RuntimeError, "expected-udid"):
            verify_expected_udid({"udid": "other-device"}, "fixture-device")

    def test_phone_commands_require_explicit_device_before_any_device_access(self):
        invocations = {
            "run_phone_replay.py": ["--dataset", "missing", "--output", "missing"],
            "check_phone_lifecycle.py": ["--fixture", "missing", "--output", "missing"],
            "run_mobile_phone_suite.py": ["--app", "missing", "--plan", "missing", "--output", "missing"],
        }
        for script, arguments in invocations.items():
            with self.subTest(script=script):
                result = subprocess.run([sys.executable, str(Path(__file__).with_name(script)), *arguments],
                    capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("--device", result.stderr)
                self.assertNotIn("Traceback", result.stderr)

    def test_private_recording_sources_require_explicit_paths(self):
        for script in ("prepare_actual_lounge.py",):
            with self.subTest(script=script):
                result = subprocess.run([sys.executable, str(Path(__file__).with_name(script)), "--output", "missing"],
                    capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("--source", result.stderr)
                self.assertNotIn("Traceback", result.stderr)

    def test_mobile_suite_propagates_device_bundle_and_identity_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "dataset"
            source.mkdir()
            (source / "manifest.json").write_text("{}")
            plan = root / "input-plan.json"
            plan.write_text(json.dumps({"cases": [{"name": "fixture", "dataset": str(source),
                "manifest_sha256": run_mobile_phone_suite.sha(source / "manifest.json"),
                "init_first": 0, "init_second": 1}]}))
            output = root / "results"
            argv = ["suite", "--app", str(root / "fixture.app"), "--plan", str(plan),
                "--output", str(output), "--device", "chosen-device", "--expected-udid", "chosen-udid",
                "--bundle", "com.example.fixture.PTAMMobile", "--repeats", "1",
                "--expected-plan-sha256", run_mobile_phone_suite.sha(plan)]
            with mock.patch.object(sys, "argv", argv), \
                 mock.patch.object(run_mobile_phone_suite.subprocess, "run", return_value=mock.Mock(returncode=0)) as child:
                run_mobile_phone_suite.main()
            command = child.call_args.args[0]
            for flag, value in (("--device", "chosen-device"), ("--expected-udid", "chosen-udid"),
                                ("--bundle", "com.example.fixture.PTAMMobile")):
                self.assertEqual(command[command.index(flag) + 1], value)
            complete = json.loads((output / "suite-complete.json").read_text())
            self.assertEqual(complete["plan_sha256"], run_mobile_phone_suite.sha(plan))
            self.assertTrue(complete["pipeline_complete"])
            argv[argv.index("--output") + 1] = str(root / "rejected")
            argv[-1] = "0" * 64
            with mock.patch.object(sys, "argv", argv), \
                 mock.patch.object(run_mobile_phone_suite.subprocess, "run") as child:
                with self.assertRaisesRegex(ValueError, "plan hash changed"):
                    run_mobile_phone_suite.main()
                child.assert_not_called()
                self.assertFalse((root / "rejected").exists())

    def test_normal_storage_and_unknown_external_mount(self):
        with mock.patch.dict(os.environ, {}, clear=True), tempfile.TemporaryDirectory() as directory:
            storage.check_output_storage(Path(directory) / "fresh" / "output")
            with self.assertRaisesRegex(RuntimeError, "no mounted volume"):
                storage.check_output_storage("/Volumes/iptam-unmounted-fixture/output")
            with self.assertRaisesRegex(RuntimeError, "no mounted volume"):
                storage.check_output_storage("/mnt/iptam-unmounted-fixture/output")

    def test_configured_storage_requires_real_mount_and_matching_optional_uuid(self):
        with tempfile.TemporaryDirectory() as directory:
            env = {"IPTAM_STORAGE_MOUNT": directory, "IPTAM_STORAGE_UUID": "fixture-uuid"}
            with mock.patch.dict(os.environ, env, clear=True):
                with self.assertRaisesRegex(RuntimeError, "not actually mounted"):
                    storage.check_output_storage(Path(directory) / "output")
                with mock.patch.object(Path, "is_mount", return_value=True), \
                     mock.patch.object(storage, "_volume_uuid", return_value="FIXTURE-UUID"):
                    storage.check_output_storage(Path(directory) / "output")
                with mock.patch.object(Path, "is_mount", return_value=True), \
                     mock.patch.object(storage, "_volume_uuid", return_value="wrong-uuid"):
                    with self.assertRaisesRegex(RuntimeError, "UUID does not match"):
                        storage.check_output_storage(Path(directory) / "output")
        with mock.patch.dict(os.environ, {"IPTAM_STORAGE_UUID": "fixture-uuid"}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "requires IPTAM_STORAGE_MOUNT"):
                storage.check_output_storage("testing/output")

    def test_storage_symlinks_cannot_bypass_mount_guard(self):
        with mock.patch.dict(os.environ, {}, clear=True), tempfile.TemporaryDirectory() as directory:
            alias = Path(directory) / "alias"
            alias.symlink_to("/Volumes/iptam-unmounted-fixture", target_is_directory=True)
            with self.assertRaisesRegex(RuntimeError, "no mounted volume"):
                storage.check_output_storage(alias / "output")


if __name__ == "__main__":
    unittest.main()
