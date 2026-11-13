"""Portable names for independent iOS estimator apps and saved-run analysis."""
import os


BUNDLE_PREFIX = os.environ.get("IPTAM_BUNDLE_PREFIX", "org.example.iptam").rstrip(".")
BUNDLE_SUFFIXES = {"current": "iPTAM", "upstream": "PTAMUpstream", "mobile": "PTAMMobile"}


def bundle_identifier(engine):
    return BUNDLE_PREFIX + "." + BUNDLE_SUFFIXES[engine]


def engine_from_bundle(identifier):
    """Recognize target suffixes under any signing namespace, including old logs."""
    suffix = str(identifier or "").rsplit(".", 1)[-1]
    return next((engine for engine, value in BUNDLE_SUFFIXES.items() if value == suffix), None)


def verify_expected_udid(hardware, expected):
    if expected is not None and hardware.get("udid") != expected:
        raise RuntimeError("The selected device does not match --expected-udid")
