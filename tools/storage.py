"""Check writable output storage without assuming a particular workstation.

Set IPTAM_STORAGE_MOUNT to require a real mounted volume for outputs beneath it.
IPTAM_STORAGE_UUID optionally checks that volume's identity (macOS or Linux).
Paths under common external-mount roots always require a mounted ancestor.
"""
import os
from pathlib import Path
import plistlib
import subprocess
import sys


EXTERNAL_ROOTS = (Path("/Volumes"), Path("/mnt"), Path("/media"))


def _within(path, root):
    return path == root or root in path.parents


def _volume_uuid(mount):
    if sys.platform == "darwin":
        info = plistlib.loads(subprocess.check_output(["diskutil", "info", "-plist", str(mount)]))
        if info.get("MountPoint") != str(mount) or not info.get("WritableVolume", False):
            raise RuntimeError("Configured storage volume is not mounted writable")
        return str(info.get("VolumeUUID", ""))
    if sys.platform.startswith("linux"):
        return subprocess.check_output(
            ["findmnt", "--noheadings", "--output", "UUID", "--mountpoint", str(mount)],
            text=True).strip()
    raise RuntimeError("Storage UUID verification is supported on macOS and Linux")


def check_output_storage(output):
    target = Path(output).expanduser().resolve()
    configured = os.environ.get("IPTAM_STORAGE_MOUNT")
    expected_uuid = os.environ.get("IPTAM_STORAGE_UUID")
    if expected_uuid and not configured:
        raise RuntimeError("IPTAM_STORAGE_UUID requires IPTAM_STORAGE_MOUNT")
    if configured:
        mount = Path(configured).expanduser().resolve()
        if _within(target, mount):
            if not mount.is_mount():
                raise RuntimeError("Configured storage is not actually mounted; refusing to create an assumed mount directory")
            if expected_uuid and _volume_uuid(mount).casefold() != expected_uuid.casefold():
                raise RuntimeError("Configured storage UUID does not match the mounted volume")
    for root in EXTERNAL_ROOTS:
        if _within(target, root):
            ancestors = (target, *target.parents)
            if not any(path != root and _within(path, root) and path.is_mount() for path in ancestors):
                raise RuntimeError("External output path has no mounted volume; refusing to create an assumed mount directory")
    existing = target
    while not existing.exists():
        if existing == existing.parent:
            raise RuntimeError("No existing output storage ancestor")
        existing = existing.parent
    if not existing.is_dir():
        existing = existing.parent
    if not os.access(existing, os.W_OK | os.X_OK):
        raise RuntimeError("Output storage is not writable")
