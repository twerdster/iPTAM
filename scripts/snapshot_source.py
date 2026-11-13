#!/usr/bin/env python3
"""Record exact tracked and new source files for a reproducible dirty-tree build."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile


def inventory(root):
    names = subprocess.check_output(["git", "ls-files", "-c", "-o", "--exclude-standard", "-z"], cwd=root).split(b"\0")
    paths = sorted({name.decode() for name in names if name})
    return {name: hashlib.sha256((root / name).read_bytes()).hexdigest()
            for name in paths if (root / name).is_file()}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--verify", action="store_true")
    args = ap.parse_args()
    root = Path(__file__).resolve().parents[1]
    files = inventory(root)
    compiled = {name: digest for name, digest in files.items()
                if Path(name).suffix in {".cc", ".cpp", ".cxx", ".c", ".h", ".hh", ".m", ".mm", ".plist", ".pbxproj"}
                or Path(name).name == "CMakeLists.txt"}
    fingerprint = args.output / "source.json"
    if args.verify:
        if json.loads(fingerprint.read_text())["compiled_files"] != compiled:
            raise RuntimeError("Source changed during the build; preserve evidence and rebuild a frozen tree")
        return
    args.output.mkdir(parents=True, exist_ok=True)
    data = {"base_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
            "branch": subprocess.check_output(["git", "branch", "--show-current"], cwd=root, text=True).strip(),
            "files": files, "compiled_files": compiled}
    fingerprint.write_text(json.dumps(data, indent=2) + "\n")
    with zipfile.ZipFile(args.output / "source.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for name in files:
            archive.write(root / name, name)


if __name__ == "__main__":
    main()
