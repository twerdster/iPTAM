#!/usr/bin/env python3
"""Freeze the external upstream compatibility port, including generated deps."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--verify', action='store_true')
    args = ap.parse_args()
    root = args.port.resolve()
    selected = [p for p in root.iterdir() if p.is_file() and p.suffix in {'.txt', '.json', '.md'}]
    selected += [p for p in root.iterdir() if p.is_file() and 'LICENSE' in p.name]
    for name in ('adapter', 'compat', 'generated', 'generated-deps', 'patches', 'tools'):
        selected += [p for p in (root / name).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    files = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(selected))}
    compiled = {name: digest for name, digest in files.items()
                if Path(name).suffix in {'.cc', '.cpp', '.cxx', '.c', '.h', '.hh', '.hpp'}
                or Path(name).name == 'CMakeLists.txt'}
    receipt = args.output / 'upstream-source.json'
    if args.verify:
        if json.loads(receipt.read_text())['compiled_files'] != compiled:
            raise RuntimeError('Upstream compiled source changed during build')
        return
    args.output.mkdir(parents=True, exist_ok=True)
    receipt.write_text(json.dumps({'port': str(root), 'files': files, 'compiled_files': compiled}, indent=2) + '\n')
    with zipfile.ZipFile(args.output / 'upstream-source.zip', 'w', zipfile.ZIP_DEFLATED) as archive:
        for name in files:
            archive.write(root / name, name)


if __name__ == '__main__':
    main()
