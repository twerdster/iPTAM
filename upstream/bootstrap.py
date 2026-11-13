#!/usr/bin/env python3
"""Create a pinned, auditable upstream-only comparison build on native storage."""
import argparse
import json
from pathlib import Path
import plistlib
import shutil
import subprocess
import sys


def run(*args):
    subprocess.run([str(a) for a in args], check=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--work-root', type=Path, required=True)
    p.add_argument('--build', action='store_true', help='Also build the macOS host driver')
    a = p.parse_args()
    source = Path(__file__).resolve().parent
    target = a.work_root.expanduser().resolve()
    ancestor = target
    while not ancestor.exists():
        ancestor = ancestor.parent
    if sys.platform == 'darwin':
        mount = ancestor
        while not mount.is_mount():
            mount = mount.parent
        volume = plistlib.loads(subprocess.check_output(['diskutil', 'info', '-plist', str(mount)]))
        if 'exfat' in str(volume.get('FilesystemType', '')).lower() or 'exfat' in str(volume.get('FilesystemName', '')).lower():
            raise RuntimeError('Use native filesystem storage for checkout/dependencies/builds; exFAT is for output evidence only')
    target.mkdir(parents=True, exist_ok=True)
    for directory in ['adapter', 'compat', 'tools']:
        shutil.copytree(source / directory, target / directory, dirs_exist_ok=True)
    for name in ['CMakeLists.txt', 'pins.json', 'README.md', 'UPSTREAM-LICENSE']:
        shutil.copy2(source / name, target / name)
    pins = json.loads((source / 'pins.json').read_text())
    for name, pin in pins.items():
        checkout = target / pin['directory']
        fresh = not checkout.exists()
        if fresh:
            checkout.parent.mkdir(parents=True, exist_ok=True)
            run('git', 'clone', '--no-checkout', pin['url'], checkout)
        if not fresh and subprocess.check_output(['git', '-C', str(checkout), 'status', '--porcelain'], text=True).strip():
            raise RuntimeError(f'Refusing to replace local changes in {checkout}')
        actual_url = subprocess.check_output(['git', '-C', str(checkout), 'remote', 'get-url', 'origin'], text=True).strip()
        if actual_url != pin['url']:
            raise RuntimeError(f'Unexpected dependency origin: {checkout}')
        run('git', '-C', checkout, 'checkout', '--detach', pin['commit'])
    shutil.copy2(target / 'upstream-source/LICENSE', target / 'UPSTREAM-LICENSE')
    shutil.copy2(target / 'upstream-source/README.md', target / 'UPSTREAM-README.md')
    run(sys.executable, target / 'tools/generate_sources.py', '--output', target,
        '--upstream', target / 'upstream-source')
    if a.build:
        run('cmake', '-S', target, '-B', target / 'build', '-DCMAKE_BUILD_TYPE=Release')
        run('cmake', '--build', target / 'build', '-j4')
    print(f'Prepared upstream comparison: {target}')


if __name__ == '__main__':
    main()
