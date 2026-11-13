#!/usr/bin/env python3
"""Copy pristine upstream sources and apply only enumerated build compatibility.

No tracking, mapping, solver, RNG, timing, quality or camera-model changes.
The upstream checkout is never written. Generated unified patch and SHA256
inventory make every source change reviewable.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import subprocess
import shutil


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--upstream', type=Path)
    p.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1])
    a = p.parse_args()
    if a.upstream is None:
        a.upstream = a.output / 'upstream-source'
    changes, inventory = [], []
    dest = a.output / 'generated'
    dest.mkdir(parents=True, exist_ok=True)
    (a.output / 'patches').mkdir(parents=True, exist_ok=True)
    for source in sorted((a.upstream / 'Src').iterdir()):
        if not source.is_file():
            continue
        raw = source.read_bytes()
        generated = raw
        reasons = []
        if source.name == 'Tracker.cc':
            generated = generated.replace(b'#include "OpenGL.h"', b'#include "HeadlessGL.h"')
            generated = generated.replace(b'#include <cvd/gl_helpers.h>', b'// Rendering disabled by HeadlessGL.h; TrackFrame is called with bDraw=false.')
            generated = generated.replace(b'#include <fcntl.h>', b'#include <fcntl.h>\n#include <unistd.h>')
            reasons = ['Remove window/OpenGL linkage; retain all estimator statements and branches.',
                       'Declare POSIX usleep on modern clang.']
        (dest / source.name).write_bytes(generated)
        if generated != raw:
            changes.extend(difflib.unified_diff(raw.decode().splitlines(True), generated.decode().splitlines(True),
                           fromfile='a/Src/' + source.name, tofile='b/generated/' + source.name))
        inventory.append({'file': source.name, 'upstream_sha256': hashlib.sha256(raw).hexdigest(),
                          'generated_sha256': hashlib.sha256(generated).hexdigest(), 'compatibility_changes': reasons})
    (a.output / 'patches/compatibility.patch').write_text(''.join(changes))
    revision = subprocess.check_output(['git', '-C', str(a.upstream), 'rev-parse', 'HEAD'], text=True).strip()
    (a.output / 'source-inventory.json').write_text(json.dumps({'upstream': str(a.upstream), 'commit': revision,
        'policy': 'Compatibility-only; asynchronous original mapper and original defaults retained', 'sources': inventory}, indent=2) + '\n')
    dependency_inventory = []
    dependency_patch = []
    for name in ['TooN', 'libcvd', 'gvars']:
        original = a.output / 'deps' / name
        target = a.output / 'generated-deps' / name
        shutil.copytree(original, target, dirs_exist_ok=True, ignore=shutil.ignore_patterns('.git'))
        edits = {}
        if name == 'TooN':
            edits['TooN.h'] = lambda s: s.replace('#include <TooN/internal/debug.hh>\n', '').replace('#include <TooN/internal/allocator.hh>', '#include <TooN/internal/debug.hh>\n#include <TooN/internal/allocator.hh>')
            edits['internal/size_mismatch.hh'] = lambda s: s.replace('Internal::BadSize size_mismatch;', 'static_assert(Size1 == Size2, "TooN static size mismatch");')
            edits['internal/vector.hh'] = lambda s: s.replace('try_destructive_resize(', 'this->try_destructive_resize(')
            edits['wls.h'] = lambda s: s.replace('template<int Size, class Precision> class Decomposition', 'template<int DecompSize, class DecompPrecision> class Decomposition')
        if name == 'gvars':
            edits['src/GUI.cc'] = lambda s: s.replace('    system(sParams.c_str());',
                '#if defined(__ENVIRONMENT_IPHONE_OS_VERSION_MIN_REQUIRED__)\n'
                '    cerr << "GVars shell command is unavailable on iOS." << endl;\n'
                '#else\n    system(sParams.c_str());\n#endif')
        for relative, transform in edits.items():
            file = target / relative
            before = file.read_text()
            after = transform(before)
            file.write_text(after)
            dependency_patch.extend(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                fromfile='a/deps/' + name + '/' + relative, tofile='b/generated-deps/' + name + '/' + relative))
        if name == 'libcvd':
            (target / 'cvd/config.h').write_text('#pragma once\n#define CVD_ARCH_LITTLE_ENDIAN 1\n#define CVD_HAVE_TOON 1\n#define CVD_HAVE_LAPACK 1\n#define CVD_HAVE_PTHREAD 1\n#define CVD_HAVE_XMMINTRIN 0\n')
        if name == 'gvars':
            (target / 'gvars3/config.h').write_text('#pragma once\n#define GVARS3_HAVE_TOON 1\n')
        dependency_inventory.append({'name': name, 'commit': subprocess.check_output(['git', '-C', str(original), 'rev-parse', 'HEAD'], text=True).strip(),
            'generated_files': {str(f.relative_to(target)): hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(target.rglob('*')) if f.is_file()}})
    (a.output / 'patches/dependency-compatibility.patch').write_text(''.join(dependency_patch))
    (a.output / 'dependency-inventory.json').write_text(json.dumps(dependency_inventory, indent=2) + '\n')


if __name__ == '__main__':
    main()
