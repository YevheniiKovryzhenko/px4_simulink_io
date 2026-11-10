#!/usr/bin/env python3
"""Test the real scan/map code using real matrix/math and generated uORB schema.

Only uORB registration declarations and platform macros are stubbed; no map,
geometry, binning or expiration implementation is replaced.
"""
import pathlib
import subprocess
import tempfile

root = pathlib.Path(__file__).resolve().parents[5]
build = root / 'build/cubepilot_cubeorangeplus_surgeon'
assert (build / 'uORB/topics/obstacle_distance.h').exists(), 'Build Cube Orange+ first'
with tempfile.TemporaryDirectory(prefix='sf45-tests-') as tmp:
    temp = pathlib.Path(tmp)
    (temp / 'px4_platform_common').mkdir()
    (temp / 'uORB').mkdir()
    (temp / 'px4_platform_common/defines.h').write_text(
        '#pragma once\n#include <cmath>\n#define PX4_ISFINITE(x) std::isfinite(x)\n#define __EXPORT\n')
    (temp / 'uORB/uORB.h').write_text(
        '#pragma once\n#include <cstdint>\n#define __EXPORT\n#define ORB_DECLARE(x)\nstruct orb_metadata;\n')
    base = ['c++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-DM_PI_F=3.14159265358979323846f']
    for path in [temp, root / 'src', root / 'src/lib', root / 'src/lib/matrix',
                 root / 'src/lib/collision_prevention', build]:
        base += ['-I' + str(path)]
    for name in ['scan', 'map']:
        command = base + [str(pathlib.Path(__file__).with_name(name + '_test.cpp'))]
        if name == 'map':
            command += [str(root / 'src/lib/collision_prevention/ObstacleMath.cpp')]
        executable = temp / name
        subprocess.run(command + ['-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True)
        print(name + ' test passed')
