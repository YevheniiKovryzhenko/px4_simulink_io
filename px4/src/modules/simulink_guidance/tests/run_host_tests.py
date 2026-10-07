#!/usr/bin/env python3
"""Run guidance tests against a built lockstep SITL configuration and system GoogleTest."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=root / "build/px4_sitl_simulink")
    parser.add_argument("--asset-dir", type=Path)
    args = parser.parse_args()
    build = args.build_dir.resolve()
    entries = json.loads((build / "compile_commands.json").read_text())
    entry = next(e for e in entries if e["file"].endswith("/simulink_guidance/trajectory.cpp"))
    command = shlex.split(entry["command"])
    flags = [a for a in command if a.startswith(("-I", "-D"))
             and not a.startswith(("-DNDEBUG", "-DPX4_MAIN", "-DMODULE_NAME"))]
    if "-DENABLE_LOCKSTEP_SCHEDULER" not in flags:
        parser.error("Use a lockstep SITL build; tests advance simulated time explicitly.")
    flags += ["-I" + str(root), '-DMODULE_NAME="guidance_test"', "-std=gnu++14",
              "-include", "visibility.h", "-O2", "-pthread"]
    names = ["modules__simulink_guidance", "px4_layer", "px4_platform", "uORB", "systemlib",
             "cdev", "px4_work_queue", "px4_daemon", "work_queue", "parameters", "events",
             "perf", "tinybson", "uorb_msgs", "lockstep_scheduler", "heatshrink"]
    libraries = []
    for name in names:
        matches = list(build.rglob(f"lib{name}.a"))
        if len(matches) != 1:
            parser.error(f"Build SITL first: expected one lib{name}.a, found {len(matches)}")
        libraries.append(str(matches[0]))
    test_dir = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory(prefix="px4-guidance-tests-") as temporary:
        objects = []
        for source in (test_dir / "TrajectoryTest.cpp", test_dir / "TrajectoryMathTest.cpp",
                       test_dir / "host_main.cpp"):
            obj = str(Path(temporary) / (source.stem + ".o"))
            subprocess.run(["c++", *flags, "-c", str(source), "-o", obj], cwd=root, check=True)
            objects.append(obj)
        executable = str(Path(temporary) / "guidance-tests")
        subprocess.run(["c++", "-pthread", *objects, "-Wl,--start-group", *libraries,
                        "-Wl,--end-group", "-lgtest", "-ldl", "-lrt", "-o", executable],
                       cwd=root, check=True)
        env = os.environ.copy()
        assets = args.asset_dir or build / "rootfs/Trajectories"
        if assets.is_dir():
            env["GUIDANCE_TEST_ASSET_DIR"] = str(assets.resolve())
        subprocess.run([executable], env=env, cwd=temporary, check=True)


if __name__ == "__main__":
    main()
