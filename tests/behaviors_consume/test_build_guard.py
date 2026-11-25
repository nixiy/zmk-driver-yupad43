"""Compile the real processor with a configured firmware's flags to check its stack guard.

Requires a Zephyr build with compile_commands.json and this processor instantiated.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
from tempfile import TemporaryDirectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--expect", required=True, choices=("allow", "reject"))
    args = parser.parse_args()
    build = args.build_dir.resolve()
    entry = next(e for e in json.loads((build / "compile_commands.json").read_text())
                 if e["file"].endswith("input_processor_behaviors_consume.c"))
    with TemporaryDirectory(prefix="iqs9151-stack-guard-") as directory:
        output = Path(directory) / "processor.obj"
        def quote(path):
            if os.name == "nt":
                return subprocess.list2cmdline([str(path)])
            return shlex.quote(str(path))

        command = re.sub(r'(?<= -o )("[^"]+"|\S+)',
                         lambda _: quote(output), entry["command"])
        # CMake normally omits dependency output flags here. Redirect them too
        # if a generator supplies them, keeping the original build untouched.
        command = re.sub(r'(?<= -MF )("[^"]+"|\S+)',
                         lambda _: quote(output.with_suffix(".d")), command)
        argv = command if os.name == "nt" else shlex.split(command)
        result = subprocess.run(argv, cwd=entry["directory"], capture_output=True, text=True)
        if args.expect == "reject":
            assert result.returncode != 0, "Unsafe configuration must fail compilation"
            message = "behaviors-consume requires CONFIG_INPUT_THREAD_STACK_SIZE >= 2048"
            assert message in result.stderr, result.stderr
        else:
            assert result.returncode == 0, result.stdout + result.stderr
            assert output.is_file()
    print(f"PASS: actual processor compilation {args.expect} ({build.name})")


if __name__ == "__main__":
    main()
