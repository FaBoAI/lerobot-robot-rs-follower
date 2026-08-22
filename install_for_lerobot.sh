#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-python}"

"${PYTHON_BIN}" -m pip uninstall -y lerobot_robot_rs_follower || true
"${PYTHON_BIN}" -m pip install --no-cache-dir .

"${PYTHON_BIN}" - <<'PY'
from importlib.metadata import version
import lerobot_robot_rs_follower
from lerobot_robot_rs_follower import RSFollowerConfig

installed = version("lerobot_robot_rs_follower")
print(f"plugin import OK: rs_follower (version={installed})")
print(RSFollowerConfig)
if installed != "0.0.17":
    raise SystemExit(f"Unexpected installed version: {installed}")
PY

if command -v lerobot-teleoperate >/dev/null 2>&1; then
    if lerobot-teleoperate --help 2>&1 | grep -q "rs_follower"; then
        echo "LeRobot plugin discovery OK: rs_follower"
    else
        echo "ERROR: lerobot-teleoperate did not discover rs_follower" >&2
        exit 1
    fi
else
    echo "WARNING: lerobot-teleoperate is not on PATH; plugin discovery check skipped" >&2
fi
