#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

for script in .xgc2/scripts/*.sh; do
  bash -n "${script}"
done

PYTHONPYCACHEPREFIX="${TMPDIR:-/tmp}/xgc2-camera-pycache" python3 -m py_compile \
  xgc_camera_calibration/scripts/*.py \
  xgc_camera_calibration/src/xgc_camera_calibration/*.py \
  xgc_camera_calibration/test/*.py \
  xgc_camera_driver/test/*.py \
  .xgc2/scripts/xgc2_artifact_manifest.py

python3 - <<'PY'
import json
import pathlib
import xml.etree.ElementTree as ET

root = pathlib.Path(".")
for path in sorted(root.glob("xgc_camera_*/package.xml")):
    ET.parse(path)
for path in sorted(root.glob("xgc_camera_*/*/*.launch")):
    ET.parse(path)

plugin_path = root / "process-definitions/xgc2-camera-ros1.json"
plugin = json.loads(plugin_path.read_text(encoding="utf-8"))
assert plugin["apiVersion"] == "xgc.execution.process/v1"
definitions = plugin["definitions"]
ids = [definition["id"] for definition in definitions]
assert len(ids) == len(set(ids)) == 4
driver = next(item for item in definitions if item["id"] == "xgc2-camera-v4l2-ros1")
for probe_name in ("readiness", "liveness"):
    probe = driver[probe_name]
    assert probe["kind"] == "exec"
    assert probe["command"]["executable"] == "/opt/ros/noetic/bin/rostopic"
    assert probe["command"]["args"] == [
        "echo", "-n", "1", "${imageTopic}/header/stamp"
    ]
    assert probe["timeout"] >= 12_000_000_000
assert driver["command"]["executable"] == (
    "/opt/ros/noetic/lib/xgc_camera_driver/xgc_camera_driver_node"
)
PY

grep -q '^id: xgc2-camera-ros1$' .xgc2/product.yml
grep -q '^version: 0.1.0-1$' .xgc2/product.yml
grep -q 'xgc2::camera' xgc_camera_driver/CMakeLists.txt
grep -q '/usr/share/xgc2/process-definitions' xgc_camera_driver/CMakeLists.txt
if grep -R -E -i '(fs150|scout|agilex)' \
  README.md process-definitions xgc_camera_driver xgc_camera_calibration >/dev/null; then
  echo "vehicle-specific integration leaked into the independent camera product" >&2
  exit 1
fi

echo "ROS1 camera product compliance passed"
