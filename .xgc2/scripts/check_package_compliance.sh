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

MANIFEST_TEST_ROOT="$(mktemp -d)"
trap 'rm -rf "${MANIFEST_TEST_ROOT}"' EXIT
MANIFEST_TEST_ARCH="$(dpkg --print-architecture)"
mkdir -p \
  "${MANIFEST_TEST_ROOT}/package/DEBIAN" \
  "${MANIFEST_TEST_ROOT}/debs" \
  "${MANIFEST_TEST_ROOT}/manifests"
printf '%s\n' \
  'Package: xgc2-camera-manifest-contract' \
  'Version: 0.2.0-1' \
  'Section: misc' \
  'Priority: optional' \
  "Architecture: ${MANIFEST_TEST_ARCH}" \
  'Maintainer: XGC2 <dev@xiaokang.ink>' \
  'Description: XGC2 camera build-manifest contract test' \
  >"${MANIFEST_TEST_ROOT}/package/DEBIAN/control"
dpkg-deb --build \
  "${MANIFEST_TEST_ROOT}/package" \
  "${MANIFEST_TEST_ROOT}/debs/xgc2-camera-manifest-contract_0.2.0-1_${MANIFEST_TEST_ARCH}.deb" \
  >/dev/null
python3 .xgc2/scripts/xgc2_artifact_manifest.py build \
  --deb-dir "${MANIFEST_TEST_ROOT}/debs" \
  --output-dir "${MANIFEST_TEST_ROOT}/manifests" \
  --product xgc2-camera-ros1 \
  --product-version 0.2.0-1 \
  --distribution focal \
  --architecture "${MANIFEST_TEST_ARCH}" \
  --source-sha 0000000000000000000000000000000000000000 \
  --ci-run-id compliance \
  --ci-workflow ci \
  --ci-workflow-ref refs/heads/main

MANIFEST_TEST_ROOT="${MANIFEST_TEST_ROOT}" python3 - <<'PY'
import json
import os
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
assert len(ids) == len(set(ids)) == 3
driver = next(item for item in definitions if item["id"] == "xgc2-camera-v4l2-ros1")
calibrator = next(
    item for item in definitions
    if item["id"] == "xgc2-camera-calibrator-ros1"
)
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
assert calibrator["version"] == "3.0.0"
assert calibrator["command"]["executable"] == (
    "/opt/ros/noetic/lib/xgc_camera_calibration/calibrator_web.py"
)
assert calibrator["parameters"]["properties"]["bindAddress"]["default"] == "127.0.0.1"
assert calibrator["parameters"]["properties"]["httpPort"]["default"] == 8765
assert calibrator["parameters"]["properties"]["intrinsicOutputFile"]["default"] == (
    "/var/lib/xgc2/camera/calibrations/usb_cam/intrinsics.yaml"
)
assert "DISPLAY" not in calibrator["command"]["env"]

manifest_paths = list(
    (pathlib.Path(os.environ["MANIFEST_TEST_ROOT"]) / "manifests").glob("*.json")
)
assert len(manifest_paths) == 1
manifest = json.loads(manifest_paths[0].read_text(encoding="utf-8"))
assert set(manifest) == {
    "schema", "product", "source_sha", "version", "distribution",
    "architecture", "ci", "created_at", "debs",
}
assert manifest["schema"] == "xgc2.build-artifact.v1"
assert manifest["product"] == "xgc2-camera-ros1"
assert manifest["version"] == "0.2.0-1"
assert set(manifest["ci"]) == {"run_id", "workflow", "workflow_ref"}
assert len(manifest["debs"]) == 1
deb = manifest["debs"][0]
assert set(deb) == {
    "file", "package", "version", "architecture", "sha256", "size",
}
assert deb["package"] == "xgc2-camera-manifest-contract"
assert deb["version"] == "0.2.0-1"
assert len(deb["sha256"]) == 64
assert deb["size"] > 0
PY

grep -q '^id: xgc2-camera-ros1$' .xgc2/product.yml
grep -q '^version: 0.2.0-1$' .xgc2/product.yml
grep -q '^    focal: 0.2.0-1$' .xgc2/product.yml
if grep -q '^    focal: .*~focal' .xgc2/product.yml; then
  echo "single-distribution ROS1 package version must not retain a focal suffix" >&2
  exit 1
fi
grep -q 'xgc2::camera' xgc_camera_driver/CMakeLists.txt
grep -q '/usr/share/xgc2/process-definitions' xgc_camera_driver/CMakeLists.txt
grep -q '/workspace/repo/process-definitions/' .xgc2/scripts/build_debs_in_docker.sh
grep -q '/workspace/work/src/process-definitions/' .xgc2/scripts/build_debs_in_docker.sh
test -f xgc_camera_calibration/web/index.html
test -f xgc_camera_calibration/web/app.js
test -f xgc_camera_calibration/web/styles.css
if grep -R --exclude-dir=__pycache__ -E '(PyQt|python3-pyqt5|extrinsic_calibrator_ui)' \
  process-definitions xgc_camera_calibration README.md \
  .xgc2/product.yml .xgc2/scripts/package_debs.sh \
  .xgc2/scripts/build_debs_in_docker.sh \
  .xgc2/scripts/check_installed_packages.sh >/dev/null; then
  echo "desktop Qt dependency leaked into the WebUI camera calibrator" >&2
  exit 1
fi
if grep -R -E -i '(fs150|scout|agilex)' \
  README.md process-definitions xgc_camera_driver xgc_camera_calibration >/dev/null; then
  echo "vehicle-specific integration leaked into the independent camera product" >&2
  exit 1
fi

echo "ROS1 camera product compliance passed"
