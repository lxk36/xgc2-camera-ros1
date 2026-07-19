#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
DRIVER="${PREFIX}/lib/xgc_camera_driver/xgc_camera_driver_node"
PLUGIN="/usr/share/xgc2/process-definitions/xgc2-camera-ros1.json"

source "${PREFIX}/setup.bash"
dpkg -s ros-noetic-xgc-camera-driver >/dev/null
dpkg -s ros-noetic-xgc2-camera-calibration >/dev/null
test "$(rospack find xgc_camera_driver)" = "${PREFIX}/share/xgc_camera_driver"
test "$(rospack find xgc_camera_calibration)" = "${PREFIX}/share/xgc_camera_calibration"
test -x "${DRIVER}"
test -x "${PREFIX}/lib/xgc_camera_calibration/run_intrinsic_calibrator.py"
test -x "${PREFIX}/lib/xgc_camera_calibration/extrinsic_calibrator_web.py"
test -x "${PREFIX}/lib/xgc_camera_calibration/extrinsic_tf_publisher.py"
test -f "${PREFIX}/share/xgc_camera_calibration/web/index.html"
test -f "${PREFIX}/share/xgc_camera_calibration/web/app.js"
test -f "${PREFIX}/share/xgc_camera_calibration/web/styles.css"
test -f "${PLUGIN}"
python3 -m json.tool "${PLUGIN}" >/dev/null
python3 -c 'from xgc_camera_calibration.solver import solve_extrinsic; from xgc_camera_calibration.transforms import split_parent_to_optical_pose; from xgc_camera_calibration.web_service import CalibrationService'
roslaunch --files xgc_camera_driver camera.launch >/dev/null
roslaunch --files xgc_camera_driver usb_cam_compat.launch >/dev/null
roslaunch --files xgc_camera_calibration intrinsic_calibrator.launch >/dev/null
roslaunch --files xgc_camera_calibration extrinsic_calibrator.launch >/dev/null
roslaunch --files xgc_camera_calibration extrinsic_tf.launch >/dev/null
if ldd "${DRIVER}" | grep -q 'not found'; then
  ldd "${DRIVER}" >&2
  exit 1
fi

RUNTIME="$(mktemp -d)"
ROSCORE_PID=""
DRIVER_PID=""
WEB_PID=""
cleanup() {
  if [[ -n "${WEB_PID}" ]]; then kill "${WEB_PID}" 2>/dev/null || true; fi
  if [[ -n "${DRIVER_PID}" ]]; then kill "${DRIVER_PID}" 2>/dev/null || true; fi
  if [[ -n "${ROSCORE_PID}" ]]; then kill "${ROSCORE_PID}" 2>/dev/null || true; fi
  wait "${WEB_PID}" 2>/dev/null || true
  wait "${DRIVER_PID}" 2>/dev/null || true
  wait "${ROSCORE_PID}" 2>/dev/null || true
  rm -rf "${RUNTIME}"
}
trap cleanup EXIT
export ROS_MASTER_URI="http://127.0.0.1:11359"
export ROS_HOME="${RUNTIME}/ros-home"
export ROS_LOG_DIR="${RUNTIME}/ros-log"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
roscore -p 11359 >"${RUNTIME}/roscore.log" 2>&1 &
ROSCORE_PID="$!"
for _ in $(seq 1 50); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.1
done
rosparam list >/dev/null
"${DRIVER}" __name:=driver __ns:=/usb_cam \
  _backend:=synthetic _video_device:=synthetic:0 \
  _width:=320 _height:=240 _framerate:=10 _pixel_format:=bgr24 \
  _output_encoding:=bgr8 _frame_id:=usb_cam_optical_frame \
  >"${RUNTIME}/driver.log" 2>&1 &
DRIVER_PID="$!"
timeout 15 rostopic echo -n 1 /usb_cam/image_raw/header/stamp >"${RUNTIME}/stamp.yaml"
timeout 15 rostopic echo -n 1 /usb_cam/camera_info/header/stamp >/dev/null
test "$(rostopic type /usb_cam/image_raw)" = "sensor_msgs/Image"
test "$(rostopic type /usb_cam/camera_info)" = "sensor_msgs/CameraInfo"
grep -q '^secs:' "${RUNTIME}/stamp.yaml"
"${PREFIX}/lib/xgc_camera_calibration/extrinsic_calibrator_web.py" \
  __name:=xgc_camera_extrinsic_calibrator_web \
  _image_topic:=/usb_cam/image_raw _camera_info_topic:=/usb_cam/camera_info \
  _http_port:=18765 _output_file:="${RUNTIME}/extrinsics.yaml" \
  >"${RUNTIME}/web.log" 2>&1 &
WEB_PID="$!"
WEB_READY=false
for _ in $(seq 1 150); do
  if python3 -c 'import json, sys, urllib.request; payload=json.load(urllib.request.urlopen("http://127.0.0.1:18765/healthz", timeout=1)); sys.exit(0 if payload["status"] == "ok" and payload["image_ready"] and payload["camera_info_ready"] else 1)' >/dev/null 2>&1; then
    WEB_READY=true
    break
  fi
  if ! kill -0 "${WEB_PID}" 2>/dev/null; then
    break
  fi
  sleep 0.1
done
if [[ "${WEB_READY}" != true ]]; then
  echo "Installed calibration WebUI did not become ready" >&2
  for log_file in web driver roscore; do
    echo "--- ${log_file}.log ---" >&2
    sed -n '1,240p' "${RUNTIME}/${log_file}.log" >&2 || true
  done
  exit 1
fi
python3 -c 'import json, urllib.request; payload=json.load(urllib.request.urlopen("http://127.0.0.1:18765/healthz", timeout=2)); assert payload["status"] == "ok" and payload["image_ready"] and payload["camera_info_ready"]'
python3 -c 'import urllib.request; payload=urllib.request.urlopen("http://127.0.0.1:18765/", timeout=2).read(); assert b"Camera extrinsic calibration" in payload'

echo "Installed ROS1 camera packages, synthetic topics, and calibration WebUI passed"
