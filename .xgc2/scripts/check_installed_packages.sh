#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
DRIVER="${PREFIX}/lib/xgc_camera_driver/xgc_camera_driver_node"
PLUGIN="/usr/share/xgc2/process-definitions/xgc2-camera-driver-ros1.json"

source "${PREFIX}/setup.bash"
dpkg -s ros-noetic-xgc-camera-driver >/dev/null
test "$(rospack find xgc_camera_driver)" = "${PREFIX}/share/xgc_camera_driver"
test -x "${DRIVER}"
test -f "${PLUGIN}"
python3 -m json.tool "${PLUGIN}" >/dev/null
roslaunch --files xgc_camera_driver camera.launch >/dev/null
roslaunch --files xgc_camera_driver usb_cam_compat.launch >/dev/null
if ldd "${DRIVER}" | grep -q 'not found'; then
  ldd "${DRIVER}" >&2
  exit 1
fi

RUNTIME="$(mktemp -d)"
ROSCORE_PID=""
DRIVER_PID=""
cleanup() {
  if [[ -n "${DRIVER_PID}" ]]; then kill "${DRIVER_PID}" 2>/dev/null || true; fi
  if [[ -n "${ROSCORE_PID}" ]]; then kill "${ROSCORE_PID}" 2>/dev/null || true; fi
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

echo "Installed ROS1 camera driver and synthetic topic contract passed"
