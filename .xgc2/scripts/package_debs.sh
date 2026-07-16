#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
INSTALL_ROOT=""
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

VERSION="${PACKAGE_VERSION:-$(sed -nE 's/^version:[[:space:]]*([^[:space:]#]+).*/\1/p' "${REPO_ROOT}/.xgc2/product.yml" | head -n1)}"
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
BUILD_ROOT="$(mktemp -d)"
trap 'rm -rf "${BUILD_ROOT}"' EXIT
mkdir -p "${OUTPUT_DIR}"

copy_path() {
  local source="$1"
  local package_root="$2"
  if [[ ! -e "${source}" ]]; then
    return
  fi
  local relative="${source#${INSTALL_ROOT}}"
  mkdir -p "${package_root}$(dirname "${relative}")"
  cp -a "${source}" "${package_root}${relative}"
}

copy_ros_package() {
  local ros_package="$1"
  local package_root="$2"
  copy_path "${INSTALL_ROOT}${PREFIX}/share/${ros_package}" "${package_root}"
  copy_path "${INSTALL_ROOT}${PREFIX}/lib/${ros_package}" "${package_root}"
  copy_path "${INSTALL_ROOT}${PREFIX}/lib/python3/dist-packages/${ros_package}" "${package_root}"
}

write_control() {
  local package_root="$1"
  local package_name="$2"
  local dependencies="$3"
  local description="$4"
  mkdir -p "${package_root}/DEBIAN" "${package_root}/usr/share/doc/${package_name}"
  cat >"${package_root}/DEBIAN/control" <<EOF
Package: ${package_name}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <dev@xiaokang.ink>
Depends: ${dependencies}
Description: ${description}
EOF
  install -m 0644 "${REPO_ROOT}/LICENSE" "${package_root}/usr/share/doc/${package_name}/copyright"
  chmod 0755 "${package_root}/DEBIAN"
}

build_driver() {
  local package_name="ros-noetic-xgc-camera-driver"
  local package_root="${BUILD_ROOT}/${package_name}"
  mkdir -p "${package_root}"
  copy_ros_package xgc_camera_driver "${package_root}"
  copy_path "${INSTALL_ROOT}/usr/share/xgc2/process-definitions/xgc2-camera-ros1.json" "${package_root}"
  write_control "${package_root}" "${package_name}" \
    "libavcodec58, libavutil56, libopencv-core4.2, libopencv-imgcodecs4.2, libopencv-imgproc4.2, libswscale5, libxgc2-camera-dev (>= 0.1.0-1~focal), ros-noetic-camera-info-manager, ros-noetic-cv-bridge, ros-noetic-diagnostic-msgs, ros-noetic-diagnostic-updater, ros-noetic-image-transport, ros-noetic-roscpp, ros-noetic-roslaunch, ros-noetic-rostopic, ros-noetic-sensor-msgs" \
    "XGC2 ROS Noetic adapter for the independent Linux camera core"
  test -x "${package_root}${PREFIX}/lib/xgc_camera_driver/xgc_camera_driver_node"
  test -f "${package_root}/usr/share/xgc2/process-definitions/xgc2-camera-ros1.json"
  find "${package_root}" -type d -exec chmod 0755 {} +
  find "${package_root}" -type f -exec chmod 0644 {} +
  chmod 0755 "${package_root}${PREFIX}/lib/xgc_camera_driver/xgc_camera_driver_node"
  strip --strip-unneeded "${package_root}${PREFIX}/lib/xgc_camera_driver/xgc_camera_driver_node" 2>/dev/null || true
  fakeroot dpkg-deb --build "${package_root}" "${OUTPUT_DIR}/${package_name}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_calibration() {
  local package_name="ros-noetic-xgc-camera-calibration"
  local package_root="${BUILD_ROOT}/${package_name}"
  mkdir -p "${package_root}"
  copy_ros_package xgc_camera_calibration "${package_root}"
  write_control "${package_root}" "${package_name}" \
    "python3-numpy, python3-opencv, python3-pyqt5, python3-yaml, ros-noetic-camera-calibration, ros-noetic-cv-bridge, ros-noetic-geometry-msgs, ros-noetic-rosbash, ros-noetic-roslaunch, ros-noetic-rospy, ros-noetic-sensor-msgs, ros-noetic-tf2-ros" \
    "XGC2 optional intrinsic and assisted extrinsic camera calibration tools"
  test -f "${package_root}${PREFIX}/lib/python3/dist-packages/xgc_camera_calibration/solver.py"
  find "${package_root}" -type d -name __pycache__ -prune -exec rm -rf {} +
  find "${package_root}" -type d -exec chmod 0755 {} +
  find "${package_root}" -type f -exec chmod 0644 {} +
  if [[ -d "${package_root}${PREFIX}/lib/xgc_camera_calibration" ]]; then
    find "${package_root}${PREFIX}/lib/xgc_camera_calibration" -type f -exec chmod 0755 {} +
  fi
  fakeroot dpkg-deb --build "${package_root}" "${OUTPUT_DIR}/${package_name}_${VERSION}_${ARCH}.deb" >/dev/null
}

build_driver
build_calibration
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
