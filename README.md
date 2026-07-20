# XGC2 ROS1 Camera Driver

This product turns one fixed or general-purpose Linux camera into a supervised
ROS Noetic process. It is strictly the ROS interface adapter in the camera
stack:

```text
libxgc2-camera-dev          Linux capture, V4L2, buffers and timestamps
          ↓
xgc_camera_driver          ROS Image, CameraInfo and diagnostics
          ↓
perception consumers       calibration, detection, SLAM and other algorithms
```

`libxgc2-camera-dev` owns the ROS-independent V4L2 and synthetic backends. This
repository maps its frames, timestamps and health into ROS without containing
calibration algorithms or perception applications.

## Install and run

```bash
sudo apt update
sudo apt install ros-noetic-xgc-camera-driver

source /opt/ros/noetic/setup.bash
roslaunch xgc_camera_driver usb_cam_compat.launch \
  video_device:=/dev/video0 width:=1920 height:=1080 \
  framerate:=30 pixel_format:=mjpeg camera_info_url:=file:///path/intrinsics.yaml
```

The compatibility contract is:

```text
/usb_cam/image_raw   sensor_msgs/Image
/usb_cam/camera_info sensor_msgs/CameraInfo
/diagnostics         diagnostic_msgs/DiagnosticArray
frame_id             usb_cam_optical_frame
```

`Image` and `CameraInfo` share the same timestamp, dimensions and frame. Until
valid intrinsics are supplied through `camera_info_url`, `CameraInfo.K` remains
zero and diagnostics report that the camera is uncalibrated; the driver does
not invent calibration values.

Main private parameters are `backend`, `video_device`, `width`, `height`,
`framerate`, `pixel_format`, `capture_mode`, `buffer_count`,
`capture_timeout_ms`, `output_encoding`, `camera_name`, `frame_id` and
`camera_info_url`. Supported capture formats are MJPEG, H264, YUYV, UYVY,
RGB24, BGR24, NV12 and GREY. Output is `bgr8`, `rgb8` or `mono8`.

For deterministic tests without camera hardware:

```bash
roslaunch xgc_camera_driver camera.launch \
  backend:=synthetic pixel_format:=bgr24 width:=320 height:=240 framerate:=10
```

## Automation

`/usr/share/xgc2/process-definitions/xgc2-camera-driver-ros1.json` registers
`xgc2-camera-v4l2-ros1`. Readiness and liveness consume only the
`header/stamp` field of a real image message, so a registered node that is not
publishing frames is not considered ready.

## Calibration

General intrinsic calibration and fixed-world-camera extrinsic calibration are
maintained and released separately in the public
`lxk36/xgc2-camera-calibration-ros1` repository. The driver only consumes an
existing intrinsic YAML through the standard `camera_info_manager` contract.

## Build and release

CI builds and tests the driver against the release-resolved
`libxgc2-camera-dev`, creates `ros-noetic-xgc-camera-driver` for Focal `amd64`
and `arm64`, installs it in a clean container, and verifies the synthetic ROS
topic contract and linked libraries.
