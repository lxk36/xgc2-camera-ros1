# XGC2 ROS1 Camera

This product turns a fixed or general-purpose Linux camera into one
supervisable ROS Noetic process. It is deliberately split from capture logic:
`libxgc2-camera-dev` owns the ROS-independent V4L2/synthetic backends and this
repository only maps frames, timestamps, calibration and health into ROS.

## Packages

- `xgc_camera_driver` is the mandatory one-camera/one-process adapter. It
  publishes a synchronized `image_transport::CameraPublisher` pair and ROS
  diagnostics.
- `xgc_camera_calibration` is optional. It contains the standard Noetic
  intrinsic-calibration wrapper, a synchronized assisted 3D-to-2D extrinsic
  WebUI, a reusable RANSAC/LM PnP solver and a TF publisher. The extrinsic
  frontend is plain HTML, CSS and JavaScript with no npm or framework runtime.

No platform-specific camera implementation is copied into this product. The
only capture API dependency is the CMake package `xgc2_camera`, public header
`xgc2/camera/camera.hpp`, and target `xgc2::camera`.

## Install and run

```bash
sudo apt update
sudo apt install ros-noetic-xgc-camera-driver
sudo apt install ros-noetic-xgc2-camera-calibration  # only on calibration hosts

source /opt/ros/noetic/setup.bash
roslaunch xgc_camera_driver usb_cam_compat.launch \
  video_device:=/dev/video0 width:=1920 height:=1080 \
  framerate:=30 pixel_format:=mjpeg camera_info_url:=file:///path/intrinsics.yaml
```

The default compatibility contract is:

```text
/usb_cam/image_raw   sensor_msgs/Image
/usb_cam/camera_info sensor_msgs/CameraInfo
/diagnostics         diagnostic_msgs/DiagnosticArray
frame_id             usb_cam_optical_frame
```

`Image` and `CameraInfo` always use the same timestamp, dimensions and frame.
Until valid intrinsics are loaded, `CameraInfo.K` remains zero and diagnostics
report `WARN: camera is publishing but CameraInfo is not calibrated`; the
driver never invents intrinsics.
The driver maps V4L2 realtime timestamps directly and anchors monotonic
timestamps into ROS time instead of replacing capture time with dequeue time.

Main private parameters are `backend` (`v4l2` or `synthetic`),
`video_device`, `width`, `height`, `framerate`, `pixel_format`,
`capture_mode`, `buffer_count`, `capture_timeout_ms`, `output_encoding`,
`camera_name`, `frame_id` and `camera_info_url`. Supported capture formats are
MJPEG, H264, YUYV, UYVY,
RGB24, BGR24, NV12 and GREY. H264 uses a persistent FFmpeg parser/decoder, so
NAL units split across V4L2 packets are preserved. Parser-only H264 packets do
not increment the published-frame counter. Output is `bgr8`, `rgb8` or `mono8`.

For deterministic tests without a camera:

```bash
roslaunch xgc_camera_driver camera.launch \
  backend:=synthetic pixel_format:=bgr24 width:=320 height:=240 framerate:=10
```

## Calibration assets

Calibration results are runtime assets and are never written into a package
share directory. Interactive launch defaults use the invoking user's state directory;
Automation definitions default to `/var/lib/xgc2/camera/calibrations` and allow
every path to be overridden.

Intrinsic calibration:

```bash
roslaunch xgc_camera_calibration intrinsic_calibrator.launch \
  image_topic:=/usb_cam/image_raw board_size:=11x8 square_size:=0.03 \
  output_dir:=$HOME/.local/state/xgc2/camera/calibrations/usb_cam/intrinsic
```

After pressing `CALIBRATE` and `SAVE`, the wrapper safely extracts the
`ost.yaml` member from `/tmp/calibrationdata.tar.gz` to `intrinsics.yaml` in the
configured directory and keeps the original archive beside it.

Assisted extrinsic calibration:

```bash
roslaunch xgc_camera_calibration extrinsic_calibrator.launch \
  image_topic:=/usb_cam/image_raw \
  camera_info_topic:=/usb_cam/camera_info \
  pose_prefix:=/vrpn_client_node \
  bind_address:=127.0.0.1 http_port:=8765
```

Open `http://127.0.0.1:8765/`, freeze a synchronized frame, select each marker
name and click its image center, then select **Solve and save**. Use at least
four correspondences; six or more non-coplanar points provide useful outlier
rejection. The Python backend keeps a bounded pose history and selects the
sample nearest to the frozen image timestamp. Samples outside
`maximum_marker_age` are excluded. The solver performs RANSAC, iterative
refinement, degeneracy checks, error gating and an atomic YAML write.

The WebUI uses polling and JPEG snapshots over a small versioned HTTP API:

- `GET /healthz` and `GET /api/v1/state` expose process/input state;
- `GET /api/v1/image.jpg` returns the live or frozen camera frame;
- `POST /api/v1/freeze`, `/api/v1/live` and `/api/v1/solve` own the operator
  workflow.

No ROS logic or calibration math runs in the browser. A frozen frame and its
time-matched marker poses form one immutable backend generation, and solve
requests from an older generation are rejected. This makes the same page safe
to host directly or embed as an iframe in a future XGC2 panel.

The server binds to loopback by default. `frame_ancestors` controls which XGC2
origins may embed the page, and `allowed_origins` optionally enables explicit
cross-origin API calls. Binding to a non-loopback address should only be done
behind the platform's authenticated reverse proxy or on a trusted network.

The solved convention is `parent_T_camera_optical`. The TF publisher converts
it into the stable REP-103 chain:

```text
map -> usb_cam_link -> usb_cam_optical_frame
```

```bash
roslaunch xgc_camera_calibration extrinsic_tf.launch \
  extrinsic_file:=$HOME/.local/state/xgc2/camera/calibrations/usb_cam/extrinsics.yaml
```

## Automation

The driver package installs
`/usr/share/xgc2/process-definitions/xgc2-camera-ros1.json`. Configure that file
or its containing directory in `XGC_PROCESS_DEFINITION_PLUGINS`. It registers:

- `xgc2-camera-v4l2-ros1`
- `xgc2-camera-intrinsic-calibrator-ros1`
- `xgc2-camera-extrinsic-calibrator-ros1`
- `xgc2-camera-extrinsic-tf-ros1`

Driver readiness and liveness consume the small `header/stamp` field of an
actual `Image` message; a registered node with no frames is not considered
ready, and probes never stream full image payloads. Failures exit nonzero, camera
resources are released through RAII, and the driver definition allows at most
three supervised restarts.

The extrinsic definition runs
`xgc_camera_calibration/extrinsic_calibrator_web.py` directly without a desktop
session or `DISPLAY`. Its default panel URL is `http://127.0.0.1:8765/`.

## Build and release

Noetic packages are released for Focal `amd64` and `arm64`. CI builds and tests
against the release-resolved `libxgc2-camera-dev`, creates two Debian packages, installs
them in a clean container, runs the synthetic ROS topic contract, checks linked
libraries, starts the installed calibration HTTP service, checks its health and
static frontend, and emits trusted artifact manifests.
