#!/usr/bin/env python3
"""ROS1 image adapter and HTTP entrypoint for intrinsic calibration.

Serves the intrinsic sample-guide WebUI (3D guide, 2D reference images and
X/Y/Size/Skew coverage) against any camera on an image topic.  It is
camera-agnostic: a throttled feeder thread streams the latest frame into the
cv2-direct intrinsic session, and the calibration works with no camera control
at all.  When the optional Gazebo camera-control adapter is enabled, the guide
additionally lights up -- fly the camera to a sample pose, auto-run the whole
sweep, and reset -- by talking to Gazebo's own /gazebo topics.
"""

import sys
import threading
from pathlib import Path

import cv2
import rospkg
import rospy
from sensor_msgs.msg import Image

from xgc_camera_calibration.intrinsic_service import IntrinsicCalibrationService
from xgc_camera_calibration.web_service import CalibrationHttpServer, image_message_to_bgr


def normalize_topic(value):
    text = str(value).strip()
    return text if text.startswith("/") else "/" + text


def split_list_parameter(value):
    if isinstance(value, list):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value).split(",") if item.strip()]


class RosImageSource:
    """Thread-safe latest-frame buffer for one camera image topic."""

    def __init__(self, image_topic):
        self.image_topic = normalize_topic(image_topic)
        self.lock = threading.RLock()
        self.image_message = None
        self.subscriber = rospy.Subscriber(
            self.image_topic, Image, self._on_image, queue_size=1, buff_size=2**24
        )

    def _on_image(self, message):
        with self.lock:
            self.image_message = message

    def latest_bgr(self):
        with self.lock:
            message = self.image_message
        if message is None:
            return None
        try:
            return image_message_to_bgr(message)
        except (TypeError, ValueError, cv2.error):
            return None


def feed_frames(source, service, rate_hz, stop_event):
    """Push the latest camera frame into the intrinsic session at a fixed rate.

    Runs off the subscriber thread so 4K board detection never stalls image
    delivery; broad exception handling keeps the feeder alive across transient
    conversion errors.
    """
    rate = rospy.Rate(rate_hz if rate_hz > 0.0 else 1.0)
    while not rospy.is_shutdown() and not stop_event.is_set():
        try:
            frame = source.latest_bgr()
            if frame is not None:
                service.process_frame(frame)
        except Exception as error:
            rospy.logwarn_throttle(10.0, "Intrinsic feeder skipped a frame: %s", error)
        rate.sleep()


def maybe_camera_control(board_center):
    """Attach the optional Gazebo camera adapter, or run camera-agnostic.

    Only attaches when ``~camera_control`` is requested and the model actually
    appears on /gazebo/model_states within the timeout, so a real-camera run and
    a simulation without the model both fall back cleanly to guidance-only.
    """
    if not bool(rospy.get_param("~camera_control", False)):
        return None
    model_name = str(rospy.get_param("~camera_model_name", "gazebo_static_camera"))
    timeout = float(rospy.get_param("~camera_control_timeout", 8.0))
    try:
        from xgc_camera_calibration.camera_control import GazeboCameraControl

        control = GazeboCameraControl(model_name, board_center)
    except Exception as error:
        rospy.logwarn("Sim camera control unavailable (%s); running camera-agnostic", error)
        return None
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    poll = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if control.available():
            rospy.loginfo("Sim camera control attached for model '%s'", model_name)
            return control
        poll.sleep()
    rospy.logwarn(
        "Gazebo model '%s' not seen in %.1fs; running camera-agnostic", model_name, timeout
    )
    return None


def main():
    rospy.init_node("xgc_camera_intrinsic_calibrator_web")
    try:
        image_topic = normalize_topic(rospy.get_param("~image_topic", "/usb_cam/image_raw"))
        camera_info_topic = normalize_topic(
            rospy.get_param("~camera_info_topic", "/usb_cam/camera_info")
        )
        source = RosImageSource(image_topic)
        package_root = Path(rospkg.RosPack().get_path("xgc_camera_calibration"))
        web_root = Path(rospy.get_param("~web_root", str(package_root / "web" / "intrinsic")))
        calibrations = Path.home() / ".local/state/xgc2/camera/calibrations/usb_cam"
        board_center = (
            float(rospy.get_param("~board_x", 2.0)),
            float(rospy.get_param("~board_y", 0.0)),
            float(rospy.get_param("~board_z", 1.5)),
        )
        # Interior corners for the shared checkerboard_8x6 model (8x6 squares).
        service = IntrinsicCalibrationService(
            board_size=(
                int(rospy.get_param("~board_cols", 7)),
                int(rospy.get_param("~board_rows", 5)),
            ),
            square=float(rospy.get_param("~square_size", 0.20)),
            output_file=rospy.get_param("~output_file", str(calibrations / "intrinsics.yaml")),
            image_topic=image_topic,
            camera_info_topic=camera_info_topic,
            jpeg_quality=int(rospy.get_param("~jpeg_quality", 80)),
            display_width=int(rospy.get_param("~display_width", 720)),
            board_center=board_center,
            references_dir=str(
                rospy.get_param("~references_dir", str(calibrations / "intrinsic_refs"))
            ),
        )
        camera = maybe_camera_control(board_center)
        if camera is not None:
            service.attach_camera_control(camera)
        bind_address = str(rospy.get_param("~bind_address", "127.0.0.1"))
        http_port = int(rospy.get_param("~http_port", 8766))
        if not 1 <= http_port <= 65535:
            raise ValueError("~http_port must be between 1 and 65535")
        server = CalibrationHttpServer(
            (bind_address, http_port),
            None,
            web_root,
            frame_ancestors=str(
                rospy.get_param(
                    "~frame_ancestors", "'self' http://127.0.0.1:* http://localhost:*"
                )
            ),
            allowed_origins=split_list_parameter(rospy.get_param("~allowed_origins", [])),
            logger=lambda message: rospy.logdebug("Intrinsic web: %s", message),
            intrinsic_service=service,
        )
    except Exception as error:
        rospy.logfatal("Could not start intrinsic calibration WebUI: %s", error)
        return 1

    stop_event = threading.Event()
    server_thread = threading.Thread(
        target=server.serve_forever, name="intrinsic-calibration-http", daemon=True
    )
    feeder_thread = threading.Thread(
        target=feed_frames,
        args=(source, service, float(rospy.get_param("~intrinsic_rate", 6.0)), stop_event),
        name="intrinsic-calibration-feeder",
        daemon=True,
    )
    server_thread.start()
    feeder_thread.start()
    rospy.loginfo(
        "Intrinsic calibration WebUI on http://%s:%d (image=%s, camera_control=%s)",
        bind_address,
        http_port,
        image_topic,
        camera is not None,
    )
    try:
        rospy.spin()
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=5.0)
        feeder_thread.join(timeout=5.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
