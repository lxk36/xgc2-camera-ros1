#!/usr/bin/env python3
"""ROS1 camera/pose adapter and HTTP entrypoint for extrinsic calibration."""

import re
import sys
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospkg
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

from xgc_camera_calibration.web_service import (
    ApiError,
    CalibrationHttpServer,
    CalibrationService,
    FrameSnapshot,
    MarkerObservation,
    image_message_to_bgr,
    nearest_observation,
)


def normalize_topic(value):
    normalized = str(value).strip()
    return normalized if normalized.startswith("/") else "/" + normalized


class RosCalibrationSource:
    """Thread-safe latest camera frame and timestamped pose-marker history."""

    def __init__(self):
        self.lock = threading.RLock()
        self.image_topic = normalize_topic(rospy.get_param("~image_topic", "/usb_cam/image_raw"))
        self.camera_info_topic = normalize_topic(
            rospy.get_param("~camera_info_topic", "/usb_cam/camera_info")
        )
        self.pose_prefix = normalize_topic(
            rospy.get_param("~pose_prefix", "/vrpn_client_node")
        ).rstrip("/")
        tracker_value = rospy.get_param("~trackers", [])
        if isinstance(tracker_value, list):
            self.tracker_filter = {
                str(item).strip() for item in tracker_value if str(item).strip()
            }
        else:
            self.tracker_filter = {
                item.strip() for item in str(tracker_value).split(",") if item.strip()
            }
        self.pose_history_size = int(rospy.get_param("~pose_history_size", 240))
        if self.pose_history_size < 2:
            raise ValueError("~pose_history_size must be at least 2")
        self.image_message = None
        self.camera_info = None
        self.marker_latest = {}
        self.marker_history = {}
        self.marker_subscribers = {}
        self.marker_topics = {}
        self.image_subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self._image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._info_callback, queue_size=1
        )
        self.discovery_timer = rospy.Timer(rospy.Duration(1.0), self._refresh_markers)
        self._refresh_markers(None)

    def _image_callback(self, message):
        with self.lock:
            self.image_message = message

    def _info_callback(self, message):
        with self.lock:
            self.camera_info = message

    def _refresh_markers(self, _event):
        pattern = re.compile(r"^" + re.escape(self.pose_prefix) + r"/([^/]+)/pose$")
        desired = {}
        try:
            topics = rospy.get_published_topics()
        except rospy.ROSException as error:
            rospy.logwarn_throttle(5.0, "Could not discover pose markers: %s", error)
            return
        for topic, message_type in topics:
            match = pattern.match(topic)
            if not match or message_type != "geometry_msgs/PoseStamped":
                continue
            name = match.group(1)
            if self.tracker_filter and name not in self.tracker_filter:
                continue
            desired[topic] = name
        with self.lock:
            for topic in set(self.marker_subscribers) - set(desired):
                self.marker_subscribers.pop(topic).unregister()
                removed_name = self.marker_topics.pop(topic, "")
                if removed_name and removed_name not in desired.values():
                    self.marker_latest.pop(removed_name, None)
                    self.marker_history.pop(removed_name, None)
            for topic, name in desired.items():
                if topic in self.marker_subscribers:
                    continue
                self.marker_history.setdefault(
                    name, deque(maxlen=self.pose_history_size)
                )
                self.marker_topics[topic] = name
                self.marker_subscribers[topic] = rospy.Subscriber(
                    topic, PoseStamped, self._marker_callback(name), queue_size=20
                )

    def _marker_callback(self, name):
        def callback(message):
            stamp = message.header.stamp
            if stamp.is_zero():
                stamp = rospy.Time.now()
            position = message.pose.position
            observation = MarkerObservation(
                name=name,
                position=(float(position.x), float(position.y), float(position.z)),
                stamp_sec=float(stamp.to_sec()),
                frame_id=message.header.frame_id,
            )
            with self.lock:
                history = self.marker_history.setdefault(
                    name, deque(maxlen=self.pose_history_size)
                )
                if history and observation.stamp_sec < history[-1].stamp_sec:
                    history.clear()
                history.append(observation)
                self.marker_latest[name] = observation

        return callback

    def _convert_image(self, message):
        try:
            return image_message_to_bgr(message)
        except (TypeError, ValueError, cv2.error) as error:
            raise ApiError(409, "Could not convert camera image: {}".format(error)) from error

    def preview_image(self):
        with self.lock:
            message = self.image_message
        if message is None:
            return None
        return self._convert_image(message)

    def status(self):
        with self.lock:
            image = self.image_message
            info = self.camera_info
            marker_names = sorted(
                name for name, history in self.marker_history.items() if history
            )
            stamp_sec = None
            if image is not None:
                stamp = image.header.stamp
                stamp_sec = float((stamp if not stamp.is_zero() else rospy.Time.now()).to_sec())
            return {
                "image_topic": self.image_topic,
                "camera_info_topic": self.camera_info_topic,
                "pose_prefix": self.pose_prefix,
                "image_ready": image is not None,
                "camera_info_ready": info is not None,
                "marker_count": len(marker_names),
                "marker_names": marker_names,
                "latest_image_stamp_sec": stamp_sec,
            }

    def freeze(self, parent_frame, maximum_marker_age):
        with self.lock:
            image_message = self.image_message
            camera_info = self.camera_info
            histories = {
                name: tuple(history) for name, history in self.marker_history.items()
            }
        if image_message is None or camera_info is None:
            raise ApiError(409, "Image and CameraInfo have not both arrived")
        image_stamp = image_message.header.stamp
        if image_stamp.is_zero():
            image_stamp = rospy.Time.now()
        stamp_sec = float(image_stamp.to_sec())
        markers = {}
        stale = {}
        wrong_frames = []
        for name, history in histories.items():
            if not history:
                continue
            observation, age = nearest_observation(
                history, stamp_sec, maximum_marker_age
            )
            if observation is None:
                if age is None:
                    continue
                stale[name] = age
                continue
            if observation.frame_id and observation.frame_id != parent_frame:
                wrong_frames.append(name)
                continue
            markers[name] = MarkerObservation(
                name=observation.name,
                position=observation.position,
                stamp_sec=observation.stamp_sec,
                frame_id=observation.frame_id,
                age_sec=age,
            )
        if wrong_frames:
            raise ApiError(
                409,
                "Pose markers are not expressed in parent frame '{}': {}".format(
                    parent_frame, ", ".join(sorted(wrong_frames))
                ),
            )
        image = self._convert_image(image_message)
        if (
            camera_info.width
            and int(camera_info.width) != image.shape[1]
            or camera_info.height
            and int(camera_info.height) != image.shape[0]
        ):
            raise ApiError(409, "CameraInfo dimensions do not match the image")
        return FrameSnapshot(
            image=image,
            stamp_sec=stamp_sec,
            frame_id=image_message.header.frame_id,
            camera_matrix=np.asarray(camera_info.K, dtype=np.float64).reshape(3, 3),
            distortion=np.asarray(camera_info.D, dtype=np.float64),
            markers=markers,
            stale_markers=stale,
        )


def split_list_parameter(value):
    if isinstance(value, list):
        return [str(item).strip() for item in value if str(item).strip()]
    return [item.strip() for item in str(value).split(",") if item.strip()]


def main():
    rospy.init_node("xgc_camera_extrinsic_calibrator_web")
    try:
        source = RosCalibrationSource()
        package_root = Path(rospkg.RosPack().get_path("xgc_camera_calibration"))
        web_root = Path(rospy.get_param("~web_root", str(package_root / "web" / "extrinsic")))
        service = CalibrationService(
            source,
            output_file=rospy.get_param(
                "~output_file",
                str(
                    Path.home()
                    / ".local/state/xgc2/camera/calibrations/usb_cam/extrinsics.yaml"
                ),
            ),
            parent_frame=rospy.get_param("~parent_frame", "map"),
            child_frame=rospy.get_param("~child_frame", "usb_cam_optical_frame"),
            maximum_marker_age=float(rospy.get_param("~maximum_marker_age", 0.1)),
            ransac_threshold_px=float(rospy.get_param("~ransac_threshold_px", 3.0)),
            maximum_inlier_error_px=float(
                rospy.get_param("~maximum_inlier_error_px", 5.0)
            ),
            jpeg_quality=int(rospy.get_param("~jpeg_quality", 80)),
        )
        bind_address = str(rospy.get_param("~bind_address", "127.0.0.1"))
        http_port = int(rospy.get_param("~http_port", 8765))
        if not 1 <= http_port <= 65535:
            raise ValueError("~http_port must be between 1 and 65535")
        server = CalibrationHttpServer(
            (bind_address, http_port),
            service,
            web_root,
            frame_ancestors=str(
                rospy.get_param(
                    "~frame_ancestors",
                    "'self' http://127.0.0.1:* http://localhost:*",
                )
            ),
            allowed_origins=split_list_parameter(
                rospy.get_param("~allowed_origins", [])
            ),
            logger=lambda message: rospy.logdebug("Web calibrator: %s", message),
        )
    except Exception as error:
        rospy.logfatal("Could not start camera extrinsic WebUI: %s", error)
        return 1

    server_thread = threading.Thread(
        target=server.serve_forever,
        name="camera-calibration-http",
        daemon=True,
    )
    server_thread.start()
    rospy.loginfo(
        "Camera extrinsic WebUI listening on http://%s:%d (image=%s, poses=%s)",
        bind_address,
        http_port,
        source.image_topic,
        source.pose_prefix,
    )
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=5.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
