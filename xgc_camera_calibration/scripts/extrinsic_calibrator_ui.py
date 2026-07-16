#!/usr/bin/env python3
"""Assisted VRPN/pose-to-image extrinsic calibration UI."""

import copy
import os
import re
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QObject, QPoint, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from sensor_msgs.msg import CameraInfo, Image

from xgc_camera_calibration.solver import CalibrationError, save_extrinsic, solve_extrinsic


def normalize_topic(value):
    value = value.strip()
    return value if value.startswith("/") else "/" + value


def default_output_file():
    state_home = os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state"))
    return str(Path(state_home) / "xgc2/camera/calibrations/usb_cam/extrinsics.yaml")


class Signals(QObject):
    changed = pyqtSignal()
    log = pyqtSignal(str)


class RosState:
    def __init__(self, signals):
        self.signals = signals
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.image_topic = normalize_topic(rospy.get_param("~image_topic", "/usb_cam/image_raw"))
        self.camera_info_topic = normalize_topic(
            rospy.get_param("~camera_info_topic", "/usb_cam/camera_info")
        )
        self.pose_prefix = normalize_topic(rospy.get_param("~pose_prefix", "/vrpn_client_node")).rstrip("/")
        self.image = None
        self.image_message = None
        self.camera_info = None
        self.marker_messages = {}
        self.marker_subscribers = {}
        self.image_subscriber = rospy.Subscriber(
            self.image_topic, Image, self._image_callback, queue_size=1, buff_size=2**24
        )
        self.info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._info_callback, queue_size=1
        )
        self.signals.log.emit("Image: {}; CameraInfo: {}".format(self.image_topic, self.camera_info_topic))

    def refresh_markers(self):
        pattern = re.compile(r"^" + re.escape(self.pose_prefix) + r"/([^/]+)/pose$")
        desired = {}
        try:
            published = rospy.get_published_topics()
        except rospy.ROSException as error:
            self.signals.log.emit("Could not inspect pose topics: {}".format(error))
            return
        for topic, message_type in published:
            match = pattern.match(topic)
            if match and message_type == "geometry_msgs/PoseStamped":
                desired[topic] = match.group(1)
        with self.lock:
            for topic in set(self.marker_subscribers) - set(desired):
                self.marker_subscribers.pop(topic).unregister()
            for topic, name in desired.items():
                if topic not in self.marker_subscribers:
                    self.marker_subscribers[topic] = rospy.Subscriber(
                        topic, PoseStamped, self._marker_callback(name), queue_size=5
                    )

    def snapshot(self, maximum_age):
        with self.lock:
            if self.image is None or self.image_message is None or self.camera_info is None:
                return None
            image_stamp = self.image_message.header.stamp
            if image_stamp.is_zero():
                image_stamp = rospy.Time.now()
            markers = {}
            stale = []
            for name, message in self.marker_messages.items():
                marker_stamp = message.header.stamp if not message.header.stamp.is_zero() else image_stamp
                age = abs((marker_stamp - image_stamp).to_sec())
                if age <= maximum_age:
                    markers[name] = copy.deepcopy(message)
                else:
                    stale.append((name, age))
            return (
                self.image.copy(),
                copy.deepcopy(self.camera_info),
                markers,
                stale,
                image_stamp,
            )

    def live(self):
        with self.lock:
            return (
                None if self.image is None else self.image.copy(),
                copy.deepcopy(self.camera_info),
                copy.deepcopy(self.marker_messages),
            )

    def _image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.signals.log.emit("Image conversion failed: {}".format(error))
            return
        with self.lock:
            self.image = image
            self.image_message = message
        self.signals.changed.emit()

    def _info_callback(self, message):
        with self.lock:
            self.camera_info = message
        self.signals.changed.emit()

    def _marker_callback(self, name):
        def callback(message):
            with self.lock:
                self.marker_messages[name] = message
            self.signals.changed.emit()

        return callback


class MarkerDialog(QDialog):
    def __init__(self, names, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select pose marker")
        self.combo = QComboBox()
        self.combo.addItems(names)
        layout = QFormLayout(self)
        layout.addRow("Marker", self.combo)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def selected(self):
        return self.combo.currentText()


class ImageCanvas(QLabel):
    clicked = pyqtSignal(float, float, QPoint)

    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(720, 405)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background: #111; color: #bbb")
        self.setText("Waiting for camera image")
        self.original = None
        self.shape = None
        self.draw_rect = None

    def display(self, image, selected_points=(), projected_points=()):
        if image is None:
            return
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        qimage = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(qimage)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        for item in selected_points:
            x, y = map(lambda value: int(round(value)), item["pixel"])
            painter.setPen(QPen(QColor(255, 70, 70), 3))
            painter.drawEllipse(QPoint(x, y), 7, 7)
            painter.drawText(x + 9, y - 7, item["marker"])
        for item in projected_points:
            x, y = map(lambda value: int(round(value)), item["pixel"])
            painter.setPen(QPen(QColor(70, 230, 90), 3))
            painter.drawEllipse(QPoint(x, y), 6, 6)
            painter.drawText(x + 9, y - 7, item["marker"])
        painter.end()
        self.original = pixmap
        self.shape = (height, width)
        self._scale()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._scale()

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton or self.original is None or not self.draw_rect:
            return
        if not self.draw_rect.contains(event.pos()):
            return
        height, width = self.shape
        x = (event.x() - self.draw_rect.x()) * width / float(self.draw_rect.width())
        y = (event.y() - self.draw_rect.y()) * height / float(self.draw_rect.height())
        self.clicked.emit(x, y, event.globalPos())

    def _scale(self):
        if self.original is None:
            return
        scaled = self.original.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        x = int((self.width() - scaled.width()) / 2)
        y = int((self.height() - scaled.height()) / 2)
        self.draw_rect = scaled.rect().translated(x, y)
        self.setPixmap(scaled)


def pose_position(message):
    position = message.pose.position
    return np.array([position.x, position.y, position.z], dtype=np.float64)


class Window(QMainWindow):
    def __init__(self, state, signals):
        super().__init__()
        self.state = state
        self.signals = signals
        self.output_file = rospy.get_param("~output_file", default_output_file())
        self.parent_frame = rospy.get_param("~parent_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "usb_cam_optical_frame")
        self.maximum_marker_age = float(rospy.get_param("~maximum_marker_age", 0.1))
        self.ransac_threshold = float(rospy.get_param("~ransac_threshold_px", 3.0))
        self.maximum_error = float(rospy.get_param("~maximum_inlier_error_px", 5.0))
        self.frozen = None
        self.points = []
        self.result = None
        self.setWindowTitle("XGC2 assisted camera extrinsic calibration")
        self.resize(1320, 820)
        self._build()
        signals.log.connect(self.log)
        signals.changed.connect(self.redraw)
        self.canvas.clicked.connect(self.add_point)

        self.marker_timer = QTimer(self)
        self.marker_timer.timeout.connect(self.state.refresh_markers)
        self.marker_timer.start(1500)
        self.state.refresh_markers()
        self.draw_timer = QTimer(self)
        self.draw_timer.timeout.connect(self.redraw)
        self.draw_timer.start(100)
        self.log("Output asset: {}".format(self.output_file))

    def _build(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        self.canvas = ImageCanvas()
        layout.addWidget(self.canvas, stretch=4)
        side = QVBoxLayout()
        layout.addLayout(side, stretch=2)
        buttons = QHBoxLayout()
        for label, callback in (
            ("Freeze synchronized frame", self.freeze),
            ("Live", self.unfreeze),
            ("Solve and save", self.solve),
            ("Remove", self.remove),
            ("Clear", self.clear),
        ):
            button = QPushButton(label)
            button.clicked.connect(callback)
            buttons.addWidget(button)
        side.addLayout(buttons)
        self.status = QLabel("Live")
        side.addWidget(self.status)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["marker", "u", "v", "world xyz"])
        self.table.horizontalHeader().setStretchLastSection(True)
        side.addWidget(self.table, stretch=2)
        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(160)
        side.addWidget(self.result_text)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        side.addWidget(self.log_text, stretch=1)

    def freeze(self):
        snapshot = self.state.snapshot(self.maximum_marker_age)
        if snapshot is None:
            QMessageBox.warning(self, "Cannot freeze", "Image and CameraInfo have not both arrived.")
            return
        image, info, markers, stale, stamp = snapshot
        if not markers:
            QMessageBox.warning(self, "Cannot freeze", "No time-matched pose marker is available.")
            return
        if (info.width and info.width != image.shape[1]) or (info.height and info.height != image.shape[0]):
            QMessageBox.warning(self, "Cannot freeze", "CameraInfo dimensions do not match the image.")
            return
        intrinsic = np.asarray(info.K, dtype=np.float64).reshape(3, 3)
        if not np.all(np.isfinite(intrinsic)) or intrinsic[0, 0] <= 0.0 or intrinsic[1, 1] <= 0.0:
            QMessageBox.warning(
                self,
                "Cannot freeze",
                "CameraInfo is uncalibrated; load or generate camera intrinsics first.",
            )
            return
        wrong_frames = sorted(
            name
            for name, message in markers.items()
            if message.header.frame_id and message.header.frame_id != self.parent_frame
        )
        if wrong_frames:
            QMessageBox.warning(
                self,
                "Cannot freeze",
                "Pose markers are not expressed in parent frame '{}': {}".format(
                    self.parent_frame, ", ".join(wrong_frames)
                ),
            )
            return
        self.frozen = (image, info, markers, stamp)
        self.points = []
        self.status.setText("Frozen at {:.6f}: {} synchronized markers".format(stamp.to_sec(), len(markers)))
        if stale:
            self.log("Excluded stale markers: " + ", ".join("{} ({:.3f}s)".format(*item) for item in stale))
        self.redraw()
        self._refresh_table()

    def unfreeze(self):
        self.frozen = None
        self.status.setText("Live; green points show the latest solution" if self.result else "Live")
        self.redraw()

    def add_point(self, x, y, global_position):
        if self.frozen is None:
            return
        markers = self.frozen[2]
        used = {point["marker"] for point in self.points}
        names = sorted(set(markers) - used)
        if not names:
            return
        dialog = MarkerDialog(names, self)
        dialog.move(global_position)
        if dialog.exec_() != QDialog.Accepted:
            return
        name = dialog.selected()
        self.points.append({"marker": name, "pixel": (float(x), float(y)), "world": pose_position(markers[name])})
        self._refresh_table()
        self.redraw()

    def remove(self):
        for row in sorted({index.row() for index in self.table.selectedIndexes()}, reverse=True):
            del self.points[row]
        self._refresh_table()
        self.redraw()

    def clear(self):
        self.points = []
        self._refresh_table()
        self.redraw()

    def solve(self):
        if self.frozen is None or len(self.points) < 4:
            QMessageBox.warning(self, "Cannot solve", "Freeze a frame and select at least four correspondences.")
            return
        info = self.frozen[1]
        intrinsic = np.asarray(info.K, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.D, dtype=np.float64)
        try:
            self.result = solve_extrinsic(
                [point["world"] for point in self.points],
                [point["pixel"] for point in self.points],
                intrinsic,
                distortion,
                ransac_reprojection_error_px=self.ransac_threshold,
                maximum_accepted_error_px=self.maximum_error,
            )
            persisted_points = []
            inliers = set(map(int, self.result.inlier_indices))
            for index, point in enumerate(self.points):
                persisted_points.append(
                    {
                        "marker": point["marker"],
                        "pixel": list(map(float, point["pixel"])),
                        "world": list(map(float, point["world"])),
                        "inlier": index in inliers,
                        "reprojection_error_px": float(self.result.reprojection_errors_px[index]),
                    }
                )
            save_extrinsic(
                self.output_file,
                self.result,
                parent_frame=self.parent_frame,
                child_frame=self.child_frame,
                points=persisted_points,
                metadata={
                    "image_topic": self.state.image_topic,
                    "camera_info_topic": self.state.camera_info_topic,
                    "pose_prefix": self.state.pose_prefix,
                    "image_width": int(self.frozen[0].shape[1]),
                    "image_height": int(self.frozen[0].shape[0]),
                },
            )
        except (CalibrationError, OSError, cv2.error) as error:
            QMessageBox.critical(self, "Calibration failed", str(error))
            return
        translation = self.result.translation
        quaternion = self.result.quaternion_xyzw
        self.result_text.setPlainText(
            "{} -> {}\nxyz [{:.6f}, {:.6f}, {:.6f}]\nq_xyzw [{:.6f}, {:.6f}, {:.6f}, {:.6f}]\n"
            "mean {:.3f}px, max {:.3f}px\n{}".format(
                self.parent_frame,
                self.child_frame,
                *translation,
                *quaternion,
                self.result.mean_reprojection_error_px,
                self.result.max_reprojection_error_px,
                "; ".join(self.result.warnings),
            )
        )
        self.log("Saved calibrated extrinsic to {}".format(self.output_file))
        self.redraw()

    def redraw(self):
        if self.frozen is not None:
            self.canvas.display(self.frozen[0], self.points)
            return
        image, info, markers = self.state.live()
        projections = []
        if image is not None and info is not None and self.result is not None and markers:
            world = np.array([pose_position(markers[name]) for name in sorted(markers)], dtype=np.float64)
            rvec, _ = cv2.Rodrigues(self.result.rotation_world_to_camera)
            pixels, _ = cv2.projectPoints(
                world,
                rvec,
                self.result.translation_world_to_camera,
                np.asarray(info.K, dtype=np.float64).reshape(3, 3),
                np.asarray(info.D, dtype=np.float64),
            )
            camera_points = (
                self.result.rotation_world_to_camera.dot(world.T).T
                + self.result.translation_world_to_camera
            )
            for name, pixel, camera_point in zip(sorted(markers), pixels.reshape(-1, 2), camera_points):
                if camera_point[2] > 0.0:
                    projections.append({"marker": name, "pixel": tuple(map(float, pixel))})
        self.canvas.display(image, projected_points=projections)

    def _refresh_table(self):
        self.table.setRowCount(len(self.points))
        for row, point in enumerate(self.points):
            values = (
                point["marker"],
                "{:.1f}".format(point["pixel"][0]),
                "{:.1f}".format(point["pixel"][1]),
                "{:.3f}, {:.3f}, {:.3f}".format(*point["world"]),
            )
            for column, value in enumerate(values):
                self.table.setItem(row, column, QTableWidgetItem(value))

    def log(self, message):
        self.log_text.append(message)


def main():
    rospy.init_node("xgc_camera_extrinsic_calibrator", disable_signals=True)
    app = QApplication(sys.argv)
    signals = Signals()
    state = RosState(signals)
    window = Window(state, signals)
    window.show()
    result = app.exec_()
    rospy.signal_shutdown("calibration UI closed")
    return result


if __name__ == "__main__":
    sys.exit(main())
