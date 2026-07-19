"""HTTP-independent intrinsic calibration session, driven frame by frame.

Camera-agnostic: it consumes decoded BGR frames (``process_frame``) from any
source -- a real camera or a simulated one -- and never moves the camera.  It
mirrors the ROS ``camera_calibration`` operator loop (auto-collect geometrically
diverse chessboard views, report X/Y/Size/Skew coverage, then calibrate) using
the cv2-direct, cv_bridge-free ``intrinsic_solver``.

It also owns a **sample guide**: a catalogue of recommended sample viewpoints
(the 3D guide's spheres) plus their pre-recorded reference images.  The guide is
pure visual guidance for any camera.  When an optional camera-control adapter is
attached (only meaningful in simulation), the guide additionally greens each
viewpoint as the camera aligns to it, exposes the live pose, and can fly the
camera through the catalogue (goto / auto-run / reset).
"""

from __future__ import annotations

import os
import threading
import time
from http import HTTPStatus
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from xgc_camera_calibration import intrinsic_solver
from xgc_camera_calibration.solver import CalibrationError
from xgc_camera_calibration.web_service import ApiError


def recommended_views(board_center: Sequence[float]) -> List[Dict[str, Any]]:
    """Spatially-distinct sample poses that together fill X / Y / Size / Skew.

    Filling X and Y needs the board off-centre in the image, so those poses carry
    a yaw/pitch aim offset (the camera adapter applies it through
    look_at_orientation) -- a camera that simply aims at the board keeps it
    centred and never moves the X/Y bars.  Size needs near and far views; Skew
    needs the oblique corners.  Each pose sits at its own point so it is a
    distinct, clickable marker in the 3D guide.
    """
    tx, ty, tz = float(board_center[0]), float(board_center[1]), float(board_center[2])
    specs = [
        ("far (small)", (tx - 6.0, ty, tz), 0.0, 0.0),
        ("near (big)", (tx - 1.8, ty, tz), 0.0, 0.0),
        ("left", (tx - 7.0, ty + 0.3, tz), 0.55, 0.0),
        ("right", (tx - 7.0, ty - 0.3, tz), -0.55, 0.0),
        ("top", (tx - 4.0, ty, tz + 0.8), 0.0, 0.25),
        ("bottom", (tx - 4.0, ty, tz - 0.8), 0.0, -0.30),
        ("oblique UL", (tx - 2.5, ty + 2.2, tz + 1.9), 0.0, 0.0),
        ("oblique UR", (tx - 2.5, ty - 2.2, tz + 1.9), 0.0, 0.0),
        ("oblique LL", (tx - 2.5, ty + 2.2, tz - 1.0), 0.0, 0.0),
        ("oblique LR", (tx - 2.5, ty - 2.2, tz - 1.0), 0.0, 0.0),
    ]
    return [{
        "name": name,
        "position": [round(value, 2) for value in position],
        "yaw_offset": yaw_offset,
        "pitch_offset": pitch_offset,
        "roll": 0.0,
    } for (name, position, yaw_offset, pitch_offset) in specs]


class IntrinsicCalibrationService:
    """Own one operator intrinsic-calibration session over a stream of frames."""

    def __init__(
        self,
        *,
        board_size: Sequence[int],
        square: float,
        output_file: str,
        image_topic: str = "",
        camera_info_topic: str = "",
        jpeg_quality: int = 80,
        sample_distance: float = intrinsic_solver.SAMPLE_DISTANCE,
        maximum_detect_width: int = 960,
        display_width: int = 960,
        board_center: Sequence[float] = (2.0, 0.0, 1.5),
        references_dir: str = "",
        align_threshold: float = 1.8,
    ):
        if not output_file:
            raise ValueError("output_file must not be empty")
        if int(board_size[0]) < 2 or int(board_size[1]) < 2:
            raise ValueError("board_size must be at least 2x2 interior corners")
        if float(square) <= 0.0:
            raise ValueError("square size must be positive")
        if not 1 <= int(jpeg_quality) <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self.board_size = (int(board_size[0]), int(board_size[1]))
        self.square = float(square)
        self.output_file = str(Path(output_file).expanduser())
        self.image_topic = str(image_topic)
        self.camera_info_topic = str(camera_info_topic)
        self.jpeg_quality = int(jpeg_quality)
        self.sample_distance = float(sample_distance)
        self.maximum_detect_width = int(maximum_detect_width)
        self.display_width = int(display_width)
        self.lock = threading.RLock()
        self.samples: List[Tuple[float, float, float, float]] = []
        self.image_points: List[np.ndarray] = []
        self.image_size: Optional[Tuple[int, int]] = None
        self._display: Optional[np.ndarray] = None
        self.result: Optional[intrinsic_solver.IntrinsicResult] = None
        self.result_payload: Optional[Dict[str, Any]] = None

        # Sample guide: a full board spans (interior corners + 1) squares.
        self.board_center = tuple(float(value) for value in board_center)
        self.board_geometry = {
            "center": list(self.board_center),
            "width": (self.board_size[0] + 1) * self.square,
            "height": (self.board_size[1] + 1) * self.square,
        }
        self.views: List[Dict[str, Any]] = recommended_views(self.board_center)
        self.target_done: List[bool] = [False] * len(self.views)
        self.references_dir = str(Path(references_dir).expanduser()) if references_dir else ""
        self.refs: Dict[int, bytes] = {}
        self.align_threshold = float(align_threshold)
        self.camera: Optional[Any] = None
        self._recording = False
        self._load_refs()

    # -- guide wiring ---------------------------------------------------------
    def attach_camera_control(self, camera: Any) -> None:
        """Attach an optional sim camera adapter (goto/reset/current pose)."""
        with self.lock:
            self.camera = camera

    def _load_refs(self) -> None:
        if not self.references_dir:
            return
        for index in range(len(self.views)):
            path = os.path.join(self.references_dir, "{}.jpg".format(index))
            if os.path.isfile(path):
                try:
                    with open(path, "rb") as handle:
                        self.refs[index] = handle.read()
                except OSError:
                    pass

    def _save_ref(self, index: int, jpeg: bytes) -> None:
        self.refs[index] = jpeg
        if not self.references_dir:
            return
        try:
            os.makedirs(self.references_dir, exist_ok=True)
            with open(os.path.join(self.references_dir, "{}.jpg".format(index)), "wb") as handle:
                handle.write(jpeg)
        except OSError:
            pass

    def ref(self, index: int) -> Optional[bytes]:
        with self.lock:
            return self.refs.get(index)

    def _nearest_target(self, position: Sequence[float]) -> Tuple[Optional[int], float]:
        best_index, best_distance = None, float("inf")
        for index, view in enumerate(self.views):
            target = view["position"]
            distance = (
                (position[0] - target[0]) ** 2
                + (position[1] - target[1]) ** 2
                + (position[2] - target[2]) ** 2
            ) ** 0.5
            if distance < best_distance:
                best_distance, best_index = distance, index
        return best_index, best_distance

    def _mark_aligned(self, display: np.ndarray) -> None:
        """Green the nearest target once the camera aligns to it and the board is
        visible this frame -- independent of whether this frame became a *new*
        sample (is_new_sample de-duplicates similar views, so a
        redundant-but-valid pose would otherwise stay grey).  Requires the sim
        camera adapter for the live pose; a no-op for a real camera.
        """
        if self._recording or self.camera is None:
            return
        position = self.camera.current_position()
        if position is None:
            return
        index, distance = self._nearest_target(position)
        if index is None or distance > self.align_threshold or self.target_done[index]:
            return
        self.target_done[index] = True
        ok, encoded = cv2.imencode(
            ".jpg", display, [int(cv2.IMWRITE_JPEG_QUALITY), 75]
        )
        if ok:
            self._save_ref(index, encoded.tobytes())

    def _encode_jpeg(self, image: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise ApiError(HTTPStatus.INTERNAL_SERVER_ERROR, "Could not encode camera frame")
        return encoded.tobytes()

    def process_frame(self, bgr: np.ndarray) -> None:
        """Ingest one decoded BGR frame: detect the board, auto-collect, annotate."""
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            return
        height, width = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        detection = intrinsic_solver.detect_board(gray, self.board_size, self.maximum_detect_width)

        scale = 1.0
        if width > self.display_width:
            scale = float(self.display_width) / float(width)
            display = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        else:
            display = bgr.copy()

        with self.lock:
            self.image_size = (width, height)
            if detection is not None:
                corners, params = detection
                cv2.drawChessboardCorners(
                    display, self.board_size, (corners * scale).astype(np.float32), True
                )
                if self.result is None and intrinsic_solver.is_new_sample(
                    params, self.samples, self.sample_distance
                ):
                    self.samples.append(params)
                    self.image_points.append(corners)
                self._mark_aligned(display)
            self._display = display

    def image_jpeg(self) -> bytes:
        with self.lock:
            display = self._display
        if display is None:
            raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "No camera image has arrived")
        return self._encode_jpeg(display)

    def targets_document(self) -> Dict[str, Any]:
        """Static guide geometry for the 3D scene: board + recommended views."""
        with self.lock:
            return {
                "board": dict(self.board_geometry),
                "views": [
                    {"name": view["name"], "position": view["position"]}
                    for view in self.views
                ],
                "camera_control": self.camera is not None,
            }

    def state(self) -> Dict[str, Any]:
        with self.lock:
            bars, goodenough = intrinsic_solver.coverage(self.samples)
            targets = [{
                "name": view["name"],
                "position": view["position"],
                "done": self.target_done[index],
                "has_ref": index in self.refs,
            } for index, view in enumerate(self.views)]
            next_index = next((i for i, done in enumerate(self.target_done) if not done), None)
            pose = self.camera.current() if self.camera is not None else None
            return {
                "mode": "intrinsic",
                "samples": len(self.samples),
                "coverage": bars,
                "goodenough": bool(goodenough),
                "calibrated": self.result is not None,
                "result": self.result_payload,
                "output_file": self.output_file,
                "image_ready": self._display is not None,
                "image_topic": self.image_topic,
                "board": {"size": list(self.board_size), "square_size_m": self.square},
                "targets": targets,
                "next": next_index,
                "pose": pose,
                "camera_control": self.camera is not None,
            }

    # -- sim camera guidance actions -----------------------------------------
    def _require_camera(self) -> Any:
        if self.camera is None:
            raise ApiError(HTTPStatus.NOT_FOUND, "No camera control is available")
        return self.camera

    def goto(self, index: int) -> Dict[str, Any]:
        camera = self._require_camera()
        if not 0 <= index < len(self.views):
            raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, "Unknown target index")
        view = self.views[index]
        camera.goto(view["position"], view["yaw_offset"], view["pitch_offset"], view["roll"])
        return {"ok": True, "name": view["name"]}

    def reset_pose(self) -> Dict[str, Any]:
        self._require_camera().reset()
        return {"ok": True}

    def auto_run(self, settle: float = 1.3) -> Dict[str, Any]:
        """Reset, then fly through every view hands-free so the feeder collects a
        full sample set (spheres green as it goes).  The operator then calibrates.
        """
        camera = self._require_camera()
        self.reset()
        for view in self.views:
            camera.goto(view["position"], view["yaw_offset"], view["pitch_offset"], view["roll"])
            time.sleep(settle)
        with self.lock:
            return {"ok": True, "samples": len(self.samples)}

    def record_references(self, settle: float = 1.3) -> Dict[str, Any]:
        """One-off: fly to every view, snapshot the annotated frame as its
        reference image, then start fresh so the operator still calibrates
        manually with all spheres grey.
        """
        camera = self._require_camera()
        with self.lock:
            self._recording = True
        saved = 0
        try:
            for index, view in enumerate(self.views):
                camera.goto(view["position"], view["yaw_offset"], view["pitch_offset"], view["roll"])
                time.sleep(settle)
                with self.lock:
                    display = None if self._display is None else self._display.copy()
                if display is not None:
                    ok, encoded = cv2.imencode(".jpg", display, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                    if ok:
                        with self.lock:
                            self._save_ref(index, encoded.tobytes())
                        saved += 1
            camera.reset()
        finally:
            with self.lock:
                self._recording = False
            self.reset()
        return {"ok": True, "saved": saved}

    def calibrate(self) -> Dict[str, Any]:
        with self.lock:
            if self.result is not None:
                return self.result_payload  # type: ignore[return-value]
            if not self.image_points or self.image_size is None:
                raise ApiError(HTTPStatus.CONFLICT, "No chessboard samples collected yet")
            try:
                result = intrinsic_solver.calibrate_intrinsic(
                    self.image_points, self.board_size, self.square, self.image_size
                )
            except (CalibrationError, cv2.error) as error:
                raise ApiError(HTTPStatus.UNPROCESSABLE_ENTITY, str(error)) from error
            try:
                intrinsic_solver.save_intrinsic(
                    self.output_file,
                    result,
                    board_size=self.board_size,
                    square=self.square,
                    metadata={"image_topic": self.image_topic, "web_calibrator": True},
                )
            except OSError as error:
                raise ApiError(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    "Could not save calibration result: {}".format(error),
                ) from error
            matrix = result.camera_matrix
            self.result = result
            self.result_payload = {
                "camera_matrix": [float(v) for v in matrix.reshape(-1)],
                "distortion": [float(v) for v in result.distortion],
                "fx": float(matrix[0, 0]),
                "fy": float(matrix[1, 1]),
                "cx": float(matrix[0, 2]),
                "cy": float(matrix[1, 2]),
                "image_width": result.image_size[0],
                "image_height": result.image_size[1],
                "rms_reprojection_error_px": result.rms_reprojection_error_px,
                "sample_count": result.sample_count,
                "output_file": self.output_file,
            }
            return self.result_payload

    def reset(self) -> Dict[str, Any]:
        with self.lock:
            self.samples = []
            self.image_points = []
            self.result = None
            self.result_payload = None
            self.target_done = [False] * len(self.views)
        return self.state()
