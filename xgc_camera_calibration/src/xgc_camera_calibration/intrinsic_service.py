"""HTTP-independent intrinsic calibration session, driven frame by frame.

Camera-agnostic: it consumes decoded BGR frames (``process_frame``) from any
source -- a real camera or a simulated one -- and never moves the camera.  It
mirrors the ROS ``camera_calibration`` operator loop (auto-collect geometrically
diverse chessboard views, report X/Y/Size/Skew coverage, then calibrate) using
the cv2-direct, cv_bridge-free ``intrinsic_solver``.
"""

from __future__ import annotations

import threading
from http import HTTPStatus
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from xgc_camera_calibration import intrinsic_solver
from xgc_camera_calibration.solver import CalibrationError
from xgc_camera_calibration.web_service import ApiError


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
        display = bgr
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
            self._display = display

    def image_jpeg(self) -> bytes:
        with self.lock:
            display = self._display
        if display is None:
            raise ApiError(HTTPStatus.SERVICE_UNAVAILABLE, "No camera image has arrived")
        return self._encode_jpeg(display)

    def state(self) -> Dict[str, Any]:
        with self.lock:
            bars, goodenough = intrinsic_solver.coverage(self.samples)
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
            }

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
        return self.state()
