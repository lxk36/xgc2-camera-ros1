"""Intrinsic (pinhole) camera calibration, cv2-direct and cv_bridge-free.

Mirrors the ROS ``camera_calibration`` coverage heuristics (X / Y / Size / Skew)
and delegates the actual estimation to ``cv2.calibrateCamera`` -- the same call
``MonoCalibrator`` makes internally -- without dragging in ``cv_bridge`` or any
ROS calibration class.  Frames arrive already decoded (see
``web_service.image_message_to_bgr``); board detection runs on a down-scaled copy
for speed on large (4K) frames, then corners are refined at full resolution.
"""

from __future__ import annotations

import math
import os
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from xgc_camera_calibration.solver import CalibrationError

_DETECT_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
# Same acceptance ranges the ROS camera_calibration GUI uses for goodenough.
PARAM_RANGES: Tuple[float, float, float, float] = (0.7, 0.7, 0.4, 0.5)
PARAM_NAMES: Tuple[str, str, str, str] = ("X", "Y", "Size", "Skew")
SAMPLE_DISTANCE = 0.2


@dataclass(frozen=True)
class IntrinsicResult:
    camera_matrix: np.ndarray          # 3x3
    distortion: np.ndarray             # (k1,k2,p1,p2,k3)
    image_size: Tuple[int, int]        # (width, height)
    rms_reprojection_error_px: float
    sample_count: int


def board_object_points(board_size: Sequence[int], square: float) -> np.ndarray:
    """3D corner grid (Z=0) for a (cols, rows) interior-corner board."""
    cols, rows = int(board_size[0]), int(board_size[1])
    grid = np.zeros((cols * rows, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    grid *= float(square)
    return grid


def detect_board(
    gray: np.ndarray, board_size: Sequence[int], maximum_width: int = 960
) -> Optional[Tuple[np.ndarray, Tuple[float, float, float, float]]]:
    """Detect chessboard corners; return (corners Nx1x2 full-res, coverage params) or None."""
    if gray.ndim != 2:
        raise ValueError("detect_board expects a single-channel image")
    height, width = gray.shape[:2]
    scale = 1.0
    search = gray
    if width > maximum_width:
        scale = float(maximum_width) / float(width)
        search = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    found, corners = cv2.findChessboardCorners(search, tuple(board_size), _DETECT_FLAGS)
    if not found:
        return None
    corners = (corners / scale).astype(np.float32)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), _SUBPIX_CRITERIA)
    return corners, _coverage_params(corners, board_size, width, height)


def _coverage_params(
    corners: np.ndarray, board_size: Sequence[int], width: int, height: int
) -> Tuple[float, float, float, float]:
    columns = int(board_size[0])
    upper_left = corners[0, 0]
    upper_right = corners[columns - 1, 0]
    lower_right = corners[-1, 0]
    lower_left = corners[-columns, 0]
    edge_a = upper_right - upper_left
    edge_b = lower_right - upper_right
    edge_c = lower_left - lower_right
    diagonal_p = edge_b + edge_c
    diagonal_q = edge_a + edge_b
    area = abs(diagonal_p[0] * diagonal_q[1] - diagonal_p[1] * diagonal_q[0]) / 2.0
    border = math.sqrt(area) if area > 0 else 0.0
    mean_x = float(np.mean(corners[:, :, 0]))
    mean_y = float(np.mean(corners[:, :, 1]))
    p_x = min(1.0, max(0.0, (mean_x - border / 2.0) / max(1e-6, width - border)))
    p_y = min(1.0, max(0.0, (mean_y - border / 2.0) / max(1e-6, height - border)))
    p_size = math.sqrt(area / float(width * height)) if area > 0 else 0.0
    vector_a = upper_left - upper_right
    vector_b = lower_right - upper_right
    norm = float(np.linalg.norm(vector_a)) * float(np.linalg.norm(vector_b))
    cosine = float(np.dot(vector_a, vector_b)) / norm if norm > 0 else 0.0
    angle = math.acos(min(1.0, max(-1.0, cosine)))
    p_skew = min(1.0, 2.0 * abs(math.pi / 2.0 - angle))
    return (p_x, p_y, p_size, p_skew)


def is_new_sample(
    params: Sequence[float], samples: Sequence[Sequence[float]], threshold: float = SAMPLE_DISTANCE
) -> bool:
    """True if params are far enough (L1) from every already-collected sample."""
    if not samples:
        return True
    distance = min(sum(abs(a - b) for a, b in zip(params, sample)) for sample in samples)
    return distance > threshold


def coverage(
    samples: Sequence[Sequence[float]], ranges: Sequence[float] = PARAM_RANGES
) -> Tuple[List[Dict[str, Any]], bool]:
    """Return ([{label, progress}]x4, goodenough), mirroring compute_goodenough."""
    if not samples:
        return [{"label": name, "progress": 0.0} for name in PARAM_NAMES], False
    minimum = [min(sample[i] for sample in samples) for i in range(4)]
    maximum = [max(sample[i] for sample in samples) for i in range(4)]
    minimum[2] = 0.0  # size / skew are rewarded by their maximum only
    minimum[3] = 0.0
    progress = [min(1.0, (hi - lo) / rng) for lo, hi, rng in zip(minimum, maximum, ranges)]
    goodenough = (len(samples) >= 40) or all(value >= 1.0 for value in progress)
    bars = [{"label": name, "progress": float(value)} for name, value in zip(PARAM_NAMES, progress)]
    return bars, goodenough


def calibrate_intrinsic(
    image_points: Sequence[np.ndarray],
    board_size: Sequence[int],
    square: float,
    image_size: Sequence[int],
) -> IntrinsicResult:
    """Estimate K and distortion from the collected corner sets via cv2.calibrateCamera."""
    if len(image_points) < 3:
        raise CalibrationError("need at least three samples to calibrate")
    obj = board_object_points(board_size, square)
    object_points = [obj for _ in image_points]
    corners = [np.asarray(points, dtype=np.float32).reshape(-1, 1, 2) for points in image_points]
    size = (int(image_size[0]), int(image_size[1]))
    rms, camera_matrix, distortion, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points, corners, size, None, None
    )
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
    if not np.all(np.isfinite(camera_matrix)) or camera_matrix[0, 0] <= 0.0:
        raise CalibrationError("calibration produced a degenerate camera matrix")
    return IntrinsicResult(
        camera_matrix=camera_matrix,
        distortion=np.asarray(distortion, dtype=np.float64).reshape(-1),
        image_size=size,
        rms_reprojection_error_px=float(rms),
        sample_count=len(image_points),
    )


def intrinsic_document(
    result: IntrinsicResult,
    *,
    board_size: Sequence[int],
    square: float,
    metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    k = result.camera_matrix
    document: Dict[str, Any] = {
        "schema": "xgc2.camera.intrinsic.v1",
        "created_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "image_width": result.image_size[0],
        "image_height": result.image_size[1],
        "camera_matrix": {"rows": 3, "cols": 3, "data": [float(v) for v in k.reshape(-1)]},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(result.distortion.size),
            "data": [float(v) for v in result.distortion],
        },
        "focal_length": {"fx": float(k[0, 0]), "fy": float(k[1, 1])},
        "principal_point": {"cx": float(k[0, 2]), "cy": float(k[1, 2])},
        "rms_reprojection_error_px": result.rms_reprojection_error_px,
        "sample_count": result.sample_count,
        "board": {"size": [int(board_size[0]), int(board_size[1])], "square_size_m": float(square)},
    }
    if metadata:
        document["metadata"] = dict(metadata)
    return document


def save_intrinsic(
    path: os.PathLike,
    result: IntrinsicResult,
    *,
    board_size: Sequence[int],
    square: float,
    metadata: Optional[Dict[str, Any]] = None,
) -> Path:
    """Atomically persist a versioned intrinsic document outside package share."""
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    document = intrinsic_document(result, board_size=board_size, square=square, metadata=metadata)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix="." + destination.name + ".", suffix=".tmp", dir=str(destination.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, default_flow_style=False, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary_name, 0o644)
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
    return destination


def load_intrinsic(path: os.PathLike) -> Dict[str, Any]:
    source = Path(path).expanduser()
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise CalibrationError("intrinsic document must be a mapping")
    if document.get("schema") != "xgc2.camera.intrinsic.v1":
        raise CalibrationError("unsupported or missing intrinsic schema")
    matrix = document.get("camera_matrix", {})
    data = matrix.get("data") if isinstance(matrix, dict) else None
    if not isinstance(data, list) or len(data) != 9:
        raise CalibrationError("camera_matrix.data must contain nine values")
    document["camera_matrix_array"] = np.asarray(data, dtype=np.float64).reshape(3, 3)
    return document
