"""Camera-link and optical-frame transform helpers."""

import math

import numpy as np

from .solver import CalibrationError, rotation_matrix_to_quaternion


def quaternion_to_rotation_matrix(quaternion_xyzw):
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise CalibrationError("quaternion has zero norm")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def link_to_optical_rotation():
    """Return camera_link_R_camera_optical per REP-103."""

    return _rpy_matrix(-math.pi / 2.0, 0.0, -math.pi / 2.0)


def split_parent_to_optical_pose(translation, quaternion_xyzw):
    """Convert parent_T_optical into parent_T_link and link_T_optical.

    The link and optical origins are coincident. camera_link uses x-forward,
    y-left, z-up; camera_optical uses x-right, y-down, z-forward.
    """

    parent_r_optical = quaternion_to_rotation_matrix(quaternion_xyzw)
    link_r_optical = link_to_optical_rotation()
    parent_r_link = parent_r_optical.dot(link_r_optical.T)
    return {
        "parent_t_link": np.asarray(translation, dtype=np.float64).reshape(3),
        "parent_q_link_xyzw": rotation_matrix_to_quaternion(parent_r_link),
        "link_t_optical": np.zeros(3, dtype=np.float64),
        "link_q_optical_xyzw": rotation_matrix_to_quaternion(link_r_optical),
    }
