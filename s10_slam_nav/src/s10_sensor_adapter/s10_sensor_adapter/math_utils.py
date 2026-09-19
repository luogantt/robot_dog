"""Small frame-rotation helpers independent from ROS."""

import math
from typing import Iterable, List


def rpy_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """Return Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotate_vector(matrix: List[List[float]], values: Iterable[float]) -> List[float]:
    vector = list(values)
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def rotate_covariance(matrix: List[List[float]], covariance: Iterable[float]) -> List[float]:
    """Rotate a row-major 3x3 covariance with R*C*R^T."""
    source = list(covariance)
    if len(source) != 9 or source[0] < 0.0:
        return source
    cov = [source[0:3], source[3:6], source[6:9]]
    rc = [
        [sum(matrix[i][k] * cov[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    return [
        sum(rc[i][k] * matrix[j][k] for k in range(3))
        for i in range(3)
        for j in range(3)
    ]
