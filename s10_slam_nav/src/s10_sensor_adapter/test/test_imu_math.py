import math

from s10_sensor_adapter.math_utils import rotate_covariance, rotate_vector, rpy_matrix


def test_identity_rotation():
    rotation = rpy_matrix(0.0, 0.0, 0.0)
    assert rotate_vector(rotation, [1.0, 2.0, 3.0]) == [1.0, 2.0, 3.0]


def test_yaw_quarter_turn():
    rotation = rpy_matrix(0.0, 0.0, math.pi / 2.0)
    output = rotate_vector(rotation, [1.0, 0.0, 0.0])
    assert abs(output[0]) < 1e-9
    assert abs(output[1] - 1.0) < 1e-9


def test_isotropic_covariance_is_unchanged():
    rotation = rpy_matrix(0.2, -0.4, 0.8)
    output = rotate_covariance(rotation, [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0])
    expected = [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0]
    assert all(abs(a - b) < 1e-9 for a, b in zip(output, expected))
