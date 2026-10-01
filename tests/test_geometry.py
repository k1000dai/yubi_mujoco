"""Independent SE(3) checks against the pinned official replay equations.

The published replay contract uses xyzw, right-multiplied rotations and, in
body mode, translation expressed in the previous pose.  Matrix products below
are independent of the simulator's quaternion composition implementation.
"""

import numpy as np
import pytest
from numpy.testing import assert_allclose
from scipy.spatial.transform import Rotation

from yubi_mujoco.geometry import (
    compose_delta,
    delta_between,
    interpolate,
    pose,
    relative_pose,
    wxyz_to_xyzw,
    xyzw_to_wxyz,
)


def matrix(value):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(value[3:]).as_matrix()
    result[:3, 3] = value[:3]
    return result


def assert_same_pose(actual, expected, atol=1e-12):
    assert_allclose(matrix(actual), matrix(expected), atol=atol)


def test_xyzw_wxyz_roundtrip_keeps_distinct_components():
    q = np.array([1.0, 2.0, 3.0, 4.0]) / np.sqrt(30.0)
    assert_allclose(xyzw_to_wxyz(q), q[[3, 0, 1, 2]])
    assert_allclose(wxyz_to_xyzw(xyzw_to_wxyz(q)), q)
    assert_allclose(xyzw_to_wxyz([0, 0, 0, 1]), [1, 0, 0, 0])


def test_pose_normalizes_without_mutating_input():
    raw = np.array([0.1, 0.2, 0.3, 0.4, -0.6, 0.8, 1.0])
    original = raw.copy()
    actual = pose(raw)
    assert_allclose(np.linalg.norm(actual[3:]), 1.0)
    assert_allclose(actual[:3], original[:3])
    assert_allclose(raw, original)
    assert not np.shares_memory(actual, raw)


@pytest.mark.parametrize(
    "invalid",
    [
        [0] * 6,
        [0] * 8,
        [[0, 0, 0, 0, 0, 0, 1]],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1e-10],
        [np.nan, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, np.inf, 0, 0, 1],
        [0, 0, 0, 1e308, 0, 0, 0],
    ],
)
def test_invalid_pose_rejected(invalid):
    with np.errstate(over="ignore", invalid="ignore"):
        with pytest.raises(ValueError):
            pose(invalid)


def test_body_and_world_translation_with_rotated_nonidentity_anchor():
    anchor = np.r_[0.3, -0.2, 0.4, Rotation.from_euler("z", 90, degrees=True).as_quat()]
    delta = np.array([0.01, 0, 0, 0, 0, 0, 1.0])
    body = compose_delta(anchor, delta, "body")
    world = compose_delta(anchor, delta, "world")
    assert_allclose(body[:3], [0.3, -0.19, 0.4], atol=1e-12)
    assert_allclose(world[:3], [0.31, -0.2, 0.4], atol=1e-12)


@pytest.mark.parametrize("frame", ["body", "world"])
def test_noncommuting_rotation_right_multiplies_for_both_frames(frame):
    anchor = np.r_[
        0.2, -0.3, 0.4, Rotation.from_euler("xyz", [31, -22, 47], degrees=True).as_quat()
    ]
    delta = np.r_[
        0.01, -0.03, 0.02, Rotation.from_euler("xyz", [-12, 19, 8], degrees=True).as_quat()
    ]
    actual = matrix(compose_delta(anchor, delta, frame))
    expected = matrix(anchor) @ matrix(delta)
    if frame == "world":
        expected[:3, 3] = anchor[:3] + delta[:3]
    assert_allclose(actual, expected, atol=1e-12)
    # This case must distinguish the common wrong left-multiply implementation.
    assert not np.allclose(actual[:3, :3], matrix(delta)[:3, :3] @ matrix(anchor)[:3, :3])


@pytest.mark.parametrize("frame", ["body", "world"])
def test_random_composition_matches_official_replay_equations(frame):
    rng = np.random.default_rng(1429)
    current = np.r_[rng.normal(size=3), Rotation.random(random_state=rng).as_quat()]
    expected = matrix(current)
    for _ in range(50):
        delta = np.r_[
            rng.normal(scale=0.02, size=3),
            Rotation.from_rotvec(rng.normal(scale=0.1, size=3)).as_quat(),
        ]
        rotation_before = expected[:3, :3].copy()
        expected[:3, 3] += rotation_before @ delta[:3] if frame == "body" else delta[:3]
        expected[:3, :3] = rotation_before @ matrix(delta)[:3, :3]
        current = compose_delta(current, delta, frame)
        assert_allclose(matrix(current), expected, atol=2e-12)


@pytest.mark.parametrize("frame", ["body", "world"])
def test_delta_between_is_inverse_of_composition(frame):
    rng = np.random.default_rng(123)
    for _ in range(20):
        before = np.r_[rng.normal(size=3), Rotation.random(random_state=rng).as_quat()]
        after = np.r_[rng.normal(size=3), Rotation.random(random_state=rng).as_quat()]
        delta = delta_between(before, after, frame)
        assert_same_pose(compose_delta(before, delta, frame), after)


def test_interhand_is_right_expressed_in_left_frame():
    left = np.r_[0.15, -0.3, 0.2, Rotation.from_euler("xyz", [15, 45, -30], degrees=True).as_quat()]
    right = np.r_[
        -0.2, 0.3, 0.25, Rotation.from_euler("xyz", [-10, 23, 70], degrees=True).as_quat()
    ]
    expected = np.linalg.inv(matrix(left)) @ matrix(right)
    assert_allclose(matrix(relative_pose(left, right)), expected, atol=1e-12)
    assert not np.allclose(expected[:3, 3], right[:3] - left[:3])


@pytest.mark.parametrize("operation", [compose_delta, delta_between])
def test_invalid_translation_frame_rejected(operation):
    identity = [0, 0, 0, 0, 0, 0, 1]
    with pytest.raises(ValueError):
        operation(identity, identity, "camera")


def test_interpolation_takes_shortest_rotation_and_preserves_endpoints():
    a = np.r_[0.0, 0.0, 0.0, Rotation.from_euler("z", 170, degrees=True).as_quat()]
    b = np.r_[2.0, 4.0, 6.0, Rotation.from_euler("z", -170, degrees=True).as_quat()]
    assert_same_pose(interpolate(a, b, 0), a)
    assert_same_pose(interpolate(a, b, 1), b)
    midpoint = interpolate(a, b, 0.5)
    assert_allclose(midpoint[:3], [1, 2, 3])
    assert_allclose(Rotation.from_quat(midpoint[3:]).magnitude(), np.pi)
