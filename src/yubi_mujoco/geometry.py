"""SE(3), SI metres and xyzw quaternions at every public API boundary."""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def pose(value):
    p = np.asarray(value, dtype=np.float64)
    if p.shape != (7,) or not np.isfinite(p).all():
        raise ValueError("pose must be seven finite values [x,y,z,qx,qy,qz,qw]")
    n = np.linalg.norm(p[3:])
    if not np.isfinite(n) or n < 1e-8:
        raise ValueError("zero quaternion is invalid; identity is [0,0,0,1]")
    p = p.copy()
    p[3:] /= n
    return p


def compose_delta(current, delta, translation_frame="body"):
    """Right-composed rotation, previous-pose translation, matching official replay."""
    current, delta = pose(current), pose(delta)
    r = Rotation.from_quat(current[3:])
    if translation_frame not in ("body", "world"):
        raise ValueError("translation_frame must be body or world")
    dp = r.apply(delta[:3]) if translation_frame == "body" else delta[:3]
    return np.r_[current[:3] + dp, (r * Rotation.from_quat(delta[3:])).as_quat()]


def relative_pose(left, right):
    left, right = pose(left), pose(right)
    r = Rotation.from_quat(left[3:])
    return np.r_[
        r.inv().apply(right[:3] - left[:3]), (r.inv() * Rotation.from_quat(right[3:])).as_quat()
    ]


def delta_between(before, after, translation_frame="body"):
    result = relative_pose(before, after)
    if translation_frame == "world":
        result[:3] = pose(after)[:3] - pose(before)[:3]
    elif translation_frame != "body":
        raise ValueError("translation_frame must be body or world")
    return result


def interpolate(a, b, fraction):
    a, b = pose(a), pose(b)
    q = Slerp([0, 1], Rotation.from_quat([a[3:], b[3:]]))([float(fraction)]).as_quat()[0]
    return np.r_[a[:3] * (1 - fraction) + b[:3] * fraction, q]


def xyzw_to_wxyz(q):
    return np.asarray(q)[[3, 0, 1, 2]]


def wxyz_to_xyzw(q):
    return np.asarray(q)[[1, 2, 3, 0]]
