"""Privileged-state scripted baselines for testing physics, NOT learned policies."""

import numpy as np
from .geometry import interpolate


def oracle_rollout(env, *, on_step=None):
    """Demonstrate task with pose/gripper commands only, never object writes."""
    history = []
    info = env.info()

    def move(target, jaw, seconds):
        nonlocal info
        start = env.targets.copy()
        for k in range(max(1, round(seconds * env.config.control_hz))):
            if env._done:
                return
            fraction = (k + 1) / max(1, round(seconds * env.config.control_hz))
            p = np.array([interpolate(a, b, fraction) for a, b in zip(start, target)])
            _, reward, terminated, truncated, info = env.step_absolute(p, env.motor_for_jaw(jaw))
            history.append(info.copy())
            if on_step:
                on_step(env, info)

    poses = env.targets.copy()
    opening = min(0.55, 0.9 * env.config.joint_max)
    jaw = np.array([opening, opening])
    if env.config.task == "push":
        # Turn the fingers upward and push with the palm contact proxy.
        # This avoids pinching/lifting and exercises a distinct contact primitive.
        obj = env.object_positions[0]
        poses[0, 3:] = [0, -np.sin(np.pi / 4), 0, np.cos(np.pi / 4)]
        poses[0, :3] = [obj[0] - 0.085, obj[1], 0.24]
        jaw[0] = 0
        move(poses, jaw, 1.4)
        poses[0, 2] = 0.052
        move(poses, jaw, 1.0)
        poses[0, 0] = env.goals[0, 0] - 0.064
        poses[0, 1] = env.goals[0, 1]
        move(poses, jaw, 2.2)
        poses[0, 0] -= 0.08
        move(poses, jaw, 0.6)
        poses[0, 2] = 0.26
        move(poses, jaw, 0.6)
        move(poses, jaw, 0.8)
        return info, history
    count = len(env.object_positions)
    for i in range(count):
        obj = env.object_positions[i]
        poses[i, :3] = [obj[0], obj[1], 0.24]
    move(poses, jaw, 0.8)
    for i in range(count):
        poses[i, 2] = env.object_positions[i, 2] + 0.080
    move(poses, jaw, 1.0)
    for i in range(count):
        jaw[i] = 0
    move(poses, jaw, 0.7)
    for i in range(count):
        poses[i, 2] = 0.27
    move(poses, jaw, 1.4)
    if env.config.task == "lift":
        move(poses, jaw, 0.8)
        return info, history
    for i in range(count):
        poses[i, :2] = env.goals[i, :2]
    move(poses, jaw, 1.8)
    for i in range(count):
        poses[i, 2] = 0.102
    move(poses, jaw, 1.4)
    for i in range(count):
        jaw[i] = opening
    move(poses, jaw, 0.7)
    for i in range(count):
        poses[i, 2] = 0.26
    move(poses, jaw, 1.0)
    move(poses, jaw, 1.0)
    return info, history
