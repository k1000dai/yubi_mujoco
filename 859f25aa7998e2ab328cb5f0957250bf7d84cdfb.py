"""End-to-end contact regression, deliberately separate from learned-policy claims."""

import numpy as np
import pytest
from yubi_mujoco import YubiEnv, SimConfig
from yubi_mujoco.scripted import oracle_rollout


@pytest.mark.parametrize("task", ["pick_place", "lift", "push", "dual_pick_place"])
@pytest.mark.parametrize("seed", [0, 3])
def test_scripted_contact_task_success(task, seed):
    with YubiEnv(SimConfig(task=task)) as env:
        env.reset(seed=seed)
        info, history = oracle_rollout(env)
        assert info["success"]
        assert info["success_now"]
        assert np.isfinite(env.data.qpos).all()
        assert len(history) == info["steps"]
        if task in ("pick_place", "dual_pick_place"):
            assert all(info["bilateral_contact_seen"])
            assert all(info["lifted"])
            assert not any(info["touching_gripper"])
            assert max(info["goal_distance_xy"]) < 0.01
            assert min(info["max_object_height"]) > 0.12
        if task == "push":
            assert not any(info["lifted"])
            assert max(info["goal_distance_xy"]) < 0.015


def test_open_hands_do_not_magically_lift_object():
    # The same reach/lift without closing must not attach the object.
    with YubiEnv() as env:
        p = env.targets.copy()
        p[0, :3] = [*env.object_positions[0, :2], 0.10]
        for _ in range(50):
            env.step_absolute(p, env.motor_for_jaw([0.55, 0.55]))
        p[0, 2] = 0.27
        for _ in range(50):
            env.step_absolute(p, env.motor_for_jaw([0.55, 0.55]))
        assert env.object_positions[0, 2] < 0.03
        assert not env.info()["success"]
