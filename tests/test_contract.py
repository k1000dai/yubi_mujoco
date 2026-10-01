"""Wire contract and physical-integrity regressions, without policy privileges."""

import json
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import pytest
from numpy.testing import assert_allclose, assert_array_equal
from scipy.spatial.transform import Rotation

from yubi_mujoco.env import HANDS, SimConfig, YubiEnv
from yubi_mujoco.geometry import relative_pose, xyzw_to_wxyz
from yubi_mujoco.model import ASSETS, JAW_HARD_STOP, JAW_PAD_CONTACT

RELATIVE_KEY = "observation.pose.left_hand_root_to_right_hand_root.absolute"


@pytest.fixture
def env():
    with YubiEnv() as instance:
        yield instance


def identity_action(env):
    action = np.zeros(16)
    action[[6, 13]] = 1.0
    action[14:] = env.target_grippers
    return action


def test_observation_has_interhand7_and_absolute_motor2_only(env):
    observation = env.observe(images=False)
    assert set(observation) == {RELATIVE_KEY, "observation.joint_states", "prompt"}
    relative = observation[RELATIVE_KEY]
    joints = observation["observation.joint_states"]
    assert relative.shape == (7,) and relative.dtype == np.float32
    assert joints.shape == (2,) and joints.dtype == np.float32
    assert np.r_[relative, joints].shape == (9,)
    assert isinstance(observation["prompt"], str) and observation["prompt"]
    assert_allclose(relative, relative_pose(*env.eef_poses), atol=2e-7)
    assert_allclose(joints, env.grippers, atol=1e-7)
    assert "observation.state" not in observation
    assert "object_positions" not in observation


def test_interhand_observation_uses_rotated_left_frame(env):
    # Controlled setup verifies public observation order and MuJoCo wxyz conversion.
    values = np.array(
        [
            np.r_[
                0.08, 0.15, 0.3, Rotation.from_euler("xyz", [20, 35, -40], degrees=True).as_quat()
            ],
            np.r_[
                -0.08,
                -0.15,
                0.27,
                Rotation.from_euler("xyz", [-30, 25, 70], degrees=True).as_quat(),
            ],
        ]
    )
    for hand, value in zip(HANDS, values):
        adr = env.model.joint(f"{hand}_free").qposadr[0]
        env.data.qpos[adr : adr + 7] = np.r_[value[:3], xyzw_to_wxyz(value[3:])]
    mujoco.mj_forward(env.model, env.data)
    assert_allclose(env.eef_poses, values, atol=1e-12)
    assert_allclose(env.observe()[RELATIVE_KEY], relative_pose(*values), atol=1e-7)


@pytest.mark.rendering
def test_rgb_wrist_images_have_official_wire_shape(env):
    observation = env.observe(images=True)
    for hand in HANDS:
        image = observation[f"observation.image.{hand}"]
        assert image.shape == (480, 640, 3)
        assert image.dtype == np.uint8
        assert np.ptp(image) > 0
    assert not any("center" in key for key in observation)


@pytest.mark.parametrize(
    "bad",
    [
        np.zeros(15),
        np.zeros(17),
        np.zeros((1, 16)),
        np.full(16, np.nan),
        np.full(16, np.inf),
        np.zeros(16),
    ],
)
def test_invalid_delta_action_rejected_without_advancing(env, bad):
    before, time, targets = env.data.qpos.copy(), env.data.time, env.targets.copy()
    with pytest.raises(ValueError):
        env.step(bad)
    assert_array_equal(env.data.qpos, before)
    assert_array_equal(env.targets, targets)
    assert env.data.time == time and env.steps == 0


@pytest.mark.parametrize(
    "bad", [[0.5], [0.5, 0.5, 0.5], [np.nan, 0.5], [np.inf, 0.5], [-0.01, 0.5], [0.95, 0.5]]
)
def test_invalid_grippers_rejected_before_physics(env, bad):
    before = env.data.time
    with pytest.raises(ValueError):
        env.step_absolute(env.targets, bad)
    assert env.data.time == before and env.steps == 0


@pytest.mark.parametrize("scale,offset", [(2.0, -0.2), (-2.0, 0.8), (0.5, 0.1)])
def test_motor_scale_offset_and_absolute_semantics(scale, offset):
    with YubiEnv(SimConfig(motor_scale=scale, motor_offset=offset)) as env:
        jaw = np.array([0.2, 0.8])
        motor = (jaw - offset) / scale
        assert_allclose(env.motor_for_jaw(jaw), motor)
        assert_allclose(env._motor_to_jaw(motor), jaw)
        env.step_absolute(env.targets, motor)
        assert_allclose(env.data.ctrl, jaw)
        env.step_absolute(env.targets, motor)
        assert_allclose(env.data.ctrl, jaw)  # Repeat is absolute, not accumulated.
        for bad_jaw in ([-0.02, 0.5], [env.config.joint_max + 0.02, 0.5]):
            with pytest.raises(ValueError):
                env._motor_to_jaw(env.motor_for_jaw(bad_jaw))


def test_reset_reproducible_and_seed_changes_layout(env):
    obs_a, info_a = env.reset(seed=721)
    qpos_a, goals_a = env.data.qpos.copy(), env.goals.copy()
    env.step(identity_action(env))
    obs_b, info_b = env.reset(seed=721)
    assert_array_equal(env.data.qpos, qpos_a)
    assert_array_equal(env.goals, goals_a)
    assert_array_equal(obs_a[RELATIVE_KEY], obs_b[RELATIVE_KEY])
    assert_array_equal(obs_a["observation.joint_states"], obs_b["observation.joint_states"])
    assert info_a["simulation_time"] == info_b["simulation_time"] == 0
    assert env.steps == 0
    env.reset(seed=722)
    assert not np.array_equal(env.goals, goals_a)
    assert not np.array_equal(env.data.qpos, qpos_a)


@pytest.mark.parametrize("hz,expected_substeps", [(10, 60), (30, 20), (60, 10)])
def test_control_time_is_exact_integer_physics_steps(hz, expected_substeps):
    with YubiEnv(SimConfig(control_hz=hz)) as env:
        assert env._substeps == expected_substeps
        start = env.data.time
        action = identity_action(env)
        for _ in range(hz):
            env.step(action)
        assert env.data.time - start == pytest.approx(1.0, abs=2e-12)
        assert env.info()["simulation_time"] == pytest.approx(1.0, abs=2e-12)
        assert env.steps == hz


def test_replaying_same_rows_at_10_hz_is_three_times_30_hz():
    elapsed = {}
    for hz in (10, 30):
        with YubiEnv(SimConfig(control_hz=hz)) as env:
            for _ in range(6):
                env.step(identity_action(env))
            elapsed[hz] = env.info()["simulation_time"]
    assert elapsed[10] == pytest.approx(3 * elapsed[30], abs=1e-12)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"physics_dt": float("inf")},
        {"physics_dt": float("nan")},
        {"physics_dt": 0},
        {"physics_dt": 0.003},
        {"control_hz": 15},
        {"motor_scale": 0},
        {"motor_scale": float("inf")},
        {"motor_offset": float("nan")},
        {"object_mass": 0},
    ],
)
def test_invalid_config_fails_before_model_construction(kwargs):
    with pytest.raises(ValueError):
        SimConfig(**kwargs)


def test_step_does_not_directly_mutate_any_freejoint_qpos(env, monkeypatch):
    # Freeze integration; direct teleport/attachment writes would then be visible.
    original = env.data.qpos.copy()
    equality_original = env.data.eq_active.copy()
    model_equality = env.model.eq_data.copy()
    calls = []

    def frozen_step(model, data):
        calls.append(1)
        assert_array_equal(data.qpos, original)

    monkeypatch.setattr(mujoco, "mj_step", frozen_step)
    target = env.targets.copy()
    target[:, 0] += 0.015
    env.step_absolute(target, env.motor_for_jaw([0.1, 0.2]))
    assert len(calls) == env._substeps
    assert_array_equal(env.data.qpos, original)
    assert_array_equal(env.data.eq_active, equality_original)
    assert_array_equal(env.model.eq_data, model_equality)
    assert not np.array_equal(env.data.mocap_pos, env.eef_poses[:, :3])


def test_only_hand_servos_and_jaw_coupling_are_equality_constraints(env):
    equality = ET.fromstring(env.xml).find("equality")
    assert equality is not None
    assert len(equality) == 4
    for constraint in equality:
        assert constraint.tag in {"weld", "joint"}
        assert not any("object" in value for value in constraint.attrib.values())
    for hand in HANDS:
        gear = equality.find(f"joint[@name='{hand}_gear']")
        assert gear is not None
        assert gear.get("joint1") == f"{hand}_left_joint"
        assert gear.get("joint2") == f"{hand}_right_joint"
        assert_allclose(np.fromstring(gear.get("polycoef"), sep=" "), [0, -1, 0, 0, 0])
    assert env.model.nu == 2
    assert env.model.joint("object_0_free").type[0] == mujoco.mjtJoint.mjJNT_FREE


def test_both_jaws_actually_move_with_opposite_equal_angles(env):
    for target in (0.12, 0.75):
        for _ in range(30):
            env.step_absolute(env.targets, env.motor_for_jaw([target, target]))
        for hand in HANDS:
            right = env.data.qpos[env.model.joint(f"{hand}_right_joint").qposadr[0]]
            left = env.data.qpos[env.model.joint(f"{hand}_left_joint").qposadr[0]]
            assert right == pytest.approx(target, abs=2e-3)
            assert left == pytest.approx(-right, abs=2e-3)


def _jaw(env, hand="left"):
    return env.data.qpos[env.model.joint(f"{hand}_right_joint").qposadr[0]]


def test_jaws_stop_at_pad_contact_and_cad_opening_stop(env):
    # Glove commands span 0..0.94, but the CAD jaws only travel 0.03..0.80.
    for command, stop in ((0.0, JAW_PAD_CONTACT), (0.94, JAW_HARD_STOP)):
        for _ in range(30):
            env.step_absolute(env.targets, env.motor_for_jaw([command, command]))
        for hand in HANDS:
            assert _jaw(env, hand) == pytest.approx(stop, abs=3e-3)


def test_jaw_zero_is_closed_pose_of_yubi_sw_urdf(env):
    # q=0 is the CAD-parallel jaw turned 7.5 deg closed, as in yubi_hand.urdf.xacro.
    for side, sign in (("right", 1), ("left", -1)):
        quat = env.model.body(f"left_{side}_finger").quat
        assert 2 * np.arctan2(quat[3], quat[0]) == pytest.approx(sign * np.radians(7.5))


def test_jaw_speed_respects_servo_no_load_speed(env):
    dof = env.model.joint("left_right_joint").dofadr[0]
    speeds = []
    for command in (0.0, 0.94, 0.0):
        for _ in range(30):
            env.step_absolute(env.targets, env.motor_for_jaw([command, command]))
            speeds.append(abs(env.data.qvel[dof]))
    assert max(speeds) <= env.config.gripper_speed
    assert max(speeds) > 0.5 * env.config.gripper_speed


def test_inertials_come_from_cad_mass_properties(env):
    bodies = json.loads((ASSETS / "mass_properties.json").read_text())["bodies"]
    for body, key in (
        ("left_hand_root", "palm"),
        ("left_right_finger", "right_jaw"),
        ("left_left_finger", "left_jaw"),
    ):
        assert env.model.body(body).mass[0] == pytest.approx(bodies[key]["mass_kg"])
    hand = env.model.body("left_hand_root").id
    assert 0.5 < env.model.body_subtreemass[hand] < 0.56


def test_velocity_limiter_is_physical_per_second(env):
    initial = env.targets.copy()
    target = initial.copy()
    target[:, 0] += 0.2
    _, _, _, _, info = env.step_absolute(target, env.target_grippers)
    displacement = np.linalg.norm(env.targets[:, :3] - initial[:, :3], axis=1)
    assert_allclose(displacement, env.config.max_translation_speed / env.config.control_hz)
    assert info["target_rate_limited"] == [True, True]


def test_terminal_episode_requires_reset():
    with YubiEnv(SimConfig(horizon=1)) as env:
        _, _, terminated, truncated, _ = env.step(identity_action(env))
        assert truncated
        with pytest.raises(RuntimeError, match="reset"):
            env.step(identity_action(env))
        env.reset(seed=0)
        assert env.steps == 0
        env.step(identity_action(env))


@pytest.mark.parametrize("maximum", [0.3, 0.5, 0.94])
def test_reset_respects_custom_nominal_jaw_limits(maximum):
    with YubiEnv(SimConfig(joint_max=maximum)) as env:
        assert np.all(env.data.ctrl >= 0)
        assert np.all(env.data.ctrl <= maximum)
        env.step_absolute(env.targets, env.target_grippers)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"horizon": float("nan")},
        {"horizon": float("inf")},
        {"horizon": 1.2},
        {"camera_fovy": float("nan")},
        {"camera_fovy": float("inf")},
        {"camera_fovy": 0},
        {"camera_fovy": 180},
    ],
)
def test_config_requires_finite_horizon_and_physical_camera_fov(kwargs):
    with pytest.raises(ValueError):
        SimConfig(**kwargs)


def test_delta_control_composes_from_previous_target_not_tracking_error(env):
    # Use sufficiently small moves to avoid deliberate velocity clipping.
    initial = env.targets.copy()
    action = identity_action(env)
    action[:3] = [0.002, -0.001, 0.003]
    action[7:10] = [-0.001, 0.002, 0.001]
    action[3:7] = Rotation.from_euler("x", 0.01).as_quat()
    action[10:14] = Rotation.from_euler("z", -0.01).as_quat()
    for _ in range(3):
        old = env.targets.copy()
        env.step(action)
        for hand in range(2):
            part = action[hand * 7 : hand * 7 + 7]
            r = Rotation.from_quat(old[hand, 3:])
            assert_allclose(env.targets[hand, :3], old[hand, :3] + r.apply(part[:3]), atol=1e-12)
            assert_allclose(
                Rotation.from_quat(env.targets[hand, 3:]).as_matrix(),
                r.as_matrix() @ Rotation.from_quat(part[3:]).as_matrix(),
                atol=1e-12,
            )
    assert not np.allclose(env.targets, initial)


@pytest.mark.rendering
def test_hold_policy_emits_full_valid_chunk_without_privileged_state(env):
    from yubi_mujoco.adapter import HoldPolicy, validate_chunk

    observation = env.observe(images=True)
    actions = HoldPolicy().infer(observation)
    assert actions.shape == (32, 16) and actions.dtype == np.float32
    assert_allclose(actions[:, [6, 13]], 1)
    assert_allclose(actions[:, 14:16], np.tile(observation["observation.joint_states"], (32, 1)))
    assert validate_chunk(actions).shape == actions.shape


@pytest.mark.parametrize(
    "bad",
    [
        np.zeros((15, 16)),
        np.zeros((16, 15)),
        np.zeros(16),
        np.full((16, 16), np.nan),
        np.full((16, 16), np.inf),
        np.zeros((16, 16)),
        np.full((16, 16), "bad"),
    ],
)
def test_invalid_policy_chunks_rejected(bad):
    from yubi_mujoco.adapter import validate_chunk

    with pytest.raises(ValueError):
        validate_chunk(bad)


def test_complex_actions_rejected_before_lossy_conversion():
    from yubi_mujoco.adapter import validate_chunk

    actions = np.zeros((16, 16), dtype=complex)
    actions[:, [6, 13]] = 1
    actions[0, 0] = 1j
    with pytest.raises(ValueError):
        validate_chunk(actions)


@pytest.mark.rendering
def test_policy_receives_exact_keys_and_only_adopted_rows_execute(env):
    from yubi_mujoco.adapter import run_policy

    calls = []

    class Policy:
        def infer(self, obs):
            calls.append(obs)
            assert set(obs) == {
                RELATIVE_KEY,
                "observation.joint_states",
                "prompt",
                "observation.image.left",
                "observation.image.right",
            }
            for hand in HANDS:
                assert obs[f"observation.image.{hand}"].shape == (480, 640, 3)
            result = np.tile(identity_action(env), (32, 1))
            # Unadopted gripper values must not be applied or rejected as physical commands.
            result[3:, 14:] = 1000
            return result

    initial = env.targets.copy()
    report = run_policy(env, Policy(), adopt_rows=3, max_calls=2)
    assert report["status"] == "call_limit"
    assert report["policy_calls"] == 2
    assert env.steps == 6 and len(calls) == 2
    assert_allclose(env.targets, initial, atol=1e-12)
    assert report["simulation_time"] == pytest.approx(6 / env.config.control_hz)
    assert report["camera_calibrated"] is False
    assert report["motor_mapping_calibrated"] is False


@pytest.mark.rendering
def test_invalid_adopted_gripper_rejects_chunk_before_partial_execution(env):
    from yubi_mujoco.adapter import run_policy

    class Policy:
        def infer(self, obs):
            result = np.tile(identity_action(env), (32, 1))
            result[2, 14] = 1000
            return result

    report = run_policy(env, Policy(), adopt_rows=3, max_calls=1)
    assert report["status"] == "invalid_policy_output"
    assert env.steps == 0
    assert report["error"]


@pytest.mark.rendering
def test_policy_rollout_honors_horizon_midchunk():
    from yubi_mujoco.adapter import HoldPolicy, run_policy

    with YubiEnv(SimConfig(horizon=3)) as env:
        report = run_policy(env, HoldPolicy(), adopt_rows=16)
        assert report["status"] == "horizon"
        assert report["steps"] == 3
        assert report["policy_calls"] == 1


@pytest.mark.rendering
def test_finite_padding_outside_executed_rows_is_accepted(env):
    from yubi_mujoco.adapter import run_policy, validate_chunk

    padded = np.zeros((32, 16), dtype=np.float32)
    padded[:3] = identity_action(env)
    assert validate_chunk(padded, executed_rows=3).shape == (32, 16)
    with pytest.raises(ValueError):
        validate_chunk(padded, executed_rows=4)

    class Policy:
        def infer(self, obs):
            return padded

    report = run_policy(env, Policy(), adopt_rows=3, max_calls=1)
    assert report["status"] == "call_limit" and report["steps"] == 3


@pytest.mark.parametrize(
    "kwargs",
    [
        {"adopt_rows": 0},
        {"adopt_rows": -1},
        {"adopt_rows": 1.5},
        {"max_calls": 0},
        {"max_calls": -1},
        {"max_calls": 1.5},
    ],
)
def test_policy_rollout_requires_positive_integer_counts(env, kwargs):
    from yubi_mujoco.adapter import HoldPolicy, run_policy

    with pytest.raises(ValueError):
        run_policy(env, HoldPolicy(), **kwargs)
    assert env.steps == 0


@pytest.mark.rendering
def test_adoption_cannot_exceed_returned_chunk(env):
    from yubi_mujoco.adapter import run_policy

    class Policy:
        def infer(self, obs):
            return np.tile(identity_action(env), (16, 1))

    report = run_policy(env, Policy(), adopt_rows=17, max_calls=1)
    assert report["status"] == "invalid_policy_output"
    assert env.steps == 0


def test_cli_export_is_portable_and_loadable(tmp_path):
    from yubi_mujoco.cli import main

    assert main(["export-mjcf", "--output", str(tmp_path), "--task", "dual_pick_place"]) == 0
    xml_path = tmp_path / "scene.xml"
    text = xml_path.read_text()
    assert 'meshdir="meshes"' in text
    assert str(tmp_path) not in text
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    assert model.body("object_1").id > 0 and model.nu == 2
    assert (tmp_path / "meshes" / "cad_palm_structure.stl").exists()


@pytest.mark.rendering
def test_cli_hold_evaluation_writes_machine_readable_report(tmp_path):
    import json
    from yubi_mujoco.cli import main

    assert (
        main(
            [
                "evaluate",
                "--policy",
                "hold",
                "--horizon",
                "2",
                "--hz",
                "10",
                "--output",
                str(tmp_path),
            ]
        )
        == 0
    )
    report = json.loads((tmp_path / "report.json").read_text())
    assert report["configuration"]["control_hz"] == 10
    assert report["successes"] == 0 and report["success_rate"] == 0
    episode = report["episodes"][0]
    assert episode["status"] == "horizon" and episode["steps"] == 2
    assert episode["simulation_time"] == pytest.approx(0.2)
    trace = json.loads((tmp_path / "trace_seed_0.json").read_text())
    assert len(trace) == 2
