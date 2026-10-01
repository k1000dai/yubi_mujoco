"""Finite-compliance floating hands; all manipulated objects use real MuJoCo contact."""

from dataclasses import dataclass
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from .geometry import compose_delta, interpolate, pose, relative_pose, xyzw_to_wxyz, wxyz_to_xyzw
from .model import make_model_xml

HANDS = ("left", "right")
TASKS = ("pick_place", "push", "lift", "dual_pick_place")


@dataclass(frozen=True)
class SimConfig:
    task: str = "pick_place"
    control_hz: int = 30
    physics_dt: float = 1 / 600
    horizon: int = 900
    object_mass: float = 0.05
    friction: float = 1.2
    joint_max: float = 0.94
    motor_scale: float = 1.0  # jaw radians = scale * contract motor radians + offset
    motor_offset: float = 0.0  # nominal, not a calibrated real-robot conversion
    gripper_kp: float = 4.0
    gripper_torque: float = 1.0
    max_translation_speed: float = 0.6
    max_rotation_speed: float = 3.0
    camera_fovy: float = 90.0

    def __post_init__(self):
        if self.task not in TASKS:
            raise ValueError(f"task must be one of {TASKS}")
        if self.control_hz not in (10, 30, 60):
            raise ValueError(
                "control_hz must be 10, 30, or 60; dataset replay is 30Hz, robot nominal10Hz"
            )
        if (
            not np.isfinite(self.physics_dt)
            or self.physics_dt <= 0
            or abs(
                1 / self.control_hz / self.physics_dt - round(1 / self.control_hz / self.physics_dt)
            )
            > 1e-7
        ):
            raise ValueError("physics_dt must divide the control interval exactly")
        for k in (
            "object_mass",
            "friction",
            "joint_max",
            "gripper_kp",
            "gripper_torque",
            "max_translation_speed",
            "max_rotation_speed",
        ):
            if not np.isfinite(getattr(self, k)) or getattr(self, k) <= 0:
                raise ValueError(f"{k} must be positive and finite")
        if (
            not np.isfinite(self.motor_scale)
            or self.motor_scale == 0
            or not np.isfinite(self.motor_offset)
        ):
            raise ValueError("motor scale must be nonzero and calibration finite")
        if not np.isfinite(self.camera_fovy) or not 1 < self.camera_fovy < 179:
            raise ValueError("camera_fovy must lie between1 and179 degrees")
        if not isinstance(self.horizon, int) or self.horizon <= 0:
            raise ValueError("horizon must be positive")


class YubiEnv:
    """Gym-like reset/step without optional Gym dependency.

    Public pose = hand root [x,y,z,qx,qy,qz,qw] in metres/world coordinates.
    Gripper = absolute motor radians, NOT width, delta, or normalized [0,1].
    MuJoCo qpos quaternions are wxyz and converted internally.
    Object qpos is initialized only by reset; never changed by actions.
    """

    def __init__(self, config=None, *, render_images=False):
        self.config = config or SimConfig()
        self.xml = make_model_xml(self.config)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.render_images = render_images
        self._renderer = None
        self._render_size = None
        self._substeps = round(1 / self.config.control_hz / self.config.physics_dt)
        self._root_ids = [self.model.body(f"{h}_hand_root").id for h in HANDS]
        self._mocap_ids = [self.model.body(f"{h}_target").mocapid[0] for h in HANDS]
        self._motor_joints = [self.model.joint(f"{h}_right_joint").id for h in HANDS]
        self._object_ids = [
            self.model.body(f"object_{i}").id
            for i in range(2 if self.config.task == "dual_pick_place" else 1)
        ]
        self.reset()

    def _motor_to_jaw(self, values):
        a = np.asarray(values, dtype=float)
        if a.shape != (2,) or not np.isfinite(a).all():
            raise ValueError("grippers must contain two finite absolute motor positions in radians")
        jaw = a * self.config.motor_scale + self.config.motor_offset
        if np.any(jaw < -1e-7) or np.any(jaw > self.config.joint_max + 1e-7):
            raise ValueError(
                f"motor commands map outside nominal jaw range [0,{self.config.joint_max}]; check calibration"
            )
        return np.clip(jaw, 0, self.config.joint_max)

    def motor_for_jaw(self, jaw):
        return (np.asarray(jaw) - self.config.motor_offset) / self.config.motor_scale

    @property
    def eef_poses(self):
        return np.array(
            [np.r_[self.data.xpos[i], wxyz_to_xyzw(self.data.xquat[i])] for i in self._root_ids]
        )

    @property
    def object_positions(self):
        return self.data.xpos[self._object_ids].copy()

    @property
    def grippers(self):
        jaw = np.array([self.data.qpos[self.model.jnt_qposadr[j]] for j in self._motor_joints])
        return self.motor_for_jaw(jaw)

    def reset(self, *, seed=0, options=None):
        self.seed = int(seed)
        self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        self.steps = 0
        self._success_hold = 0
        self._done = False
        self._lifted = np.zeros(len(self._object_ids), dtype=bool)
        self._max_height = np.zeros(len(self._object_ids))
        self._bilateral_contact_seen = np.zeros(len(self._object_ids), dtype=bool)
        self.goals = []
        for i in range(len(self._object_ids)):
            base_y = 0.16 if i == 0 else -0.16
            pos = np.array([-0.10, base_y, 0.019])
            pos[:2] += self.rng.uniform(-0.025, 0.025, size=2)
            joint = self.model.joint(f"object_{i}_free")
            self.data.qpos[joint.qposadr[0] : joint.qposadr[0] + 3] = pos
            goal = np.array([0.17, base_y, 0.018])
            goal[:2] += self.rng.uniform(-0.02, 0.02, size=2)
            self.goals.append(goal)
            self.model.site_pos[self.model.site(f"goal_{i}").id] = [*goal[:2], 0.001]
        self.goals = np.array(self.goals)
        open_q = min(0.55, 0.9 * self.config.joint_max)
        for hand in HANDS:
            for side, sign in (("right", 1), ("left", -1)):
                self.data.qpos[self.model.joint(f"{hand}_{side}_joint").qposadr[0]] = sign * open_q
        self.data.ctrl[:] = open_q
        for _ in range(120):
            mujoco.mj_step(self.model, self.data)
        self.targets = self.eef_poses.copy()
        self.target_grippers = self.motor_for_jaw(np.array([open_q, open_q]))
        self._initial_objects = self.object_positions.copy()
        self._initial_time = self.data.time
        return self.observe(), self.info()

    def observe(self, *, images=None):
        """Official wire keys only. Privileged world/object state is in info()."""
        p = self.eef_poses
        obs = {
            "observation.pose.left_hand_root_to_right_hand_root.absolute": relative_pose(
                p[0], p[1]
            ).astype(np.float32),
            "observation.joint_states": self.grippers.astype(np.float32),
            "prompt": {
                "pick_place": "Pick up the red cube and place it on the green target.",
                "push": "Push the red cube onto the green target without lifting it.",
                "lift": "Lift the red cube above the table and hold it.",
                "dual_pick_place": "Use both hands to place each cube on its green target.",
            }[self.config.task],
        }
        if images if images is not None else self.render_images:
            for hand in HANDS:
                obs[f"observation.image.{hand}"] = self.render(
                    f"wrist_{hand}", width=640, height=480
                )
        return obs

    def step(self, action):
        """One official 16-vector: left delta7, right delta7, absolute motors2."""
        return self.step_delta(action)

    def step_delta(self, action, *, translation_frame="body"):
        a = np.asarray(action, dtype=float)
        if a.shape != (16,) or not np.isfinite(a).all():
            raise ValueError("delta action must be a finite 16-vector")
        target = np.array(
            [
                compose_delta(self.targets[i], a[i * 7 : (i + 1) * 7], translation_frame)
                for i in range(2)
            ]
        )
        return self.step_absolute(target, a[14:16])

    def step_absolute(self, eef_poses, grippers):
        if self._done:
            raise RuntimeError("episode is finished; call reset before stepping again")
        target = np.asarray(eef_poses, dtype=float)
        if target.shape != (2, 7):
            raise ValueError("eef_poses must be (2,7), hand root poses in world frame")
        target = np.array([pose(p) for p in target])
        if (
            np.any(np.abs(target[:, 0]) > 0.6)
            or np.any(np.abs(target[:, 1]) > 0.4)
            or np.any((target[:, 2] < 0.01) | (target[:, 2] > 0.7))
        ):
            raise ValueError("EEF target outside simulator workspace x±.6,y±.4,z[.01,.7]m")
        jaw = self._motor_to_jaw(grippers)
        current = self.targets.copy()
        applied = target.copy()
        clipped = []
        for i in range(2):
            distance = np.linalg.norm(target[i, :3] - current[i, :3])
            angle = (
                Rotation.from_quat(current[i, 3:]).inv() * Rotation.from_quat(target[i, 3:])
            ).magnitude()
            fraction = min(
                1.0,
                self.config.max_translation_speed / self.config.control_hz / max(distance, 1e-12),
                self.config.max_rotation_speed / self.config.control_hz / max(angle, 1e-12),
            )
            applied[i] = interpolate(current[i], target[i], fraction)
            clipped.append(fraction < 1 - 1e-10)
        # Sweep only mocap targets. Dynamic grippers follow finite-compliance welds.
        # No changes to object poses, freejoint qpos, or contact constraints here.
        for k in range(self._substeps):
            for i, mid in enumerate(self._mocap_ids):
                p = interpolate(current[i], applied[i], (k + 1) / self._substeps)
                self.data.mocap_pos[mid] = p[:3]
                self.data.mocap_quat[mid] = xyzw_to_wxyz(p[3:])
            self.data.ctrl[:] = jaw
            mujoco.mj_step(self.model, self.data)
            self._update_contact_history()
        self.targets = applied
        self.target_grippers = np.array(grippers, dtype=float)
        self.steps += 1
        info = self.info()
        self._success_hold = self._success_hold + 1 if info["success_now"] else 0
        terminated = self._success_hold >= max(1, round(self.config.control_hz * 0.4))
        truncated = self.steps >= self.config.horizon or not np.isfinite(self.data.qpos).all()
        self._done = terminated or truncated
        info["success"] = terminated
        info["target_rate_limited"] = clipped
        info["terminated"] = terminated
        info["truncated"] = truncated
        reward = float(terminated) - float(np.mean(info["goal_distance_xy"]))
        return self.observe(), reward, terminated, truncated, info

    def _contacts(self):
        result = []
        for c in self.data.contact[: self.data.ncon]:
            g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1) or ""
            g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2) or ""
            result.append((g1, g2))
        return result

    def _update_contact_history(self):
        heights = self.object_positions[:, 2]
        self._max_height = np.maximum(self._max_height, heights)
        self._lifted |= heights > 0.075
        contacts = self._contacts()
        for i in range(len(self._object_ids)):
            names = [
                a if b == f"object_{i}_geom" else b
                for a, b in contacts
                if f"object_{i}_geom" in (a, b)
            ]
            for hand in HANDS:
                if any(n.startswith(f"{hand}_left_") for n in names) and any(
                    n.startswith(f"{hand}_right_") for n in names
                ):
                    self._bilateral_contact_seen[i] = True

    def info(self):
        positions = self.object_positions
        distances = np.linalg.norm(positions[:, :2] - self.goals[:, :2], axis=1)
        speeds = []
        for i in range(len(self._object_ids)):
            j = self.model.joint(f"object_{i}_free")
            speeds.append(float(np.linalg.norm(self.data.qvel[j.dofadr[0] : j.dofadr[0] + 3])))
        touching = []
        for i in range(len(self._object_ids)):
            touching.append(
                any(
                    f"object_{i}_geom" in c and any(n.startswith(("left_", "right_")) for n in c)
                    for c in self._contacts()
                )
            )
        stable = np.array(speeds) < 0.07
        on_table = np.abs(positions[:, 2] - 0.018) < 0.012
        if self.config.task == "lift":
            success_now = np.all((positions[:, 2] > 0.12) & stable & self._bilateral_contact_seen)
        elif self.config.task == "push":
            success_now = np.all((distances < 0.04) & on_table & stable & ~self._lifted)
        else:
            success_now = np.all(
                (distances < 0.04)
                & on_table
                & stable
                & ~np.array(touching)
                & self._lifted
                & self._bilateral_contact_seen
            )
        return {
            "task": self.config.task,
            "seed": self.seed,
            "steps": self.steps,
            "simulation_time": float(self.data.time - getattr(self, "_initial_time", 0)),
            "object_positions": positions.tolist(),
            "goals": self.goals.tolist(),
            "goal_distance_xy": distances.tolist(),
            "object_speed": speeds,
            "max_object_height": self._max_height.tolist(),
            "lifted": self._lifted.tolist(),
            "bilateral_contact_seen": self._bilateral_contact_seen.tolist(),
            "touching_gripper": touching,
            "success_now": bool(success_now),
            "success": self._success_hold >= round(self.config.control_hz * 0.4),
            "eef_tracking_error_m": np.linalg.norm(
                self.eef_poses[:, :3] - self.targets[:, :3], axis=1
            ).tolist()
            if hasattr(self, "targets")
            else [0, 0],
        }

    def render(self, camera="overview", *, width=960, height=720):
        if self._renderer is None or self._render_size != (width, height):
            if self._renderer is not None:
                self._renderer.close()
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
            self._render_size = (width, height)
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render().copy()

    def save_mjcf(self, path):
        """Write XML; package assets remain referenced by absolute path. See export CLI for portable bundle."""
        Path(path).write_text(self.xml)

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
