"""Generate explicit MJCF from source-derived geometry and disclosed sim defaults."""

from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from .geometry import xyzw_to_wxyz

ASSETS = Path(__file__).parent / "assets"


def _e(parent, tag, **kwargs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in kwargs.items()})


def _s(x):
    return " ".join(f"{v:.9g}" for v in x)


def make_model_xml(config, *, mesh_dir=None):
    """Return portable MJCF text; caller loads assets from package meshes."""
    root = ET.Element("mujoco", model="YUBI arm-free contact simulation")
    _e(
        root,
        "compiler",
        angle="radian",
        meshdir=str(mesh_dir or ASSETS / "meshes"),
        autolimits="true",
    )
    _e(
        root,
        "option",
        timestep=config.physics_dt,
        gravity="0 0 -9.81",
        integrator="implicitfast",
        cone="elliptic",
        iterations="80",
        noslip_iterations="5",
    )
    visual = _e(root, "visual")
    _e(visual, "global", offwidth="1280", offheight="960")
    _e(visual, "quality", shadowsize="2048")
    _e(visual, "headlight", ambient="0.4 0.4 0.4", diffuse="0.7 0.7 0.7", specular="0.1 0.1 0.1")
    default = _e(root, "default")
    _e(
        default,
        "geom",
        friction=f"{config.friction} 0.01 0.001",
        solref="0.008 1",
        solimp="0.95 0.99 0.001",
        condim="4",
    )
    _e(default, "joint", damping="0.06", armature="0.002")
    asset = _e(root, "asset")
    _e(
        asset,
        "texture",
        name="table_tex",
        type="2d",
        builtin="checker",
        rgb1="0.25 0.30 0.36",
        rgb2="0.23 0.28 0.34",
        width="512",
        height="512",
    )
    _e(
        asset,
        "material",
        name="table_mat",
        texture="table_tex",
        texrepeat="8 8",
        reflectance="0.01",
    )
    parts = [f"palm_{p}" for p in ("structure", "servo", "camera")]
    parts += [
        f"{side}_jaw_{p}"
        for side in ("left", "right")
        for p in ("attachment", "pad", "rubber", "flap", "hardware")
    ]
    for name in parts:
        path = ASSETS / "meshes" / f"cad_{name}.stl"
        if not path.is_file():
            raise FileNotFoundError(
                f"Missing pinned CAD asset {path}; reinstall package or regenerate meshes"
            )
        _e(asset, "mesh", name=f"cad_{name}", file=path.name)
    assembly = json.loads((ASSETS / "cad_assembly.json").read_text())
    world = _e(root, "worldbody")
    _e(
        world,
        "light",
        name="key",
        pos="0 -0.5 1.6",
        dir="0 0 -1",
        directional="true",
        castshadow="false",
    )
    _e(
        world,
        "geom",
        name="floor",
        type="plane",
        size="2 2 0.05",
        pos="0 0 -0.071",
        rgba="0.10 0.13 0.18 1",
    )
    _e(
        world,
        "geom",
        name="table",
        type="box",
        size="0.62 0.43 0.035",
        pos="0 0 -0.035",
        material="table_mat",
    )
    _e(
        world,
        "camera",
        name="overview",
        pos="0.70 -0.9 0.75",
        xyaxes="0.789 0.614 0 -0.307 0.394 0.866",
        fovy="45",
    )
    _e(world, "camera", name="top", pos="0 0 1.25", xyaxes="1 0 0 0 1 0", fovy="48")
    equality, actuators = _e(root, "equality"), _e(root, "actuator")
    quat = xyzw_to_wxyz([0, np.sin(np.pi / 4), 0, np.cos(np.pi / 4)])
    palette = json.loads((ASSETS / "colors.json").read_text())
    for hand, y in (("left", 0.18), ("right", -0.18)):
        pos = [0, y, 0.25]
        _e(world, "body", name=f"{hand}_target", mocap="true", pos=_s(pos), quat=_s(quat))
        body = _e(world, "body", name=f"{hand}_hand_root", pos=_s(pos), quat=_s(quat), gravcomp="1")
        _e(body, "freejoint", name=f"{hand}_free")
        _e(body, "inertial", pos="-0.022 0 -0.02", mass="0.32", diaginertia="0.0004 0.0004 0.0003")
        _e(body, "site", name=f"{hand}_eef", size="0.002", rgba="1 0 0 1")
        _e(
            body,
            "site",
            name=f"{hand}_grasp_center",
            pos="0.080 0 0",
            size="0.002",
            rgba="0 1 0 0.5",
        )
        for part in ("structure", "servo", "camera"):
            _e(
                body,
                "geom",
                name=f"{hand}_palm_{part}_visual",
                type="mesh",
                mesh=f"cad_palm_{part}",
                contype="0",
                conaffinity="0",
                group="2",
                rgba=_s(palette[f"palm_{part}"]),
            )
        _e(
            body,
            "geom",
            name=f"{hand}_palm_collision",
            type="box",
            size="0.022 0.0335 0.025",
            pos="-0.022 0 -0.018",
            contype="2",
            conaffinity="1",
            group="3",
            rgba="0.5 0.5 0.5 0",
        )
        # Pinhole optical convention: camera looks along +x, up along +z in hand frame.
        # Extrinsic and FOV are APPROXIMATE, source fisheye not calibrated.
        _e(
            body,
            "camera",
            name=f"wrist_{hand}",
            pos="0 0 0.0426",
            xyaxes="0 -1 0 0 0 1",
            fovy=str(config.camera_fovy),
        )
        for side, sign in (("right", -1), ("left", 1)):
            finger = _e(
                body,
                "body",
                name=f"{hand}_{side}_finger",
                pos=_s(assembly["bodies"][f"{side}_jaw"]["origin_m"]),
                gravcomp="1",
            )
            _e(
                finger,
                "inertial",
                pos=f"0.055 {sign * 0.004} 0",
                mass="0.035",
                diaginertia="0.00001 0.00004 0.00004",
            )
            limits = [0, config.joint_max] if side == "right" else [-config.joint_max, 0]
            _e(
                finger,
                "joint",
                name=f"{hand}_{side}_joint",
                type="hinge",
                axis="0 0 -1",
                range=_s(limits),
            )
            for part in ("attachment", "pad", "flap", "hardware"):
                _e(
                    finger,
                    "geom",
                    name=f"{hand}_{side}_{part}_visual",
                    type="mesh",
                    mesh=f"cad_{side}_jaw_{part}",
                    contype="0",
                    conaffinity="0",
                    group="2",
                    rgba=_s(palette[f"jaw_{part}"]),
                )
            # One source rubber solid per jaw, convexified by MuJoCo for collision.
            # This explicit approximation fills up to ~3.3 mm of local concavity.
            _e(
                finger,
                "geom",
                name=f"{hand}_{side}_pad",
                type="mesh",
                mesh=f"cad_{side}_jaw_rubber",
                contype="2",
                conaffinity="1",
                rgba=_s(palette["jaw_rubber"]),
            )
        _e(
            equality,
            "weld",
            name=f"{hand}_pose_servo",
            body1=f"{hand}_target",
            body2=f"{hand}_hand_root",
            solref="0.012 1",
            solimp="0.95 0.99 0.001",
            torquescale="0.10",
        )
        _e(
            equality,
            "joint",
            name=f"{hand}_gear",
            joint1=f"{hand}_left_joint",
            joint2=f"{hand}_right_joint",
            polycoef="0 -1 0 0 0",
            solref="0.004 1",
        )
        _e(
            actuators,
            "position",
            name=f"{hand}_gripper",
            joint=f"{hand}_right_joint",
            kp=str(config.gripper_kp),
            kv="0.16",
            ctrlrange=f"0 {config.joint_max}",
            forcerange=f"-{config.gripper_torque} {config.gripper_torque}",
        )
    for i in range(2 if config.task == "dual_pick_place" else 1):
        obj = _e(world, "body", name=f"object_{i}", pos=f"0 {0.18 - i * 0.36} 0.017")
        _e(obj, "freejoint", name=f"object_{i}_free")
        _e(
            obj,
            "geom",
            name=f"object_{i}_geom",
            type="box",
            size="0.018 0.018 0.018",
            mass=str(config.object_mass),
            contype="1",
            conaffinity="3",
            rgba="0.93 0.22 0.29 1" if i == 0 else "0.63 0.38 0.88 1",
        )
        _e(
            world,
            "site",
            name=f"goal_{i}",
            type="cylinder",
            pos=f"0.22 {0.18 - i * 0.36} 0.001",
            size="0.045 0.0005",
            rgba="0.2 0.9 0.6 0.55",
        )
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")
