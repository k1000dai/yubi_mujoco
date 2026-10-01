"""Runnable demos, batch evaluation, portable MJCF and interactive pose teleop."""

import argparse
from contextlib import contextmanager
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import sysconfig
import time
import sys
import numpy as np


def _config(args):
    from .env import SimConfig

    values = json.loads(Path(args.config).read_text()) if getattr(args, "config", None) else {}
    if args.task is not None:
        values["task"] = args.task
    if args.hz is not None:
        values["control_hz"] = args.hz
    if getattr(args, "horizon", None) is not None:
        values["horizon"] = args.horizon
    return SimConfig(**values)


def _writer(output, enabled, fps):
    if not enabled:
        return None
    try:
        import imageio.v2 as iio
        import imageio_ffmpeg  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            'Video support is optional. Install it with: pip install "yubi_mujoco[video]"'
        ) from exc
    return iio.get_writer(
        str(output / "rollout.mp4"), fps=fps, codec="libx264", quality=7, macro_block_size=16
    )


def _headless_backend():
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        os.environ.setdefault("MUJOCO_GL", "egl")


class _ViewerClosed(Exception):
    """Raised from a step callback when the user closes the live viewer."""


@contextmanager
def _live_viewer(env, enabled):
    if not enabled:
        yield None
        return
    import mujoco.viewer

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        yield viewer


def _relaunch_under_mjpython(argv):
    """On macOS, passive viewers need mjpython; re-execute this CLI under it."""
    if sys.platform != "darwin" or os.environ.get("MJPYTHON_BIN"):
        return
    mjpython = Path(sysconfig.get_path("scripts")) / "mjpython"
    if not mjpython.exists():
        return  # launch_passive then reports the mjpython requirement
    env = os.environ.copy()
    libdir = sysconfig.get_config_var("LIBDIR")
    if libdir:
        # uv and other standalone Pythons load libpython via @rpath, which does
        # not resolve once mjpython re-executes from inside its app bundle.
        fallback = env.get("DYLD_FALLBACK_LIBRARY_PATH") or "/usr/local/lib:/usr/lib"
        env["DYLD_FALLBACK_LIBRARY_PATH"] = f"{libdir}:{fallback}"
    os.execve(mjpython, [str(mjpython), "-m", "yubi_mujoco", *argv], env)


def _run(args):
    if not args.viewer:
        _headless_backend()
    from .env import YubiEnv
    from .scripted import oracle_rollout
    from .adapter import HoldPolicy, load_policy, run_policy
    from ._images import write_png

    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    config = _config(args)
    report = {
        "configuration": asdict(config),
        "baseline": "privileged_scripted" if args.command == "demo" else args.policy,
        "warning": "Simulation regression metric; not real-robot UMI Arena score or evidence of policy generalization.",
        "episodes": [],
    }
    writer = _writer(out, args.video, config.control_hz)
    try:
        for index in range(args.episodes):
            with YubiEnv(config) as env, _live_viewer(env, args.viewer) as viewer:
                env.reset(seed=args.seed + index)
                records = []
                last_frame = time.perf_counter()

                def callback(e, info):
                    nonlocal last_frame
                    records.append(info)
                    if writer and index == 0:
                        writer.append_data(e.render(width=960, height=720))
                    if viewer:
                        if not viewer.is_running():
                            raise _ViewerClosed
                        viewer.sync()
                        # Pace the rollout at the control rate so it plays in real time.
                        delay = 1 / e.config.control_hz - (time.perf_counter() - last_frame)
                        time.sleep(max(0, delay))
                        last_frame = time.perf_counter()

                if viewer:
                    viewer.sync()

                if index == 0 and args.video:
                    write_png(out / "scene.png", env.render(width=960, height=720))
                    write_png(
                        out / "wrist_left.png", env.render("wrist_left", width=640, height=480)
                    )
                    write_png(
                        out / "wrist_right.png", env.render("wrist_right", width=640, height=480)
                    )
                started = time.perf_counter()
                if args.command == "demo":
                    result, _ = oracle_rollout(env, on_step=callback)
                else:
                    policy = (
                        HoldPolicy()
                        if args.policy == "hold"
                        else load_policy(args.policy, args.checkpoint)
                    )
                    result = run_policy(
                        env,
                        policy,
                        adopt_rows=args.adopt_rows,
                        on_step=callback,
                        translation_frame=args.translation_frame,
                    )
                result["wall_seconds"] = time.perf_counter() - started
                report["episodes"].append(result)
                (out / f"trace_seed_{env.seed}.json").write_text(json.dumps(records, indent=2))
                if index == 0 and args.video:
                    write_png(out / "final.png", env.render(width=960, height=720))
                print(
                    f"{args.command} {config.task} seed={env.seed}: success={result['success']}, steps={result['steps']}",
                    flush=True,
                )
                (out / "report.json").write_text(json.dumps(report, indent=2))
    except _ViewerClosed:
        report["stopped_early"] = "viewer closed"
        print("Viewer closed; stopping.", flush=True)
    finally:
        if writer:
            writer.close()
    report["successes"] = sum(r["success"] for r in report["episodes"])
    report["success_rate"] = (
        report["successes"] / len(report["episodes"]) if report["episodes"] else None
    )
    (out / "report.json").write_text(json.dumps(report, indent=2))
    print(f"Results: {out.resolve() / 'report.json'}")
    return 0


def _export(args):
    from .model import make_model_xml, ASSETS

    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copytree(ASSETS / "meshes", out / "meshes", dirs_exist_ok=True)
    (out / "scene.xml").write_text(make_model_xml(_config(args), mesh_dir="meshes"))
    for pattern in ("*.json", "*.md"):
        for p in ASSETS.glob(pattern):
            shutil.copy2(p, out / p.name)
    shutil.copytree(ASSETS / "licenses", out / "licenses", dirs_exist_ok=True)
    print(out.resolve() / "scene.xml")
    return 0


def _render(args):
    _headless_backend()
    from .env import YubiEnv
    from ._images import write_png

    output = Path(args.output)
    if output.suffix.lower() != ".png":
        raise ValueError("render output must have a .png extension")
    output.parent.mkdir(parents=True, exist_ok=True)
    with YubiEnv(_config(args)) as env:
        env.reset(seed=args.seed)
        write_png(output, env.render(args.camera, width=args.width, height=args.height))
    print(output.resolve())
    return 0


def _teleop(args):
    from .env import YubiEnv
    from scipy.spatial.transform import Rotation
    import mujoco.viewer
    import threading

    lock = threading.Lock()
    pending = []

    def key_callback(key):
        with lock:
            pending.append(key)

    print(
        "1/2 select hand; W/S ±X, A/D ±Y, R/F ±Z; I/K pitch, J/L yaw, U/M roll; O/C open/close; Space reset; Esc quit"
    )
    with YubiEnv(_config(args)) as env:
        env.reset(seed=args.seed)
        targets, jaw = env.targets.copy(), np.full(2, min(0.55, 0.9 * env.config.joint_max))
        active = 0
        with mujoco.viewer.launch_passive(env.model, env.data, key_callback=key_callback) as viewer:
            while viewer.is_running():
                start = time.perf_counter()
                with lock:
                    keys, pending[:] = pending.copy(), []
                for key in keys:
                    k = chr(key).upper() if 0 <= key < 256 else ""
                    if k in "12" and k:
                        active = int(k) - 1
                    if k == " ":
                        env.reset(seed=args.seed)
                        targets, jaw = (
                            env.targets.copy(),
                            np.full(2, min(0.55, 0.9 * env.config.joint_max)),
                        )
                    for plus, minus, axis in (("W", "S", 0), ("A", "D", 1), ("R", "F", 2)):
                        if k in (plus, minus):
                            targets[active, axis] += 0.01 if k == plus else -0.01
                    for plus, minus, axis in (("U", "M", 0), ("I", "K", 1), ("J", "L", 2)):
                        if k in (plus, minus):
                            rv = np.zeros(3)
                            rv[axis] = 0.05 if k == plus else -0.05
                            targets[active, 3:] = (
                                Rotation.from_quat(targets[active, 3:]) * Rotation.from_rotvec(rv)
                            ).as_quat()
                    if k in ("O", "C"):
                        jaw[active] = np.clip(
                            jaw[active] + (0.04 if k == "O" else -0.04), 0, env.config.joint_max
                        )
                if not env._done:
                    try:
                        env.step_absolute(targets, env.motor_for_jaw(jaw))
                    except ValueError as exc:
                        print(exc)
                        targets = env.targets.copy()
                viewer.sync()
                time.sleep(max(0, 1 / env.config.control_hz - (time.perf_counter() - start)))
    return 0


def main(argv=None):
    from . import __version__

    parser = argparse.ArgumentParser(
        prog="yubi-mujoco", description="Arm-free YUBI MuJoCo contact simulation"
    )
    parser.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    sub = parser.add_subparsers(dest="command", required=True)
    for cmd in ("demo", "evaluate", "export-mjcf", "render", "teleop"):
        p = sub.add_parser(cmd)
        p.add_argument(
            "--task", choices=("pick_place", "push", "lift", "dual_pick_place"), default=None
        )
        p.add_argument("--hz", type=int, choices=(10, 30, 60), default=None)
        p.add_argument(
            "--config", help="JSON SimConfig overrides, e.g. calibrated motor_scale/motor_offset"
        )
        p.add_argument("--seed", type=int, default=0)
        p.add_argument("--horizon", type=int)
        p.add_argument(
            "--output", default="yubi-output/scene.png" if cmd == "render" else f"yubi-output/{cmd}"
        )
        if cmd == "render":
            p.add_argument(
                "--camera",
                choices=("overview", "top", "wrist_left", "wrist_right"),
                default="overview",
            )
            p.add_argument("--width", type=int, default=960)
            p.add_argument("--height", type=int, default=720)
        if cmd in ("demo", "evaluate"):
            p.add_argument("--episodes", type=int, default=1)
            p.add_argument("--video", action="store_true")
            p.add_argument(
                "--viewer",
                action="store_true",
                help="watch the rollout live in the MuJoCo viewer (needs a desktop display)",
            )
        if cmd == "evaluate":
            p.add_argument(
                "--policy", default="hold", help="hold or path to a trusted local policy.py"
            )
            p.add_argument("--checkpoint", default=".")
            p.add_argument("--adopt-rows", type=int, default=16)
            p.add_argument("--translation-frame", choices=("body", "world"), default="body")
    argv = sys.argv[1:] if argv is None else list(argv)
    args = parser.parse_args(argv)
    if getattr(args, "episodes", 1) < 1:
        parser.error("episodes must be positive")
    if args.command == "render" and not (0 < args.width <= 1280 and 0 < args.height <= 960):
        parser.error("render width must be 1..1280 and height 1..960")
    if args.command == "teleop" or getattr(args, "viewer", False):
        _relaunch_under_mjpython(argv)
    handlers = {
        "export-mjcf": _export,
        "render": _render,
        "teleop": _teleop,
        "demo": _run,
        "evaluate": _run,
    }
    try:
        return handlers[args.command](args)
    except (ValueError, FileNotFoundError, RuntimeError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
