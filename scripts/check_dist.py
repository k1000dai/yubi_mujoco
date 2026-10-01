#!/usr/bin/env python3
"""Validate release archives and optionally install each in an isolated venv.

Run after ``uv build``. Smoke tests use temporary working directories
outside the checkout and install only core dependencies before testing video.
No GUI, FreeCAD installation, credentials, or publishing are involved.
"""

from __future__ import annotations

import argparse
from email.parser import BytesParser
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import tarfile
import tempfile
import venv
import zipfile

ASSET_FILES = {
    "cad_assembly.json",
    "cad_manifest.json",
    "colors.json",
    "SOURCE.md",
    "NOTICE.md",
    "licenses/Apache-2.0.txt",
    "licenses/CERN-OHL-W-2.0.txt",
    "licenses/MIT.txt",
}
MESH_FILES = {
    f"cad_{side}_jaw_{part}.stl"
    for side in ("left", "right")
    for part in ("attachment", "flap", "hardware", "pad", "rubber")
} | {f"cad_palm_{part}.stl" for part in ("camera", "servo", "structure")}
SOURCE_STEP = "cad/source/YUBI Gripper Assy_Dynamixel_ver2.STEP"
SOURCE_SHA256 = "0e60bf62de970e5fae5e7cff104f547a954ef8d5d7b9dde9835b381b95f4fcc8"
FORBIDDEN_PARTS = {
    ".git",
    ".venv",
    "venv",
    ".env",
    ".cache",
    ".pytest_cache",
    ".ruff_cache",
    ".mypy_cache",
    ".codex",
    ".agents",
    "__pycache__",
    "node_modules",
    "artifacts",
    "drafts",
    "build",
    "dist",
    "vendor",
    "yubi-output",
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_paths(files: dict[str, bytes]) -> None:
    for name, contents in files.items():
        path = PurePosixPath(name)
        require(not path.is_absolute() and ".." not in path.parts, f"Unsafe path: {name}")
        require(not FORBIDDEN_PARTS.intersection(path.parts), f"Unwanted file: {name}")
        require(path.suffix.lower() not in {".pyc", ".pyo", ".mp4"}, f"Unwanted file: {name}")
        require(path.name != ".DS_Store", f"Unwanted file: {name}")
        if name.endswith((".stl", ".STEP", ".txt", ".md", ".json")):
            require(bool(contents), f"Empty asset or documentation: {name}")


def validate_assets(files: dict[str, bytes], prefix: str) -> None:
    for relative in ASSET_FILES | {f"meshes/{name}" for name in MESH_FILES}:
        require(prefix + relative in files, f"Missing required asset: {prefix}{relative}")
    meshes = {
        name.removeprefix(prefix + "meshes/")
        for name in files
        if name.startswith(prefix + "meshes/")
    }
    require(meshes == MESH_FILES, f"Expected exactly the 13 runtime meshes, got: {meshes}")


def metadata_version(contents: bytes) -> str:
    metadata = BytesParser().parsebytes(contents)
    require(metadata["Name"] == "yubi_mujoco", f"Unexpected project: {metadata['Name']}")
    require(metadata["Requires-Python"] == ">=3.10", "Requires-Python must be >=3.10")
    version = metadata["Version"]
    require(bool(version), "Missing package version")
    return version


def inspect_wheel(path: Path) -> str:
    with zipfile.ZipFile(path) as archive:
        files = {name: archive.read(name) for name in archive.namelist() if not name.endswith("/")}
    validate_paths(files)
    validate_assets(files, "yubi_mujoco/assets/")
    require("yubi_mujoco/__init__.py" in files, "Wheel is missing the Python package")
    for name in files:
        require(
            name.startswith("yubi_mujoco/") or name.split("/", 1)[0].endswith(".dist-info"),
            f"Unexpected wheel top-level file: {name}",
        )
        require("cad" not in PurePosixPath(name).parts, f"CAD source belongs in sdist: {name}")
        require(
            PurePosixPath(name).suffix.lower() not in {".step", ".stp"}, f"CAD in wheel: {name}"
        )
    metadata = [name for name in files if name.endswith(".dist-info/METADATA")]
    require(len(metadata) == 1, "Wheel must contain one METADATA file")
    version = metadata_version(files[metadata[0]])
    source_notice = files["yubi_mujoco/assets/SOURCE.md"].decode("utf-8")
    require(
        f"yubi_mujoco-{version}.tar.gz" in source_notice, "Source notice has a stale sdist filename"
    )
    require(
        f"https://pypi.org/project/yubi-mujoco/{version}/#files" in source_notice,
        "Source notice has a stale release URL",
    )
    return version


def inspect_sdist(path: Path) -> str:
    with tarfile.open(path, "r:gz") as archive:
        members = archive.getmembers()
        require(all(m.isfile() or m.isdir() for m in members), "Sdist contains special files/links")
        roots = {PurePosixPath(m.name).parts[0] for m in members}
        require(len(roots) == 1, "Sdist must have exactly one root directory")
        files = {}
        for member in members:
            if member.isfile():
                stream = archive.extractfile(member)
                require(stream is not None, f"Cannot read {member.name}")
                files[str(PurePosixPath(*PurePosixPath(member.name).parts[1:]))] = stream.read()
    validate_paths(files)
    validate_assets(files, "src/yubi_mujoco/assets/")
    for name in (
        "pyproject.toml",
        "uv.lock",
        ".python-version",
        "README.md",
        "LICENSE",
        "cad/export.py",
        "cad/source/LICENSE",
        SOURCE_STEP,
    ):
        require(name in files, f"Missing source distribution file: {name}")
    require(
        hashlib.sha256(files[SOURCE_STEP]).hexdigest() == SOURCE_SHA256,
        "Sdist is missing the complete, pinned original CAD source",
    )
    require(any(name.startswith("tests/") for name in files), "Sdist is missing regression tests")
    require("PKG-INFO" in files, "Sdist is missing PKG-INFO")
    return metadata_version(files["PKG-INFO"])


SMOKE_CODE = """
import importlib.metadata
import importlib.util
from pathlib import Path
import sys
import mujoco
import numpy as np
import yubi_mujoco
from yubi_mujoco import SimConfig, YubiEnv

checkout = Path(sys.argv[1]).resolve()
assert not Path(yubi_mujoco.__file__).resolve().is_relative_to(checkout)
assert yubi_mujoco.__version__ == importlib.metadata.version("yubi_mujoco")
assert importlib.util.find_spec("imageio") is None, "Video must remain optional"
with YubiEnv(SimConfig(horizon=2)) as env:
    env.reset(seed=7)
    obs, _, _, _, info = env.step_absolute(env.targets.copy(), env.target_grippers.copy())
    assert obs["observation.joint_states"].shape == (2,)
    assert np.isfinite(env.data.qpos).all()
    assert info["steps"] == 1
model = mujoco.MjModel.from_xml_path(str(Path("export/scene.xml").resolve()))
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
assert np.isfinite(data.qpos).all()
print("Installed-package import, physics, optional dependencies and portable MJCF passed")
"""


def smoke_test(archive: Path, *, render: bool, video: bool) -> None:
    checkout = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)
    env.pop("PYTHONHOME", None)
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONUTF8"] = "1"
    with tempfile.TemporaryDirectory(prefix="yubi-dist-") as temporary:
        root = Path(temporary).resolve()
        require(not root.is_relative_to(checkout), "Temporary directory must be outside checkout")
        venv.EnvBuilder(with_pip=True).create(root / "venv")
        bindir = root / "venv" / ("Scripts" if os.name == "nt" else "bin")
        python = bindir / ("python.exe" if os.name == "nt" else "python")
        cli = bindir / ("yubi-mujoco.exe" if os.name == "nt" else "yubi-mujoco")
        work = root / "work"
        work.mkdir()

        def run(*args: str | Path) -> None:
            subprocess.run([str(arg) for arg in args], cwd=work, env=env, check=True)

        print(f"Testing isolated install of {archive.name}", flush=True)
        run(python, "-m", "pip", "install", "--upgrade", "pip")
        run(python, "-m", "pip", "install", "--no-cache-dir", archive)
        run(python, "-m", "pip", "check")
        run(cli, "--version")
        run(python, "-I", "-m", "yubi_mujoco", "--help")
        run(cli, "export-mjcf", "--output", "export")
        run(python, "-I", "-c", SMOKE_CODE, checkout)
        run(cli, "demo", "--task", "lift", "--episodes", "1", "--horizon", "2", "--output", "demo")
        report = json.loads((work / "demo/report.json").read_text(encoding="utf-8"))
        require(len(report["episodes"]) == 1, "Demo did not write an episode report")
        require(report["episodes"][0]["steps"] == 2, "Demo did not reach the smoke-test horizon")
        if render:
            run(cli, "render", "--width", "160", "--height", "120", "--output", "scene.png")
            require(
                (work / "scene.png").read_bytes().startswith(b"\x89PNG\r\n\x1a\n"), "Invalid PNG"
            )
        if video:
            run(python, "-m", "pip", "install", f"{archive}[video]")
            run(python, "-m", "pip", "check")
            run(cli, "demo", "--horizon", "2", "--video", "--output", "video")
            require((work / "video/rollout.mp4").stat().st_size > 0, "Empty video output")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dist", nargs="?", type=Path, default=Path("dist"))
    parser.add_argument("--release-tag", help="Require exact v<metadata version> release tag")
    parser.add_argument(
        "--smoke", action="store_true", help="Install wheel and sdist in clean venvs"
    )
    parser.add_argument("--render", action="store_true", help="Also test the core PNG renderer")
    parser.add_argument(
        "--video", action="store_true", help="Also install and test the video extra"
    )
    args = parser.parse_args()
    if (args.render or args.video) and not args.smoke:
        parser.error("--render and --video require --smoke")
    wheels = sorted(args.dist.resolve().glob("*.whl"))
    sdists = sorted(args.dist.resolve().glob("*.tar.gz"))
    require(len(wheels) == len(sdists) == 1, "Expected exactly one wheel and one sdist in dist/")
    wheel_version = inspect_wheel(wheels[0])
    source_version = inspect_sdist(sdists[0])
    require(wheel_version == source_version, "Wheel and sdist versions differ")
    if args.release_tag is not None:
        require(args.release_tag == f"v{wheel_version}", f"Release tag must be v{wheel_version}")
    print(f"Archives verified: yubi_mujoco {wheel_version}; 13 meshes; CAD source in sdist only")
    if args.smoke:
        for archive in (*wheels, *sdists):
            smoke_test(archive, render=args.render, video=args.video)


if __name__ == "__main__":
    main()
