"""Installed-package resources, export notices, CLI, and optional dependencies."""

from importlib import metadata
import json
from pathlib import Path
import struct
import subprocess
import sys
import zlib

import numpy as np
import pytest

from yubi_mujoco import __version__
from yubi_mujoco.cli import main
from yubi_mujoco.model import ASSETS


def test_package_version_matches_distribution():
    assert metadata.version("yubi_mujoco") == __version__


def test_development_tools_are_not_published_as_runtime_requirements():
    package = metadata.metadata("yubi_mujoco")
    assert package.get_all("Provides-Extra") == ["video"]
    requirements = package.get_all("Requires-Dist") or []
    assert not any(req.startswith(("pytest", "ruff", "twine", "build")) for req in requirements)


def test_runtime_assets_are_complete_and_hashed():
    import hashlib

    manifest = json.loads((ASSETS / "cad_manifest.json").read_text())
    meshes = sorted((ASSETS / "meshes").glob("*.stl"))
    assert len(meshes) == len(manifest["meshes"]) == 13
    for entry in manifest["meshes"].values():
        path = ASSETS / "meshes" / Path(entry["file"]).name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]
    for name in [
        "SOURCE.md",
        "NOTICE.md",
        "licenses/CERN-OHL-W-2.0.txt",
        "licenses/MIT.txt",
        "licenses/Apache-2.0.txt",
    ]:
        assert (ASSETS / name).is_file()


def test_export_copies_source_and_license_notices(tmp_path):
    main(["export-mjcf", "--output", str(tmp_path)])
    for name in [
        "SOURCE.md",
        "NOTICE.md",
        "licenses/CERN-OHL-W-2.0.txt",
        "licenses/MIT.txt",
        "licenses/Apache-2.0.txt",
    ]:
        assert (tmp_path / name).read_bytes() == (ASSETS / name).read_bytes()


def test_png_writer_roundtrips_rgb_bytes(tmp_path):
    from yubi_mujoco._images import write_png

    image = np.arange(4 * 7 * 3, dtype=np.uint8).reshape(4, 7, 3)
    path = tmp_path / "test.png"
    write_png(path, image)
    raw = path.read_bytes()
    assert raw.startswith(b"\x89PNG\r\n\x1a\n")
    offset, parts = 8, {}
    while offset < len(raw):
        size = struct.unpack(">I", raw[offset : offset + 4])[0]
        kind = raw[offset + 4 : offset + 8]
        data = raw[offset + 8 : offset + 8 + size]
        crc = struct.unpack(">I", raw[offset + 8 + size : offset + 12 + size])[0]
        assert crc == zlib.crc32(kind + data)
        parts[kind] = data
        offset += 12 + size
    assert struct.unpack(">II", parts[b"IHDR"][:8]) == (7, 4)
    assert zlib.decompress(parts[b"IDAT"]) == b"".join(b"\0" + row.tobytes() for row in image)


def test_video_dependencies_are_lazy_and_explain_install(monkeypatch, tmp_path):
    import builtins
    from yubi_mujoco.cli import _writer

    original = builtins.__import__

    def missing(name, *args, **kwargs):
        if name.startswith("imageio"):
            raise ImportError("intentionally unavailable")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", missing)
    assert _writer(tmp_path, False, 30) is None
    with pytest.raises(RuntimeError, match=r"yubi_mujoco\[video\]"):
        _writer(tmp_path, True, 30)


def test_module_help_and_version_work_outside_checkout(tmp_path):
    for flag in ["--help", "--version"]:
        result = subprocess.run(
            [sys.executable, "-m", "yubi_mujoco", flag],
            cwd=tmp_path,
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
        assert "yubi-mujoco" in result.stdout


@pytest.mark.rendering
def test_render_command_writes_png(tmp_path):
    target = tmp_path / "scene.png"
    assert main(["render", "--output", str(target), "--width", "320", "--height", "240"]) == 0
    assert target.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


@pytest.mark.parametrize(
    "args",
    [["demo", "--episodes", "0"], ["render", "--width", "0"], ["render", "--height", "2000"]],
)
def test_cli_invalid_limits_exits_cleanly(args):
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2
