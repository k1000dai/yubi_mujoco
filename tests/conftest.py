"""Select a headless backend before any test imports MuJoCo on Linux."""

import os
import sys

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL", "egl")
