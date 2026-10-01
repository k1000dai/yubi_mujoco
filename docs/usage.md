# Usage

## Installation

Use the [uv project workflow](https://docs.astral.sh/uv/getting-started/installation/)
from a source checkout:

```bash
uv sync --locked
uv run --locked yubi-mujoco --version
```

uv creates `.venv` automatically and uses the committed `uv.lock` across
platforms. `.python-version` selects Python 3.12 for the checkout; the package
supports Python 3.10+. To select a different supported version, add
`--python VERSION` to both `uv sync` and `uv run`. The `dev` dependency group is
included by default; add `--no-dev` to both commands for runtime-only use.
No environment activation is needed.

The package remains pip-compatible. In an existing virtual environment, install
this checkout with `python -m pip install .`, or a built wheel outside the source
tree:

```bash
python -m pip install /path/to/yubi_mujoco-0.1.0-py3-none-any.whl
```

Once the initial release is available on PyPI, use
`python -m pip install yubi_mujoco`. Before publication, install the source
checkout or a wheel from a trusted build. pip resolves package metadata ranges;
it does not consume `uv.lock`.

The base install requires only MuJoCo, NumPy, and SciPy. Video adds imageio and
imageio-ffmpeg: use `uv sync --locked --extra video` and
`uv run --locked --extra video yubi-mujoco ...` in the checkout. With pip, use
`python -m pip install '.[video]'` from a checkout, or
`python -m pip install 'yubi_mujoco[video]'` after publication. FreeCAD is needed
only to regenerate the CAD assets.

## Command line

`uv run --locked yubi-mujoco` and `uv run --locked python -m yubi_mujoco`
run the same CLI. After a pip install, omit the `uv run --locked` prefix and
use that environment directly.

```bash
uv run --locked yubi-mujoco --version
uv run --locked yubi-mujoco --help
uv run --locked yubi-mujoco demo --help
```

The available tasks are `pick_place` (default), `dual_pick_place`, `lift`, and
`push`. Set `--hz 10`, `--hz 30` (default), or `--hz 60` explicitly when comparing
runs. A `--horizon` is a count of control steps, not seconds. Seed batches use
`--seed`, then successive integers for `--episodes` episodes.

### Scripted demos

```bash
uv run --locked yubi-mujoco demo
uv run --locked yubi-mujoco demo --task pick_place --seed 0 --episodes 10 --output yubi-output/batch
uv run --locked --extra video yubi-mujoco demo --task dual_pick_place --video --output yubi-output/dual
```

The demo is an **object-state-aware scripted baseline**. It reads simulator
positions and issues normal pose/motor commands; it does not attach objects to
the hands. Use it to check the scene and contact behavior, not to measure visual
policy generalization.

`demo` defaults to `yubi-output/demo`. Runs write `report.json` and per-seed
`trace_seed_*.json` files. With `--video`, the first episode also produces
`rollout.mp4`, `scene.png`, `final.png`, and two wrist-camera images. Use a fresh
output directory for each experiment; existing files with the same names may be
replaced. A JSON report's status and errors matter even when a CLI run finishes.

### Render an image

```bash
uv run --locked yubi-mujoco render --output scene.png --camera overview --width 960 --height 720
uv run --locked yubi-mujoco render --camera wrist_left --width 640 --height 480 --output wrist.png
```

Named cameras are `overview`, `top`, `wrist_left`, and `wrist_right`. Rendering
uses the reset scene for the selected task and seed. Output must have a `.png`
extension; the default is `yubi-output/scene.png`. Width may be 1–1280 pixels and
height 1–960 pixels. A working OpenGL backend is required; the base install can
write PNG images without the video extra.

### Portable MJCF

```bash
uv run --locked yubi-mujoco export-mjcf --task dual_pick_place --output yubi-output/mjcf
```

This produces `scene.xml`, relative-path mesh assets, the model's JSON
metadata, source/attribution notices, and license texts. Keep the exported directory together when moving it. For example:

```python
import mujoco

model = mujoco.MjModel.from_xml_path("yubi-output/mjcf/scene.xml")
```

The Python `env.save_mjcf(path)` convenience method writes XML referencing the
installed package's absolute asset paths. Use the export command when you need
a self-contained scene.

### Keyboard teleoperation

```bash
uv run --locked yubi-mujoco teleop --task pick_place
# On macOS, the passive viewer requires the MuJoCo Python launcher:
uv run --locked mjpython -m yubi_mujoco teleop --task pick_place
```

Teleoperation needs a desktop OpenGL window. Select a hand with `1`/`2`.
`W`/`S`, `A`/`D`, and `R`/`F` move it along world ±X, ±Y, and ±Z. `I`/`K`
change pitch, `J`/`L` yaw, and `U`/`M` roll. `O`/`C` open/close the gripper.
Press Space to reset, including after an episode finishes, and Escape to exit.

## Python environment

```python
import numpy as np
from yubi_mujoco import SimConfig, YubiEnv

with YubiEnv(SimConfig(task="pick_place", control_hz=30)) as env:
    observation, info = env.reset(seed=7)
    target = env.targets.copy()
    target[0, 2] += 0.03
    motor = env.motor_for_jaw(np.array([0.4, 0.55]))
    for _ in range(10):
        observation, reward, terminated, truncated, info = env.step_absolute(target, motor)
        if terminated or truncated:
            break
```

`reset(seed=...)` returns `(observation, info)`. Both `step_absolute(poses, motors)`
and `step_delta(action)` return `(observation, reward, terminated, truncated,
info)`. `step(action)` is an alias for `step_delta(action)`. Reset after either
terminal flag. This is a Gym-like interface without a Gym dependency or registry.

### Frames, values, and tracking

- Hand order is always left, then right.
- Absolute poses have shape `(2, 7)`: `[x, y, z, qx, qy, qz, qw]` in the world
  frame. Position units are metres; API quaternions use **xyzw**.
- The world is right-handed with +Z upward and the table surface at Z = 0.
- Poses refer to `left_hand_root` and `right_hand_root`, not fingertips. Local
  +X points toward the fingers, +Y toward the left jaw, and +Z toward the camera.
- `env.eef_poses` reports actual body poses. `env.targets` stores applied pose
  commands. Finite-stiffness tracking means these can differ.
- Grippers have shape `(2,)` and contain absolute motor coordinates in radians.
  `jaw_rad = motor_scale * motor_rad + motor_offset`; the defaults 1 and 0 are
  uncalibrated. `motor_for_jaw()` applies the inverse mapping.
- The nominal jaw command interval is `0..0.94` rad. Zero is the CAD parallel-jaw
  reference, not fully closed or a measured hardware encoder zero.
- Requested positions outside X ±0.6 m, Y ±0.4 m, or Z 0.01–0.7 m are rejected.
  These are simulator bounds, not a real arm's reachable workspace.
- Targets are rate-limited to 0.6 m/s and 3 rad/s by default. The two flags in
  `info["target_rate_limited"]` record when this occurs.

Default control timing is 30 Hz, with a 1/600 s physics step. A custom
`physics_dt` must divide the control interval exactly. `SimConfig` also exposes
mass, friction, gripper control, camera FOV, and motor-mapping parameters. These
are simulation assumptions, not calibration measurements. Load CLI overrides
with `--config examples/nominal.json`, or provide your own JSON object of
`SimConfig` fields. Explicit CLI task/rate/horizon arguments override the file.

### Observations and rendering

By default, direct API observations omit images, so physics-only usage does not
create a renderer. Use `YubiEnv(..., render_images=True)` to include wrist images
in every observation, or call `env.observe(images=True)` when needed.
`env.render(camera="overview", width=960, height=720)` returns a uint8 RGB array.
Use a context manager or call `env.close()` to release rendering resources.

`info` includes object positions, goals, contact history, and success metrics.
It is privileged simulator state and is deliberately excluded from observations
passed to a policy. Do not feed it into a policy and then describe the result as
an image-only evaluation.

## Local policy evaluation

```bash
uv run --locked yubi-mujoco evaluate --policy hold --horizon 64 --output yubi-output/hold
uv run --locked yubi-mujoco evaluate --policy /path/to/policy.py --checkpoint /path/to/checkpoint \
  --task pick_place --episodes 5 --hz 30 --adopt-rows 16 \
  --translation-frame body --output yubi-output/model
```

The built-in hold policy exercises the interface; task failure is expected.
A custom file must provide `Policy(checkpoint_dir: str)` and
`infer(obs: dict) -> np.ndarray`; start with [examples/policy.py](../examples/policy.py).
The file is imported and executed as Python. Only load trusted code and weights.
The runner does not sandbox your code or install your model's dependencies.

### Observation contract

The runner passes these five keys, using the public
[UMI Arena interface](https://umi-arena.airoa.io/submission-format):

| Key | Value |
| --- | --- |
| `observation.image.left` | uint8 RGB `(480, 640, 3)`, HWC |
| `observation.image.right` | uint8 RGB `(480, 640, 3)`, HWC |
| `observation.pose.left_hand_root_to_right_hand_root.absolute` | float32 `(7,)`, right root expressed in the left root, `T_left^-1 T_right` |
| `observation.joint_states` | float32 `(2,)`, left/right absolute motor radians |
| `prompt` | Task instruction string |

There is no center camera. Concatenating the relative pose and motor coordinates
makes a 9-value state; the dataset's separate `observation.state` mode flag is
not that vector. Images match the contract's shape, but not calibrated real
camera optics. Evaluation creates wrist images even if video recording is off.

### Action contract

Return `(N, 16)` with `N >= 16`; float32 is recommended and float64 is accepted.
Each row contains:

| Columns | Command |
| --- | --- |
| `0:7` | Left `[dx, dy, dz, dqx, dqy, dqz, dqw]` |
| `7:14` | Right pose delta in the same format |
| `14:16` | Left/right **absolute** motor coordinates, rad |

An identity quaternion delta is `[0, 0, 0, 1]`. Every row must be finite. The
adopted rows must also contain valid nonzero quaternions and in-range motor
commands. Unused rows do not execute. By default the runner adopts 16 rows,
then observes and calls `infer` again; `--adopt-rows` cannot exceed the returned
chunk length. It does not advance physics while `infer` is running.

Deltas compose with the previous **applied command pose**, not the measured
tracking pose. In default `body` translation mode:

```text
p_next = p + R(q) @ delta_p
q_next = q * delta_q
```

`--translation-frame world` changes translation to `p + delta_p`; rotation still
right-multiplies the prior orientation. Rate limiting may shorten the commanded
motion. These conventions follow the
[pinned replay geometry reference](https://github.com/airoa-org/umi-arena-evaluation/blob/8763f10a022139dbcf4293b60f797a717d8bd1ad/umi_arena/replay_metrics.py).

### 10 Hz versus 30 Hz

The published robot execution description uses 10 Hz, while the dataset/public
replay examples use 30 Hz. This simulator defaults to 30 Hz and also supports
10 and 60 Hz. Executing the same six rows takes 0.6 seconds at 10 Hz and 0.2
seconds at 30 Hz. Changing the frequency alone changes motion speed; it is not
a resampling algorithm. Never scale quaternion components by a rate ratio.

Offline dataset timing needs separate care: pose-delta columns encode the
previous-to-current transform, while `action.joint_states` refers to the next
frame. See the [pinned replay audit](https://github.com/airoa-org/umi-arena-evaluation/blob/8763f10a022139dbcf4293b60f797a717d8bd1ad/umi_arena/replay_data.py).
The local live-policy runner applies no additional row shift. It does not
implement dataset download or automatic dataset replay. Real submissions still
need the official interface checker and evaluation process.

## Rendering and platform notes

Physics stepping requires no GPU. Rendering requires a working OpenGL context
and system graphics libraries; Python dependencies alone do not provide all of
those libraries. A software rendering backend may be used where supported.
A learned policy may have its own GPU requirements, independent of this package.

On Linux without `DISPLAY`, rendering CLI commands select EGL unless
`MUJOCO_GL` is already set. For the Python API, set the backend **before importing
MuJoCo or the environment**, for example:

```bash
MUJOCO_GL=egl uv run --locked python your_program.py
```

If EGL is unavailable, configure a supported backend on your system. OSMesa
software rendering requires the system OSMesa library and can be selected with
`MUJOCO_GL=osmesa`; a desktop display normally uses GLFW. Do not force EGL on
macOS. For viewer guidance, consult the
[MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html).

A failed GL initialization does not establish that physics or policy loading is
broken. First try `yubi-mujoco demo` without video; then test `render` to isolate
the graphics path. GUI teleoperation, headless images, and policy inference are
separate checks. Do not infer macOS/Windows or driver compatibility from Linux
headless results. See [validation scope](validation.md).
