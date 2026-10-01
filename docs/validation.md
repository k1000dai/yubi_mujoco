# Validation

Validation here means software, geometry, and synthetic-task regression checks.
It does not mean physical calibration or approval for real-robot deployment.
This guide describes reproducible checks; a command being listed is not a claim
that it has passed on every Python version, platform, or graphics driver.

## Local release-candidate checks (2026-10-01)

- Linux x86_64, Python 3.12.14, MuJoCo 3.14.0, NumPy 2.5.3, SciPy 1.18.1
- Full suite: 110 tests passed, including EGL rendering and all contact tasks
- The built wheel installed outside the source checkout also passed all 110 tests
- The declared minimum MuJoCo 3.2.0 passed the same 110-test suite on Python 3.12
- Ruff lint and formatting checks passed
- FreeCAD regenerated all 13 retained material meshes byte-for-byte identically
  to the pinned assets; the black/red palette was preserved unchanged

These local results do not establish macOS/Windows or remote CI results.

## Run the checks

From a clean source checkout, sync the locked development group and video extra:

```bash
uv lock --check
uv sync --locked --extra video
uv run --locked --extra video ruff check .
uv run --locked --extra video ruff format --check .
uv run --locked --extra video pytest -q
uv build
uv run --locked --no-sync twine check --strict dist/*
uv run --locked --no-sync python scripts/check_dist.py dist
```

Image tests require a working OpenGL backend. On a suitable headless Linux
system, run `MUJOCO_GL=egl uv run --locked --extra video pytest -q`. A system configured for software
rendering may instead use OSMesa. See [rendering setup](usage.md#rendering-and-platform-notes).
Do not treat a rendering test that was skipped or could not initialize as a
passed graphics test.

Also install the built wheel into a clean environment **outside the checkout**
and exercise the public commands. This catches missing package data and imports
that accidentally rely on source-tree files:

```bash
python -m yubi_mujoco --version
python -m yubi_mujoco demo --task pick_place --output yubi-output/wheel-demo
python -m yubi_mujoco render --output yubi-output/scene.png
python -m yubi_mujoco export-mjcf --output yubi-output/mjcf
```

Run the base-wheel smoke check without the video extra first. Only MP4 creation
should require imageio/imageio-ffmpeg. Load the exported `scene.xml` after moving
the entire export directory, and check that it still resolves its mesh files.

For a combined distribution smoke check, after the build:

```bash
uv run --locked --no-sync python scripts/check_dist.py dist --smoke --render --video
```

The repository CI configuration includes Linux Python 3.10, 3.12, and 3.13, plus
macOS and Windows Python 3.12. Rendering tests run on Linux; other platforms run
the non-rendering subset. Each test job uses `uv sync --locked --extra video
--no-editable` and tests the installed package outside the checkout. Distribution
checks build an sdist and a wheel from
that sdist, then install them with pip in separate clean environments, preserving
coverage for users who do not use uv. These are configured
checks, not a claim that remote CI has already passed.

## Regression coverage

The suite covers the following distinct properties:

- Pose composition, including nonidentity rotations and xyzw/wxyz conversion
- Relative inter-hand observations and absolute left/right motor semantics
- Configuration validation, calibrated motor mappings, and invalid actions
- Seeded resets and exact 10/30/60 Hz control-step timing
- Finite-stiffness hand tracking and target-rate limits
- Equal/opposite jaw coupling with one actuator per hand
- No action-time object teleportation or grasp-attachment constraints
- Wrist-image shapes and the policy-only observation contract
- Action-chunk validation, unused rows, adopted rows, and report generation
- Portable MJCF loading and the bundled scripted tasks

The open-hand negative control matters: moving an open hand through a nominal
reach/lift path must leave the object on the table. Task success alone would not
rule out hidden attachments or direct state manipulation.

Geometry checks are a separate layer. Verify pinned input hashes, all 97 source
solids assigned exactly once, hinge/frame derivation, and hashes for the 13
active material meshes. Regeneration and visual inspection should confirm that
pads, rubber, attachments, and fixed flaps remain assembled throughout opening.
Read [CAD provenance](CAD_PROVENANCE.md) before changing the mesh pipeline.

## Reproduce task evaluations

```bash
uv run --locked yubi-mujoco demo --task pick_place --seed 0 --episodes 10 --output yubi-output/pick-10
uv run --locked yubi-mujoco demo --task dual_pick_place --seed 0 --episodes 5 --output yubi-output/dual-5
uv run --locked yubi-mujoco demo --task push --seed 0 --episodes 5 --output yubi-output/push-5
uv run --locked yubi-mujoco demo --task lift --hz 10 --seed 0 --output yubi-output/lift-10hz
uv run --locked yubi-mujoco evaluate --policy hold --horizon 64 --output yubi-output/hold
```

These commands produce fresh evidence rather than a fixed advertised success
rate. The hold policy is an interface smoke test; failure to solve a manipulation
task is expected. The scripted controller reads object positions, so its results
must be labeled **privileged scripted baseline**, not policy accuracy.

### Local success criteria

All conditions must hold continuously for 0.4 seconds of control steps:

- **Pick/place:** object XY error below 4 cm, object on the table, translational
  speed below 0.07 m/s, released from gripper contact, prior bilateral jaw
  contact, and a prior object-center height above 7.5 cm
- **Dual pick/place:** the same conditions for both objects together
- **Lift:** object center above 12 cm, speed below 0.07 m/s, and prior bilateral
  jaw contact
- **Push:** XY error below 4 cm, object on the table, speed below 0.07 m/s, and
  no prior object-center height above 7.5 cm

For these 36 mm cubes, “on the table” means center height within 12 mm of
18 mm. Bilateral contact and lift are recorded as episode history; they are not
claims that both jaw contacts are present at every later instant.

A rollout terminates on success or truncates at the horizon/nonfinite state.
The reward is the success indicator minus mean object-to-goal XY distance.
These deliberately local definitions are not the official competition score.

### Read the output

`report.json` records the configuration, seeds, per-episode results, success
count, and success rate. `trace_seed_*.json` records step-level metrics such as
object/goal positions, XY distance, speed, maximum height, contact and lift
history, release state, actual-to-target EEF position error, and rate-limit flags.

Policy reports additionally include inference latencies and their p95, adopted
row count, action rate, translation frame, and invalid-output status/errors.
Latency is local `infer` wall time, not the official remote checker's end-to-end
latency. An invalid-output report needs inspection even if the process returned
normally. Save the command, versions, and complete report alongside comparisons.

## Visual and platform checks

Inspect at least an initial view, an airborne grasp, a released final state, and
both native-resolution wrist views. Check MP4 dimensions, frame rate, and actual
playback separately from still-image rendering. A video looking plausible does
not verify contact forces or calibrated optics.

GUI teleoperation needs an interactive desktop and a separate manual test.
Linux headless results alone do not verify the viewer, macOS, Windows, every
supported Python version, or every rendering backend. CI configuration shows
intended checks; only completed runs for a specific commit establish results.

## Interpretation limits

No external trained model or real hardware is required by these checks. The
base package does not train a model, download a gated dataset, or establish
hardware compatibility. Nominal mass/inertia, friction, controller gains,
convex rubber contact, box palm collision, uncalibrated cameras, absent arms,
and absent self/inter-hand collisions constrain what can be concluded.
See [model assumptions](model.md) before comparing policies or reusing the model.
