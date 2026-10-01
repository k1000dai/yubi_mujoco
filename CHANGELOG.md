# Changelog

## 0.1.0 — initial release candidate

This version is prepared for an initial release. Availability on PyPI depends
on a successful publication; this entry alone does not indicate it is published.

### Added

- Installable `yubi_mujoco` package with `yubi-mujoco` and
  `python -m yubi_mujoco` entry points
- Arm-free, finite-stiffness mocap-driven YUBI hands with contact-based object
  manipulation and one coupled-jaw actuator per hand
- Toyota CAD-derived black/red grippers from 97 source solids, distributed as
  13 active material meshes with provenance, hashes, and hardware-license notices
- Seeded pick/place, dual pick/place, lift, and push scripted regressions
- Absolute hand-root pose/motor API and delta-pose policy adapter
- Local `Policy(checkpoint_dir).infer(obs)` evaluation with wrist images,
  configurable chunk adoption, explicit timing, and JSON diagnostics
- PNG scene rendering, optional MP4 recording, portable MJCF export, and
  keyboard teleoperation
- Real-time MuJoCo viewer for `demo` and `evaluate` (`--viewer`), with automatic
  `mjpython` relaunch on macOS for the viewer and teleoperation
- Minimal base dependencies: MuJoCo, NumPy, and SciPy; video dependencies in
  the optional `video` extra
- English usage, model-limit, validation, contributor, and release documentation
- Source-only CAD regeneration inputs/tools, separate from the runtime wheel
- uv project workflow with a cross-platform lockfile, a local development
  dependency group, pinned build tools, and locked CI installs

### Scope

The included demos use privileged simulator state. Motor mapping, dynamics,
contact, and camera models remain uncalibrated. This is an independent simulation,
not an official UMI Arena benchmark or evidence of real-robot policy performance.
