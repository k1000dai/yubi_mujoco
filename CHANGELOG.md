# Changelog

## Unreleased

### Changed

- Jaw angle zero is now the closed pose of yubi-sw's `yubi_hand.urdf.xacro`
  (7.5° closed from the CAD parallel jaws), so recorded glove `joint_states`
  map 1:1 onto jaw angles; the default open command is 0.68 rad
- Jaw hinges stop at 0.03 rad (pads touch) and 0.80 rad (CAD opening
  interference); commands up to the 0.94 rad glove range are still accepted
- Body masses and full inertias come from the pinned STEP via the new
  `cad/mass_properties.py` and `mass_properties.json` asset
- Gripper defaults follow the DYNAMIXEL XM430-W350: 4.1 N m stall torque and a
  4.8 rad/s no-load torque-speed line (`gripper_speed`); gain raised to 20
- Wrist camera moved to the CAD lens center with its 10° downward pitch
- Palm collision box extended to the servo bracket's underside

### Removed

- Keyboard `teleop` command; teleoperation belongs in a separate repository

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
- PNG scene rendering, optional MP4 recording, portable MJCF export
- Real-time MuJoCo viewer for `demo` and `evaluate` (`--viewer`), with automatic
  `mjpython` relaunch on macOS for the viewer
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
