# YUBI robot-gripper CAD provenance

## Original source and notices

Copyright 2026 Toyota Motor Corporation

The meshes in `src/yubi_mujoco/assets/meshes/cad_*.stl` are derived from the
**robot-mounted YUBI Gripper**, not the wearable YUBI Glove.

- Source location: <https://github.com/Toyota/yubi-hw>
- Revision: `dd8bd13d2fd8e5003057243576f88be333d95fc5`
- Source file: `STEP/gripper/YUBI Gripper Assy_Dynamixel_ver2.STEP`
- Source SHA-256: `0e60bf62de970e5fae5e7cff104f547a954ef8d5d7b9dde9835b381b95f4fcc8`
- Original source and license are retained in `cad/source/`
- Hardware license: **CERN Open Hardware Licence Version 2 – Weakly Reciprocal**
  (`CERN-OHL-W-2.0`). Full original notice/license: [`../cad/source/LICENSE`](../cad/source/LICENSE)
- Original commercial-use guidance:
  [`../cad/source/COMMERCIAL.md`](../cad/source/COMMERCIAL.md)
- Original assembly instructions:
  [`YUBI Gripper_DYNAMIXEL_AssemblyGuide.pdf`](https://github.com/Toyota/yubi-hw/blob/dd8bd13d2fd8e5003057243576f88be333d95fc5/docs/AssemblyInstruction/YUBI%20Gripper_DYNAMIXEL_AssemblyGuide.pdf)

Modification notice, **2026-10-01**, by the YUBI simulation implementation:
tessellated STEP solid faces; regrouped components into rigid simulated bodies;
converted millimeters to meters; re-expressed the assembly in a canonical frame
and the jaws in their hinge-local frames. No intentional solid-geometry design
changes were made. These derived CAD meshes and their transformation/grouping
information retain CERN-OHL-W-2.0. The regeneration script and pinned source are
provided alongside the meshes as the complete reproducible transformation.

This independent simulation is based on the YUBI open-source hardware project.
Toyota Motor Corporation does not endorse or certify it. Original design and
derived simulation assets are provided without warranties; the original license
contains the complete disclaimer. The license grants no trademark rights.

## What was exported

The STEP assembly contains **97 solid leaf components**. FreeCAD also exposes
five `App::Part` assembly/subassembly aggregate shapes, which repeat their
children, plus reference axes and planes. Only terminal solid-bearing leaves
were exported. The exporter verifies the expected leaf identities, rejects
changed import structure, and assigns each leaf to exactly one primary body:

- Fixed palm: fixed structure, servo housing, camera, bearings, robot mount,
  UR5e flange, and fixed fasteners (62 solids)
- Right jaw: right shaft, gear, attachment, t30 pad, rubber, rigid flap,
  keys and associated moving fasteners (20 solids)
- Left jaw: left shaft, gear, attachment, t20 pad, rubber, rigid flap,
  keys and associated moving fasteners (15 solids)

The source assembly is asymmetric: it contains a **30 mm-deep right pad** and a
**20 mm-deep left pad**. The asymmetry has been preserved. The flaps are rigidly
screwed to the attachments; they are not extra articulated fingers.

These bodies are distributed as 13 disjoint material-level submeshes:

- Palm: `cad_palm_structure.stl`, `cad_palm_servo.stl`, `cad_palm_camera.stl`
- Each jaw: `cad_{side}_jaw_attachment.stl`, `_pad.stl`, `_rubber.stl`,
  `_flap.stl`, `_hardware.stl`

Only material-level partitions are distributed; the three redundant aggregate meshes
are deliberately omitted. Together these 13 partitions cover the 97 source solids. Visual colors follow the official completed black/red gripper assembly, using
approximate RGB values; see the pinned reference and palette in `src/yubi_mujoco/assets/colors.json`. The STEP-derived STL
triangulation itself has no material colors.

Full leaf-to-body mapping, per-file hashes, triangle counts, local bounding boxes,
tessellation settings and source provenance are in
[`../src/yubi_mujoco/assets/cad_manifest.json`](../src/yubi_mujoco/assets/cad_manifest.json).

## Coordinate derivation and hinge verification

The input STEP uses millimeters and is translated far from its CAD origin.
Geometry is **not** scaled by guessed image dimensions or aligned by eye.

The script locates the radius-4 mm cylindrical shaft bores of the two
`FINGER ATTACHMENT` solids, checking that their axes are parallel to CAD Y. Their
axis lines are:

- Right: CAD X = 839.0706616294 mm, CAD Z = 1270.5734593074 mm
- Left: CAD X = 869.0706616289 mm, CAD Z = 1270.5734593077 mm
- Separation: 30.000000 mm within STEP numerical precision
- Both axis directions: CAD `(0, 1, 0)`

Each attachment has two radius-0.8 mm flap mounting holes, separated by 18 mm.
The vector between their centers is CAD `(0, 0, 18)` mm. This independently
verifies that both attachments point parallel along CAD +Z in the supplied
assembly. The script asserts these observations on every export.

The canonical right-handed rotation is:

```text
canonical +x = CAD +Z      (along the fingers)
canonical +y = CAD +X      (toward source FINGER ATTACHMENT_L)
canonical +z = CAD +Y      (joint axes, toward the camera)

R = [[0, 0, 1],
     [1, 0, 0],
     [0, 1, 0]]

O_cad_mm ≈ [854.0706616292, 856.5046236935, 1286.5734593075]
canonical_m = 0.001 * R @ (cad_mm - O_cad_mm)
```

CAD origin X is the average of the shaft axes. Origin Y is the average
mid-depth of the two attachments. Origin Z is 16 mm forward of the average
shaft-axis Z. The **16 mm reference offset is a coordinate convention**, chosen
to match the glove URDF's familiar root/hinge coordinates; it is not an additional
measurement of the robot geometry. The measured 30 mm hinge spacing agrees with
that convention. The exact computed values and geometric evidence are recorded
in [`../src/yubi_mujoco/assets/cad_assembly.json`](../src/yubi_mujoco/assets/cad_assembly.json).

Resulting parent-body hinge positions in meters are:

```text
right_jaw = (-0.016, -0.015, 0)
left_jaw  = (-0.016, +0.015, 0)
```

Palm mesh coordinates are in the canonical root frame. Jaw mesh coordinates
are **already relative to their own hinge**. Place each jaw body at its hinge
position with identity orientation and use the corresponding STL at zero geom
position. Do not subtract the hinge a second time. All STL vertices are in
meters; do not apply a second `0.001` mesh scale.

With a +Z joint axis on both bodies, opening requires positive left-jaw angle
and negative right-jaw angle. The mechanism has one servo and two equal gears,
so the idealized relationship is `q_right = -q_left`. Gear tooth contact is not
needed to realize that kinematic constraint.

## Neutral pose, scale and limits

**CAD neutral is the supplied parallel-attachment pose**, with both geometric
joint angles zero. It is not a measured encoder zero and is not fully closed.
The opposing rubber meshes have a 17 mm gap between their innermost global
bounding planes in this pose. The curved pads have variable separation along
their length, so that value is not a universal fingertip aperture or a complete
contact model.

Neutral assembly extents after export, in meters:

```text
x: -0.061000 to +0.089139   overall length ≈ 150.139 mm
y: -0.046000 to +0.046000   overall width  = 92.000 mm
z: -0.056500 to +0.060868   overall depth ≈ 117.368 mm
```

Jaw tip maximum x is approximately 105.139 mm from its hinge. The exported
assembly includes the source UR5e mounting flange. A different robot flange
must be modelled explicitly if required; none is silently substituted.

The separate YUBI software glove URDF uses a nominal opening range of
`0..0.94 rad`. That does **not** establish the robot gripper's motor zero,
mechanical stops, safe travel or calibrated aperture. `cad_assembly.json`
explicitly marks robot zero and joint limits unverified. If a demonstration
uses the glove range, it is a documented nominal simulation assumption.

The meshes preserve visible assembly shape. They do not establish mass density,
inertia, motor control gains, friction, rubber compliance, gear backlash,
bearing behavior, self-collision exclusions, contact proxies or actuator
calibration. Those require independent modelling and validation. In particular,
do not use the convex hull of a complete jaw/palm assembly as an accurate
contact representation of its concave surfaces.

Each separate rubber mesh is one source solid. Convexifying just that rubber
solid is a useful simplified contact model, but still fills its concave profile.
At the central-depth plane (canonical z=0), the exported left/right inner
profiles differ from their convex envelopes by approximately 3.3 mm per jaw at
root x=0.040 m, 3.1 mm at x=0.060 m, and 1.0 mm at x=0.070 m. The forward
section at x=0.075–0.085 m lies on its convex envelope. These are mesh-based
geometric checks, not measured rubber behavior. A 36 mm cube centered at root
x=0.070 m spans x=0.052–0.088 m and fits inside the neutral pad-tip x extent.

## Source distribution

The wheel intentionally excludes the 8 MB original STEP. The same-version sdist
contains it at `cad/source/`, together with the exporter, configuration, and this
document. Both distributions include source-location and license notices in
`yubi_mujoco/assets/SOURCE.md` and `NOTICE.md`. The sdist must be published with
the wheel so the full source remains available to recipients.

## Reproduce and verify

Runtime simulation uses the committed binary STL assets and does not need
FreeCAD. Regeneration uses FreeCAD's OpenCascade tessellation through Python,
with 0.12 mm linear deflection, 0.35 rad angular deflection, and absolute rather
than relative deflection:

```sh
# From the repository root, using a Python with FreeCAD's native libraries.
/usr/bin/python3 cad/export.py

# Optional location for another FreeCAD installation.
FREECAD_LIB=/path/to/freecad/lib /path/to/python cad/export.py
```

The source STEP hash is checked before import. No network access is used.
`--linear-deflection-mm` and `--angular-deflection-rad` permit intentional mesh
resolution changes; new mesh hashes/settings are written to the manifest.
Binary STL numeric content can vary across FreeCAD/OpenCascade versions; the
actual FreeCAD version is recorded rather than claiming cross-version
bit-for-bit identity.

Verification performed on 2026-10-01:

- All 97 expected leaf solids assigned exactly once across three rigid-body groups
- Both radius-4 mm shaft bores and 18 mm flap-hole direction checked analytically
- CAD-to-canonical rotation is a proper cyclic axis permutation (determinant +1)
- Aggregate bounds checked against source assembly dimensions
- The earlier exporter regenerated twice with matching SHA-256 hashes for all 16
  meshes (13 material partitions plus three aggregate alternatives); the runtime
  distribution now retains only the 13 material partitions
- All 13 disjoint material meshes compiled in MuJoCo 3.14.0
- Neutral `q_left=q_right=0` and open `q_left=+0.5, q_right=-0.5` rendered and
  visually inspected; jaws rotate about their actual shaft axes and their
  attachments, pads, rubber and flaps stay assembled

This validates the export and the geometric articulation reference. It is not
physical validation of the robot.
