# Model and assumptions

This is an independent, arm-free contact simulation of the robot-mounted YUBI
gripper. Its visible shape is derived from Toyota CAD. Its dynamics, camera
model, and controller parameters are simulation assumptions unless explicitly
identified as source geometry. It is not certified by Toyota or AIRoA.

## Source geometry

The source is `STEP/gripper/YUBI Gripper Assy_Dynamixel_ver2.STEP` at
[Toyota/yubi-hw commit dd8bd13d2fd8e5003057243576f88be333d95fc5](https://github.com/Toyota/yubi-hw/tree/dd8bd13d2fd8e5003057243576f88be333d95fc5).
It is the robot gripper, not the wearable glove. Its 97 terminal solid components
are assigned once each to three rigid groups:

- Palm: 62 solids, including fixed structure, servo housing, camera, fasteners,
  and the source robot-mount/UR5e flange
- Right jaw: 20 solids, including its shaft, gear, attachment, pad, rubber, and flap
- Left jaw: 15 solids, including its shaft, attachment, pad, rubber, and flap

The model preserves the source assembly's asymmetric pads: 30 mm on the right
and 20 mm on the left. The flaps are fixed to the jaws, not extra articulated
fingers. Black/charcoal structure and flaps, red pads, and black contact rubber
follow the completed assembly reference; RGB values and lighting are approximate.

Only the **13 active, disjoint material meshes** are included in the runtime
package: three palm partitions and five partitions per jaw. They are a material
partition of the same 97 solids, not 13 source components. Loading aggregate and
partition meshes together would duplicate geometry.

The installed assets include the assembly transform, palette, source/mesh
manifest, and attribution. The original STEP is retained under `cad/source/`
with the regeneration tools in the source distribution, not the runtime wheel.
See [CAD provenance](CAD_PROVENANCE.md) and the installed
[source notice](../src/yubi_mujoco/assets/SOURCE.md).

## Coordinates and articulation

The exporter derives the axes from the actual 8 mm-diameter shaft bores. The
hinge centers are 30 mm apart. Millimetres are converted to metres, and geometry
is re-expressed in a canonical hand-root frame:

```text
hand +X = source CAD +Z  (along the fingers)
hand +Y = source CAD +X  (toward the left jaw)
hand +Z = source CAD +Y  (joint-axis/camera direction)
```

The root-to-hinge positions are `(-0.016, -0.015, 0)` m for the right jaw and
`(-0.016, +0.015, 0)` m for the left jaw. The 16 mm root offset is a coordinate
convention, not a measured registration to a deployed robot's end-effector
frame. Jaw vertices are already hinge-local and all vertices are in metres;
do not subtract the hinge or apply the millimetre scale a second time.

There is one independently commanded jaw coordinate per hand. The two idealized
gears couple equal and opposite hinge coordinates. In the generated MJCF both
hinge axes are local −Z; the right coordinate is positive on opening and the
left coordinate is its negative. The model has two gripper actuators total.

At geometric zero the finger attachments are parallel. The opposing rubber has
about 17 mm separation between its innermost bounding planes; the curved pads
have variable separation along their length. This is not a fully closed grasp
or a calibrated aperture measurement. The default `0..0.94` rad command range
comes from an operator-side glove URDF reference, not measured robot stops.

## Pose control without arms

Each hand is a dynamic free body following a mocap target through a
finite-stiffness weld. Pose commands change mocap targets; they do not overwrite
the dynamic hand or object pose. Actual hand motion can lag its target under
contact. Target interpolation is limited to 0.6 m/s translation and 3 rad/s
rotation by default.

Objects use MuJoCo free joints and ordinary contact. Reset initializes their
poses; stepping does not teleport them or add grasp-attachment constraints.
The scripted controller receives object state and then uses the same public
pose/motor action path as other controllers.

There are no robot arms, inverse kinematics, arm joint limits, reachability
checks, arm self-collisions, or hardware stop logic. The rectangular simulator
workspace is only an input bound. It does not establish a feasible or safe real
robot motion.

## Collision geometry

Visual geometry is more detailed than collision geometry:

- Each jaw's rubber solid supplies a mesh collision shape, convexified by
  MuJoCo. This fills part of the rubber's concavity; mesh-based checks found
  deviations of approximately 3.3 mm in some local sections.
- Palm collision uses a lightweight box approximation.
- Other structural components are visual-only.
- Gripper self-collision and inter-hand collision are not modeled.

Consequently, a camera or visual finger component can pass through an obstacle
without a corresponding contact force. A successful simulated grasp does not
validate real grasp clearance, rubber deformation, gear contact, backlash,
contact force, or collision safety.

## Nominal dynamics

The default physics timestep is 1/600 s. Control runs at 30 Hz unless changed to
10 or 60 Hz. Integer physics substeps preserve the chosen control interval.
The following are tuning choices, not identified physical parameters:

| Parameter | Default |
| --- | --- |
| Palm/root body mass | 0.32 kg per hand |
| Each jaw body mass | 0.035 kg |
| Cube mass and edge length | 0.05 kg; 36 mm |
| Sliding friction coefficient | 1.2 |
| Gripper proportional gain | 4.0 |
| Gripper torque limit | ±1 Nm |
| Motor-to-jaw scale / offset | 1.0 / 0.0 |

Inertias, damping, contact softness, weld stiffness, actuator gains, and gravity
compensation are also assumptions. The 0.32 kg value is the root body's assigned
mass, not a measured total mass of an assembled gripper. The servo's published
stall torque is not a continuous grasp-force calibration for this model.

## Cameras

The wrist cameras are pinhole approximations located 42.6 mm along root +Z,
looking along root +X, with a default 90° vertical field of view. They produce
640×480 RGB images for policy observations. The source hardware uses fisheye
cameras; matching array dimensions does not reproduce those optics.

Not calibrated: intrinsic matrices, distortion, optical extrinsics, root/world
registration, exposure, color response, lighting, motion blur, transport delay,
and sensor noise. The overview and top cameras are visualization aids.

## What a simulation score means

The four included tasks are small synthetic cube-manipulation regressions.
The bundled baseline knows the objects' world positions and is not a trained
vision policy. A successful rollout shows that the configured simulator and
controller satisfied this project's criteria under that seed. It does not
establish UMI Arena performance, policy generalization, sim-to-real rankings, or
hardware deployment safety. See [validation](validation.md) for the criteria.

Before using results to reason about real hardware, independently establish:

1. The exact hardware revision, pad/rubber build, and hand-root registration
2. Motor-coordinate sign, scale, zero, stops, and aperture relationship
3. Camera intrinsics, distortion, extrinsics, and image-processing pipeline
4. Mass/inertia, contact, friction, compliance, actuator dynamics, and delay
5. Real object/task distributions and scoring conditions
6. Controller timing, chunk adoption, limits, and arm feasibility

## Regenerating assets

Normal installation uses prebuilt meshes and does not import FreeCAD. Source
maintainers can reproduce them using the pinned input and a Python interpreter
with FreeCAD's native modules:

```bash
python cad/fetch_source.py
/usr/bin/python3 cad/export.py
# If needed for your installation:
FREECAD_LIB=/path/to/freecad/lib /path/to/python cad/export.py
```

Use the provenance guide for the current flags, source checks, and exporter
requirements. The STEP digest, leaf membership, and hinge geometry must pass
validation before an intentional mesh update is accepted. Tessellation output
can differ across FreeCAD/OpenCascade versions; record tool versions and new
hashes instead of assuming cross-version bit-for-bit identity.

## Licensing boundaries

Original simulation code is MIT-licensed. Toyota-derived hardware geometry,
meshes, and transformation/grouping data remain under CERN-OHL-W-2.0, with
Copyright 2026 Toyota Motor Corporation. Keep their provenance and source
notices when redistributing them. Referenced upstream software projects use
Apache-2.0. See the [attribution notice](../src/yubi_mujoco/assets/NOTICE.md) and
[CAD provenance](CAD_PROVENANCE.md) for the source pins and license texts.
