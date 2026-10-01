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

Jaw angle zero follows the operator-side
[yubi-sw](https://github.com/airoa-org/yubi-sw) `yubi_hand.urdf.xacro`, whose
encoder node publishes the glove angle measured from its calibrated fully closed
pose. The URDF finger meshes match the CAD jaws turned 7.5° (0.131 rad) closed
from the parallel CAD pose with a 0.7 mm median surface residual, so each jaw
body is pre-rotated by that angle and recorded `joint_states` map 1:1 onto jaw
angles. Glove values and gripper motor values are still not the same calibrated
quantity; `motor_scale` and `motor_offset` remain nominal.

Swept-solid checks of the pinned STEP give the robot gripper's travel in these
coordinates:

| Event | Jaw angle |
| --- | --- |
| Opposing rubber pads touch | 0.030 rad |
| CAD parallel attachments (17 mm rubber gap) | 0.131 rad |
| FINGER ATTACHMENT enters the UPPER PLATE pocket wall | ≈0.806 rad |

Commands are accepted over the glove range `0..0.94` rad. Because gripper
self-collision is not modeled, the hinge range `0.03..0.80` rad stands in for
pad contact and the opening stop, so larger commands drive the jaw into the
stop instead of past it. Measured servo zero and software position limits on a
deployed robot remain unverified.

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
- Palm collision uses a lightweight box spanning the case, servo, and servo
  bracket (root z from −54 to +7 mm); the camera tower and robot mount are visual-only.
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

| Parameter | Default | Source |
| --- | --- | --- |
| Palm/root body mass | 0.306 kg per hand | CAD estimate |
| Right / left jaw body mass | 0.119 / 0.116 kg | CAD estimate |
| Cube mass and edge length | 0.05 kg; 36 mm | assumption |
| Sliding friction coefficient | 1.2 | assumption |
| Gripper proportional gain | 20 N m/rad | tuning |
| Gripper torque limit | ±4.1 N m | XM430-W350 stall torque, 12 V |
| Gripper no-load speed | 4.8 rad/s | XM430-W350, 46 rpm at 12 V |
| Motor-to-jaw scale / offset | 1.0 / 0.0 | nominal |

Body masses, centers of mass, and full inertia tensors come from
`assets/mass_properties.json`, generated by `cad/mass_properties.py`. It takes
exact solid volumes and inertias of all 97 STEP leaves, BOM materials, and
catalog masses for the servo (82 g) and camera (about 30 g). Print fill factors
(0.75 for 3-wall, ≥50% infill parts; 0.60 otherwise), the rubber density, and the
A2024 finger attachments are assumptions. The palm includes the source UR5e
bracket and flange. Each jaw's 63 g steel gear keeps its center of mass about
16 mm from the hinge.

The servo drives the right jaw directly. Its torque-speed line is modeled as
back-EMF damping, `gripper_torque / gripper_speed`, on that joint, so a
saturated jaw cannot exceed the no-load speed. The idealized 1:1 spur gears
couple the left jaw. Armature, bearing damping, contact softness, weld stiffness,
and gravity compensation remain assumptions. The supply voltage, current limit,
and position gains of a deployed robot are unverified; the stall torque bounds
grasp force rather than calibrating it.

## Cameras

The wrist cameras are pinhole approximations at the CAD lens center,
(−3.1, 0, 40.1) mm in the root frame, looking along the CAD lens axis: root +X
pitched 10° toward −Z. The default vertical field of view is 90°; the source
ELP-USBFHD01M-L180 lens is a fisheye with a 187° diagonal field of view. They produce
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
