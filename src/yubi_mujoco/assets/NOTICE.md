# YUBI MuJoCo notices

Original simulation software: Copyright (c) 2026 YUBI simulation contributors,
MIT license. The root LICENSE does not relicense third-party assets.

Toyota-derived CAD and transformation/grouping metadata:
Copyright 2026 Toyota Motor Corporation, CERN-OHL-W-2.0.
Full license: licenses/CERN-OHL-W-2.0.txt.

Modification notice, 2026-10-01: tessellated the pinned robot-gripper STEP
assembly; partitioned 97 solid leaves into fixed palm and moving jaws;
converted millimeters to meters and re-expressed geometry in a canonical
root frame and verified hinge-local frames. No intentional solid-geometry
design changes. Distributed meshes are 13 disjoint material partitions;
redundant whole-body aggregate meshes are omitted. Approximate black/red
rendering colors follow the official assembled gripper reference.
mass_properties.json holds body mass, center-of-mass and inertia estimates
computed from the same STEP solids with stated material assumptions.
The CAD-derived meshes and transformation/grouping metadata retain
CERN-OHL-W-2.0. See SOURCE.md for complete source and regeneration instructions.

Published software/API/coordinate conventions were consulted from:
- airoa-org/yubi-sw, commit 1eccb041efc4d8966b03e4ccf4b402aa79d78be4
  Copyright 2026 AI Robot Association
- airoa-org/umi-arena-evaluation, commit 8763f10a022139dbcf4293b60f797a717d8bd1ad
These upstream references use Apache-2.0; a copy is in licenses/Apache-2.0.txt.
No pretrained weights or restricted datasets are redistributed.

OpenArm MuJoCo (https://github.com/enactic/openarm_mujoco) was consulted as a
packaging/usability reference; its code and robot assets are not included.

This is an independent project. It is not endorsed or certified by Toyota,
AIRoA, or Enactic. No trademark rights are granted. See the full licenses for
warranty disclaimers and other terms.
