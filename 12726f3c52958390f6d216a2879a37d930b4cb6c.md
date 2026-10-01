# Complete source and asset provenance

For a published 0.1.0 release, obtain its complete corresponding source as
`yubi_mujoco-0.1.0.tar.gz`, the same-version source distribution (sdist), at:
https://pypi.org/project/yubi-mujoco/0.1.0/#files

The sdist contains the unmodified source STEP, original hardware license and
commercial guidance, the CAD exporter, and all grouping/transformation data.
It must be published alongside the wheel. A wheel alone is not the complete
CAD source. Repository source is also intended at:
https://github.com/k1000dai/yubi_mujoco

Original hardware source: https://github.com/Toyota/yubi-hw
Pinned revision: dd8bd13d2fd8e5003057243576f88be333d95fc5
Original path: STEP/gripper/YUBI Gripper Assy_Dynamixel_ver2.STEP
SHA-256: 0e60bf62de970e5fae5e7cff104f547a954ef8d5d7b9dde9835b381b95f4fcc8
Copyright 2026 Toyota Motor Corporation. License: CERN-OHL-W-2.0.
See NOTICE.md for the dated modification notice and licenses/ for full terms.

## Regenerate

Extract the sdist, change into its root, and use a Python interpreter with
FreeCAD 1.0 native Python modules (Import, Part, and MeshPart) available:

    python cad/fetch_source.py
    /usr/bin/python3 cad/export.py

The first command verifies the bundled STEP; it downloads only that pinned
public source if it is absent. The exporter itself uses no network. Set
FREECAD_LIB for an alternate FreeCAD Python library location. FreeCAD is not
a runtime dependency and is installed separately, not as a pip extra.

The exporter verifies the source hash, all 97 expected solid leaves, their
unique body assignments, 8 mm shaft bores, and parallel-finger orientation.
Default tessellation: 0.12 mm linear deflection, 0.35 rad angular deflection.
Version-specific FreeCAD/OpenCascade tessellation may change mesh hashes.

Runtime meshes and corresponding SHA-256 hashes are listed in
cad_manifest.json; exact frame/pivot transforms are in cad_assembly.json.
Detailed geometry derivation is in docs/CAD_PROVENANCE.md in the sdist.
