#!/usr/bin/env python3
"""Estimate rigid-body mass properties from the pinned YUBI robot-gripper STEP.

Optional build-time dependency: OpenCascade Python bindings (cadquery-ocp).
The simulator only reads the generated JSON and never imports OpenCascade.
Example:

    uv run --no-project --with cadquery-ocp --with numpy python cad/mass_properties.py

Each of the 97 solid leaves gets a material from the yubi-hw v2 BOM. Exact solid
volume, centroid and inertia come from OpenCascade; densities and 3D-print fill
factors are stated assumptions. The DYNAMIXEL and camera use catalog masses
spread over their CAD volume. Leaf-to-body membership is shared with export.py.

Derived CAD: Copyright 2026 Toyota Motor Corporation; CERN-OHL-W-2.0.
Source: https://github.com/Toyota/yubi-hw
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from export import COMMIT, MOVING, SOURCE, SOURCE_SHA256  # noqa: E402

ASSETS = ROOT / "src/yubi_mujoco/assets"
PLA = 1.24  # g/cm^3
# Build settings PDF (yubi-hw STL/gripper): CASE, UPPER PLATE and pads use
# 3 walls with >=50% infill; other prints use slicer defaults.
PRINT_FILL_DENSE = 0.75
PRINT_FILL_DEFAULT = 0.60
SERVO_MASS_G = 82.0  # ROBOTIS e-Manual, XM430-W350
CAMERA_MASS_G = 30.0  # ELP-USBFHD01M-L180 listing, "about 30g"

# (label prefix, material, g/cm^3 or None for catalog-mass parts)
MATERIALS = [
    ("DYNAMIXEL", "catalog: DYNAMIXEL XM430-W350 82 g", None),
    ("USB CAMERA", "catalog: ELP-USBFHD01M-L180 about 30 g", None),
    ("CASE", "PLA, 3 walls >=50% infill", PLA * PRINT_FILL_DENSE),
    ("UPPER PLATE", "PLA, 3 walls >=50% infill", PLA * PRINT_FILL_DENSE),
    ("PAD_", "PLA, 3 walls >=50% infill", PLA * PRINT_FILL_DENSE),
    ("BRACKET_DYNAMIXEL", "PLA, default print settings", PLA * PRINT_FILL_DEFAULT),
    ("FLAP_", "PLA, default print settings", PLA * PRINT_FILL_DEFAULT),
    ("FINGER ATTACHMENT", "A2024 (machined option)", 2.78),
    ("GEAR SHAFT", "A2024", 2.78),
    ("BRACKET_GRIPPER", "A2024", 2.78),
    ("UR5e_FLANGE", "A2024", 2.78),
    ("WSSAB", "A2017 washer", 2.79),
    ("SB-", "brass insert", 8.5),
    ("RUBBER SHEET", "HyperV rubber sheet (assumed density)", 1.2),
    ("SSFRHQ", "SUS304 shaft", 7.93),
    ("XDSHC6", "SUS316L pin", 7.98),
    ("MTA05", "stainless bearing", 7.75),
    ("GEAKB", "S45C spur gear", 7.85),
    ("KEG", "S45C key", 7.85),
    ("CB", "SCM435 screw", 7.85),
    ("JP", "SKS3 pin", 7.85),
]


def material(part):
    for prefix, name, density in MATERIALS:
        if part.startswith(prefix):
            return name, density
    raise ValueError(f"No BOM material for STEP part {part!r}")


def read_solids(path):
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.collections import Sequence_TDF_Label

    doc = TDocStd_Document(TCollection_ExtendedString("yubi"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(path)) != 1:
        raise SystemExit(f"Cannot read {path}")
    reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        attr = TDataStd_Name()
        found = label.FindAttribute(TDataStd_Name.GetID_s(), attr)
        return attr.Get().ToExtString() if found else ""

    solids = []

    def walk(label, location):
        if tool.IsReference_s(label):
            referred = TDF_Label()
            tool.GetReferredShape_s(label, referred)
            label, location = referred, location * tool.GetLocation_s(label)
        if tool.IsAssembly_s(label):
            children = Sequence_TDF_Label()
            tool.GetComponents_s(label, children)
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), location)
            return
        explorer = TopExp_Explorer(tool.GetShape_s(label).Moved(location), TopAbs_SOLID)
        while explorer.More():
            props = GProp_GProps()
            BRepGProp.VolumeProperties_s(explorer.Current(), props)
            c, m = props.CentreOfMass(), props.MatrixOfInertia()
            solids.append(
                {
                    "part": name(label),
                    "volume_mm3": props.Mass(),
                    "com_mm": np.array([c.X(), c.Y(), c.Z()]),
                    "inertia_mm5": np.array(
                        [[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]
                    ),
                }
            )
            explorer.Next()

    roots = Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location())
    return solids


def main():
    source = ROOT / SOURCE
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise SystemExit("Source STEP hash differs from the pinned version")
    assembly = json.loads((ASSETS / "cad_assembly.json").read_text())
    manifest = json.loads((ASSETS / "cad_manifest.json").read_text())
    solids = read_solids(source)
    total = sum(s["volume_mm3"] for s in solids)
    if len(solids) != 97 or abs(total - manifest["cad_volume_mm3"]) > 1e-3:
        raise ValueError("STEP solid traversal does not match the exporter's 97 leaves")
    # The traversal order must equal FreeCAD's Part__Feature numbering used by MOVING.
    labels = {
        int(leaf["name"].removeprefix("Part__Feature") or 0): leaf["label"]
        for record in manifest["meshes"].values()
        for leaf in record["leaf_features"]
    }
    for index, solid in enumerate(solids):
        stem = solid["part"].split("-")[0].split("_")[0]
        if not labels[index].startswith(stem):
            raise ValueError(f"Leaf {index}: STEP {solid['part']!r} vs mesh {labels[index]!r}")

    servo_cm3 = sum(s["volume_mm3"] for s in solids if s["part"].startswith("DYNAMIXEL")) / 1e3
    camera_cm3 = sum(s["volume_mm3"] for s in solids if s["part"].startswith("USB CAMERA")) / 1e3
    origin = np.array(assembly["cad_origin_mm"])
    rotation = np.array(assembly["cad_to_canonical_rotation"], dtype=float)
    groups = {**MOVING, "palm": [i for i in range(97) if all(i not in v for v in MOVING.values())]}
    bodies = {}
    for body, indices in sorted(groups.items()):
        frame = np.array(assembly["bodies"][body]["origin_m"] if body != "palm" else [0, 0, 0])
        items, breakdown = [], {}
        for i in indices:
            solid = solids[i]
            name, density = material(solid["part"])
            if density is None:
                density = (
                    SERVO_MASS_G / servo_cm3
                    if solid["part"].startswith("DYNAMIXEL")
                    else CAMERA_MASS_G / camera_cm3
                )
            rho = density * 1e-3  # g/mm^3
            mass = solid["volume_mm3"] * rho * 1e-3  # kg
            com = rotation @ (solid["com_mm"] - origin) * 1e-3 - frame  # m, body frame
            inertia = rotation @ solid["inertia_mm5"] @ rotation.T * rho * 1e-9  # kg m^2
            items.append((mass, com, inertia))
            breakdown[name] = breakdown.get(name, 0.0) + mass
        mass = sum(m for m, _, _ in items)
        com = sum(m * c for m, c, _ in items) / mass
        inertia = sum(
            i + m * ((c - com) @ (c - com) * np.eye(3) - np.outer(c - com, c - com))
            for m, c, i in items
        )
        bodies[body] = {
            "frame": "canonical_root" if body == "palm" else "joint_local_cad_neutral",
            "mass_kg": round(mass, 6),
            "com_m": [round(float(v), 7) for v in com],
            "fullinertia_kg_m2": [
                float(f"{v:.6e}")
                for v in (
                    inertia[0, 0],
                    inertia[1, 1],
                    inertia[2, 2],
                    inertia[0, 1],
                    inertia[0, 2],
                    inertia[1, 2],
                )
            ],
            "fullinertia_order": "xx yy zz xy xz yz, about the center of mass",
            "mass_breakdown_kg": {k: round(v, 5) for k, v in sorted(breakdown.items())},
        }
    result = {
        "schema_version": 1,
        "source_commit": COMMIT,
        "source_step": str(SOURCE),
        "source_sha256": SOURCE_SHA256,
        "generator": "cad/mass_properties.py",
        "method": "Exact OpenCascade solid volume, centroid and inertia per STEP leaf; "
        "BOM material densities; catalog masses for the servo and camera.",
        "assumptions": {
            "pla_density_g_cm3": PLA,
            "print_fill_dense": PRINT_FILL_DENSE,
            "print_fill_default": PRINT_FILL_DEFAULT,
            "servo_mass_g": SERVO_MASS_G,
            "camera_mass_g": CAMERA_MASS_G,
            "finger_attachment": "A2024; printed PLA attachments lower each jaw by about 17 g",
            "palm_includes_ur5e_mount": True,
        },
        "bodies": bodies,
    }
    target = ASSETS / "mass_properties.json"
    target.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: (v["mass_kg"], v["com_m"]) for k, v in bodies.items()}, indent=2))


if __name__ == "__main__":
    main()
