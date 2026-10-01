#!/usr/bin/env python3
"""Rebuild meter-scale meshes from the pinned YUBI robot-gripper STEP assembly.

Optional build-time dependency: FreeCAD (including Import, Part and MeshPart).
The simulator uses only the exported STL files and never imports FreeCAD.
Example on Debian: /usr/bin/python3 cad/export.py

Derived CAD: Copyright 2026 Toyota Motor Corporation; CERN-OHL-W-2.0.
Source: https://github.com/Toyota/yubi-hw
Modification notice, 2026-10-01: tessellated and regrouped by rigid body,
converted millimeters to meters, and re-expressed about verified hinge axes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path("cad/source/YUBI Gripper Assy_Dynamixel_ver2.STEP")
SOURCE_SHA256 = "0e60bf62de970e5fae5e7cff104f547a954ef8d5d7b9dde9835b381b95f4fcc8"
COMMIT = "dd8bd13d2fd8e5003057243576f88be333d95fc5"

# Exact leaf-feature membership for the pinned assembly. Components not listed
# here belong to the fixed palm. Bearing outer geometry is fixed; shaft, keys,
# gears, gear screws, attachments, pads and flaps follow their respective jaw.
MOVING = {
    "right_jaw": [8, 13, 14, 17, 19, 21, 26, 33, 53, 54, 57, 58, 69, 70, 71, 72, 73, 74, 76, 77],
    "left_jaw": [12, 15, 16, 18, 20, 22, 25, 34, 51, 52, 59, 60, 75, 78, 79],
}
SUBPARTS = {
    "right_jaw_attachment": [17],
    "left_jaw_attachment": [18],
    "right_jaw_pad": [19],
    "left_jaw_pad": [20],
    "right_jaw_rubber": [21],
    "left_jaw_rubber": [22],
    "right_jaw_flap": [33],
    "left_jaw_flap": [34],
    "palm_servo": list(range(1, 8)),
    "palm_camera": list(range(28, 33)),
}


def feature_name(index):
    return "Part__Feature" + (f"{index:03d}" if index else "")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vec(value):
    return [float(value.x), float(value.y), float(value.z)]


def mean(values):
    return [sum(v[i] for v in values) / len(values) for i in range(3)]


def cylinder_centers(shape, Part, radius, direction):
    centers = []
    for face in shape.Faces:
        surface = face.Surface
        if not isinstance(surface, Part.Cylinder):
            continue
        if abs(surface.Radius - radius) > 1e-5:
            continue
        axis = vec(surface.Axis)
        if abs(abs(sum(a * b for a, b in zip(axis, direction))) - 1) > 1e-7:
            continue
        c = vec(surface.Center)
        if not any(sum((a - b) ** 2 for a, b in zip(c, d)) < 1e-10 for d in centers):
            centers.append(c)
    return centers


def derive_frame(leaves, Part):
    """Derive both axis lines and the CAD parallel-finger pose from real faces."""
    hinges = {}
    directions = {}
    depth_centers = []
    evidence = {}
    for side, index in (("right_jaw", 17), ("left_jaw", 18)):
        shape = leaves[feature_name(index)].Shape
        centers = cylinder_centers(shape, Part, 4.0, [0, 1, 0])
        if len(centers) != 1:
            raise ValueError(f"Expected one unique 8 mm shaft bore in {side}: {centers}")
        hinges[side] = centers[0]
        holes = sorted(cylinder_centers(shape, Part, 0.8, [0, 1, 0]), key=lambda p: p[2])
        if len(holes) != 2:
            raise ValueError(f"Expected two flap screw holes in {side}: {holes}")
        delta = [b - a for a, b in zip(holes[0], holes[1])]
        length = math.sqrt(sum(d * d for d in delta))
        direction = [d / length for d in delta]
        if max(abs(a - b) for a, b in zip(direction, [0, 0, 1])) > 1e-6:
            raise ValueError(f"CAD neutral is not parallel +CAD Z: {direction}")
        directions[side] = direction
        bb = shape.BoundBox
        depth_centers.append((bb.YMin + bb.YMax) / 2)
        evidence[side] = {
            "shaft_bore_radius_mm": 4.0,
            "shaft_bore_center_mm": centers[0],
            "shaft_axis_cad": [0, 1, 0],
            "flap_screw_centers_mm": holes,
            "flap_screw_spacing_mm": length,
            "finger_direction_cad": direction,
        }
    midpoint = mean(list(hinges.values()))
    # The origin's +16 mm offset along fingers deliberately matches the naming
    # convention in yubi_hand.urdf.xacro; the 30 mm separation is measured here.
    origin = [midpoint[0], sum(depth_centers) / 2, midpoint[2] + 16.0]
    rotation = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]

    def canonical(p):
        d = [p[i] - origin[i] for i in range(3)]
        return [sum(r[i] * d[i] for i in range(3)) * 0.001 for r in rotation]

    pivots = {side: canonical([c[0], origin[1], c[2]]) for side, c in hinges.items()}
    separation = abs(hinges["left_jaw"][0] - hinges["right_jaw"][0])
    if abs(separation - 30) > 1e-5:
        raise ValueError(f"Unexpected hinge separation: {separation}")
    return origin, rotation, pivots, evidence, canonical


def bounds(vertices):
    return [
        [min(p[i] for p in vertices) for i in range(3)],
        [max(p[i] for p in vertices) for i in range(3)],
    ]


def write_stl(path, triangles):
    header = b"YUBI CAD; Toyota 2026; CERN-OHL-W-2.0; metres; see CAD_PROVENANCE.md"
    with path.open("wb") as f:
        f.write(header.ljust(80, b" ")[:80])
        f.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            length = math.sqrt(sum(t * t for t in n))
            n = [t / length for t in n] if length else [0.0, 0.0, 0.0]
            f.write(struct.pack("<12fH", *(n + list(a) + list(b) + list(c)), 0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--freecad-lib", default=os.environ.get("FREECAD_LIB", "/usr/lib/freecad/lib")
    )
    parser.add_argument("--linear-deflection-mm", type=float, default=0.12)
    parser.add_argument("--angular-deflection-rad", type=float, default=0.35)
    args = parser.parse_args()
    if args.linear_deflection_mm <= 0 or args.angular_deflection_rad <= 0:
        parser.error("Deflections must be positive")
    source = ROOT / SOURCE
    if sha256(source) != SOURCE_SHA256:
        raise SystemExit(
            "Source STEP hash differs from pinned version; inspect grouping before regenerating"
        )
    sys.path.append(args.freecad_lib)
    try:
        import FreeCAD as App
        import Import
        import MeshPart
        import Part
    except ImportError as exc:
        raise SystemExit(
            "Regeneration requires FreeCAD Python libraries. Try /usr/bin/python3 "
            "cad/export.py or set FREECAD_LIB. Runtime simulation does not need it."
        ) from exc
    doc = App.newDocument("YubiCadExport")
    try:
        Import.insert(str(source), doc.Name)
        # Import creates App::Part aggregate Shapes containing their children.
        # Select terminal solid-bearing Part::Feature objects exactly once.
        leaves = {
            o.Name: o
            for o in doc.Objects
            if hasattr(o, "Shape")
            and o.Shape.Solids
            and not any(hasattr(c, "Shape") and c.Shape.Solids for c in o.OutList)
        }
        expected = {feature_name(i) for i in range(97)}
        if set(leaves) != expected or any(len(o.Shape.Solids) != 1 for o in leaves.values()):
            raise ValueError(
                "STEP import leaf structure changed; refusing ambiguous/duplicate geometry"
            )
        origin, rotation, pivots, evidence, canonical = derive_frame(leaves, Part)
        moving = {k: {feature_name(i) for i in ids} for k, ids in MOVING.items()}
        if moving["left_jaw"] & moving["right_jaw"]:
            raise ValueError("A CAD leaf was assigned to both jaws")
        groups = {**moving, "palm": set(leaves) - set.union(*moving.values())}
        # Optional material-level submeshes are an alternative to the three
        # aggregate meshes, not additional geometry to overlay on them.
        optional = {k: {feature_name(i) for i in ids} for k, ids in SUBPARTS.items()}
        for side in ("left_jaw", "right_jaw"):
            used = set.union(*(v for k, v in optional.items() if k.startswith(side)))
            optional[side + "_hardware"] = groups[side] - used
        optional["palm_structure"] = (
            groups["palm"] - optional["palm_servo"] - optional["palm_camera"]
        )
        tessellations = {}
        for name, obj in sorted(leaves.items()):
            mesh = MeshPart.meshFromShape(
                Shape=obj.Shape,
                LinearDeflection=args.linear_deflection_mm,
                AngularDeflection=args.angular_deflection_rad,
                Relative=False,
            )
            points, facets = mesh.Topology
            points = [canonical(vec(p)) for p in points]
            tessellations[name] = [[points[i] for i in facet] for facet in facets]
        dest = ROOT / "src/yubi_mujoco/assets/meshes"
        dest.mkdir(parents=True, exist_ok=True)
        records = {}
        for group, members in sorted(optional.items()):
            body = next((b for b in ("left_jaw", "right_jaw") if group.startswith(b)), "palm")
            offset = pivots.get(body, [0.0, 0.0, 0.0])
            triangles = [
                [[p[i] - offset[i] for i in range(3)] for p in tri]
                for name in sorted(members)
                for tri in tessellations[name]
            ]
            path = dest / f"cad_{group}.stl"
            write_stl(path, triangles)
            vertices = [p for t in triangles for p in t]
            records[group] = {
                "file": str(path.relative_to(ROOT)),
                "body": body,
                "coordinate_frame": "canonical_root" if body == "palm" else "joint_local",
                "body_origin_canonical_m": offset,
                "triangles": len(triangles),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
                "bounds_local_m": bounds(vertices),
                "leaf_features": [{"name": n, "label": leaves[n].Label} for n in sorted(members)],
                "aggregate": group in groups,
            }
        config = {
            "schema_version": 1,
            "source_step": str(SOURCE),
            "cad_unit": "millimeter",
            "mesh_unit": "meter",
            "cad_origin_mm": origin,
            "cad_to_canonical_rotation": rotation,
            "cad_to_canonical_scale": 0.001,
            "formula": "canonical_m = 0.001 * rotation @ (cad_mm - cad_origin_mm)",
            "canonical_axes": {
                "x": "+CAD Z, along parallel fingers",
                "y": "+CAD X, toward source left jaw",
                "z": "+CAD Y, hinge axis / camera side",
            },
            "bodies": {
                "palm": {"origin_m": [0, 0, 0]},
                **{
                    side: {
                        "origin_m": pivots[side],
                        "joint_axis": [0, 0, 1],
                        "opening_angle_sign": 1 if side == "left_jaw" else -1,
                        "cad_neutral_joint_angle_rad": 0.0,
                    }
                    for side in ("left_jaw", "right_jaw")
                },
            },
            "cad_neutral": "Parallel attachments point +canonical X; proven by paired flap mounting holes. This is a geometric reference, not a measured motor encoder zero.",
            "calibration": {
                "motor_zero_verified": False,
                "robot_joint_limits_verified": False,
                "nominal_glove_opening_range_rad": [0.0, 0.94],
                "nominal_range_warning": "Glove URDF range, not measured robot travel. CAD neutral already has a nonzero pad gap. Do not call q=0 fully closed.",
            },
            "geometric_evidence": evidence,
            "optional_material_meshes": {
                body: [f"cad_{k}.stl" for k in sorted(optional) if k.startswith(body)]
                for body in ("palm", "left_jaw", "right_jaw")
            },
        }
        manifest = {
            "schema_version": 1,
            "copyright": "Copyright 2026 Toyota Motor Corporation",
            "license": "CERN-OHL-W-2.0",
            "license_file": "cad/source/LICENSE",
            "source_location": "https://github.com/Toyota/yubi-hw",
            "source_commit": COMMIT,
            "source_step": str(SOURCE),
            "source_sha256": SOURCE_SHA256,
            "modification_date": "2026-10-01",
            "modification_notice": "Tessellated, grouped by rigid body, converted mm to m, and re-expressed in a canonical frame and pivot-local jaw frames; no intentional solid-geometry design changes.",
            "exporter": "cad/export.py",
            "assembly_config": "src/yubi_mujoco/assets/cad_assembly.json",
            "freecad_version": App.Version(),
            "tessellation": {
                "linear_deflection_mm": args.linear_deflection_mm,
                "angular_deflection_rad": args.angular_deflection_rad,
            },
            "solid_leaf_count": len(leaves),
            "aggregate_solid_count": {k: len(v) for k, v in groups.items()},
            "no_duplicate_compounds": True,
            "cad_volume_mm3": sum(o.Shape.Volume for o in leaves.values()),
            "bounds_canonical_m": bounds(
                [p for tris in tessellations.values() for t in tris for p in t]
            ),
            "distribution_note": "Only 13 disjoint material-level meshes are distributed; aggregate meshes omitted to avoid duplicated geometry.",
            "meshes": records,
        }
        for filename, value in (
            ("src/yubi_mujoco/assets/cad_assembly.json", config),
            ("src/yubi_mujoco/assets/cad_manifest.json", manifest),
        ):
            target = ROOT / filename
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(json.dumps(value, indent=2) + "\n")
        print(
            json.dumps(
                {
                    "mesh_count": len(records),
                    "leaf_count": len(leaves),
                    "pivots_m": pivots,
                    "material_triangles": {k: records[k]["triangles"] for k in records},
                },
                indent=2,
            )
        )
    finally:
        App.closeDocument(doc.Name)


if __name__ == "__main__":
    main()
