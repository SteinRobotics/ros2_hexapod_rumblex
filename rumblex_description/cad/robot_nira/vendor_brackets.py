"""Mounting frames measured from the HX-35HM vendor STEP faces and bores.

Hole profiles in the bottom bracket are splines, so use the enclosed area
and centroid instead of requiring analytic circular edges. Radius tolerances
only select features; placements use the measured centres and face normals.
"""
from functools import lru_cache
from math import pi
from pathlib import Path

from build123d import CenterOf, Face, GeomType, Plane, RigidJoint, Rot, Vector, import_step

from common import servo_simplified as servo

FEATURE_TOLERANCE = 0.002


def hole_centers(face, radius):
    return [hole.center(CenterOf.MASS) for wire in face.inner_wires()
            if abs((hole := Face(wire)).area - pi * radius**2) < FEATURE_TOLERANCE]


def mounting_faces(part, radius, count, direction):
    matches = []
    for face in part.faces():
        if face.geom_type != GeomType.PLANE:
            continue
        if face.normal_at().dot(Vector(direction)) < 0.99:
            continue
        holes = hole_centers(face, radius)
        if len(holes) == count:
            matches.append((face, holes))
    if not matches:
        raise ValueError(f"Vendor bracket has no mounting face with {count} radius-{radius} bores")
    return matches


def midpoint(points):
    return sum(points, Vector()) / len(points)


@lru_cache(maxsize=2)
def _template(kind):
    filename = 'Side' if kind == 'side' else 'Botton'
    rotation = Rot(Z=90) if kind == 'side' else Rot(180, 0, 90)
    part = rotation * import_step(str(
        Path(__file__).resolve().parents[1] / 'imported' / f'HX-35HM {filename} Bracket.STEP'))
    if kind == 'side':
        # Inner rear flange; its vertical M2 clearance pair mates to +HOLE_X.
        face, holes = max(mounting_faces(part, 1.15, 2, (0, -1, 0)),
                          key=lambda item: midpoint(item[1]).Y)
        holes.sort(key=lambda p: p.Z)
        target = Plane(origin=(servo.HOLE_X, servo.BODY_Y + servo.CASE_FLANGE_THICKNESS,
                               (servo.HOLE_Z_LOW + servo.HOLE_Z_HIGH) / 2),
                       x_dir=(0, 0, 1), z_dir=(0, -1, 0)).location
    else:
        # Inner front flange; its horizontal pair mates to the lower servo row.
        face, holes = min(mounting_faces(part, servo.M2_R, 2, (0, 1, 0)),
                          key=lambda item: midpoint(item[1]).Y)
        holes.sort(key=lambda p: p.X)
        target = Plane(origin=(0, -servo.CASE_FLANGE_THICKNESS, servo.HOLE_Z_LOW),
                       x_dir=(1, 0, 0), z_dir=(0, 1, 0)).location
    expected_pitch = (servo.HOLE_Z_HIGH - servo.HOLE_Z_LOW if kind == 'side'
                      else 2 * servo.HOLE_X)
    if abs((holes[1] - holes[0]).length - expected_pitch) > FEATURE_TOLERANCE:
        raise ValueError('Vendor bracket mounting pitch does not match the servo')
    source = Plane(origin=midpoint(holes), x_dir=holes[1] - holes[0],
                   z_dir=face.normal_at()).location
    part = target * source.inverse() * part
    RigidJoint('servo_mount', part, target)

    direction = (1, 0, 0) if kind == 'side' else (0, 0, -1)
    faces = mounting_faces(part, 1.6, 4, direction)
    if len(faces) != 1:
        raise ValueError(f'Expected one outer bracket plate, found {len(faces)}')
    face, holes = faces[0]
    # Side pod uses X across its face, -Y outward, Z up. The bottom plate
    # uses the inclined bracket's upward-facing mounting frame.
    normal = Vector(0, 0, 1) if kind == 'side' else -face.normal_at()
    frame = Plane(origin=midpoint(holes), x_dir=(0, 1, 0), z_dir=normal).location
    RigidJoint('plate_mount', part, frame)
    return part


def build_side_bracket():
    return _template('side').moved(Rot())


def build_bottom_bracket():
    return _template('bottom').moved(Rot())


def side_plate_holes():
    """Adapter hole centres in the pod drawing's XZ plane."""
    part = _template('side')
    _, holes = mounting_faces(part, 1.6, 4, (1, 0, 0))[0]
    frame = Plane(part.joints['plate_mount'].location)
    return tuple((p.X, p.Z) for point in holes for p in [frame.to_local_coords(point)])
