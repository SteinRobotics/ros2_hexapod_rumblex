#!/usr/bin/env python3

# extracted connection holes:
# top view:
#   hole 0: radius=0.80  position=( 10.2500, -12.8500, 27.0488)  # right
#   hole 1: radius=0.80  position=(-10.2500, -12.8500, 27.0488)  # left
#   hole x: radius=0.80  position=(-10.2500, -12.8500,  2.5988)  # left
#   hole x: radius=0.80  position=( 10.2500, -12.8500,  2.5988)  # right

# back view:
#   hole 19: radius=0.80  position=(-10.2500, 13.9500,  2.5988)
#   hole 20: radius=0.80  position=(-10.2500, 13.9500, 27.0488)
#   hole 21: radius=0.80  position=( 10.2500, 13.9500,  2.5988)
#   hole 22: radius=0.80  position=( 10.2500, 13.9500, 27.0488)

# extracted servo horn positions holes:
# top view:
# center hole: radius=0.80  position=(0.0008, -15.7424, 35.3488) # center
#   hole  4: radius=0.80  position=( 7.0000, -15.7424, 35.3511)  # right
#   hole  6: radius=0.80  position=( 0.0024, -15.7424, 28.3488)  # bottom
#   hole  8: radius=0.80  position=(-7.0000, -15.7424, 35.3464)  # left
#   hole 10: radius=0.80  position=(-0.0024, -15.7424, 42.3488)  # top

# extracted servo horn positions holes:
# back view:
# center hole: radius=0.80  position=(0.0008, 25.4048, 35.3488) # center
#   hole 12: radius=0.80  position=(-6.9982, 25.4048, 35.3464)
#   hole 14: radius=0.80  position=(-0.0013, 25.4048, 42.3480)
#   hole 16: radius=0.80  position=( 0.0034, 25.4048, 28.3495)
#   hole 18: radius=0.80  position=( 7.0003, 25.4048, 35.3511)

from pathlib import Path

from build123d import *
from utils.ocp_utils import show

DARK_GRAY = Color(0.25, 0.25, 0.25)

OUTER_DIAMETER = 2.0
OVERALL_LENGTH = 2.0
STUD_HOLE_DIAMETER = 0.1

# Fraction of the servo's Y extent treated as "near the top / back face"
EDGE_BAND_FRACTION = 0.2


def build_model(
    outer_diameter: float = OUTER_DIAMETER,
    length: float = OVERALL_LENGTH,
    build_method=BuildPart,
) -> Part:
    with build_method() as model:
        Cylinder(outer_diameter / 2, length)

    return model.part


def build_assembly() -> Compound:
    servo = import_step(str(Path(__file__).parent / "imported" / "HX-35H.stp"))
    servo.color = DARK_GRAY
    servo_bb = servo.bounding_box()

    y_min = servo_bb.min.Y
    y_max = servo_bb.max.Y
    y_band = (y_max - y_min) * EDGE_BAND_FRACTION

    def is_y_axis_aligned(face, tol: float = 1e-3) -> bool:
        direction = face.axis_of_rotation.direction
        return abs(abs(direction.dot(Vector(0, 1, 0))) - 1) < tol

    def is_near_top_or_back(face) -> bool:
        origin_y = face.axis_of_rotation.position.Y
        return origin_y > y_max - y_band or origin_y < y_min + y_band

    holes = servo.faces().filter_by(
        lambda f: f.geom_type == GeomType.CYLINDER and 0.8 < f.radius < 0.9
    ).filter_by(
        lambda f: is_y_axis_aligned(f) and is_near_top_or_back(f)
    )
    print(f"Found {len(holes)} candidate holes")
    for i, hole in enumerate(holes):
        pos = hole.axis_of_rotation.position
        print(f"  hole {i}: radius={hole.radius:.2f}  position=({pos.X:.4f}, {pos.Y:.4f}, {pos.Z:.4f})")

    marker_parts = []
    for hole in holes:
        with BuildPart() as marker:
            with BuildSketch(Plane(hole.axis_of_rotation.location)):
                Circle(hole.radius * 0.5)
            extrude(amount=1.0, both=True)
        marker_parts.append(marker.part)

    children = [servo]
    if marker_parts:
        children.append(Compound(children=marker_parts))
    else:
        print("Warning: no markers were generated")

    return Compound(children=children)


def main() -> None:
    assembly = build_assembly()
    Path("generated").mkdir(exist_ok=True)
    show(assembly, name="servoHX-35H_plus_markers", clear=True)


if __name__ == "__main__":
    main()