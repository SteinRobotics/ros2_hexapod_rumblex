"""Laser-cut BOMs derived from the corresponding assembly builders.

Only sheet parts belong here: purchased brackets, electronics, turned spacers,
rubber toes and Nox's 20 mm head block are not laser-cut sheet components.
"""

from dataclasses import dataclass
from importlib import import_module


@dataclass
class Component:
    name: str
    quantity: int
    thickness: float
    surface: object


def cutting_face(part, plane, thickness):
    """Extract a face only if it represents the complete constant-thickness part."""
    faces = [f for f in part.faces()
             if f.normal_at().dot(plane.z_dir) > 1 - 1e-7
             and abs((f.center() - plane.origin).dot(plane.z_dir)) < 1e-6]
    if len(faces) != 1:
        raise ValueError(f"Expected one cutting face, found {len(faces)}")
    face = faces[0]
    from build123d import extrude
    reconstructed = extrude(face, amount=-thickness)
    if ((part - reconstructed).volume + (reconstructed - part).volume) > 1e-4:
        raise ValueError("Part is not a uniform sheet extrusion; a flat pattern is missing")
    return plane.to_local_coords(face)


def _rumblex_components(robot):
    """Build fresh nominal geometry without invoking exporters or the viewer."""
    if robot not in ("nox", "nira"):
        raise ValueError(f"Unknown robot {robot!r}; supported robots: nox, nira")
    from build123d import Plane

    def module(name):
        return import_module(f"robot_{robot}.{name}")

    common = module("torso_common")
    legs = len(common.SERVO_POSITIONS.keys() - {"head"})
    parts = []

    def add(name, quantity=1, *, surface=None, thickness=None):
        source = module(name) if surface is None else None
        if surface is None:
            surface = source.build_surface()
        if thickness is None:
            thickness = getattr(source, "THICKNESS", common.THICKNESS)
        parts.append(Component(name, quantity, thickness, surface))

    for index in range(5):
        add(f"torso_layer_{index}")
    for name in ("foot_front", "foot_back", "foot_connection"):
        add(name, legs)

    slots = common.rectangle_slots_locations
    ends = [p for p in slots if abs(p.orientation.Z) > 1e-6]
    sides = [p for p in slots if abs(p.orientation.Z) <= 1e-6 and p.position.X > 0]
    if robot == "nox":
        chassis = module("chassis_side")
        torso = module("assembly_torso")
        height = 3 * common.THICKNESS + torso.SPACER_LENGTH_1_to_2 + torso.SPACER_LENGTH_TOP
        for name, quantity, side, h in (
            ("chassis_side_end", len(ends), False, height),
            ("chassis_side", len(sides), True, height),
            ("chassis_side_diagonal", len(common.diagonal_slots_locations), False,
             common.SPACER_LENGTH_TOP + 2 * common.THICKNESS),
        ):
            add(name, quantity, surface=chassis.build_surface(h, side=side),
                thickness=chassis.THICKNESS)
    else:
        layout = module("torso_layout")
        chassis = module("chassis_common")
        add("torso_layer_2_addon")
        add("torso_layer_2_cover")
        roof = module("torso_layer_4")
        add("torso_layer_4_frame", surface=roof.build_frame_surface(), thickness=roof.FRAME_THICKNESS)
        add("foot_outer", legs)
        for name, quantity in (("chassis_front", sum(p.position.X > 0 for p in ends)),
                               ("chassis_back", sum(p.position.X < 0 for p in ends)),
                               ("chassis_side", len(sides))):
            add(name, quantity, surface=module(name).build_surface(layout.SIDE_PLATE_HEIGHT),
                thickness=chassis.THICKNESS)
        rear = [p for p in common.diagonal_slots_locations if p.position.X < 0]
        for oled in (False, True):
            add("chassis_diagonal_back_left" if oled else "chassis_diagonal_back",
                sum((p.position.Y > 0) == oled for p in rear),
                surface=module("chassis_diagonal_back").build_surface(
                    layout.DIAGONAL_PLATE_HEIGHT, with_oled=oled), thickness=chassis.THICKNESS)
        add("chassis_diagonal_front", len(common.front_diagonal_slots_locations),
            surface=module("chassis_diagonal_front").build_surface(layout.DIAGONAL_PLATE_HEIGHT),
            thickness=chassis.THICKNESS)
        slope = module("chassis_slope_cover")
        add("chassis_slope_cover", surface=cutting_face(
            slope.build_slope_cover(layout.LAYER_3_BOTTOM),
            slope.speaker_plane(layout.LAYER_3_BOTTOM, 0), chassis.THICKNESS),
            thickness=chassis.THICKNESS)
        housing = module("lidar_interface_housing")
        for name, panel in housing.flat_parts().items():
            add(f"lidar_housing_{name}", surface=cutting_face(
                panel, Plane(origin=(0, 0, housing.WALL)), housing.WALL), thickness=housing.WALL)
        head = module("assembly_head")
        t = head.PLATE_THICKNESS
        for name, panel, plane in (
            ("head_chin", head.build_deck(), Plane(origin=(0, 0, t))),
            ("head_brow", head.build_deck(brow=True), Plane(origin=(0, 0, t))),
            ("head_rear_adapter", head.build_adapter(),
             Plane(origin=(0, -t, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0))),
        ):
            add(name, surface=cutting_face(panel, plane, t), thickness=t)
    return parts


# Extend this registry with a callable returning a list[Component]. Builders
# define their BOM and return one connected planar CAD face per component type.
ROBOT_BUILDERS = {
    "nox": lambda: _rumblex_components("nox"),
    "nira": lambda: _rumblex_components("nira"),
}


def robot_components(robot):
    if robot not in ROBOT_BUILDERS:
        raise ValueError(f"Unknown robot {robot!r}; supported robots: {', '.join(ROBOT_BUILDERS)}")
    return ROBOT_BUILDERS[robot]()
