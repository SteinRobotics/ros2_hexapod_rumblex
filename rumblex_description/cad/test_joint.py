from build123d import *
from utils.ocp_utils import show

# ============================================================
# Parameter
# ============================================================

shaft_radius = 10
shaft_length = 60

plate_width = 50
plate_depth = 50
plate_thickness = 5

hole_radius = 12


# ============================================================
# Zylinder
# ============================================================

with BuildPart() as shaft:
    Cylinder(
        radius=shaft_radius,
        height=shaft_length,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    RigidJoint(
        label="fixed",
        joint_location=Location((0, 0, 0)),
    )


# ============================================================
# Platte
# ============================================================

with BuildPart() as plate:
    Box(
        plate_width,
        plate_depth,
        plate_thickness,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )

    Cylinder(
        radius=hole_radius,
        height=plate_thickness,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
        mode=Mode.SUBTRACT,
    )

    RevoluteJoint(
        label="rotation",
        axis=Axis(
            (0, 0, 0),
            (0, 0, 1),
        ),
        angular_range=(0, 90),
    )


# ============================================================
# Joint verbinden
# ============================================================

shaft.joints["fixed"].connect_to(
    plate.joints["rotation"],
    angle=0,
)


# ============================================================
# Assembly
# ============================================================

assembly = Compound([shaft.part, plate.part])

show(assembly)