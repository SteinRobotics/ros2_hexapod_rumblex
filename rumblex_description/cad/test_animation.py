from build123d import *
from ocp_vscode import show, Animation
import numpy as np


# --------------------------------------------------
# Parameter
# --------------------------------------------------

shaft_radius = 10
shaft_length = 60

plate_size = 50
plate_thickness = 5
hole_radius = 12

plate_height = 20


# --------------------------------------------------
# Zylinder
# --------------------------------------------------

with BuildPart() as shaft:
    Cylinder(shaft_radius, shaft_length)

    RigidJoint(
        "fixed",
        joint_location=Location((0, 0, plate_height)),
    )


# --------------------------------------------------
# Platte mit Loch
# --------------------------------------------------

with BuildPart() as plate:
    Box(plate_size, plate_size, plate_thickness)

    Cylinder(
        hole_radius,
        plate_thickness,
        mode=Mode.SUBTRACT,
    )

    RevoluteJoint(
        "rotation",
        axis=Axis.Z
    )


# --------------------------------------------------
# Joint verbinden
# --------------------------------------------------

shaft.joints["fixed"].connect_to(
    plate.joints["rotation"],
    angle=0,
)


# --------------------------------------------------
# Anzeige
# --------------------------------------------------

show(
    shaft.part,
    plate.part,
    names=["shaft", "plate"],
)


# --------------------------------------------------
# Animation
# --------------------------------------------------

animation = Animation()

time = np.linspace(0, 2, 61)
angle = np.linspace(0, 90, 61)

# 0° -> 90° -> 0°
time = np.concatenate([
    time,
    time[1:] + 2,
])

angle = np.concatenate([
    angle,
    angle[-2::-1],
])

animation.add_track(
    "/Group/plate",
    "rz",
    time,
    angle,
)

animation.animate(speed=1)