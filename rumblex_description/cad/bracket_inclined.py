"""
Inclined Bracket - build123d model

A small U-shaped mounting bracket (e.g. servo / motor mount):
  - Two vertical "ear" legs, 2mm thick sheet, with a rounded bottom
    and an inclined (sloped) top edge, each carrying 4 mounting holes
    (one Ø8mm centre hole, one Ø2mm hole above it, two Ø2mm holes
    flanking it 14mm apart).
  - A top plate joining the two legs, TILTED to match the legs'
    inclined top edge (so its underside sits flush on the slope),
    carrying 3 elongated slots (for a servo horn / motor shaft
    clearance) plus 4 small Ø2mm corner mounting holes.

All dimensions taken from the reference drawing (mm).
"""

import math
from pathlib import Path
from build123d import *

from utils.ocp_utils import show

# ---------------------------------------------------------------- params --
PLATE_W = 41.0          # top plate overall width (X)
PLATE_D = 25.0          # top plate depth, measured in plan view (Y)
PLATE_T = 2.0           # top plate thickness (measured normal to its face)
 
LEG_T = 2.0             # leg material thickness (X)
LEG_H = 42              # leg height, left (tall / vertical) side
LEG_H_LOW = 28.0        # leg height, right (short / inclined) side
 
BIG_HOLE_D = 8.0
SMALL_HOLE_D = 2.0
HOLE_ROW_Z = PLATE_D / 2       # height of the centre hole from the bottom of the leg

SERVO_HORN_HOLE_D = 2.0     # small screw holes for attaching the servo horn
SERVO_HORN_BC_RADIUS = 7.0  # bolt-circle radius, centred on the leg's Ø8 hole
SERVO_HORN_HOLE_COUNT = 4   # cross pattern: up / down / left / right
 
SLOT_SPACING = 10.0
 
# ------------------------------------------------------- slope geometry --
# Angle of the legs' inclined top edge, and the tilted plane that the top
# plate lives on so it sits flush against that slope.
_THETA = math.atan2(LEG_H - LEG_H_LOW, PLATE_D)
THETA_DEG = math.degrees(_THETA)                  # rotation to level the top plate for assembly
# SLANT_DEPTH = PLATE_D / math.cos(_THETA)         # true plate depth along the slope
SLANT_DEPTH = PLATE_D 
_ZC = (LEG_H + LEG_H_LOW) / 2                     # height of the slope's midpoint
 
# Plane origin sits at the midpoint of the legs' top edge; its normal is
# tilted by _THETA so the plane's own "bottom" face is flush with that edge.
PLATE_PLANE = Plane(
    origin=(0, 0, _ZC),
    x_dir=(1, 0, 0),
    z_dir=(0, math.sin(_THETA), math.cos(_THETA)),
)
 
 
# ------------------------------------------------------------ leg profile --
def leg_solid() -> Part:
    r = PLATE_D / 2  # rounded-bottom radius
 
    with BuildPart() as leg:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Line((0, r), (0, LEG_H))                 # left (tall) edge
                Line((0, LEG_H), (PLATE_D, LEG_H_LOW))    # inclined top edge
                Line((PLATE_D, LEG_H_LOW), (PLATE_D, r))  # right (short) edge
                ThreePointArc((PLATE_D, r), (r, 0), (0, r))  # rounded bottom
            make_face()
        extrude(amount=LEG_T)
 
        # centre Ø8 hole (servo shaft) + a 4-hole bolt circle around it
        # (for the servo horn screws), on the leg's mid-plane
        with BuildSketch(Plane.YZ):
            with Locations((r, HOLE_ROW_Z)):
                Circle(BIG_HOLE_D / 2)
        extrude(amount=LEG_T, mode=Mode.SUBTRACT)
        
        with BuildSketch(Plane.YZ):
            with Locations((r, HOLE_ROW_Z)):
                with PolarLocations(SERVO_HORN_BC_RADIUS, SERVO_HORN_HOLE_COUNT, start_angle=0):
                    Circle(SERVO_HORN_HOLE_D / 2)
        extrude(amount=LEG_T, mode=Mode.SUBTRACT)
 
    return leg.part
 
 
# -------------------------------------------------------------- top plate --
def top_plate_solid() -> Part:
    """Top plate built directly on PLATE_PLANE, so it comes out already
    tilted to match the legs' inclined top edge. The plane's origin is the
    plate's *bottom* face (flush against the legs), and it's extruded
    upward along the plane's own normal by PLATE_T."""
 
    with BuildPart() as plate:
        with BuildSketch(PLATE_PLANE):
            Rectangle(PLATE_W, SLANT_DEPTH)
        extrude(amount=PLATE_T)
 
        # three servo-horn slots, evenly spaced along X, centred on the plate
        # centre Ø8 hole (servo shaft) + a 4-hole bolt circle around it
        # (for the servo horn screws), on the leg's mid-plane
        with BuildSketch(PLATE_PLANE):
            with Locations((-SLOT_SPACING, 0), (0, 0), (SLOT_SPACING, 0)):
                Circle(BIG_HOLE_D / 2)
        extrude(amount=PLATE_T, mode=Mode.SUBTRACT)
        
        with BuildSketch(PLATE_PLANE):
            with Locations((-SLOT_SPACING, 0), (0, 0), (SLOT_SPACING, 0)):
                with PolarLocations(SERVO_HORN_BC_RADIUS, SERVO_HORN_HOLE_COUNT, start_angle=45):
                    Circle(SERVO_HORN_HOLE_D / 2)
        extrude(amount=PLATE_T, mode=Mode.SUBTRACT)
 
 
    return plate.part
 
 
# ------------------------------------------------------------- assemble ---
def build_bracket() -> Part:
    leg = leg_solid()
    plate = top_plate_solid()  # already positioned/tilted in global coords
 
    with BuildPart() as bracket:
        add(plate)
        add(leg.locate(Location((-PLATE_W / 2, -PLATE_D / 2, 0))))
        add(leg.locate(Location((PLATE_W / 2 - LEG_T, -PLATE_D / 2, 0))))

        # joint Z axis points along the leg hole axis (global X)
        RigidJoint(
            label="fixed",
            joint_location=Location(
                Plane(
                    origin=(-PLATE_W / 2 + LEG_T / 2, 0, HOLE_ROW_Z),
                    x_dir=(0, 0, 1),
                    z_dir=(1, 0, 0),
                )
            ),
        )

    return bracket.part
 


inclined_bracket = build_bracket()

def main() -> None:
    bracket = build_bracket()
    Path("generated").mkdir(exist_ok=True)
    export_step(bracket, "generated/inclined_bracket.step")
    show(bracket, name="inclined_bracket", clear=True)
    
if __name__ == "__main__":
    main()