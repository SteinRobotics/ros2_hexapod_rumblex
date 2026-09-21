# Allow direct execution as well as package imports.
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from build123d import *
from utils.ocp_utils import show

import common.bracket_u_shape as bracket_u_shape
import common.servo_simplified as servo_simplified

JOINT_ANGLE = 20.0  # initial angle for the joint connection

# ============================================================
# Build parts
# ============================================================

servo = servo_simplified.build_model()
bracket = bracket_u_shape.build_bracket()


# ============================================================
# connect servo rotation joint to bracket fixed joint
# ============================================================

servo.joints["rotation"].connect_to(
    bracket.joints["fixed"],
    angle=JOINT_ANGLE,
)



# ============================================================
# Assembly
# ============================================================

assembly = Compound([servo, bracket])

show(assembly)
