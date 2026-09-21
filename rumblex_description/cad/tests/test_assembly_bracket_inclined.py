from build123d import *
from utils.ocp_utils import show

import bracket_inclined
import servo_simplified

JOINT_ANGLE = 0.0  # initial angle for the joint connection

# ============================================================
# Build parts
# ============================================================

servo = servo_simplified.build_model()
bracket = bracket_inclined.build_bracket()


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
