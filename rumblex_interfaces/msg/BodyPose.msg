# Positions in metres, angles in degrees. Absent components leave the current pose unchanged.
# Supplied components replace their entire target. Values in absent components are ignored.
# Feedback and brain trajectory samples set all presence flags.
bool has_head_orientation
Orientation head_orientation
bool has_torso_pose
Pose torso_pose
# Toe targets before torso transformation, ordered:
# right_front, right_mid, right_back, left_front, left_mid, left_back.
bool has_toe_positions
geometry_msgs/Vector3[6] toe_positions
