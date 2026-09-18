std_msgs/Header header

# BUTTONS
bool button_a
bool button_b
bool button_x
bool button_y
bool button_l1
bool button_l2
bool button_r1
bool button_r2
bool button_select
bool button_start
bool button_home

bool button_long_a
bool button_long_b
bool button_long_x
bool button_long_y
bool button_long_l1
bool button_long_l2
bool button_long_r1
bool button_long_r2
bool button_long_select
bool button_long_start
bool button_long_home

# DPAD
int8 dpad_vertical     # DOWN = -1, UP = 1
int8 dpad_horizontal   # LEFT = -1, RIGHT = 1

# LEFT_STICK
float32 left_stick_vertical   # TOP  = -1.0, DOWN = 1.0,  hangs on 0.004 -> means 0.0
float32 left_stick_horizontal # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0

# RIGHT_STICK
float32 right_stick_horizontal  # LEFT = -1.0, RIGHT = 1.0, hangs on 0.004 -> means 0.0
float32 right_stick_vertical #

