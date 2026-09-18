std_msgs/Header header
# Types:
uint8 NO_REQUEST = 0
# Single pose
uint8 SINGLE_POSE = 1
# Sequence gaits (one-shot animations)
uint8 SEQUENCE_LAYDOWN = 10
uint8 SEQUENCE_STAND_UP = 11
uint8 SEQUENCE_WAITING = 12
uint8 SEQUENCE_WATCH = 13
uint8 SEQUENCE_LOOK = 14
uint8 SEQUENCE_DANCE = 15
uint8 SEQUENCE_HIGH_FIVE = 16
uint8 SEQUENCE_LEGS_WAVE = 17
uint8 SEQUENCE_BODY_ROLL = 18
uint8 SEQUENCE_BITE = 19
uint8 SEQUENCE_STOMP = 20
uint8 SEQUENCE_CLAP = 21
uint8 SEQUENCE_TESTLEGS = 22
# Continuous gaits
uint8 CONTINUOUS_POSE = 30
uint8 CONTINUOUS_MOVE = 31  # auto-selects WAVE/RIPPLE/TRIPOD based on velocity
uint8 CONTINUOUS_RUNNING = 32  # fast 2-leg-support gait, unstable but fastest

# Directions:
uint8 CLOCKWISE = 0
uint8 ANTICLOCKWISE = 1

uint8 type 0
uint8 direction 0
float64 duration_s 1.0   # duration of 1 cycle
string name




