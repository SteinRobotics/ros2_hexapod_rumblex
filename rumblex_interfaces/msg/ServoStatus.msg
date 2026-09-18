std_msgs/Header header
uint8 NO_ERROR = 0
uint8 OVERTEMPERATURE = 1
uint8 OVERVOLTAGE = 2
uint8 OVERTEMPERATURE_OVERVOLTAGE = 3
uint8 STALLED = 4
uint8 OVERTEMPERATURE_STALLED = 5
uint8 OVERVOLTAGE_STALLED = 6
uint8 OVERTEMPERATURE_OVERVOLTAGE_STALLED = 7

string servo_error_code
uint8 error_code   # from ServoErrorCode.msg
string servo_max_temperature
float32 max_temperature   # °C
string servo_max_voltage
float32 max_voltage   # V
string servo_min_voltage
float32 min_voltage   # V
