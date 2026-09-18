#!/bin/bash

# Sollwerte für SERVO_SERIAL_ID: 
# [10, 20,      # HEAD: HORIZONTAL, VERTICAL
#  11, 21, 31,  # LEG_RIGHT_FRONT: COXA, FEMUR, TIBIA
#  12, 22, 32,  # LEG_RIGHT_MID:   COXA, FEMUR, TIBIA
#  13, 23, 33,  # LEG_RIGHT_BACK:  COXA, FEMUR, TIBIA
#  14, 24, 34,  # LEG_LEFT_BACK:  COXA, FEMUR, TIBIA
#  15, 25, 35,  # LEG_LEFT_MID:    COXA, FEMUR, TIBIA 
#  16, 26, 36]  # LEG_LEFT_FRONT:   COXA, FEMUR, TIBIA 

# Prüfen, ob ein Servoname übergeben wurde (z.B. LEG_LEFT_FRONT_COXA)
if [ -z "$1" ]; then
  echo "Verwendung: $0 <SERVO_NAME>"
  exit 1
fi

SERVO_NAME="$1"
echo "Lese ID von Servo '$SERVO_NAME' aus..."

ros2 topic pub --once /servo_direct_request rumblex_interfaces/msg/ServoDirectRequest "
name: '$SERVO_NAME'
cmd: 14
data1: 0
data2: 0"
