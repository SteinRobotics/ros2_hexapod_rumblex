#!/bin/bash

# Prüfen, ob beide Argumente übergeben wurden (Servoname und Winkel)
if [ $# -lt 2 ]; then
  echo "Verwendung: $0 <SERVO_NAME> <ANGLE_DEG>"
  exit 1
fi

SERVO_NAME="$1"
ANGLE_DEG="$2"
echo "Setze Servo '$SERVO_NAME' auf '$ANGLE_DEG'°"

ros2 topic pub --once /single_servo_request rumblex_interfaces/msg/ServoAngle "
name: '$SERVO_NAME'
angle_deg: '$ANGLE_DEG'"

