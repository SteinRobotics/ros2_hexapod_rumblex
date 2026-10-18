#!/usr/bin/env bash
# Run inside the robot user's lingering systemd session.
set -eo pipefail
: "${RUMBLEX_ROBOT:?Set RUMBLEX_ROBOT in ~/.config/rumblex/robot.env}"
case "$RUMBLEX_ROBOT" in nox|nira) ;; *) echo "Invalid robot: $RUMBLEX_ROBOT" >&2; exit 2 ;; esac
workspace="$HOME/Workspace/colcon_rumblex"
source /opt/ros/lyrical/setup.bash
source "$workspace/.venv/bin/activate"
source "$workspace/install/local_setup.bash"
export SDL_VIDEODRIVER=dummy
export SDL_AUDIODRIVER=pulseaudio
export PYGAME_HIDE_SUPPORT_PROMPT=1
# Use the venv interpreter explicitly even when ros2 itself comes from apt.
exec "$workspace/.venv/bin/python" /opt/ros/lyrical/bin/ros2 launch \
    rumblex_bringup target_launch.py "robot:=$RUMBLEX_ROBOT"
