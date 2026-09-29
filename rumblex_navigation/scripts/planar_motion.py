"""Integrate constant body-frame velocities for offline visualization."""

import math


def integrate_pose(x, y, yaw, vx, vy, wz, dt):
    """Return the planar pose after dt seconds, including holonomic motion."""
    if dt <= 0:
        return x, y, yaw
    turn = wz * dt
    # Integrate the circular arc exactly; sinc avoids division by a small rate.
    half_turn = turn / 2.0
    scale = math.sin(half_turn) / half_turn if half_turn else 1.0
    heading = yaw + half_turn
    x += dt * scale * (vx * math.cos(heading) - vy * math.sin(heading))
    y += dt * scale * (vx * math.sin(heading) + vy * math.cos(heading))
    yaw = math.atan2(math.sin(yaw + turn), math.cos(yaw + turn))
    return x, y, yaw
