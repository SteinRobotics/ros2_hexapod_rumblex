"""Supporting-foot motion and torso transforms for offline display (metres/radians)."""

import math

LEG_NAMES = ('right_front', 'right_mid', 'right_back',
             'left_front', 'left_mid', 'left_back')
LOCOMOTION = ('CONTINUOUS_MOVE', 'CONTINUOUS_RUNNING')


def support_displacement(previous, current, heights, tolerance):
    """Fit current feet into previous feet, returning planar body displacement."""
    pairs = [(a, b) for a, b, z in zip(previous, current, heights)
             if abs(a[2] - z) <= tolerance and abs(b[2] - z) <= tolerance]
    if len(pairs) < 2:
        return None
    ax, ay = (sum(a[i] for a, _ in pairs) / len(pairs) for i in (0, 1))
    bx, by = (sum(b[i] for _, b in pairs) / len(pairs) for i in (0, 1))
    dot = sum((b[0] - bx) * (a[0] - ax) + (b[1] - by) * (a[1] - ay)
              for a, b in pairs)
    cross = sum((b[0] - bx) * (a[1] - ay) - (b[1] - by) * (a[0] - ax)
                for a, b in pairs)
    if math.hypot(dot, cross) < 1e-12:
        return None
    yaw = math.atan2(cross, dot)
    c, s = math.cos(yaw), math.sin(yaw)
    return ax - c * bx + s * by, ay - s * bx - c * by, yaw


def compose_torso(planar, torso):
    """Return translation and XYZW quaternion of planar pose * torso pose."""
    x, y, yaw = planar
    p, r = torso.position, torso.orientation
    c, s = math.cos(yaw), math.sin(yaw)
    position = (x + c * p.x - s * p.y, y + s * p.x + c * p.y, p.z)
    roll, pitch = math.radians(r.roll), math.radians(r.pitch)
    heading = yaw + math.radians(r.yaw)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(heading / 2), math.sin(heading / 2)
    return position, (sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy,
                      cr * cp * cy + sr * sp * sy)


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def body_velocity(previous, current, dt):
    """Finite-difference translation and rotation, expressed in current base_link."""
    position, q = current
    old_position, old_q = previous
    inverse = (-q[0], -q[1], -q[2], q[3])
    difference = tuple((b - a) / dt for a, b in zip(old_position, position))
    linear = quaternion_product(quaternion_product(inverse, (*difference, 0.0)), q)[:3]
    # q_current^-1 * q_previous is the reverse rotation in the current frame.
    relative = quaternion_product(inverse, old_q)
    if relative[3] < 0:
        relative = tuple(-v for v in relative)
    magnitude = math.sqrt(sum(v * v for v in relative[:3]))
    scale = (-2 * math.atan2(magnitude, relative[3]) / (dt * magnitude)
             if magnitude > 1e-12 else -2 / dt)
    return linear, tuple(v * scale for v in relative[:3])
