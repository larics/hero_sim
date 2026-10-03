"""Planar pose math shared by the sim nodes. No ROS imports."""
import math


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_quat(yaw):
    """Return (x, y, z, w) for a pure rotation of `yaw` about Z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(x, y, z, w):
    """Extract the yaw (Z) angle from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def limit_ground_cmd(vx, vy, wz, drive, max_speed, max_yaw_rate):
    """Apply the drive constraint and limits to a body-frame cmd_vel.

    diff drops the lateral component (non-holonomic); omni keeps it. The
    planar speed |(vx, vy)| is scaled down to max_speed, and the yaw rate is
    clipped to +-max_yaw_rate.
    """
    if drive == 'diff':
        vy = 0.0
    speed = math.hypot(vx, vy)
    if speed > max_speed:
        scale = max_speed / speed
        vx, vy = vx * scale, vy * scale
    wz = max(-max_yaw_rate, min(max_yaw_rate, wz))
    return vx, vy, wz


def integrate_ground(x, y, yaw, vx, vy, wz, dt):
    """Advance a planar pose by a body-frame velocity held for dt seconds."""
    c, s = math.cos(yaw), math.sin(yaw)
    x += (vx * c - vy * s) * dt
    y += (vx * s + vy * c) * dt
    return x, y, wrap_angle(yaw + wz * dt)
