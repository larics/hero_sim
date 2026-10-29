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


class GroundGoTo:
    """A ground robot's go_to: from `start` to `goal` (x, y, yaw), within the speed limits.

    omni drives straight while turning to the goal yaw. diff turns to face the
    goal, drives straight, then turns to the goal yaw (closer than MIN_DRIVE
    it only turns). The move takes `duration` s, or longer if that would
    exceed max_speed or max_yaw_rate: then all of its phases stretch alike.
    """

    MIN_DRIVE = 0.01  # m

    def __init__(self, start, goal, drive, duration, max_speed, max_yaw_rate):
        x0, y0, yaw0 = start
        x1, y1, yaw1 = goal
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        # Phases (shortest time, start pose, change in x, y, yaw), in order.
        if drive == 'diff' and dist >= self.MIN_DRIVE:
            heading = math.atan2(dy, dx)
            turn_in, turn_out = wrap_angle(heading - yaw0), wrap_angle(yaw1 - heading)
            phases = [
                (abs(turn_in) / max_yaw_rate, (x0, y0, yaw0), (0.0, 0.0, turn_in)),
                (dist / max_speed, (x0, y0, heading), (dx, dy, 0.0)),
                (abs(turn_out) / max_yaw_rate, (x1, y1, heading), (0.0, 0.0, turn_out)),
            ]
        else:
            turn = wrap_angle(yaw1 - yaw0)
            shortest = max(dist / max_speed, abs(turn) / max_yaw_rate)
            phases = [(shortest, (x0, y0, yaw0), (dx, dy, turn))]
        shortest = sum(p[0] for p in phases)
        self.total = max(duration, shortest)  # s
        stretch = self.total / shortest if shortest > 0.0 else 0.0
        self._phases = [(t * stretch, pose, delta) for t, pose, delta in phases]
        self.goal = (x1, y1, wrap_angle(yaw1))

    def at(self, t):
        """Pose (x, y, yaw) t s into the move; the goal from `total` on."""
        for span, (x, y, yaw), (dx, dy, dyaw) in self._phases:
            if t < span:
                a = t / span
                return x + a * dx, y + a * dy, wrap_angle(yaw + a * dyaw)
            t -= span
        return self.goal
