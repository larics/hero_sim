"""One simulated Crazyflie exposing crazyswarm2's per-drone ROS interface."""
from crazyflie_interfaces.msg import FullState, Hover, Position, Status, VelocityWorld
from crazyflie_interfaces.srv import (
    Arm, GoTo, Land, NotifySetpointsStop, StartTrajectory, Takeoff, UploadTrajectory,
)
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, TransformStamped
import numpy as np
from rclpy.node import Node

from hero_sim.kinematics import quat_to_yaw, wrap_angle, yaw_to_quat


class SimCrazyflie:
    """Kinematic Crazyflie with the same topics/services as crazyswarm2's server.

    Motion sources, highest priority first:
      cmd_hover        velocity + height setpoint, held until notify_setpoints_stop
                       or a new takeoff/land/go_to
      low-level        cmd_position / cmd_full_state / cmd_velocity_world /
                       cmd_vel_legacy, active for LOW_LEVEL_TIMEOUT after the last msg
      high-level       takeoff / land / go_to, linear interpolation over the duration
    """

    # Supervisor info bits (crazyflie_interfaces/msg/Status)
    SUPERVISOR_CAN_BE_ARMED = 1
    SUPERVISOR_IS_ARMED = 2
    SUPERVISOR_AUTO_ARM = 4
    SUPERVISOR_CAN_FLY = 8
    SUPERVISOR_IS_FLYING = 16
    SUPERVISOR_IS_TUMBLED = 32
    SUPERVISOR_IS_LOCKED = 64

    LOW_LEVEL_TIMEOUT = 0.5  # s
    BATTERY_FULL = 4.2       # V
    BATTERY_FLOOR = 3.0      # V
    BATTERY_LOW = 3.3        # V, below this pm_state reports LOW_POWER

    def __init__(self, node: Node, name, initial_position=(0.0, 0.0, 0.0), dt=0.1,
                 battery_drain=0.025, frame='world'):
        self.node = node
        self.name = name
        self.frame = frame
        self.dt = dt
        self.battery_drain = battery_drain  # V/s while flying

        self.position = np.array(initial_position, dtype=float)
        self.yaw = 0.0

        # High-level motion (linear interpolation start -> goal)
        self.start_position = self.position.copy()
        self.goal = self.position.copy()
        self.start_yaw = 0.0
        self.goal_yaw = 0.0
        self.motion_active = False
        self.motion_elapsed = 0.0
        self.goal_duration = 1.0

        self.linear_speed = 1.0  # m/s, cmd_hover z-approach speed

        self.armed = True  # auto-armed on startup, like real crazyswarm2
        self.flying = False
        self.emergency = False
        self.group_mask = 0  # 0 responds to every broadcast

        self.hover_cmd = None
        self.last_low_level_time = None

        self.battery_voltage = self.BATTERY_FULL
        self.pm_state = Status.PM_STATE_BATTERY
        self.rssi = 60  # dBm

        self.trajectories = {}  # id -> pieces; stored, not flown

        self.pose_pub = node.create_publisher(PoseStamped, f'/{name}/pose', 10)
        self.status_pub = node.create_publisher(Status, f'/{name}/status', 10)

        node.create_service(Takeoff, f'/{name}/takeoff', self._takeoff_cb)
        node.create_service(Land, f'/{name}/land', self._land_cb)
        node.create_service(GoTo, f'/{name}/go_to', self._go_to_cb)
        node.create_service(Arm, f'/{name}/arm', self._arm_cb)
        node.create_service(NotifySetpointsStop, f'/{name}/notify_setpoints_stop',
                            self._notify_setpoints_stop_cb)
        node.create_service(StartTrajectory, f'/{name}/start_trajectory', self._start_trajectory_cb)
        node.create_service(UploadTrajectory, f'/{name}/upload_trajectory', self._upload_trajectory_cb)
        # Sim-only: teleport, no interpolation. Not part of crazyswarm2.
        node.create_service(GoTo, f'/{name}/set_position', self._set_position_cb)

        node.create_subscription(Hover, f'/{name}/cmd_hover', self._cmd_hover_cb, 10)
        node.create_subscription(FullState, f'/{name}/cmd_full_state', self._cmd_full_state_cb, 10)
        node.create_subscription(Position, f'/{name}/cmd_position', self._cmd_position_cb, 10)
        node.create_subscription(VelocityWorld, f'/{name}/cmd_velocity_world',
                                 self._cmd_velocity_world_cb, 10)
        node.create_subscription(Twist, f'/{name}/cmd_vel_legacy', self._cmd_vel_legacy_cb, 10)

    def _matches_group(self, group_mask):
        """group_mask 0 addresses every drone; otherwise any shared bit matches."""
        return group_mask == 0 or (self.group_mask & group_mask) != 0

    # ------------------------------------------------------------------ update

    def update(self):
        """Advance one tick of self.dt. Returns the world->name transform to broadcast."""
        dt = self.dt
        if self.emergency:
            # Motors off: fall at 2 m/s until the ground.
            if self.position[2] > 0.0:
                self.position[2] = max(0.0, self.position[2] - 2.0 * dt)
                if self.position[2] <= 0.0:
                    self.flying = False
            return self._publish_pose()

        low_level_active = False
        if self.last_low_level_time is not None:
            elapsed = (self.node.get_clock().now() - self.last_low_level_time).nanoseconds * 1e-9
            low_level_active = elapsed < self.LOW_LEVEL_TIMEOUT

        if self.hover_cmd is not None:
            self._update_hover(dt)
        elif low_level_active:
            pass  # position already set by the low-level callbacks
        elif self.motion_active:
            self._update_high_level(dt)

        if self.flying:
            self.battery_voltage = max(self.battery_voltage - self.battery_drain * dt,
                                       self.BATTERY_FLOOR)
        self.pm_state = (Status.PM_STATE_LOW_POWER if self.battery_voltage < self.BATTERY_LOW
                         else Status.PM_STATE_BATTERY)

        return self._publish_pose()

    def _update_hover(self, dt):
        cmd = self.hover_cmd
        # Body frame -> world frame
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        self.position[0] += (cmd.vx * c - cmd.vy * s) * dt
        self.position[1] += (cmd.vx * s + cmd.vy * c) * dt
        # Approach the target height at linear_speed
        max_step = self.linear_speed * dt
        self.position[2] += np.clip(cmd.z_distance - self.position[2], -max_step, max_step)
        self.yaw = wrap_angle(self.yaw + cmd.yaw_rate * dt)
        self.goal = self.position.copy()

    def _update_high_level(self, dt):
        self.motion_elapsed += dt
        alpha = min(self.motion_elapsed / self.goal_duration, 1.0)
        self.position = (1 - alpha) * self.start_position + alpha * self.goal
        self.yaw = wrap_angle(self.start_yaw + alpha * wrap_angle(self.goal_yaw - self.start_yaw))
        if alpha >= 1.0:
            self.position = self.goal.copy()
            self.yaw = self.goal_yaw
            self.motion_active = False
            if self.position[2] < 0.05:
                self.flying = False

    def _publish_pose(self):
        stamp = self.node.get_clock().now().to_msg()
        x, y, z = (float(v) for v in self.position)
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        q = Quaternion(x=qx, y=qy, z=qz, w=qw)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation = q
        self.pose_pub.publish(pose)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame
        t.child_frame_id = self.name
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation = q
        return t

    def publish_status(self):
        msg = Status()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.name
        info = self.SUPERVISOR_CAN_BE_ARMED
        if self.armed:
            info |= self.SUPERVISOR_IS_ARMED | self.SUPERVISOR_CAN_FLY
        if self.flying:
            info |= self.SUPERVISOR_IS_FLYING
        if self.emergency:
            info |= self.SUPERVISOR_IS_LOCKED
        msg.supervisor_info = info
        msg.battery_voltage = float(self.battery_voltage)
        msg.pm_state = self.pm_state
        msg.rssi = self.rssi
        msg.latency_unicast = 5  # ms
        self.status_pub.publish(msg)

    # ---------------------------------------------------------------- services

    @staticmethod
    def _duration(d):
        return d.sec + d.nanosec * 1e-9

    def _begin_motion(self, goal, goal_yaw, duration):
        self.start_position = self.position.copy()
        self.start_yaw = self.yaw
        self.goal = goal
        self.goal_yaw = goal_yaw
        self.goal_duration = max(duration, 0.01)
        self.motion_active = True
        self.motion_elapsed = 0.0
        self.hover_cmd = None

    def _takeoff_cb(self, request, response):
        if not self.armed:
            self.node.get_logger().warn(f'[{self.name}] Takeoff rejected: not armed')
            return response
        if not self._matches_group(request.group_mask):
            return response
        duration = self._duration(request.duration)
        self._begin_motion(np.array([self.position[0], self.position[1], request.height]),
                           self.yaw, duration)
        self.flying = True
        self.node.get_logger().info(
            f'[{self.name}] TAKEOFF to {request.height:.2f}m over {duration:.2f}s')
        return response

    def _land_cb(self, request, response):
        if not self._matches_group(request.group_mask):
            return response
        duration = self._duration(request.duration)
        self._begin_motion(np.array([self.position[0], self.position[1], request.height]),
                           self.yaw, duration)
        self.node.get_logger().info(
            f'[{self.name}] LAND to {request.height:.2f}m over {duration:.2f}s')
        return response

    def _go_to_cb(self, request, response):
        if not self._matches_group(request.group_mask):
            return response
        goal = np.array([request.goal.x, request.goal.y, request.goal.z])
        if request.relative:
            goal = self.position + goal
        duration = self._duration(request.duration)
        self._begin_motion(goal, request.yaw, duration)
        self.node.get_logger().info(
            f'[{self.name}] GO_TO {goal} yaw={request.yaw:.2f} dur={duration:.2f}s '
            f'rel={request.relative}')
        return response

    def _arm_cb(self, request, response):
        self.armed = request.arm
        if not self.armed:
            self.flying = False
            self.motion_active = False
            self.hover_cmd = None
        self.node.get_logger().info(f'[{self.name}] {"ARMED" if self.armed else "DISARMED"}')
        return response

    def _notify_setpoints_stop_cb(self, request, response):
        if not self._matches_group(request.group_mask):
            return response
        self.hover_cmd = None
        self.last_low_level_time = None
        self.node.get_logger().info(
            f'[{self.name}] NOTIFY_SETPOINTS_STOP (remain_valid={request.remain_valid_millisecs}ms)')
        return response

    def _start_trajectory_cb(self, request, response):
        if not self._matches_group(request.group_mask):
            return response
        self.node.get_logger().info(
            f'[{self.name}] START_TRAJECTORY id={request.trajectory_id} (not simulated)')
        return response

    def _upload_trajectory_cb(self, request, response):
        self.trajectories[request.trajectory_id] = request.pieces
        self.node.get_logger().info(
            f'[{self.name}] UPLOAD_TRAJECTORY id={request.trajectory_id} '
            f'pieces={len(request.pieces)}')
        return response

    def _set_position_cb(self, request, response):
        self.position = np.array([request.goal.x, request.goal.y, request.goal.z])
        self.yaw = request.yaw
        self.goal = self.position.copy()
        self.start_position = self.position.copy()
        self.motion_active = False
        self.node.get_logger().info(
            f'[{self.name}] SET_POSITION {self.position} yaw={self.yaw:.2f} (instant)')
        return response

    # ------------------------------------------------------------ cmd topics

    def _mark_low_level(self):
        self.goal = self.position.copy()
        self.motion_active = False
        self.last_low_level_time = self.node.get_clock().now()

    def _cmd_hover_cb(self, msg):
        self.hover_cmd = msg
        self.motion_active = False

    def _cmd_full_state_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.position = np.array([p.x, p.y, p.z])
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self._mark_low_level()

    def _cmd_position_cb(self, msg):
        self.position = np.array([msg.x, msg.y, msg.z])
        self.yaw = msg.yaw
        self._mark_low_level()

    def _cmd_velocity_world_cb(self, msg):
        # Each message advances the drone by one tick at the commanded velocity.
        self.position += np.array([msg.vel.x, msg.vel.y, msg.vel.z]) * self.dt
        self.yaw = wrap_angle(self.yaw + msg.yaw_rate * self.dt)
        self._mark_low_level()

    def _cmd_vel_legacy_cb(self, msg):
        # linear x/y as world velocities, angular.z as yaw rate; thrust (linear.z) ignored.
        self.position[0] += msg.linear.x * self.dt
        self.position[1] += msg.linear.y * self.dt
        self.yaw = wrap_angle(self.yaw + msg.angular.z * self.dt)
        self._mark_low_level()
