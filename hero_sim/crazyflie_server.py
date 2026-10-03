"""crazyflie_server stand-in: simulates every Crazyflie in the scenario config.

Same per-drone interface as crazyswarm2's crazyflie_server (see SimCrazyflie)
plus the /all/* broadcast services.
"""
from functools import partial

from crazyflie_interfaces.srv import Arm, GoTo, Land, NotifySetpointsStop, StartTrajectory, Takeoff
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from hero_sim import config
from hero_sim.crazyflie import SimCrazyflie

# (service type, /all/<name>, SimCrazyflie callback)
_BROADCASTS = (
    (Takeoff, 'takeoff', '_takeoff_cb'),
    (Land, 'land', '_land_cb'),
    (GoTo, 'go_to', '_go_to_cb'),
    (Arm, 'arm', '_arm_cb'),
    (NotifySetpointsStop, 'notify_setpoints_stop', '_notify_setpoints_stop_cb'),
    (StartTrajectory, 'start_trajectory', '_start_trajectory_cb'),
)

STATUS_PERIOD = 1.0  # s


class CrazyflieServer(Node):

    def __init__(self):
        super().__init__('crazyflie_server')
        self.declare_parameter('config_path', '')
        config_path = config.resolve_config(self.get_parameter('config_path').value)
        scenario = config.load(config_path)
        dt = 1.0 / scenario.sim['cf_rate']

        self.drones = {
            cf.name: SimCrazyflie(self, cf.name, (cf.x, cf.y, cf.z), dt=dt,
                                  battery_drain=scenario.sim['cf_battery_drain'])
            for cf in scenario.crazyflies
        }

        for srv_type, name, method in _BROADCASTS:
            self.create_service(srv_type, f'/all/{name}', partial(self._broadcast, name, method))

        self._tf = TransformBroadcaster(self)
        self.create_timer(dt, self._update_all)
        self.create_timer(STATUS_PERIOD, self._publish_status_all)

        self.get_logger().info(
            f'{len(self.drones)} crazyflies at {scenario.sim["cf_rate"]:g} Hz from {config_path}: '
            f'{list(self.drones)}')

    def _update_all(self):
        transforms = [drone.update() for drone in self.drones.values()]
        if transforms:
            self._tf.sendTransform(transforms)

    def _publish_status_all(self):
        for drone in self.drones.values():
            drone.publish_status()

    def _broadcast(self, name, method, request, response):
        self.get_logger().info(f'[all] {name.upper()} group_mask={getattr(request, "group_mask", 0)}')
        for drone in self.drones.values():
            getattr(drone, method)(request, response)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
