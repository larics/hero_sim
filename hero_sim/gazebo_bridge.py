"""gazebo_bridge: keeps Gazebo's robots where the kinematic sim has them and brings camera images to ROS.

  poses   reads world -> <robot> from /tf (ground_robot_sim, crazyflie_server)
          and sends the ones that changed to Gazebo SYNC_RATE times a second,
          in one set_pose_vector request (gazebo.SET_POSE_SERVICE)
  images  republishes each camera's frames (gazebo.camera_topic) as
          /<robot>/camera/image_raw (sensor_msgs/Image, rgb8) and
          /<robot>/camera/camera_info, in frame <robot>/camera_optical,
          stamped with the ROS clock as they arrive. They line up with TF
          without use_sim_time: Gazebo's own clock is never used
  TF      static <robot> -> <robot>/camera_link (the camera's x, y, z and
          pitch; x forward, z up) -> <robot>/camera_optical (z forward, x
          right, y down)

It talks to Gazebo through Gazebo's Python bindings (gz.transport13), so no
ros_gz_bridge is needed. sim.launch.py starts it with the gz server when
config.gazebo_enabled says so.
"""
import array
import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

from hero_sim import config, gazebo

FRAME = 'world'
SYNC_RATE = 30.0  # Hz, pose updates sent to Gazebo
REQUEST_TIMEOUT = 100  # ms, for one set_pose_vector request
RETRY_PERIOD = 1.0  # s, wait after a failed request (Gazebo still starting)
FIRST_IMAGE_WAIT = 15.0  # s, then warn about a camera that sent nothing
RGB_INT8 = 3  # gz.msgs.PixelFormatType of the cameras' R8G8B8 images


def camera_info(camera, frame):
    """sensor_msgs/CameraInfo of a config.Camera (gazebo.intrinsics), no distortion."""
    fx, fy, cx, cy = gazebo.intrinsics(camera)
    info = CameraInfo()
    info.header.frame_id = frame
    info.width, info.height = camera.width, camera.height
    info.distortion_model = 'plumb_bob'
    info.d = [0.0] * 5
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def camera_transforms(robot, camera):
    """The static TFs <robot> -> camera_link -> camera_optical of a robot's camera."""
    link, optical = gazebo.camera_frames(robot)
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = robot, link
    t = mount.transform
    t.translation.x, t.translation.y, t.translation.z = camera.x, camera.y, camera.z
    t.rotation.y, t.rotation.w = math.sin(camera.pitch / 2.0), math.cos(camera.pitch / 2.0)
    lens = TransformStamped()
    lens.header.frame_id, lens.child_frame_id = link, optical
    q = lens.transform.rotation
    q.x, q.y, q.z, q.w = gazebo.OPTICAL_ROTATION
    return [mount, lens]


class _Camera:
    """One camera's ROS side: publishers and its frame count."""
    __slots__ = ('robot', 'frame', 'image_pub', 'info_pub', 'info', 'frames', 'warned')

    def __init__(self, node, robot, camera):
        self.robot = robot
        self.frame = gazebo.camera_frames(robot)[1]
        self.image_pub = node.create_publisher(Image, f'/{robot}/camera/image_raw', 5)
        self.info_pub = node.create_publisher(CameraInfo, f'/{robot}/camera/camera_info', 5)
        self.info = camera_info(camera, self.frame)
        self.frames = 0
        self.warned = False


class GazeboBridge(Node):

    def __init__(self):
        super().__init__('gazebo_bridge')
        self.declare_parameter('config_path', '')
        scenario = config.load(config.resolve_config(self.get_parameter('config_path').value))
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.image_pb2 import Image as GzImage
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self._Pose_V, self._Boolean = Pose_V, Boolean
        self._gz = GzNode()

        self._robots = ({r.name for r in scenario.ground_robots}
                        | {cf.name for cf in scenario.crazyflies})
        self._latest = {}  # robot -> (x, y, z, qx, qy, qz, qw) from /tf
        self._sent = {}  # robot -> the pose Gazebo has
        self._retry_at = None  # rclpy Time; set after a failed request
        self._synced = False
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        self.create_timer(1.0 / SYNC_RATE, self._sync)

        self._cameras = []
        transforms = []
        for r in scenario.ground_robots:
            if r.camera:
                cam = _Camera(self, r.name, r.camera)
                self._gz.subscribe(GzImage, gazebo.camera_topic(r.name),
                                   lambda msg, c=cam: self._on_image(c, msg))
                self._cameras.append(cam)
                transforms += camera_transforms(r.name, r.camera)
        if transforms:
            stamp = self.get_clock().now().to_msg()
            for t in transforms:
                t.header.stamp = stamp
            self._static_tf = StaticTransformBroadcaster(self)
            self._static_tf.sendTransform(transforms)
            self._first_image_timer = self.create_timer(FIRST_IMAGE_WAIT, self._check_cameras)

        cams = ', '.join(f'/{c.robot}/camera/image_raw' for c in self._cameras) or 'none'
        self.get_logger().info(f'syncing {len(self._robots)} robot(s) into Gazebo world '
                               f'{gazebo.WORLD} at {SYNC_RATE:g} Hz; cameras: {cams}')

    def _on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == FRAME and t.child_frame_id in self._robots:
                p, q = t.transform.translation, t.transform.rotation
                self._latest[t.child_frame_id] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def _sync(self):
        changed = {name: pose for name, pose in self._latest.items()
                   if self._sent.get(name) != pose}
        if not changed:
            return
        now = self.get_clock().now()
        if self._retry_at is not None and now < self._retry_at:
            return
        request = self._Pose_V()
        for name, (x, y, z, qx, qy, qz, qw) in changed.items():
            pose = request.pose.add()
            pose.name = name
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.x, pose.orientation.y = qx, qy
            pose.orientation.z, pose.orientation.w = qz, qw
        ok, response = self._gz.request(gazebo.SET_POSE_SERVICE, request, self._Pose_V,
                                        self._Boolean, REQUEST_TIMEOUT)
        if not (ok and response.data):
            self._retry_at = now + Duration(seconds=RETRY_PERIOD)
            self.get_logger().info('waiting for Gazebo (gz sim server) to come up',
                                   throttle_duration_sec=5.0)
            return
        self._retry_at = None
        self._sent.update(changed)
        if not self._synced:
            self._synced = True
            self.get_logger().info('Gazebo is up: robots synced')

    def _on_image(self, cam, msg):
        """A frame from Gazebo, on gz-transport's thread."""
        stamp = self.get_clock().now().to_msg()
        if msg.pixel_format_type != RGB_INT8:
            if not cam.warned:
                cam.warned = True
                self.get_logger().error(f'{cam.robot} camera: pixel format '
                                        f'{msg.pixel_format_type}, expected RGB_INT8; dropped')
            return
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = cam.frame
        image.height, image.width = msg.height, msg.width
        image.encoding = 'rgb8'
        image.step = 3 * msg.width
        image.data = array.array('B', msg.data)  # an array, not bytes: no per-byte check
        cam.info.header.stamp = stamp
        cam.image_pub.publish(image)
        cam.info_pub.publish(cam.info)
        cam.frames += 1

    def _check_cameras(self):
        self._first_image_timer.cancel()
        for cam in self._cameras:
            if not cam.frames:
                self.get_logger().warn(
                    f'no images from {cam.robot}\'s camera after {FIRST_IMAGE_WAIT:g} s: '
                    f'check the gz sim server output (rendering needs a GPU, or Mesa)')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
