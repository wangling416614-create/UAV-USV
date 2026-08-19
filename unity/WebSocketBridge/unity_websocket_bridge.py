#!/usr/bin/env python3

import base64
import hashlib
import json
import math
import queue
import socket
import struct
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.world_control_pb2 import WorldControl
from gz.transport13 import Node as GzTransportNode
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from escort_guard_algorithm import RealtimeEscortGuardPlanner


# Exact ENU roots created by local Unity's SimulationBootstrap after applying
# PresentationCoordinateScale=0.18. Gazebo and Unity return to this one source
# of truth between classroom demonstrations.
SCENE_HOME_POSES = {
    'uav_01': (-15.6348, -40.0374, 3.53210004, 0.559),
    'uav_02': (-13.5, -38.7, 3.53210004, 0.559),
    'uav_03': (-11.3652, -37.3626, 3.53210004, 0.559),
    'usv_01': (-21.6, -54.9, 0.0, 0.10),
    'usv_02': (-13.5, -57.6, 0.0, 0.05),
    'usv_03': (-5.4, -54.9, 0.0, -0.05),
    'friendly_ship': (-27.0, -63.9, 0.0, 0.25),
    'enemy_ship': (-14.4, -62.1, 0.0, 2.60),
}

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:
    NavigateToPose = None

try:
    from uav_usv_interfaces.msg import CaptureAssignmentArray
    from uav_usv_interfaces.msg import CaptureState
    from uav_usv_interfaces.msg import CommandAck
    from uav_usv_interfaces.msg import ControlLease
    from uav_usv_interfaces.msg import FleetCommand
    from uav_usv_interfaces.msg import TrackedObjectArray
    from uav_usv_interfaces.msg import AffiliatedDetection2DArray
    from uav_usv_interfaces.msg import VehicleState
except ImportError:
    AffiliatedDetection2DArray = None
    CaptureAssignmentArray = None
    CaptureState = None
    CommandAck = None
    ControlLease = None
    FleetCommand = None
    TrackedObjectArray = None
    VehicleState = None


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    x, y, z, w = quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class UnityWebSocketBridge(Node):
    def __init__(self):
        super().__init__('unity_websocket_bridge')

        self.declare_parameter('gazebo_pose_topic', '/world/default/pose/info')
        self.declare_parameter('boat_name', 'landing_boat')
        self.declare_parameter('drone_name', 'x500_0')
        self.declare_parameter('usv_names', ['landing_boat'])
        self.declare_parameter('uav_names', ['x500_0'])
        self.declare_parameter('lighthouse_name', 'navigation_lighthouse')
        self.declare_parameter('buoy_west_name', 'medium_buoy_west_channel')
        self.declare_parameter('buoy_south_name', 'medium_buoy_south_channel')
        self.declare_parameter('buoy_east_name', 'medium_buoy_east_channel')
        self.declare_parameter('friendly_ship_name', 'friendly_ship')
        self.declare_parameter('target_vessel_name', 'target_vessel')
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('pose_stale_timeout', 1.0)
        self.declare_parameter('control_mode', 'direct')
        self.declare_parameter('boat_cmd_topic', '/model/simple_boat/cmd_vel')
        self.declare_parameter('nav2_action_name', 'navigate_to_pose')
        self.declare_parameter('control_rate', 15.0)
        self.declare_parameter('waypoint_radius', 2.0)
        self.declare_parameter('final_arrival_radius', 1.0)
        self.declare_parameter('slow_radius', 6.0)
        self.declare_parameter('max_speed', 1.25)
        self.declare_parameter('min_speed', 0.25)
        self.declare_parameter('turn_gain', 1.5)
        self.declare_parameter('max_turn', 1.8)
        self.declare_parameter('heading_slowdown_yaw', 1.0)
        self.declare_parameter('max_path_points', 512)
        self.declare_parameter('max_abs_coordinate', 200.0)
        self.declare_parameter('simulation_coordinate_scale', 1.0)
        self.declare_parameter('capture_state_topic', '/capture/state')
        self.declare_parameter('capture_roles_topic', '/capture/roles')
        self.declare_parameter('fleet_state_topic', '/fleet/state')
        self.declare_parameter('fleet_command_topic', '/fleet/command')
        self.declare_parameter('command_ack_topic', '/fleet/command_ack')
        self.declare_parameter('control_lease_topic', '/fleet/control_lease')
        self.declare_parameter(
            'operator_action_topic', '/fleet/base/operator_action'
        )
        self.declare_parameter('gateway_owner_id', 'uav_usv_platform')
        self.declare_parameter('gateway_priority', 220)
        self.declare_parameter('lease_duration', 20.0)
        self.declare_parameter('command_lifetime', 15.0)
        self.declare_parameter('mission_ack_timeout', 12.0)
        self.declare_parameter('takeoff_altitude', 18.0)
        self.declare_parameter('escort_command_period', 2.0)
        self.declare_parameter('escort_takeoff_retry_period', 20.0)
        self.declare_parameter('escort_usv_radius', 28.0)
        self.declare_parameter('escort_uav_radius', 42.0)
        self.declare_parameter('escort_uav_altitude', 24.0)
        self.declare_parameter('escort_algorithm_scale', 7.0)
        self.declare_parameter('escort_reserve_count', 0)
        self.declare_parameter('enable_camera_stream', True)
        self.declare_parameter('camera_publish_rate', 8.0)
        self.declare_parameter('camera_thumbnail_rate', 2.0)
        self.declare_parameter('camera_jpeg_quality', 55)
        self.declare_parameter('camera_max_width', 640)
        self.declare_parameter('camera_max_height', 360)
        self.declare_parameter('default_camera_id', 'usv_01')
        self.declare_parameter('enable_sensor_stream', True)
        self.declare_parameter(
            'radar_tracks_topic', '/perception/lv_dot_ros2/tracks'
        )
        self.declare_parameter('radar_device_ids', ['fleet_fused'])
        self.declare_parameter(
            'radar_tracks_topics', ['/perception/lv_dot_ros2/tracks']
        )
        self.declare_parameter(
            'pointcloud_topic', '/perception/usv_01/mid360/preview'
        )
        self.declare_parameter(
            'pointcloud_device_ids', ['usv_01', 'usv_02', 'usv_03']
        )
        self.declare_parameter('pointcloud_topics', [
            '/perception/usv_01/mid360/preview',
            '/perception/usv_02/mid360/preview',
            '/perception/usv_03/mid360/preview',
        ])
        self.declare_parameter(
            'visual_detections_topic',
            '/perception/usv_01/camera/affiliated_detections',
        )
        self.declare_parameter('sensor_publish_rate', 2.0)
        self.declare_parameter('pointcloud_max_points', 1800)

        self.pose_topic = self.get_parameter('gazebo_pose_topic').value
        self.ws_host = self.get_parameter('ws_host').value
        self.ws_port = int(self.get_parameter('ws_port').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.pose_stale_timeout = float(
            self.get_parameter('pose_stale_timeout').value
        )
        self.control_mode = str(
            self.get_parameter('control_mode').value
        ).strip().lower()
        if self.control_mode not in ('observe', 'direct', 'nav2'):
            raise ValueError(
                'control_mode must be "observe", "direct", or "nav2"'
            )
        self.boat_cmd_topic = self.get_parameter('boat_cmd_topic').value
        self.control_rate = max(
            1.0,
            float(self.get_parameter('control_rate').value),
        )
        self.waypoint_radius = max(
            0.1,
            float(self.get_parameter('waypoint_radius').value),
        )
        self.final_arrival_radius = max(
            0.1,
            float(self.get_parameter('final_arrival_radius').value),
        )
        self.slow_radius = max(
            self.final_arrival_radius,
            float(self.get_parameter('slow_radius').value),
        )
        self.max_speed = max(0.0, float(self.get_parameter('max_speed').value))
        self.min_speed = clamp(
            float(self.get_parameter('min_speed').value),
            0.0,
            self.max_speed,
        )
        self.turn_gain = float(self.get_parameter('turn_gain').value)
        self.max_turn = max(0.0, float(self.get_parameter('max_turn').value))
        self.heading_slowdown_yaw = max(
            0.01,
            float(self.get_parameter('heading_slowdown_yaw').value),
        )
        self.max_path_points = max(
            2,
            int(self.get_parameter('max_path_points').value),
        )
        self.max_abs_coordinate = max(
            1.0,
            float(self.get_parameter('max_abs_coordinate').value),
        )
        self.simulation_coordinate_scale = max(
            1e-6,
            float(self.get_parameter('simulation_coordinate_scale').value),
        )

        self.usv_names = tuple(
            str(value) for value in self.get_parameter('usv_names').value
            if str(value)
        )
        self.uav_names = tuple(
            str(value) for value in self.get_parameter('uav_names').value
            if str(value)
        )
        if not self.usv_names:
            self.usv_names = (str(self.get_parameter('boat_name').value),)
        if not self.uav_names:
            self.uav_names = (str(self.get_parameter('drone_name').value),)
        self.target_entity_name = str(
            self.get_parameter('target_vessel_name').value
        )
        self.friendly_ship_name = str(
            self.get_parameter('friendly_ship_name').value
        )

        self.entity_names = {
            self.get_parameter('lighthouse_name').value: 'lighthouse',
            self.get_parameter('buoy_west_name').value: 'buoy_west',
            self.get_parameter('buoy_south_name').value: 'buoy_south',
            self.get_parameter('buoy_east_name').value: 'buoy_east',
            self.friendly_ship_name: self.friendly_ship_name,
            self.target_entity_name: self.target_entity_name,
        }
        for name in self.usv_names:
            self.entity_names[name] = name
        for name in self.uav_names:
            self.entity_names[name] = name

        self.latest = {}
        self.capture_state = {}
        self.capture_roles = {}
        self.vehicle_states = {}
        self.last_gazebo_update = None
        self.last_boat_update = None
        self.sequence = 0
        self.lock = threading.Lock()
        self.client_lock = threading.Lock()
        self.ws_clients = []
        self.client_send_locks = {}
        self.running = True
        self.server_socket = None
        self.command_queue = queue.Queue(maxsize=64)
        self.active_path = []
        self.active_path_id = 0
        self.waypoint_index = 0
        self.control_state = (
            'observe' if self.control_mode == 'observe' else 'idle'
        )
        self.control_message = (
            'ROS fleet/base station is authoritative'
            if self.control_mode == 'observe'
            else 'Waiting for a Unity path'
        )
        self.nav2_goal_handle = None
        self.nav2_goal_pending = False

        self.gateway_owner_id = str(
            self.get_parameter('gateway_owner_id').value
        ).strip() or 'uav_usv_platform'
        self.gateway_priority = int(clamp(
            float(self.get_parameter('gateway_priority').value), 1, 255
        ))
        self.lease_duration = max(
            2.0, float(self.get_parameter('lease_duration').value)
        )
        self.command_lifetime = max(
            2.0, float(self.get_parameter('command_lifetime').value)
        )
        self.mission_ack_timeout = max(
            2.0, float(self.get_parameter('mission_ack_timeout').value)
        )
        self.takeoff_altitude = max(
            1.0, float(self.get_parameter('takeoff_altitude').value)
        )
        self.escort_command_period = max(
            0.5, float(self.get_parameter('escort_command_period').value)
        )
        self.escort_takeoff_retry_period = max(
            5.0,
            float(self.get_parameter('escort_takeoff_retry_period').value),
        )
        self.escort_usv_radius = max(
            5.0, float(self.get_parameter('escort_usv_radius').value)
        )
        self.escort_uav_radius = max(
            self.escort_usv_radius,
            float(self.get_parameter('escort_uav_radius').value),
        )
        self.escort_uav_altitude = max(
            self.takeoff_altitude,
            float(self.get_parameter('escort_uav_altitude').value),
        )
        self.escort_algorithm_scale = max(
            0.1, float(self.get_parameter('escort_algorithm_scale').value)
        )
        self.escort_reserve_count = int(
            self.get_parameter('escort_reserve_count').value
        )
        self.escort_planner = RealtimeEscortGuardPlanner(
            scale=self.escort_algorithm_scale,
            reserve_count=self.escort_reserve_count,
        )
        self.gateway_lease_id = 'platform-websocket'
        self.active_control_until = {}
        self.last_platform_lease_publish = {}
        self.pending_fleet_commands = []
        self.scene_reset_pose_deadline = 0.0
        self.scene_reset_next_pose_time = 0.0
        self.scene_home_lock_active = False
        self.pending_scene_reset_requests = []
        self.scene_reset_failures = set()
        self.pending_platform_acks = []
        self.pending_mission_commands = {}
        self.platform_command_ids = set()
        self.last_mission_state_signatures = {}
        self.capture_observed_started = False
        self.home_poses = {}
        self.last_navigation_targets = {}
        self.escort_active = False
        self.escort_paused = False
        self.escort_protected_id = self.friendly_ship_name
        self.escort_last_command_time = 0.0
        self.escort_command_sequence = 0
        self.escort_takeoff_commands = {}
        self.escort_takeoff_state = {}
        self.escort_state = {
            'active': False,
            'paused': False,
            'phase': 'IDLE',
            'protected_id': self.friendly_ship_name,
            'reason': 'waiting for mission command',
        }

        self.enable_camera_stream = bool(
            self.get_parameter('enable_camera_stream').value
        )
        self.camera_publish_rate = max(
            1.0,
            float(self.get_parameter('camera_publish_rate').value),
        )
        self.camera_thumbnail_rate = max(
            0.2,
            float(self.get_parameter('camera_thumbnail_rate').value),
        )
        self.camera_jpeg_quality = int(
            clamp(float(self.get_parameter('camera_jpeg_quality').value), 20, 95)
        )
        self.camera_max_width = max(
            80, int(self.get_parameter('camera_max_width').value)
        )
        self.camera_max_height = max(
            60, int(self.get_parameter('camera_max_height').value)
        )
        self.selected_camera_id = str(
            self.get_parameter('default_camera_id').value
        ).strip() or 'usv_01'
        self.camera_topics = {}
        self.latest_camera_msgs = {}
        self.latest_camera_times = {}
        self.camera_lock = threading.Lock()
        self.last_streamed_camera_stamp = {}
        self.last_camera_publish_time = {}

        self.enable_sensor_stream = bool(
            self.get_parameter('enable_sensor_stream').value
        )
        self.sensor_publish_rate = max(
            0.2, float(self.get_parameter('sensor_publish_rate').value)
        )
        self.pointcloud_max_points = max(
            100, int(self.get_parameter('pointcloud_max_points').value)
        )
        self.latest_radar_tracks = {}
        self.latest_pointcloud = {}
        self.latest_visual_detections = None
        self.last_sensor_stamp = {}
        self.sensor_lock = threading.Lock()

        self.gz_node = GzTransportNode()
        subscribed = self.gz_node.subscribe(Pose_V, self.pose_topic, self._on_pose)
        if not subscribed:
            raise RuntimeError(f'Unable to subscribe to {self.pose_topic}')
        self.boat_cmd_pub = self.gz_node.advertise(self.boat_cmd_topic, Twist)
        self.scene_cmd_pubs = {
            name: self.gz_node.advertise('/model/%s/cmd_vel' % name, Twist)
            for name in ('own_01', 'own_02', 'own_03',
                         'friendly_ship', 'enemy_ship')
        }
        self.target_motion_pub = self.gz_node.advertise(
            '/model/enemy_ship/motion_enable', GzBoolean
        )
        pose_parts = self.pose_topic.strip('/').split('/')
        self.gazebo_world = (
            pose_parts[1]
            if len(pose_parts) >= 2 and pose_parts[0] == 'world'
            else 'heterogeneous_332'
        )

        self.fleet_command_pub = None
        self.control_lease_pub = None
        self.operator_action_pub = None
        if FleetCommand is not None:
            lease_qos = QoSProfile(depth=10)
            lease_qos.reliability = ReliabilityPolicy.RELIABLE
            lease_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.fleet_command_pub = self.create_publisher(
                FleetCommand,
                str(self.get_parameter('fleet_command_topic').value),
                60,
            )
            self.control_lease_pub = self.create_publisher(
                ControlLease,
                str(self.get_parameter('control_lease_topic').value),
                lease_qos,
            )
            self.operator_action_pub = self.create_publisher(
                String,
                str(self.get_parameter('operator_action_topic').value),
                20,
            )
            self.create_subscription(
                String,
                str(self.get_parameter('operator_action_topic').value),
                self._on_operator_action,
                20,
            )
            self.create_subscription(
                CommandAck,
                str(self.get_parameter('command_ack_topic').value),
                self._on_command_ack,
                60,
            )

        self.nav2_client = None
        if self.control_mode == 'nav2':
            if NavigateToPose is None:
                raise RuntimeError(
                    'control_mode=nav2 requires ros-humble-nav2-msgs'
                )
            self.nav2_client = ActionClient(
                self,
                NavigateToPose,
                self.get_parameter('nav2_action_name').value,
            )

        self.server_thread = threading.Thread(
            target=self._serve, name='unity-websocket-server', daemon=True
        )
        self.server_thread.start()

        self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish_frame)
        self.create_timer(1.0 / self.control_rate, self._control_tick)
        if self.enable_camera_stream:
            self._setup_camera_subscriptions()
            self.create_timer(
                1.0 / self.camera_publish_rate,
                self._publish_selected_camera,
            )
        if self.enable_sensor_stream:
            self._setup_sensor_subscriptions()
            self.create_timer(
                1.0 / self.sensor_publish_rate,
                self._publish_sensor_frames,
            )
        if CaptureState is not None:
            fleet_state_qos = QoSProfile(depth=10)
            fleet_state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            fleet_state_qos.durability = DurabilityPolicy.VOLATILE
            self.create_subscription(
                CaptureState,
                str(self.get_parameter('capture_state_topic').value),
                self._on_capture_state,
                10,
            )
            self.create_subscription(
                CaptureAssignmentArray,
                str(self.get_parameter('capture_roles_topic').value),
                self._on_capture_roles,
                10,
            )
            self.create_subscription(
                VehicleState,
                str(self.get_parameter('fleet_state_topic').value),
                self._on_vehicle_state,
                fleet_state_qos,
            )
        camera_note = (
            f'camera stream on ({self.camera_publish_rate:.0f} Hz, '
            f'selected={self.selected_camera_id})'
            if self.enable_camera_stream
            else 'camera stream off'
        )
        self.get_logger().info(
            f'Unity WebSocket bridge listening on ws://{self.ws_host}:{self.ws_port}/uav_usv; '
            f'reading Gazebo poses from {self.pose_topic}; '
            f'fleet={len(self.usv_names)} USV + {len(self.uav_names)} UAV; '
            f'control mode={self.control_mode}; {camera_note}'
        )

    def _setup_camera_subscriptions(self):
        if cv2 is None:
            self.get_logger().warn(
                'OpenCV (cv2) unavailable; Gazebo camera stream disabled'
            )
            self.enable_camera_stream = False
            return

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        for usv_id in self.usv_names:
            topic = f'/fleet/uplink/{usv_id}/camera'
            self.camera_topics[usv_id] = topic
            self.create_subscription(
                Image,
                topic,
                self._make_camera_callback(usv_id),
                sensor_qos,
            )
            self.get_logger().info(f'Subscribed Gazebo USV camera {topic}')

        for uav_id in self.uav_names:
            topic = f'/fleet/uplink/{uav_id}/camera/image_raw'
            self.camera_topics[uav_id] = topic
            self.create_subscription(
                Image,
                topic,
                self._make_camera_callback(uav_id),
                sensor_qos,
            )
            self.get_logger().info(f'Subscribed Gazebo UAV camera {topic}')

        if self.selected_camera_id not in self.camera_topics:
            self.selected_camera_id = next(iter(self.camera_topics), 'usv_01')

    def _make_camera_callback(self, camera_id):
        def callback(msg):
            with self.camera_lock:
                self.latest_camera_msgs[camera_id] = msg
                self.latest_camera_times[camera_id] = time.monotonic()

        return callback

    def _accept_select_camera(self, command):
        camera_id = str(command.get('camera_id', '')).strip()
        if not camera_id:
            self.get_logger().warn('select_camera missing camera_id')
            return
        if self.camera_topics and camera_id not in self.camera_topics:
            self.get_logger().warn(
                f'Unknown camera_id {camera_id!r}; known={list(self.camera_topics)}'
            )
            return
        with self.camera_lock:
            self.selected_camera_id = camera_id
            self.last_streamed_camera_stamp.pop(camera_id, None)
        self.get_logger().info(f'Unity selected Gazebo camera {camera_id}')

    def _accept_sensor_subscription(self, command):
        camera_id = str(command.get('focused_camera_id', '')).strip()
        if camera_id:
            self._accept_select_camera({'camera_id': camera_id})
        try:
            thumbnail_fps = float(command.get(
                'thumbnail_fps', self.camera_thumbnail_rate
            ))
            focused_fps = float(command.get(
                'focused_fps', self.camera_publish_rate
            ))
            self.camera_thumbnail_rate = clamp(thumbnail_fps, 0.2, 6.0)
            self.camera_publish_rate = clamp(focused_fps, 1.0, 15.0)
        except (TypeError, ValueError):
            self.get_logger().warn('Ignored invalid visual sensor frame rates')

    def _publish_selected_camera(self):
        if not self.enable_camera_stream or cv2 is None:
            return
        with self.client_lock:
            if not self.ws_clients:
                return

        with self.camera_lock:
            selected_camera_id = self.selected_camera_id
            camera_ids = list(self.camera_topics)

        now = time.monotonic()
        for camera_id in camera_ids:
            rate = (
                self.camera_publish_rate
                if camera_id == selected_camera_id
                else self.camera_thumbnail_rate
            )
            last_publish = self.last_camera_publish_time.get(camera_id, 0.0)
            if now - last_publish < 1.0 / max(rate, 0.2):
                continue
            if self._publish_camera(camera_id):
                self.last_camera_publish_time[camera_id] = now

    def _publish_camera(self, camera_id):
        with self.camera_lock:
            msg = self.latest_camera_msgs.get(camera_id)
            received_at = self.latest_camera_times.get(camera_id)

        if msg is None or received_at is None:
            return False

        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.height, msg.width)
        if self.last_streamed_camera_stamp.get(camera_id) == stamp_key:
            return False

        try:
            frame = self._decode_image(msg)
            height, width = frame.shape[:2]
            scale = min(
                1.0,
                float(self.camera_max_width) / max(width, 1),
                float(self.camera_max_height) / max(height, 1),
            )
            if scale < 0.999:
                frame = cv2.resize(
                    frame,
                    (
                        max(1, int(round(width * scale))),
                        max(1, int(round(height * scale))),
                    ),
                    interpolation=cv2.INTER_AREA,
                )
            ok, encoded = cv2.imencode(
                '.jpg',
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.camera_jpeg_quality],
            )
            if not ok:
                return False
            jpeg_b64 = base64.b64encode(encoded.tobytes()).decode('ascii')
            out_h, out_w = frame.shape[:2]
            age = max(0.0, time.monotonic() - received_at)
            payload = {
                'type': 'camera_frame',
                'camera_id': camera_id,
                'encoding': 'jpeg',
                'width': int(out_w),
                'height': int(out_h),
                'timestamp_ms': int(time.time() * 1000),
                'age_seconds': round(age, 3),
                'jpeg_base64': jpeg_b64,
            }
            self._broadcast(json.dumps(payload, separators=(',', ':')))
            self.last_streamed_camera_stamp[camera_id] = stamp_key
            return True
        except Exception as exc:
            self.get_logger().warn(
                f'Unable to stream camera {camera_id}: {exc}',
                throttle_duration_sec=5.0,
            )
            return False

    @staticmethod
    def _decode_image(msg):
        channels = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
            'mono8': 1,
        }.get(msg.encoding.lower())
        if channels is None:
            raise ValueError(f'unsupported encoding {msg.encoding}')
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.step
        )
        image = rows[:, : msg.width * channels].reshape(
            msg.height, msg.width, channels
        )
        encoding = msg.encoding.lower()
        if encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding == 'mono8':
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image.copy()

    def _setup_sensor_subscriptions(self):
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        pointcloud_ids = [
            str(value).strip()
            for value in self.get_parameter('pointcloud_device_ids').value
            if str(value).strip()
        ]
        pointcloud_topics = [
            str(value).strip()
            for value in self.get_parameter('pointcloud_topics').value
            if str(value).strip()
        ]
        if len(pointcloud_ids) != len(pointcloud_topics):
            raise ValueError(
                'pointcloud_device_ids and pointcloud_topics must have equal lengths'
            )
        for device_id, topic in zip(pointcloud_ids, pointcloud_topics):
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, source=device_id: self._on_pointcloud(msg, source),
                sensor_qos,
            )
            self.get_logger().info(
                f'Subscribed frontend point cloud {device_id} <- {topic}'
            )

        if TrackedObjectArray is not None:
            radar_ids = [
                str(value).strip()
                for value in self.get_parameter('radar_device_ids').value
                if str(value).strip()
            ]
            radar_topics = [
                str(value).strip()
                for value in self.get_parameter('radar_tracks_topics').value
                if str(value).strip()
            ]
            if len(radar_ids) != len(radar_topics):
                raise ValueError(
                    'radar_device_ids and radar_tracks_topics must have equal lengths'
                )
            for device_id, topic in zip(radar_ids, radar_topics):
                self.create_subscription(
                    TrackedObjectArray,
                    topic,
                    lambda msg, source=device_id: self._on_radar_tracks(
                        msg, source
                    ),
                    sensor_qos,
                )
                self.get_logger().info(
                    f'Subscribed frontend radar tracks {device_id} <- {topic}'
                )

        if AffiliatedDetection2DArray is not None:
            detections_topic = str(
                self.get_parameter('visual_detections_topic').value
            )
            self.create_subscription(
                AffiliatedDetection2DArray,
                detections_topic,
                self._on_visual_detections,
                sensor_qos,
            )
            self.get_logger().info(
                f'Subscribed frontend visual detections {detections_topic}'
            )

    def _on_radar_tracks(self, msg, device_id='fleet_fused'):
        with self.sensor_lock:
            self.latest_radar_tracks[device_id] = msg

    def _on_pointcloud(self, msg, device_id='usv_01'):
        with self.sensor_lock:
            self.latest_pointcloud[device_id] = msg

    def _on_visual_detections(self, msg):
        with self.sensor_lock:
            self.latest_visual_detections = msg

    @staticmethod
    def _message_stamp_key(msg):
        stamp = msg.header.stamp
        return int(stamp.sec), int(stamp.nanosec)

    def _publish_sensor_frames(self):
        with self.client_lock:
            if not self.ws_clients:
                return
        with self.sensor_lock:
            radars = dict(self.latest_radar_tracks)
            clouds = dict(self.latest_pointcloud)
            detections = self.latest_visual_detections

        for device_id, radar in radars.items():
            stamp = self._message_stamp_key(radar)
            stamp_key = 'radar:' + device_id
            if self.last_sensor_stamp.get(stamp_key) != stamp:
                self._publish_radar_frame(radar, device_id)
                self.last_sensor_stamp[stamp_key] = stamp
        for device_id, cloud in clouds.items():
            stamp = self._message_stamp_key(cloud)
            stamp_key = 'pointcloud:' + device_id
            if self.last_sensor_stamp.get(stamp_key) != stamp:
                self._publish_pointcloud_frame(cloud, device_id)
                self.last_sensor_stamp[stamp_key] = stamp
        if detections is not None:
            stamp = self._message_stamp_key(detections)
            if self.last_sensor_stamp.get('detections') != stamp:
                self._publish_visual_detection_frame(detections)
                self.last_sensor_stamp['detections'] = stamp

    def _publish_radar_frame(self, msg, device_id='fleet_fused'):
        timestamp_ms = int(time.time() * 1000)
        detections = []
        for item in msg.objects:
            position = item.pose.pose.position
            x = float(position.x)
            y = float(position.y)
            z = float(position.z)
            detections.append({
                'id': str(item.track_id) or 'track',
                'range': round(math.hypot(x, y), 4),
                'bearing': round(math.degrees(math.atan2(y, x)), 4),
                'x': x,
                'y': y,
                'z': z,
                'confidence': float(item.confidence),
                'class_name': str(item.class_name),
                'class_confidence': float(item.class_confidence),
                'affiliation': int(item.affiliation),
                'affiliation_confidence': float(item.affiliation_confidence),
                'source': str(item.sensor_source),
                'timestamp_ms': timestamp_ms,
            })
        payload = {
            'type': 'radar_frame',
            'frame': {
                'device_id': device_id,
                'frame_id': str(msg.header.frame_id),
                'timestamp_ms': timestamp_ms,
                'detections': detections,
            },
        }
        self._broadcast(json.dumps(payload, separators=(',', ':')))

    def _publish_pointcloud_frame(self, msg, device_id='usv_01'):
        total_points = max(0, int(msg.width) * int(msg.height))
        stride = max(1, math.ceil(total_points / self.pointcloud_max_points))
        encoded_points = []
        try:
            for index, point in enumerate(point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )):
                if index % stride:
                    continue
                x, y, z = float(point[0]), float(point[1]), float(point[2])
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                encoded_points.append(struct.pack('<fff', x, y, z))
                if len(encoded_points) >= self.pointcloud_max_points:
                    break
        except Exception as exc:
            self.get_logger().warn(
                f'Unable to encode frontend point cloud: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        raw = b''.join(encoded_points)
        timestamp_ms = int(time.time() * 1000)
        payload = {
            'type': 'pointcloud_frame',
            'frame': {
                'timestamp_ms': timestamp_ms,
                'data': {
                    'stream_id': device_id + '_mid360',
                    'vehicle_id': device_id,
                    'frame_id': str(msg.header.frame_id),
                    'timestamp_ms': timestamp_ms,
                    'encoding': 'xyz_f32_le_base64',
                    'point_count': len(encoded_points),
                    'point_stride_bytes': 12,
                    'data_base64': base64.b64encode(raw).decode('ascii'),
                },
            },
        }
        self._broadcast(json.dumps(payload, separators=(',', ':')))

    def _publish_visual_detection_frame(self, msg):
        timestamp_ms = int(time.time() * 1000)
        detections = []
        for item in msg.detections:
            detections.append({
                'id': str(item.detection_id),
                'center_x': float(item.center_x),
                'center_y': float(item.center_y),
                'size_x': float(item.size_x),
                'size_y': float(item.size_y),
                'class_name': str(item.class_name),
                'class_confidence': float(item.class_confidence),
                'affiliation': int(item.affiliation),
                'affiliation_confidence': float(item.affiliation_confidence),
            })
        payload = {
            'type': 'visual_detection_frame',
            'camera_id': 'usv_01',
            'frame_id': str(msg.header.frame_id),
            'timestamp_ms': timestamp_ms,
            'detections': detections,
        }
        self._broadcast(json.dumps(payload, separators=(',', ':')))

    def _on_pose(self, msg):
        updates = {}
        for pose in msg.pose:
            short_name = self.entity_names.get(pose.name)
            if short_name is None:
                continue

            updates[short_name] = {
                'position': [
                    self._to_logical_distance(pose.position.x),
                    self._to_logical_distance(pose.position.y),
                    self._to_logical_distance(pose.position.z),
                ],
                'orientation': [
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ],
            }

        if updates:
            with self.lock:
                self.latest.update(updates)
                self.last_gazebo_update = time.monotonic()
                if self.usv_names[0] in updates:
                    self.last_boat_update = self.last_gazebo_update

    def _on_capture_state(self, msg):
        state = {
            'state': int(msg.state),
            'state_name': str(msg.state_name),
            'target_id': str(msg.target_id),
            'reason': str(msg.reason),
            'configured_uavs': int(msg.configured_uavs),
            'configured_usvs': int(msg.configured_usvs),
            'active_uavs': int(msg.active_uavs),
            'active_usvs': int(msg.active_usvs),
            'allocation_generation': int(msg.allocation_generation),
            'degraded': bool(msg.degraded),
        }
        with self.lock:
            self.capture_state = state
        status = self._capture_mission_status(state)
        self._observe_mission_state(
            'GB_SFLA_CS',
            status,
            state['state_name'],
            state['target_id'] or self.target_entity_name,
            state['reason'],
            {
                'activeUavs': state['active_uavs'],
                'activeUsvs': state['active_usvs'],
                'degraded': state['degraded'],
                'allocationGeneration': state['allocation_generation'],
            },
        )

    def _capture_mission_status(self, state):
        phase = str(state.get('state_name', '')).strip().upper()
        reason = str(state.get('reason', '')).strip().lower()
        if phase == 'SUCCESS':
            self.capture_observed_started = True
            return 'COMPLETED'
        if phase == 'FAILED':
            self.capture_observed_started = True
            return 'FAILED'
        if 'paused' in reason:
            self.capture_observed_started = True
            return 'PAUSED'
        if 'waiting for operator start' in reason:
            return 'CANCELLED' if self.capture_observed_started else 'IDLE'
        if (
            'operator approved' in reason
            or 'waiting for configured fleet' in reason
            or 'insufficient active vehicles' in reason
            or phase in ('APPROACHING', 'ENCIRCLING', 'HOLDING')
        ):
            self.capture_observed_started = True
            return 'RUNNING'
        return 'IDLE'

    def _on_capture_roles(self, msg):
        assignments = []
        for assignment in msg.assignments:
            assignments.append({
                'vehicle_id': str(assignment.vehicle_id),
                'vehicle_type': int(assignment.vehicle_type),
                'role_type': int(assignment.role_type),
                'role_name': str(assignment.role_name),
                'target_pose': self._ros_pose(assignment.target_pose),
                'assignment_cost': float(assignment.assignment_cost),
                'active': bool(assignment.active),
                'status': str(assignment.status),
            })
        roles = {
            'target_id': str(msg.target_id),
            'capture_center': [
                self._to_logical_distance(msg.capture_center.x),
                self._to_logical_distance(msg.capture_center.y),
                self._to_logical_distance(msg.capture_center.z),
            ],
            'capture_radius': self._to_logical_distance(msg.capture_radius),
            'generation': int(msg.generation),
            'assignments': assignments,
        }
        with self.lock:
            self.capture_roles = roles

    def _on_vehicle_state(self, msg):
        state = {
            'vehicle_type': int(msg.vehicle_type),
            'online': bool(msg.online),
            'armed': bool(msg.armed),
            'mode': str(msg.mode),
            'pose': self._ros_pose(msg.pose),
            'twist': {
                'linear': [
                    self._to_logical_distance(msg.twist.linear.x),
                    self._to_logical_distance(msg.twist.linear.y),
                    self._to_logical_distance(msg.twist.linear.z),
                ],
                'angular': [
                    float(msg.twist.angular.x),
                    float(msg.twist.angular.y),
                    float(msg.twist.angular.z),
                ],
            },
            'battery_percent': float(msg.battery_percent),
            'active_command_id': str(msg.active_command_id),
            'status_text': str(msg.status_text),
        }
        with self.lock:
            vehicle_id = str(msg.vehicle_id)
            self.vehicle_states[vehicle_id] = state
            if vehicle_id not in self.home_poses:
                self.home_poses[vehicle_id] = tuple(state['pose']['position'])

    def _on_command_ack(self, msg):
        vehicle_id = str(msg.vehicle_id)
        command_id = str(msg.command_id)
        escort_takeoff_vehicle = self.escort_takeoff_commands.get(command_id)
        if escort_takeoff_vehicle is not None:
            takeoff_state = self.escort_takeoff_state.get(escort_takeoff_vehicle)
            if takeoff_state is not None and takeoff_state['command_id'] == command_id:
                takeoff_state['status'] = int(msg.status)
                takeoff_state['message'] = str(msg.message)
                takeoff_state['ack_time'] = time.monotonic()
                if int(msg.status) in (
                    CommandAck.STATUS_SUCCEEDED,
                    CommandAck.STATUS_REJECTED,
                    CommandAck.STATUS_FAILED,
                    CommandAck.STATUS_CANCELED,
                ):
                    takeoff_state['terminal'] = True
                    self.escort_takeoff_commands.pop(command_id, None)
        if int(msg.status) == CommandAck.STATUS_ACCEPTED:
            current = self.active_control_until.get(vehicle_id)
            if current is not None:
                self.active_control_until[vehicle_id] = min(
                    current, time.monotonic() + 1.0
                )
        elif int(msg.status) in (
            CommandAck.STATUS_SUCCEEDED,
            CommandAck.STATUS_REJECTED,
            CommandAck.STATUS_FAILED,
            CommandAck.STATUS_CANCELED,
        ):
            self.active_control_until.pop(vehicle_id, None)
        # Fleet mission controllers also use /fleet/command_ack. Only forward
        # acknowledgements for commands that originated from the platform;
        # otherwise the backend would try to resolve unrelated ROS command IDs.
        if command_id not in self.platform_command_ids:
            return
        if int(msg.status) in (
            CommandAck.STATUS_SUCCEEDED,
            CommandAck.STATUS_REJECTED,
            CommandAck.STATUS_FAILED,
            CommandAck.STATUS_CANCELED,
        ):
            self.platform_command_ids.discard(command_id)
        payload = {
            'type': 'command_ack',
            'commandKey': command_id,
            'deviceCode': vehicle_id,
            'status': int(msg.status),
            'progress': float(msg.progress),
            'message': str(msg.message),
            'timestamp_ms': int(time.time() * 1000),
        }
        self._broadcast(json.dumps(payload, separators=(',', ':')))

    def _to_logical_distance(self, value):
        return float(value) / self.simulation_coordinate_scale

    def _to_simulation_distance(self, value):
        return float(value) * self.simulation_coordinate_scale

    def _ros_pose(self, pose):
        return {
            'position': [
                self._to_logical_distance(pose.position.x),
                self._to_logical_distance(pose.position.y),
                self._to_logical_distance(pose.position.z),
            ],
            'orientation': [
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ],
        }

    def _serve(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            self.server_socket = server
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.ws_host, self.ws_port))
            server.listen(8)
            server.settimeout(0.5)

            while self.running:
                try:
                    client, address = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                client.settimeout(2.0)
                if self._handshake(client):
                    client.settimeout(None)
                    with self.client_lock:
                        self.ws_clients.append(client)
                        self.client_send_locks[client] = threading.Lock()
                    self.get_logger().info(f'Unity WebSocket client connected: {address[0]}:{address[1]}')
                    threading.Thread(
                        target=self._receive_client,
                        args=(client, address),
                        name=f'unity-ws-{address[0]}:{address[1]}',
                        daemon=True,
                    ).start()
                else:
                    client.close()

    def _handshake(self, client):
        try:
            request = b''
            while b'\r\n\r\n' not in request and len(request) < 8192:
                chunk = client.recv(1024)
                if not chunk:
                    return False
                request += chunk

            headers = request.decode('utf-8', errors='ignore').split('\r\n')
            key = None
            for header in headers:
                if header.lower().startswith('sec-websocket-key:'):
                    key = header.split(':', 1)[1].strip()
                    break
            if not key:
                return False

            accept = base64.b64encode(
                hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()
            ).decode()
            response = (
                'HTTP/1.1 101 Switching Protocols\r\n'
                'Upgrade: websocket\r\n'
                'Connection: Upgrade\r\n'
                f'Sec-WebSocket-Accept: {accept}\r\n'
                '\r\n'
            )
            client.sendall(response.encode('ascii'))
            return True
        except OSError:
            return False

    def _receive_client(self, client, address):
        try:
            while self.running:
                opcode, payload = self._read_frame(client)
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    self._send_frame(client, payload, opcode=0xA)
                    continue
                if opcode != 0x1:
                    continue
                self._queue_command(payload.decode('utf-8'))
        except (ConnectionError, OSError, UnicodeDecodeError, ValueError):
            pass
        finally:
            self._remove_client(client)
            self.get_logger().info(
                f'Unity WebSocket client disconnected: {address[0]}:{address[1]}'
            )

    def _queue_command(self, text):
        try:
            command = json.loads(text)
            if not isinstance(command, dict):
                raise ValueError('command must be a JSON object')
            self.command_queue.put_nowait(command)
        except (json.JSONDecodeError, ValueError) as exc:
            self._set_control_status('error', f'Invalid Unity command: {exc}')
        except queue.Full:
            self._set_control_status('error', 'Unity command queue is full')

    @staticmethod
    def _recv_exact(client, count):
        chunks = []
        remaining = count
        while remaining:
            chunk = client.recv(remaining)
            if not chunk:
                raise ConnectionError('WebSocket closed')
            chunks.append(chunk)
            remaining -= len(chunk)
        return b''.join(chunks)

    @classmethod
    def _read_frame(cls, client):
        header = cls._recv_exact(client, 2)
        opcode = header[0] & 0x0F
        masked = (header[1] & 0x80) != 0
        length = header[1] & 0x7F
        if length == 126:
            length = struct.unpack('!H', cls._recv_exact(client, 2))[0]
        elif length == 127:
            length = struct.unpack('!Q', cls._recv_exact(client, 8))[0]
        if length > 1024 * 1024:
            raise ValueError('WebSocket command exceeds 1 MiB')
        if not masked:
            raise ValueError('Client WebSocket frames must be masked')

        mask = cls._recv_exact(client, 4)
        payload = bytearray(cls._recv_exact(client, length))
        for index in range(length):
            payload[index] ^= mask[index % 4]
        return opcode, bytes(payload)

    def _send_frame(self, client, payload, opcode=0x1):
        if isinstance(payload, str):
            payload = payload.encode('utf-8')
        length = len(payload)
        first = 0x80 | opcode
        if length < 126:
            header = bytes([first, length])
        elif length <= 0xFFFF:
            header = bytes([first, 126]) + struct.pack('!H', length)
        else:
            header = bytes([first, 127]) + struct.pack('!Q', length)

        with self.client_lock:
            send_lock = self.client_send_locks.get(client)
        if send_lock is None:
            raise ConnectionError('WebSocket client is not registered')
        with send_lock:
            frame = header + payload
            sent = client.send(frame, socket.MSG_DONTWAIT)
            if sent != len(frame):
                raise ConnectionError('WebSocket client is not consuming frames')

    def _remove_client(self, client):
        with self.client_lock:
            was_registered = client in self.ws_clients
            if client in self.ws_clients:
                self.ws_clients.remove(client)
            self.client_send_locks.pop(client, None)
            no_clients_remain = not self.ws_clients
        try:
            client.close()
        except OSError:
            pass
        if was_registered and no_clients_remain and self.active_path:
            try:
                self.command_queue.put_nowait({'type': 'boat_stop'})
            except queue.Full:
                self._publish_boat_cmd(0.0, 0.0)

    def _control_tick(self):
        self._drain_commands()
        now = time.monotonic()
        if self.scene_reset_pose_deadline > 0.0 or self.scene_home_lock_active:
            final_pass = (
                self.scene_reset_pose_deadline > 0.0
                and now >= self.scene_reset_pose_deadline
            )
            if final_pass or now >= self.scene_reset_next_pose_time:
                failures = self._set_scene_poses_fast(SCENE_HOME_POSES)
                if not failures:
                    self.scene_reset_failures = set()
                elif self.scene_reset_failures:
                    self.scene_reset_failures = set(failures)
                zero = Twist()
                for publisher in self.scene_cmd_pubs.values():
                    publisher.publish(zero)
                self.scene_reset_next_pose_time = now + 0.5
            if final_pass:
                self.scene_reset_pose_deadline = 0.0
                self.scene_reset_next_pose_time = 0.0
                self.scene_home_lock_active = False
                requests = self.pending_scene_reset_requests
                self.pending_scene_reset_requests = []
                # Reset is terminal: cancel retained controller targets, then
                # freeze physics after the final home-pose write.  This avoids
                # PX4/Nav2 pulling the models away while the bridge repeatedly
                # teleports them back.  CAPTURE/ESCORT resumes physics in
                # _on_operator_action().
                self._publish_operator_action('CANCEL_CAPTURE')
                self._publish_operator_action('CANCEL_ESCORT')
                paused = self._set_world_paused(True)
                success = not self.scene_reset_failures and paused
                status = (
                    'Gazebo/ROS scene restored and held at local Unity initial poses'
                    if success else
                    (
                        'Scene reset failed for: '
                        + ', '.join(sorted(self.scene_reset_failures))
                        if self.scene_reset_failures else
                        'Scene restored, but Gazebo physics could not be paused'
                    )
                )
                for request in requests:
                    request_id = request.get('request_id', '')
                    command_key = request.get('command_key', '')
                    if request_id:
                        self._broadcast(json.dumps({
                            'type': 'scene_reset_ack',
                            'request_id': request_id,
                            'success': success,
                            'status': status,
                        }, separators=(',', ':')))
                    if command_key:
                        self._send_platform_ack(
                            command_key, 1 if success else 4, status, ''
                        )
                self._set_control_status(
                    'scene_reset' if success else 'error', status
                )
        self._update_escort_mission()
        self._expire_pending_mission_commands()
        self._renew_platform_leases()
        self._publish_pending_fleet_commands()
        self._publish_pending_platform_acks()

        if self.control_mode == 'observe':
            return
        if self.control_mode == 'nav2':
            self._update_nav2_control()
        else:
            self._update_direct_control()

    def _drain_commands(self):
        for _ in range(16):
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return

            command_type = command.get('type')
            # Camera selection is always allowed (including observe mode).
            if command_type == 'select_camera':
                self._accept_select_camera(command)
                continue
            if command_type == 'sensor_subscription':
                self._accept_sensor_subscription(command)
                continue
            if command_type == 'reset_scene':
                self._reset_scene(command)
                continue
            if command_type == 'command':
                self._accept_platform_command(command)
                continue

            if self.control_mode == 'observe':
                self._set_control_status(
                    'observe',
                    'ROS fleet/base station is authoritative; Unity commands ignored',
                )
                continue

            if command_type == 'boat_path':
                self._accept_boat_path(command)
            elif command_type == 'boat_stop':
                self._stop_boat('Stopped from Unity', clear_path=True)
            else:
                self._set_control_status(
                    'error',
                    f'Unknown command type: {command_type!r}',
                )

    @staticmethod
    def _normalize_vehicle_id(value):
        return str(value or '').strip().lower().replace('-', '_')

    def _accept_platform_command(self, frame):
        command_key = str(frame.get('commandKey', '')).strip()
        command_type = str(frame.get('commandType', '')).strip().upper()
        vehicle_id = self._normalize_vehicle_id(frame.get('deviceCode'))
        payload = frame.get('payload')
        if not isinstance(payload, dict):
            payload = {}
        if not command_key:
            self._send_platform_ack('', 4, 'commandKey is required', vehicle_id)
            return

        mission_command_types = {
            'START_MISSION', 'RESUME_MISSION', 'PAUSE_MISSION',
            'STOP_MISSION', 'CANCEL_MISSION', 'FAIL_MISSION',
            'COMPLETE_MISSION',
        }
        if command_type in mission_command_types:
            if self.operator_action_pub is None:
                self._send_platform_ack(
                    command_key, 4, 'ROS fleet interfaces are unavailable', vehicle_id
                )
                return
            algorithm_code = str(
                payload.get('algorithmCode', payload.get('algorithm_code', 'GB_SFLA_CS'))
            ).strip().upper()
            if algorithm_code not in ('GB_SFLA_CS', 'ESCORT_GUARD'):
                self._send_platform_ack(
                    command_key,
                    4,
                    f'Unsupported ROS mission algorithm: {algorithm_code or "missing"}',
                    vehicle_id,
                )
                return
            default_target = (
                self.friendly_ship_name
                if algorithm_code == 'ESCORT_GUARD'
                else self.target_entity_name
            )
            target_id = str(
                payload.get('target_id', payload.get('targetId', default_target))
            ).strip() or default_target
            if algorithm_code == 'ESCORT_GUARD':
                mission_actions = {
                    'START_MISSION': f'ESCORT:{target_id}',
                    'RESUME_MISSION': f'ESCORT:{target_id}',
                    'PAUSE_MISSION': 'HOLD_ESCORT',
                    'STOP_MISSION': 'CANCEL_ESCORT',
                    'CANCEL_MISSION': 'CANCEL_ESCORT',
                    'FAIL_MISSION': 'CANCEL_ESCORT',
                    'COMPLETE_MISSION': 'COMPLETE_ESCORT',
                }
                if command_type in ('START_MISSION', 'RESUME_MISSION'):
                    with self.lock:
                        protected_pose_available = target_id in self.latest
                    if not protected_pose_available:
                        self._send_platform_ack(
                            command_key,
                            4,
                            f'Protected vessel pose is unavailable: {target_id}',
                            vehicle_id,
                        )
                        return
            else:
                mission_actions = {
                    'START_MISSION': f'CAPTURE:{target_id}',
                    'RESUME_MISSION': f'CAPTURE:{target_id}',
                    'PAUSE_MISSION': 'HOLD_ALL',
                    'STOP_MISSION': 'CANCEL_CAPTURE',
                    'CANCEL_MISSION': 'CANCEL_CAPTURE',
                    'FAIL_MISSION': 'CANCEL_CAPTURE',
                    'COMPLETE_MISSION': 'CANCEL_CAPTURE',
                }
            msg = String()
            msg.data = mission_actions[command_type]
            self.operator_action_pub.publish(msg)
            expected_statuses = {
                'START_MISSION': ('RUNNING',),
                'RESUME_MISSION': ('RUNNING',),
                'PAUSE_MISSION': ('PAUSED',),
                'STOP_MISSION': ('CANCELLED',),
                'CANCEL_MISSION': ('CANCELLED',),
                'FAIL_MISSION': ('FAILED', 'CANCELLED'),
                'COMPLETE_MISSION': (
                    ('COMPLETED',)
                    if algorithm_code == 'ESCORT_GUARD'
                    else ('COMPLETED', 'CANCELLED')
                ),
            }[command_type]
            self.pending_mission_commands[command_key] = {
                'algorithm_code': algorithm_code,
                'command_type': command_type,
                'action': msg.data,
                'target_id': target_id,
                'expected_statuses': expected_statuses,
                'deadline': time.monotonic() + self.mission_ack_timeout,
            }
            self._set_control_status(
                'mission_pending',
                f'Waiting for authoritative ROS state after {msg.data}',
            )
            return

        if command_type == 'RESET_SCENE':
            self._reset_scene({
                'request_id': '',
                'command_key': command_key,
            })
            return

        ui_commands = {
            'SELECT_DEVICE', 'FOCUS_DEVICE', 'SWITCH_CAMERA', 'TOGGLE_TRAJECTORY'
        }
        if command_type in ui_commands:
            camera_id = self._normalize_vehicle_id(
                payload.get('camera_id', payload.get('cameraId', vehicle_id))
            )
            if camera_id in self.camera_topics:
                self._accept_select_camera({'camera_id': camera_id})
            self._send_platform_ack(
                command_key, 1, 'Gateway UI selection synchronized', vehicle_id
            )
            return

        if command_type in ('START', 'STOP'):
            self._send_platform_ack(
                command_key,
                1,
                'Runtime lifecycle command acknowledged by ROS gateway',
                vehicle_id,
            )
            return

        if self.fleet_command_pub is None or self.control_lease_pub is None:
            self._send_platform_ack(
                command_key, 4, 'ROS fleet interfaces are unavailable', vehicle_id
            )
            return
        if not vehicle_id:
            self._send_platform_ack(
                command_key, 4, 'deviceCode is required for vehicle commands', ''
            )
            return
        known_vehicles = set(self.usv_names) | set(self.uav_names)
        if vehicle_id not in known_vehicles:
            self._send_platform_ack(
                command_key, 4, f'Unknown vehicle: {vehicle_id}', vehicle_id
            )
            return

        command = FleetCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'map'
        command.command_id = command_key
        command.vehicle_id = vehicle_id
        command.lease_id = self.gateway_lease_id
        command.priority = self.gateway_priority
        command.expires_at = (
            self.get_clock().now() + Duration(seconds=self.command_lifetime)
        ).to_msg()
        command.target_pose.orientation.w = 1.0

        mapping = {
            'UAV_TAKEOFF': FleetCommand.COMMAND_TAKEOFF,
            'TAKEOFF': FleetCommand.COMMAND_TAKEOFF,
            'UAV_HOVER': FleetCommand.COMMAND_HOLD,
            'USV_HOLD': FleetCommand.COMMAND_HOLD,
            'USV_STOP': FleetCommand.COMMAND_HOLD,
            'USV_EMERGENCY_STOP': FleetCommand.COMMAND_EMERGENCY_STOP,
            'UAV_EMERGENCY_LAND': FleetCommand.COMMAND_EMERGENCY_STOP,
        }
        target = self._target_from_payload(payload)
        if command_type in ('UAV_RESUME', 'USV_DEPART', 'USV_RESUME'):
            command.command_type = FleetCommand.COMMAND_NAVIGATE
            target = target or self.last_navigation_targets.get(vehicle_id)
            if target is None:
                target = self._default_departure_target(vehicle_id)
        elif command_type in ('UAV_RETURN', 'USV_RETURN', 'UAV_LAND', 'LAND'):
            command.command_type = FleetCommand.COMMAND_NAVIGATE
            target = target or self.home_poses.get(vehicle_id)
            if target is None:
                self._send_platform_ack(
                    command_key,
                    4,
                    f'No home pose received for {vehicle_id}',
                    vehicle_id,
                )
                return
            if command_type in ('UAV_LAND', 'LAND'):
                target = (target[0], target[1], max(1.0, target[2]))
        elif command_type in mapping:
            command.command_type = mapping[command_type]
        else:
            self._send_platform_ack(
                command_key,
                4,
                f'Unsupported platform command: {command_type}',
                vehicle_id,
            )
            return

        if command.command_type == FleetCommand.COMMAND_TAKEOFF:
            altitude = payload.get('altitude', self.takeoff_altitude)
            try:
                command.parameters = [self._to_simulation_distance(
                    max(1.0, float(altitude))
                )]
            except (TypeError, ValueError):
                command.parameters = [self._to_simulation_distance(
                    self.takeoff_altitude
                )]
        if command.command_type == FleetCommand.COMMAND_NAVIGATE:
            if target is None:
                self._send_platform_ack(
                    command_key, 4, 'Navigation target is required', vehicle_id
                )
                return
            command.target_pose.position.x = self._to_simulation_distance(
                target[0]
            )
            command.target_pose.position.y = self._to_simulation_distance(
                target[1]
            )
            command.target_pose.position.z = self._to_simulation_distance(
                target[2]
            )
            self.last_navigation_targets[vehicle_id] = tuple(target)

        # Lease and command use different ROS topics. Give DDS a short window to
        # deliver the lease before publishing the command that references it.
        self.active_control_until[vehicle_id] = (
            time.monotonic() + self.lease_duration
        )
        self.platform_command_ids.add(command_key)
        self._publish_platform_lease(vehicle_id)
        self.pending_fleet_commands.append((time.monotonic() + 0.2, command))
        self._set_control_status(
            'platform_control',
            f'Queued {command_type} for {vehicle_id}',
        )

    def _on_operator_action(self, msg):
        action = str(msg.data).strip()
        upper = action.upper()
        if upper.startswith(('CAPTURE:', 'ESCORT:')):
            # Gazebo's GUI reset leaves the world paused.  Mission state can
            # still change through ROS while every model and sensor remains
            # frozen, which looks like a broken ROS/Unity command path.  A
            # mission start/resume is authoritative, so always resume physics.
            self._set_world_paused(False)
            # A new authoritative mission supersedes any in-progress reset
            # enforcement and its delayed final hold.
            self.scene_reset_pose_deadline = 0.0
            self.scene_reset_next_pose_time = 0.0
            self.scene_home_lock_active = False
            self.pending_scene_reset_requests = []
            self.scene_reset_failures = set()
            self._set_target_motion(True)
        elif upper in (
            'HOLD_ALL', 'HOLD_ESCORT', 'CANCEL_CAPTURE', 'CANCEL_ESCORT',
            'COMPLETE_ESCORT',
        ):
            self._set_target_motion(False)
        if upper.startswith('ESCORT:'):
            protected_id = action.split(':', 1)[1].strip()
            requested_id = protected_id or self.friendly_ship_name
            continuing = (
                self.escort_active
                and requested_id == self.escort_protected_id
            )
            self.escort_protected_id = requested_id
            self.escort_active = True
            self.escort_paused = False
            self.escort_last_command_time = 0.0
            self.escort_takeoff_commands.clear()
            self.escort_takeoff_state.clear()
            if not continuing:
                self.escort_planner.reset()
            self._set_escort_state(
                'FORMING',
                'ROS escort mission resumed' if continuing
                else 'ROS escort mission started with supplied 3-D guard algorithm',
            )
            self.get_logger().warning(
                f'ROS escort mission started for {self.escort_protected_id}'
            )
        elif upper in ('HOLD_ESCORT', 'HOLD_ALL') and self.escort_active:
            self.escort_paused = True
            self._queue_escort_hold()
            self._set_escort_state('PAUSED', 'escort paused by operator')
        elif upper in ('CANCEL_ESCORT', 'COMPLETE_ESCORT'):
            if self.escort_active:
                self._queue_escort_hold()
            completed = upper == 'COMPLETE_ESCORT'
            self.escort_active = False
            self.escort_paused = False
            self.escort_takeoff_commands.clear()
            self.escort_takeoff_state.clear()
            self._set_escort_state(
                'COMPLETED' if completed else 'CANCELLED',
                'escort completed by operator' if completed
                else 'escort cancelled by operator',
            )
            self.escort_planner.reset()

    def _publish_operator_action(self, action):
        if self.operator_action_pub is None:
            return
        message = String()
        message.data = str(action)
        self.operator_action_pub.publish(message)

    def _set_target_motion(self, enabled):
        message = GzBoolean()
        message.data = bool(enabled)
        self.target_motion_pub.publish(message)

    def _set_world_paused(self, paused):
        request = WorldControl()
        request.pause = bool(paused)
        executed, response = self.gz_node.request(
            '/world/%s/control' % self.gazebo_world,
            request,
            WorldControl,
            GzBoolean,
            1000,
        )
        success = bool(executed and response.data)
        if not success:
            self.get_logger().warning(
                'Gazebo world control did not confirm pause=%s' % bool(paused)
            )
        return success

    def _set_scene_pose(self, name, values, blocking=False):
        x, y, z, yaw = values
        request = GzPose()
        request.name = name
        request.position.x = float(x)
        request.position.y = float(y)
        request.position.z = float(z)
        request.orientation.z = math.sin(0.5 * yaw)
        request.orientation.w = math.cos(0.5 * yaw)
        service = '/world/%s/set_pose%s' % (
            self.gazebo_world,
            '/blocking' if blocking else '',
        )
        executed, response = self.gz_node.request(
            service,
            request,
            GzPose,
            GzBoolean,
            1500 if blocking else 100,
        )
        return bool(executed and response.data)

    def _set_scene_poses_batch(self, poses, blocking=False):
        """Set a complete scene snapshot with one Gazebo service request.

        Calling ``set_pose`` once per entity makes a reset fragile on a busy
        classroom machine: eight independent 500 ms discovery / response
        windows can all expire even though Gazebo is healthy.  The world
        service provides an atomic Pose_V endpoint specifically for this use.
        Use its non-blocking variant because this method is repeated during
        the four-second stabilization window.  The ``/blocking`` service can
        consume its full five-second timeout on every pass and starve both the
        WebSocket acknowledgement and the ROS control timer.
        """
        request = Pose_V()
        for name, values in poses.items():
            x, y, z, yaw = values
            pose = request.pose.add()
            pose.name = name
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.z = math.sin(0.5 * yaw)
            pose.orientation.w = math.cos(0.5 * yaw)
        service = '/world/%s/set_pose_vector%s' % (
            self.gazebo_world,
            '/blocking' if blocking else '',
        )
        executed, response = self.gz_node.request(
            service,
            request,
            Pose_V,
            GzBoolean,
            5000 if blocking else 100,
        )
        return bool(executed and response.data)

    def _set_scene_poses_fast(self, poses):
        """Apply a scene snapshot without ever starving the ROS/WS timer."""
        if self._set_scene_poses_batch(poses, blocking=False):
            return set()
        return {
            name
            for name, values in poses.items()
            if not self._set_scene_pose(name, values, blocking=False)
        }

    def _reset_scene(self, command):
        request_id = str(command.get('request_id', '')).strip()
        command_key = str(command.get('command_key', '')).strip()
        # "Pause mission" means hold the vehicles, not freeze Gazebo time.
        # Keep physics running so the reset poses and fresh telemetry reach
        # both ROS and Unity even after the GUI reset button was used.
        self._set_world_paused(False)
        self._set_target_motion(False)
        # A reset ends the current task. The home lock below keeps the scene
        # stationary until a new CAPTURE/ESCORT command explicitly starts it.
        self._publish_operator_action('CANCEL_CAPTURE')
        self._publish_operator_action('CANCEL_ESCORT')

        zero = Twist()
        for publisher in self.scene_cmd_pubs.values():
            publisher.publish(zero)

        # Never block this control callback. Busy entities are retried briefly
        # before the final pose write and physics freeze.
        failures = list(self._set_scene_poses_fast(SCENE_HOME_POSES))
        self.home_poses.update({
            name: values[:3]
            for name, values in SCENE_HOME_POSES.items()
            if name in self.usv_names or name in self.uav_names
        })
        # Keep enforcing the home poses only during the short reset window.
        # The final pass pauses Gazebo, so no periodic teleport lock is needed.
        now = time.monotonic()
        self.scene_home_lock_active = True
        self.scene_reset_pose_deadline = now + 1.0
        self.scene_reset_next_pose_time = now + 0.1
        self.scene_reset_failures = set(failures)
        # ACK only after the final pass has applied the home snapshot and
        # Gazebo has confirmed that physics is paused.
        self.pending_scene_reset_requests.append({
            'request_id': request_id,
            'command_key': command_key,
        })
        self._set_control_status(
            'scene_reset', 'Stabilizing local Unity initial poses'
        )

    def _set_escort_state(self, phase, reason, details=None):
        state_details = dict(details) if isinstance(details, dict) else {}
        with self.lock:
            self.escort_state = {
                'active': bool(self.escort_active),
                'paused': bool(self.escort_paused),
                'phase': str(phase),
                'protected_id': str(self.escort_protected_id),
                'reason': str(reason),
                'command_sequence': int(self.escort_command_sequence),
                'algorithm': 'ESCORT_GUARD_3D_SINGLE_TARGET',
                **state_details,
            }
        normalized_phase = str(phase).strip().upper()
        if normalized_phase == 'PAUSED':
            status = 'PAUSED'
        elif normalized_phase == 'COMPLETED':
            status = 'COMPLETED'
        elif normalized_phase == 'CANCELLED':
            status = 'CANCELLED'
        elif normalized_phase == 'FAILED':
            status = 'FAILED'
        elif self.escort_active:
            status = 'RUNNING'
        else:
            status = 'IDLE'
        observation_details = {
            'commandSequence': int(self.escort_command_sequence),
            **state_details,
        }
        self._observe_mission_state(
            'ESCORT_GUARD',
            status,
            normalized_phase,
            self.escort_protected_id,
            reason,
            observation_details,
        )

    def _observe_mission_state(
        self, algorithm_code, status, phase, target_id, reason, details=None
    ):
        confirmations = self._resolve_pending_mission_commands(
            algorithm_code, status, reason
        )
        signature = (str(status), str(phase), str(target_id), str(reason))
        if (
            self.last_mission_state_signatures.get(algorithm_code) == signature
            and not confirmations
        ):
            return
        self.last_mission_state_signatures[algorithm_code] = signature
        payload = {
            'type': 'mission_state',
            'source': 'ROS',
            'algorithmCode': str(algorithm_code),
            'status': str(status),
            'phase': str(phase),
            'targetId': str(target_id),
            'reason': str(reason),
            'timestamp_ms': int(time.time() * 1000),
        }
        if isinstance(details, dict):
            payload['details'] = details
        if confirmations:
            payload['confirmedCommands'] = confirmations
        self._broadcast(json.dumps(payload, separators=(',', ':')))

    def _resolve_pending_mission_commands(self, algorithm_code, status, reason):
        matched = []
        for command_key, pending in self.pending_mission_commands.items():
            if pending['algorithm_code'] != algorithm_code:
                continue
            if status in pending['expected_statuses']:
                matched.append((command_key, True))
            elif status in ('COMPLETED', 'FAILED', 'CANCELLED'):
                matched.append((command_key, False))
        confirmations = []
        for command_key, succeeded in matched:
            pending = self.pending_mission_commands.pop(command_key)
            confirmations.append({
                'commandKey': command_key,
                'commandType': pending['command_type'],
            })
            if succeeded:
                self._send_platform_ack(
                    command_key,
                    3,
                    'ROS mission state confirmed %s after %s: %s'
                    % (status, pending['action'], reason or 'state observed'),
                    '',
                )
                self._set_control_status(
                    'mission_confirmed',
                    f"{pending['command_type']} confirmed by ROS state {status}",
                )
            else:
                self._send_platform_ack(
                    command_key,
                    5,
                    'ROS mission reached unexpected terminal state %s after %s: %s'
                    % (status, pending['action'], reason or 'state observed'),
                    '',
                )
                self._set_control_status(
                    'mission_failed',
                    f"{pending['command_type']} rejected by ROS state {status}",
                )
        return confirmations

    def _expire_pending_mission_commands(self):
        now = time.monotonic()
        expired = [
            command_key
            for command_key, pending in self.pending_mission_commands.items()
            if now >= pending['deadline']
        ]
        for command_key in expired:
            pending = self.pending_mission_commands.pop(command_key)
            self._send_platform_ack(
                command_key,
                5,
                'Timed out waiting for authoritative ROS mission state after '
                + pending['action'],
                '',
            )

    def _update_escort_mission(self):
        if not self.escort_active or self.escort_paused:
            return
        now = time.monotonic()
        if now - self.escort_last_command_time < self.escort_command_period:
            return
        with self.lock:
            protected_pose = self.latest.get(self.escort_protected_id)
            threat_pose = self.latest.get(self.target_entity_name)
            vehicle_poses = {
                vehicle_id: self.latest.get(vehicle_id)
                for vehicle_id in self.usv_names + self.uav_names
            }
            vehicle_states = {
                vehicle_id: dict(self.vehicle_states.get(vehicle_id, {}))
                for vehicle_id in self.usv_names + self.uav_names
            }
        if protected_pose is None:
            self._set_escort_state(
                'WAITING_FOR_TARGET',
                f'waiting for {self.escort_protected_id} Gazebo pose',
            )
            return

        center = protected_pose['position']
        yaw = yaw_from_quaternion(protected_pose['orientation'])
        plan = self.escort_planner.plan(
            protected_position=center,
            threat_position=(
                threat_pose['position'] if threat_pose is not None else None
            ),
            vehicle_positions={
                vehicle_id: pose['position']
                for vehicle_id, pose in vehicle_poses.items()
                if pose is not None
            },
            protected_yaw=yaw,
            uav_altitude=self.escort_uav_altitude,
        )
        offline_vehicles = []
        waiting_for_takeoff = []
        dispatched = 0
        for vehicle_id in self.usv_names:
            state = vehicle_states.get(vehicle_id, {})
            target = plan.targets.get(vehicle_id)
            if not state.get('online', False):
                offline_vehicles.append(vehicle_id)
            elif target is not None:
                self._queue_escort_navigation(vehicle_id, target)
                dispatched += 1
        for vehicle_id in self.uav_names:
            state = vehicle_states.get(vehicle_id, {})
            target = plan.targets.get(vehicle_id)
            if not state.get('online', False):
                offline_vehicles.append(vehicle_id)
            elif not state.get('armed', False):
                waiting_for_takeoff.append(vehicle_id)
                self._queue_escort_takeoff_if_due(vehicle_id, now)
            elif vehicle_poses.get(vehicle_id) is not None:
                self.escort_takeoff_state.pop(vehicle_id, None)
                self._queue_escort_navigation(vehicle_id, target)
                dispatched += 1

        self.escort_last_command_time = now
        if waiting_for_takeoff:
            failures = []
            for vehicle_id in waiting_for_takeoff:
                takeoff_state = self.escort_takeoff_state.get(vehicle_id, {})
                if takeoff_state.get('terminal') and takeoff_state.get('message'):
                    failures.append(f"{vehicle_id}: {takeoff_state['message']}")
            reason = 'waiting for UAV takeoff/arming: ' + ', '.join(waiting_for_takeoff)
            if failures:
                reason += '; last failure: ' + '; '.join(failures)
            self._set_escort_state('TAKING_OFF', reason, plan.details)
        elif offline_vehicles:
            self._set_escort_state(
                'WAITING_FOR_VEHICLES',
                'offline ROS vehicles: ' + ', '.join(offline_vehicles),
                plan.details,
            )
        elif dispatched:
            self._set_escort_state(
                plan.phase,
                plan.reason,
                plan.details,
            )
        else:
            self._set_escort_state(
                'WAITING_FOR_VEHICLES',
                'waiting for ROS vehicle states and Gazebo poses',
                plan.details,
            )

    @staticmethod
    def _escort_ring_targets(center, yaw, radius, count, altitude, phase):
        if count <= 0:
            return []
        return [
            (
                float(center[0]) + radius * math.cos(
                    yaw + phase + 2.0 * math.pi * index / count
                ),
                float(center[1]) + radius * math.sin(
                    yaw + phase + 2.0 * math.pi * index / count
                ),
                float(altitude),
                float(yaw),
            )
            for index in range(count)
        ]

    def _queue_escort_navigation(self, vehicle_id, target):
        self.escort_command_sequence += 1
        command = FleetCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'map'
        command.command_id = (
            f'escort-{vehicle_id}-{self.escort_command_sequence}'
        )
        command.vehicle_id = vehicle_id
        command.lease_id = self.gateway_lease_id
        command.command_type = FleetCommand.COMMAND_NAVIGATE
        command.priority = self.gateway_priority
        command.expires_at = (
            self.get_clock().now() + Duration(seconds=self.command_lifetime)
        ).to_msg()
        command.target_pose.position.x = self._to_simulation_distance(
            target[0]
        )
        command.target_pose.position.y = self._to_simulation_distance(
            target[1]
        )
        command.target_pose.position.z = self._to_simulation_distance(
            target[2]
        )
        command.target_pose.orientation.z = math.sin(float(target[3]) / 2.0)
        command.target_pose.orientation.w = math.cos(float(target[3]) / 2.0)
        self.active_control_until[vehicle_id] = (
            time.monotonic() + self.lease_duration
        )
        self._publish_platform_lease(vehicle_id)
        self.pending_fleet_commands.append((time.monotonic() + 0.2, command))

    def _queue_escort_takeoff_if_due(self, vehicle_id, now):
        previous = self.escort_takeoff_state.get(vehicle_id)
        if previous is not None:
            age = now - previous['sent_time']
            status = previous.get('status')
            retryable_failure = previous.get('terminal') and status in (
                CommandAck.STATUS_REJECTED,
                CommandAck.STATUS_FAILED,
                CommandAck.STATUS_CANCELED,
            )
            missing_result = status is None and age >= self.escort_takeoff_retry_period
            accepted_but_stalled = (
                status == CommandAck.STATUS_ACCEPTED
                and age >= self.escort_takeoff_retry_period
            )
            succeeded_but_not_armed = (
                status == CommandAck.STATUS_SUCCEEDED
                and age >= self.escort_takeoff_retry_period
            )
            if not (
                age >= self.escort_takeoff_retry_period
                and (
                    retryable_failure
                    or missing_result
                    or accepted_but_stalled
                    or succeeded_but_not_armed
                )
            ):
                return

        self.escort_command_sequence += 1
        command = FleetCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'map'
        command.command_id = (
            f'escort-takeoff-{vehicle_id}-{self.escort_command_sequence}'
        )
        command.vehicle_id = vehicle_id
        command.lease_id = self.gateway_lease_id
        command.command_type = FleetCommand.COMMAND_TAKEOFF
        command.priority = self.gateway_priority
        command.expires_at = (
            self.get_clock().now() + Duration(seconds=self.command_lifetime)
        ).to_msg()
        command.parameters = [self._to_simulation_distance(
            self.takeoff_altitude
        )]
        self.active_control_until[vehicle_id] = now + self.lease_duration
        self._publish_platform_lease(vehicle_id)
        self.pending_fleet_commands.append((now + 0.2, command))
        self.escort_takeoff_commands[command.command_id] = vehicle_id
        self.escort_takeoff_state[vehicle_id] = {
            'command_id': command.command_id,
            'sent_time': now,
            'status': None,
            'message': 'takeoff command queued',
            'terminal': False,
        }
        self.get_logger().warning(
            f'Escort queued TAKEOFF for {vehicle_id} before navigation'
        )

    def _queue_escort_hold(self):
        if self.fleet_command_pub is None or self.control_lease_pub is None:
            return
        for vehicle_id in self.usv_names + self.uav_names:
            self.escort_command_sequence += 1
            command = FleetCommand()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = 'map'
            command.command_id = (
                f'escort-hold-{vehicle_id}-{self.escort_command_sequence}'
            )
            command.vehicle_id = vehicle_id
            command.lease_id = self.gateway_lease_id
            command.command_type = FleetCommand.COMMAND_HOLD
            command.priority = self.gateway_priority
            command.expires_at = (
                self.get_clock().now() + Duration(seconds=self.command_lifetime)
            ).to_msg()
            command.target_pose.orientation.w = 1.0
            self.active_control_until[vehicle_id] = (
                time.monotonic() + self.lease_duration
            )
            self._publish_platform_lease(vehicle_id)
            self.pending_fleet_commands.append(
                (time.monotonic() + 0.2, command)
            )

    @staticmethod
    def _target_from_payload(payload):
        raw = payload.get('target')
        if not isinstance(raw, dict):
            raw = payload
        if not any(key in raw for key in ('x', 'y', 'z')):
            return None
        try:
            target = (
                float(raw.get('x', 0.0)),
                float(raw.get('y', 0.0)),
                float(raw.get('z', 0.0)),
            )
        except (TypeError, ValueError):
            return None
        return target if all(math.isfinite(value) for value in target) else None

    def _default_departure_target(self, vehicle_id):
        state = self.vehicle_states.get(vehicle_id)
        if state is None:
            return self.home_poses.get(vehicle_id)
        position = state['pose']['position']
        altitude = (
            max(self.takeoff_altitude, float(position[2]))
            if vehicle_id.startswith('uav_')
            else float(position[2])
        )
        return float(position[0]) + 12.0, float(position[1]), altitude

    def _publish_platform_lease(self, vehicle_id, force=False):
        if self.control_lease_pub is None:
            return
        now = time.monotonic()
        if (
            not force
            and now - self.last_platform_lease_publish.get(vehicle_id, 0.0) < 0.2
        ):
            return
        lease = ControlLease()
        lease.header.stamp = self.get_clock().now().to_msg()
        lease.vehicle_id = vehicle_id
        lease.lease_id = self.gateway_lease_id
        lease.owner_id = self.gateway_owner_id
        lease.priority = self.gateway_priority
        lease.valid_until = (
            self.get_clock().now() + Duration(seconds=self.lease_duration)
        ).to_msg()
        lease.revoked = False
        self.control_lease_pub.publish(lease)
        self.last_platform_lease_publish[vehicle_id] = now

    def _renew_platform_leases(self):
        now = time.monotonic()
        expired = []
        for vehicle_id, expires_at in self.active_control_until.items():
            if now >= expires_at:
                expired.append(vehicle_id)
            else:
                self._publish_platform_lease(vehicle_id)
        for vehicle_id in expired:
            self.active_control_until.pop(vehicle_id, None)
            self.last_platform_lease_publish.pop(vehicle_id, None)

    def _publish_pending_fleet_commands(self):
        if not self.pending_fleet_commands or self.fleet_command_pub is None:
            return
        now = time.monotonic()
        pending = []
        for publish_at, command in self.pending_fleet_commands:
            if now < publish_at:
                pending.append((publish_at, command))
                continue
            self._publish_platform_lease(command.vehicle_id, force=True)
            self.fleet_command_pub.publish(command)
            self.get_logger().info(
                'PLATFORM COMMAND %s -> %s type=%d'
                % (command.command_id, command.vehicle_id, command.command_type)
            )
        self.pending_fleet_commands = pending

    def _send_platform_ack(self, command_key, status, message, device_code=''):
        self.pending_platform_acks.append((
            time.monotonic() + 0.25,
            command_key,
            status,
            message,
            device_code,
        ))

    def _publish_pending_platform_acks(self):
        if not self.pending_platform_acks:
            return
        now = time.monotonic()
        pending = []
        for publish_at, command_key, status, message, device_code in (
            self.pending_platform_acks
        ):
            if now < publish_at:
                pending.append((
                    publish_at, command_key, status, message, device_code
                ))
                continue
            self._broadcast_platform_ack(
                command_key, status, message, device_code
            )
        self.pending_platform_acks = pending

    def _broadcast_platform_ack(
        self, command_key, status, message, device_code=''
    ):
        payload = {
            'type': 'command_ack',
            'commandKey': command_key,
            'deviceCode': device_code,
            'status': int(status),
            'progress': 1.0 if int(status) in (1, 3) else 0.0,
            'message': message,
            'timestamp_ms': int(time.time() * 1000),
        }
        self._broadcast(json.dumps(payload, separators=(',', ':')))

    def _accept_boat_path(self, command):
        raw_points = command.get('points')
        if not isinstance(raw_points, list):
            self._set_control_status('error', 'boat_path.points must be an array')
            return
        if len(raw_points) < 2 or len(raw_points) > self.max_path_points:
            self._set_control_status(
                'error',
                f'Path requires 2..{self.max_path_points} points',
            )
            return

        points = []
        try:
            for point in raw_points:
                x = float(point['x'])
                y = float(point['y'])
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError('coordinates must be finite')
                if abs(x) > self.max_abs_coordinate or abs(y) > self.max_abs_coordinate:
                    raise ValueError('coordinate is outside the configured world bounds')
                points.append((x, y))
        except (KeyError, TypeError, ValueError) as exc:
            self._set_control_status('error', f'Invalid boat path: {exc}')
            return

        try:
            path_id = int(command.get('path_id', int(time.time() * 1000)))
        except (TypeError, ValueError):
            path_id = int(time.time() * 1000)

        self._cancel_nav2_goal()
        self.active_path = points
        self.active_path_id = path_id
        self.waypoint_index = self._first_unreached_waypoint(points)
        if self.waypoint_index >= len(points):
            self._complete_path()
            return

        self.nav2_goal_pending = False
        self._set_control_status(
            'tracking',
            f'Accepted Unity A* path {path_id}',
        )
        self.get_logger().info(
            'Accepted Unity A* path %d with %d waypoints; starting at %d'
            % (path_id, len(points), self.waypoint_index)
        )

    def _first_unreached_waypoint(self, points):
        boat = self._boat_snapshot()
        if boat is None:
            return 0
        boat_x, boat_y, _ = boat
        index = 0
        while index < len(points) - 1:
            if math.hypot(points[index][0] - boat_x, points[index][1] - boat_y) > self.waypoint_radius:
                break
            index += 1
        return index

    def _update_direct_control(self):
        if not self.active_path:
            return

        boat = self._boat_snapshot()
        if boat is None:
            self._publish_boat_cmd(0.0, 0.0)
            self._set_control_status('waiting_pose', 'Waiting for fresh Gazebo boat pose')
            return

        boat_x, boat_y, boat_yaw = boat
        while self.waypoint_index < len(self.active_path):
            goal_x, goal_y = self.active_path[self.waypoint_index]
            distance = math.hypot(goal_x - boat_x, goal_y - boat_y)
            radius = (
                self.final_arrival_radius
                if self.waypoint_index == len(self.active_path) - 1
                else self.waypoint_radius
            )
            if distance > radius:
                break
            self.waypoint_index += 1

        if self.waypoint_index >= len(self.active_path):
            self._complete_path()
            return

        goal_x, goal_y = self.active_path[self.waypoint_index]
        dx = goal_x - boat_x
        dy = goal_y - boat_y
        distance = math.hypot(dx, dy)
        yaw_error = wrap_pi(math.atan2(dy, dx) - boat_yaw)
        speed_scale = clamp(distance / self.slow_radius, 0.0, 1.0)
        linear_x = max(self.min_speed, self.max_speed * speed_scale)
        heading_scale = clamp(
            1.0 - abs(yaw_error) / self.heading_slowdown_yaw,
            0.12,
            1.0,
        )
        linear_x *= heading_scale
        angular_z = clamp(
            self.turn_gain * yaw_error,
            -self.max_turn,
            self.max_turn,
        )
        self._publish_boat_cmd(linear_x, angular_z)
        self._set_control_status(
            'tracking',
            'Direct heading controller',
        )

    def _update_nav2_control(self):
        if (
            not self.active_path
            or self.nav2_goal_pending
            or self.nav2_goal_handle is not None
        ):
            return
        if not self.nav2_client.server_is_ready():
            self._set_control_status('waiting_nav2', 'Waiting for NavigateToPose')
            return

        if self.waypoint_index >= len(self.active_path):
            self._complete_path()
            return

        goal_x, goal_y = self.active_path[self.waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self._to_simulation_distance(goal_x)
        goal.pose.pose.position.y = self._to_simulation_distance(goal_y)
        goal.pose.pose.orientation.w = 1.0

        path_id = self.active_path_id
        waypoint_index = self.waypoint_index
        self.nav2_goal_pending = True
        future = self.nav2_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._on_nav2_goal_response(
                result,
                path_id,
                waypoint_index,
            )
        )
        self._set_control_status('tracking', 'Nav2 MPPI controller')

    def _on_nav2_goal_response(self, future, path_id, waypoint_index):
        self.nav2_goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self._fail_path(f'Nav2 goal failed: {exc}')
            return

        if path_id != self.active_path_id or waypoint_index != self.waypoint_index:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._fail_path('Nav2 rejected a Unity waypoint')
            return

        self.nav2_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_nav2_result(
                result,
                path_id,
                waypoint_index,
            )
        )

    def _on_nav2_result(self, future, path_id, waypoint_index):
        if path_id != self.active_path_id or waypoint_index != self.waypoint_index:
            return
        self.nav2_goal_handle = None
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._fail_path(f'Nav2 result failed: {exc}')
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail_path(f'Nav2 stopped with status {wrapped_result.status}')
            return

        self.waypoint_index += 1
        if self.waypoint_index >= len(self.active_path):
            self._complete_path()

    def _boat_snapshot(self):
        primary_usv = self.usv_names[0]
        with self.lock:
            if (
                self.last_boat_update is None
                or time.monotonic() - self.last_boat_update > self.pose_stale_timeout
                or primary_usv not in self.latest
            ):
                return None
            boat = self.latest[primary_usv]
            position = tuple(boat['position'])
            orientation = tuple(boat['orientation'])
        return position[0], position[1], yaw_from_quaternion(orientation)

    def _complete_path(self):
        completed_id = self.active_path_id
        self._publish_boat_cmd(0.0, 0.0)
        self.waypoint_index = len(self.active_path)
        self._set_control_status('complete', f'Path {completed_id} complete')
        self.get_logger().info(f'Unity path {completed_id} complete')
        self.active_path = []
        self.nav2_goal_handle = None
        self.nav2_goal_pending = False

    def _fail_path(self, message):
        self._publish_boat_cmd(0.0, 0.0)
        self.active_path = []
        self.nav2_goal_handle = None
        self.nav2_goal_pending = False
        self._set_control_status('error', message)
        self.get_logger().error(message)

    def _stop_boat(self, message, clear_path):
        if clear_path and not self.active_path and self.control_state == 'stopped':
            return
        self._cancel_nav2_goal()
        self._publish_boat_cmd(0.0, 0.0)
        if clear_path:
            self.active_path = []
            self.active_path_id = 0
            self.waypoint_index = 0
        self._set_control_status('stopped', message)
        self.get_logger().info(message)

    def _cancel_nav2_goal(self):
        if self.nav2_goal_handle is not None:
            try:
                self.nav2_goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.nav2_goal_handle = None
        self.nav2_goal_pending = False

    def _publish_boat_cmd(self, linear_x, angular_z):
        if self.control_mode == 'observe':
            return
        message = Twist()
        message.linear.x = self._to_simulation_distance(linear_x)
        message.angular.z = float(angular_z)
        self.boat_cmd_pub.publish(message)

    def _set_control_status(self, state, message):
        with self.lock:
            self.control_state = state
            self.control_message = message

    def _publish_frame(self):
        with self.lock:
            if (
                self.last_gazebo_update is None
                or time.monotonic() - self.last_gazebo_update > self.pose_stale_timeout
            ):
                return

            usvs = self._fleet_items(self.usv_names)
            uavs = self._fleet_items(self.uav_names)
            if (
                self.usv_names[0] not in self.latest
                or self.uav_names[0] not in self.latest
            ):
                return

            self.sequence += 1
            payload = {
                'type': 'pose_frame',
                'schema_version': 2,
                'timestamp_ms': int(time.time() * 1000),
                'sequence': self.sequence,
                'fleet': {
                    'expected_usvs': len(self.usv_names),
                    'expected_uavs': len(self.uav_names),
                    'received_usvs': len(usvs),
                    'received_uavs': len(uavs),
                    'ready': (
                        len(usvs) == len(self.usv_names)
                        and len(uavs) == len(self.uav_names)
                    ),
                },
                'usvs': usvs,
                'uavs': uavs,
                # Keep the original single-vehicle fields for the current Unity client.
                'boat': self.latest[self.usv_names[0]],
                'drone': self.latest[self.uav_names[0]],
                'mission': {
                    'capture': dict(self.capture_state),
                    'roles': dict(self.capture_roles),
                    'escort': dict(self.escort_state),
                },
                'control': {
                    'state': self.control_state,
                    'message': self.control_message,
                    'path_id': self.active_path_id,
                    'waypoint_index': self.waypoint_index,
                    'waypoint_count': len(self.active_path),
                    'mode': self.control_mode,
                },
            }
            for name in (
                'lighthouse',
                'buoy_west',
                'buoy_south',
                'buoy_east',
            ):
                if name in self.latest:
                    payload[name] = self.latest[name]
            if self.target_entity_name in self.latest:
                target = self.latest[self.target_entity_name]
                payload['target'] = {
                    'id': self.target_entity_name,
                    **target,
                }
                payload['target_vessel'] = target
            if self.friendly_ship_name in self.latest:
                friendly = self.latest[self.friendly_ship_name]
                payload['friendly_ship'] = {
                    'id': self.friendly_ship_name,
                    **friendly,
                }

        self._broadcast(json.dumps(payload, separators=(',', ':')))

    def _fleet_items(self, names):
        items = []
        for name in names:
            pose = self.latest.get(name)
            if pose is None:
                continue
            item = {
                'id': name,
                'position': list(pose['position']),
                'orientation': list(pose['orientation']),
            }
            state = self.vehicle_states.get(name)
            if state is not None:
                item['status'] = state
            items.append(item)
        return items

    def _broadcast(self, text):
        dead_clients = []

        with self.client_lock:
            clients = list(self.ws_clients)

        for client in clients:
            try:
                self._send_frame(client, text)
            except (ConnectionError, OSError):
                dead_clients.append(client)

        for client in dead_clients:
            self._remove_client(client)

    def destroy_node(self):
        self.running = False
        self._cancel_nav2_goal()
        self._publish_boat_cmd(0.0, 0.0)
        self.active_path = []
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass

        with self.client_lock:
            clients = list(self.ws_clients)

        for client in clients:
            self._remove_client(client)

        if self.nav2_client is not None:
            try:
                self.nav2_client.destroy()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnityWebSocketBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
