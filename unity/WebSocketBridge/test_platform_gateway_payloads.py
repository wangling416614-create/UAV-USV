#!/usr/bin/env python3

import json
import math
import threading

from rclpy.clock import Clock
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from uav_usv_interfaces.msg import AffiliatedDetection2D
from uav_usv_interfaces.msg import AffiliatedDetection2DArray
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray

from escort_guard_algorithm import RealtimeEscortGuardPlanner
from unity_websocket_bridge import UnityWebSocketBridge


class Logger:
    def warn(self, message, **_kwargs):
        raise RuntimeError(message)

    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def bridge_capture():
    bridge = UnityWebSocketBridge.__new__(UnityWebSocketBridge)
    bridge.pointcloud_max_points = 3
    bridge.get_logger = lambda: Logger()
    frames = []
    bridge._broadcast = frames.append
    return bridge, frames


def test_pointcloud_frame():
    bridge, frames = bridge_capture()
    header = Header()
    header.frame_id = 'map'
    cloud_header = Header()
    cloud_header.frame_id = 'usv_03/mid360_link'
    cloud = point_cloud2.create_cloud_xyz32(
        cloud_header,
        [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)],
    )
    bridge._publish_pointcloud_frame(cloud)
    payload = json.loads(frames.pop())
    data = payload['frame']['data']
    assert payload['type'] == 'pointcloud_frame'
    assert data['encoding'] == 'xyz_f32_le_base64'
    assert data['point_count'] == 3
    assert len(data['data_base64']) == 48


def test_radar_and_visual_frames():
    bridge, frames = bridge_capture()
    tracks = TrackedObjectArray()
    tracks.header.frame_id = 'map'
    track = TrackedObject()
    track.track_id = 'enemy-01'
    track.pose.pose.position.x = 3.0
    track.pose.pose.position.y = 4.0
    track.class_name = 'vessel'
    track.confidence = 0.9
    tracks.objects.append(track)
    bridge._publish_radar_frame(tracks, 'usv_02')
    radar = json.loads(frames.pop())
    assert radar['type'] == 'radar_frame'
    assert radar['frame']['device_id'] == 'usv_02'
    assert radar['frame']['detections'][0]['range'] == 5.0

    detection_array = AffiliatedDetection2DArray()
    detection = AffiliatedDetection2D()
    detection.detection_id = 'camera-enemy-01'
    detection.center_x = 0.5
    detection.center_y = 0.4
    detection.size_x = 0.2
    detection.size_y = 0.3
    detection_array.detections.append(detection)
    bridge._publish_visual_detection_frame(detection_array)
    visual = json.loads(frames.pop())
    assert visual['type'] == 'visual_detection_frame'
    assert visual['detections'][0]['id'] == 'camera-enemy-01'

    cloud_header = Header()
    cloud_header.frame_id = 'usv_03/mid360_link'
    cloud = point_cloud2.create_cloud_xyz32(
        cloud_header,
        [(1.0, 0.0, 0.0)],
    )
    bridge._publish_pointcloud_frame(cloud, 'usv_03')
    pointcloud = json.loads(frames.pop())
    assert pointcloud['frame']['data']['vehicle_id'] == 'usv_03'
    assert pointcloud['frame']['data']['stream_id'] == 'usv_03_mid360'


def test_platform_control_command():
    bridge = UnityWebSocketBridge.__new__(UnityWebSocketBridge)
    bridge.fleet_command_pub = Publisher()
    bridge.control_lease_pub = Publisher()
    bridge.operator_action_pub = Publisher()
    bridge.usv_names = ('usv_01', 'usv_02', 'usv_03')
    bridge.uav_names = ('uav_01', 'uav_02', 'uav_03')
    bridge.camera_topics = {}
    bridge.gateway_lease_id = 'test-platform'
    bridge.gateway_owner_id = 'test'
    bridge.gateway_priority = 220
    bridge.lease_duration = 10.0
    bridge.command_lifetime = 10.0
    bridge.takeoff_altitude = 18.0
    bridge.simulation_coordinate_scale = 0.18
    bridge.active_control_until = {}
    bridge.last_platform_lease_publish = {}
    bridge.pending_fleet_commands = []
    bridge.platform_command_ids = set()
    bridge.home_poses = {'uav_01': (0.0, 0.0, 1.0)}
    bridge.last_navigation_targets = {}
    bridge.vehicle_states = {}
    bridge.get_clock = lambda: Clock()
    bridge.get_logger = lambda: Logger()
    bridge._set_control_status = lambda *_args: None
    acks = []
    bridge._send_platform_ack = lambda *args: acks.append(args)

    bridge._accept_platform_command({
        'type': 'command',
        'commandKey': 'command-1',
        'commandType': 'UAV_TAKEOFF',
        'deviceCode': 'UAV-01',
        'payload': {'altitude': 20},
    })
    assert not acks
    assert len(bridge.control_lease_pub.messages) == 1
    assert len(bridge.pending_fleet_commands) == 1
    command = bridge.pending_fleet_commands[0][1]
    assert command.command_id == 'command-1'
    assert command.vehicle_id == 'uav_01'
    assert len(command.parameters) == 1
    assert math.isclose(command.parameters[0], 3.6, abs_tol=1e-5)
    bridge.pending_fleet_commands[0] = (0.0, command)
    bridge._publish_pending_fleet_commands()
    assert len(bridge.fleet_command_pub.messages) == 1


def test_mission_algorithm_routing():
    bridge = UnityWebSocketBridge.__new__(UnityWebSocketBridge)
    bridge.operator_action_pub = Publisher()
    bridge.friendly_ship_name = 'friendly_ship'
    bridge.target_entity_name = 'enemy_ship'
    bridge.latest = {
        'friendly_ship': {
            'position': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0, 1.0],
        },
    }
    bridge.lock = threading.Lock()
    bridge.pending_mission_commands = {}
    bridge.mission_ack_timeout = 12.0
    bridge.last_mission_state_signatures = {}
    bridge._set_control_status = lambda *_args: None
    bridge._broadcast = lambda *_args: None
    acks = []
    bridge._send_platform_ack = lambda *args: acks.append(args)

    bridge._accept_platform_command({
        'type': 'command',
        'commandKey': 'capture-start',
        'commandType': 'START_MISSION',
        'payload': {
            'algorithmCode': 'GB_SFLA_CS',
            'targetId': 'enemy_ship',
        },
    })
    bridge._accept_platform_command({
        'type': 'command',
        'commandKey': 'escort-start',
        'commandType': 'START_MISSION',
        'payload': {
            'algorithmCode': 'ESCORT_GUARD',
            'targetId': 'friendly_ship',
        },
    })
    assert [msg.data for msg in bridge.operator_action_pub.messages] == [
        'CAPTURE:enemy_ship', 'ESCORT:friendly_ship',
    ]
    assert not acks
    bridge._observe_mission_state(
        'GB_SFLA_CS', 'RUNNING', 'TRACKING', 'enemy_ship',
        'operator approved capture',
    )
    bridge._observe_mission_state(
        'ESCORT_GUARD', 'RUNNING', 'FORMING', 'friendly_ship',
        'ROS escort mission started',
    )
    assert [ack[1] for ack in acks] == [3, 3]

    bridge._accept_platform_command({
        'type': 'command',
        'commandKey': 'capture-restart',
        'commandType': 'START_MISSION',
        'payload': {
            'algorithmCode': 'GB_SFLA_CS',
            'targetId': 'enemy_ship',
        },
    })
    bridge._observe_mission_state(
        'GB_SFLA_CS', 'FAILED', 'FAILED', 'enemy_ship',
        'capture planner rejected mission',
    )
    assert acks[-1][0] == 'capture-restart'
    assert acks[-1][1] == 5
    assert 'capture-restart' not in bridge.pending_mission_commands

def test_supplied_escort_guard_algorithm():
    planner = RealtimeEscortGuardPlanner(scale=6.0, reserve_count=0)
    vehicles = {
        'uav_01': (5.0, 4.0, 18.0),
        'uav_02': (-8.0, 14.0, 18.0),
        'uav_03': (-12.0, -10.0, 18.0),
        'usv_01': (14.0, 1.0, 0.0),
        'usv_02': (-18.0, 9.0, 0.0),
        'usv_03': (-16.0, -12.0, 0.0),
    }
    plan = planner.plan(
        protected_position=(0.0, 0.0, 0.0),
        threat_position=(60.0, 0.0, 0.0),
        vehicle_positions=vehicles,
        protected_yaw=0.0,
        uav_altitude=24.0,
    )
    assert plan.detected
    assert len(plan.targets) == 6
    assert list(plan.roles.values()).count('core') == 1
    assert list(plan.roles.values()).count('wing') == 2
    assert list(plan.roles.values()).count('support') == 3
    assert plan.details['algorithmMode'] == 'REAL_GAZEBO_POSE'
    assert math.isclose(plan.details['blockerPoint'][0], 22.8)
    core_id = next(
        vehicle_id for vehicle_id, role in plan.roles.items() if role == 'core'
    )
    assert math.isclose(plan.targets[core_id][0], 22.8)

    planner.reset()
    normal = planner.plan(
        protected_position=(0.0, 0.0, 0.0),
        threat_position=(90.0, 0.0, 0.0),
        vehicle_positions=vehicles,
        protected_yaw=0.0,
        uav_altitude=24.0,
    )
    assert not normal.detected
    assert normal.phase == 'NORMAL_ESCORT'
    assert set(normal.roles.values()) == {'escort'}


def test_escort_takes_off_uavs_before_navigation():
    bridge = UnityWebSocketBridge.__new__(UnityWebSocketBridge)
    bridge.usv_names = ('usv_01', 'usv_02', 'usv_03')
    bridge.uav_names = ('uav_01', 'uav_02', 'uav_03')
    bridge.friendly_ship_name = 'friendly_ship'
    bridge.target_entity_name = 'enemy_ship'
    bridge.escort_protected_id = 'friendly_ship'
    bridge.escort_active = True
    bridge.escort_paused = False
    bridge.escort_last_command_time = 0.0
    bridge.escort_command_period = 0.5
    bridge.escort_takeoff_retry_period = 20.0
    bridge.escort_command_sequence = 0
    bridge.escort_takeoff_commands = {}
    bridge.escort_takeoff_state = {}
    bridge.escort_usv_radius = 28.0
    bridge.escort_uav_radius = 42.0
    bridge.escort_uav_altitude = 24.0
    bridge.escort_planner = RealtimeEscortGuardPlanner(scale=6.0)
    bridge.takeoff_altitude = 18.0
    bridge.simulation_coordinate_scale = 0.18
    bridge.gateway_lease_id = 'test-platform'
    bridge.gateway_owner_id = 'test'
    bridge.gateway_priority = 220
    bridge.lease_duration = 20.0
    bridge.command_lifetime = 15.0
    bridge.active_control_until = {}
    bridge.last_platform_lease_publish = {}
    bridge.pending_fleet_commands = []
    bridge.pending_mission_commands = {}
    bridge.last_mission_state_signatures = {}
    bridge.control_lease_pub = Publisher()
    bridge.lock = threading.Lock()
    bridge.latest = {
        name: {
            'position': [float(index), 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0, 1.0],
        }
        for index, name in enumerate(
            ('friendly_ship',) + bridge.usv_names + bridge.uav_names
        )
    }
    bridge.latest['enemy_ship'] = {
        'position': [60.0, 0.0, 0.0],
        'orientation': [0.0, 0.0, 0.0, 1.0],
    }
    bridge.vehicle_states = {
        name: {'online': True, 'armed': not name.startswith('uav_')}
        for name in bridge.usv_names + bridge.uav_names
    }
    bridge.get_clock = lambda: Clock()
    bridge.get_logger = lambda: Logger()
    bridge._broadcast = lambda *_args: None
    bridge._send_platform_ack = lambda *_args: None
    bridge._set_control_status = lambda *_args: None

    bridge._update_escort_mission()
    queued_types = {
        name: [
            command.command_type
            for _due, command in bridge.pending_fleet_commands
            if command.vehicle_id == name
        ]
        for name in bridge.usv_names + bridge.uav_names
    }
    assert all(
        queued_types[name] == [FleetCommand.COMMAND_NAVIGATE]
        for name in bridge.usv_names
    )
    assert all(
        queued_types[name] == [FleetCommand.COMMAND_TAKEOFF]
        for name in bridge.uav_names
    )
    assert bridge.escort_state['phase'] == 'TAKING_OFF'

    bridge.pending_fleet_commands.clear()
    bridge.escort_last_command_time = 0.0
    for name in bridge.uav_names:
        bridge.vehicle_states[name]['armed'] = True
    bridge._update_escort_mission()
    assert all(
        any(
            command.vehicle_id == name
            and command.command_type == FleetCommand.COMMAND_NAVIGATE
            for _due, command in bridge.pending_fleet_commands
        )
        for name in bridge.uav_names
    )
    assert bridge.escort_state['phase'] in ('FORMING', 'GUARDING')
    assert bridge.escort_state['algorithmMode'] == 'REAL_GAZEBO_POSE'


def main():
    assert UnityWebSocketBridge._normalize_vehicle_id('UAV-01') == 'uav_01'
    assert UnityWebSocketBridge._target_from_payload(
        {'target': {'x': 1, 'y': 2, 'z': 3}}
    ) == (1.0, 2.0, 3.0)
    test_pointcloud_frame()
    test_radar_and_visual_frames()
    test_platform_control_command()
    test_mission_algorithm_routing()
    test_supplied_escort_guard_algorithm()
    test_escort_takes_off_uavs_before_navigation()
    print('platform gateway payloads: OK')


if __name__ == '__main__':
    main()
