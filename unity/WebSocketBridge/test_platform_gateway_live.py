#!/usr/bin/env python3

import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from uav_usv_interfaces.msg import AffiliatedDetection2D
from uav_usv_interfaces.msg import AffiliatedDetection2DArray
from uav_usv_interfaces.msg import CommandAck
from uav_usv_interfaces.msg import FleetCommand
from uav_usv_interfaces.msg import TrackedObject
from uav_usv_interfaces.msg import TrackedObjectArray

from test_bridge_roundtrip import connect
from test_bridge_roundtrip import receive_text
from test_bridge_roundtrip import send_json


class TestRosPeer(Node):
    def __init__(self, command_key):
        super().__init__('platform_gateway_live_test')
        self.command_key = command_key
        self.command_seen = threading.Event()
        self.escort_vehicles = set()
        self.command_pub_ack = self.create_publisher(
            CommandAck, '/fleet/command_ack', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/fleet/uplink/usv_01/camera', 2
        )
        self.cloud_pubs = {
            device_id: self.create_publisher(
                point_cloud2.PointCloud2,
                f'/perception/{device_id}/mid360/preview',
                2,
            )
            for device_id in ('usv_01', 'usv_02', 'usv_03')
        }
        self.radar_pubs = {
            device_id: self.create_publisher(
                TrackedObjectArray,
                f'/perception/{device_id}/radar/tracks',
                2,
            )
            for device_id in ('usv_01', 'usv_02', 'usv_03')
        }
        self.fused_radar_pub = self.create_publisher(
            TrackedObjectArray, '/perception/lv_dot_ros2/tracks', 2
        )
        self.detection_pub = self.create_publisher(
            AffiliatedDetection2DArray,
            '/perception/usv_01/camera/affiliated_detections',
            2,
        )
        self.create_subscription(
            FleetCommand, '/fleet/command', self._on_command, 10
        )

    def _on_command(self, command):
        if (
            command.command_id.startswith('escort-')
            and command.command_type == FleetCommand.COMMAND_NAVIGATE
        ):
            self.escort_vehicles.add(command.vehicle_id)
        if command.command_id != self.command_key:
            return
        self.command_seen.set()
        ack = CommandAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        ack.command_id = command.command_id
        ack.vehicle_id = command.vehicle_id
        ack.status = CommandAck.STATUS_SUCCEEDED
        ack.progress = 1.0
        ack.message = 'live test command succeeded'
        self.command_pub_ack.publish(ack)

    def publish_sensors(self):
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'usv_01/camera'
        image.height = 4
        image.width = 4
        image.encoding = 'bgr8'
        image.step = 12
        image.data = bytes([20, 100, 180] * 16)
        self.image_pub.publish(image)

        header = Header()
        header.stamp = stamp
        header.frame_id = 'map'
        for index, (device_id, publisher) in enumerate(self.cloud_pubs.items()):
            cloud_header = Header()
            cloud_header.stamp = stamp
            cloud_header.frame_id = device_id + '/mid360_link'
            publisher.publish(point_cloud2.create_cloud_xyz32(
                cloud_header,
                [
                    (1.0 + index, 2.0, 0.1),
                    (2.0 + index, 3.0, 0.2),
                    (3.0 + index, 4.0, 0.3),
                ],
            ))

        tracks = TrackedObjectArray()
        tracks.header = header
        track = TrackedObject()
        track.track_id = 'live-target-1'
        track.pose.pose.position.x = 3.0
        track.pose.pose.position.y = 4.0
        track.class_name = 'vessel'
        track.confidence = 0.95
        tracks.objects.append(track)
        self.fused_radar_pub.publish(tracks)
        for device_id, publisher in self.radar_pubs.items():
            tracks.objects[0].sensor_source = device_id
            publisher.publish(tracks)

        detections = AffiliatedDetection2DArray()
        detections.header = header
        detection = AffiliatedDetection2D()
        detection.detection_id = 'live-camera-target-1'
        detection.center_x = 0.5
        detection.center_y = 0.5
        detection.size_x = 0.25
        detection.size_y = 0.2
        detections.detections.append(detection)
        self.detection_pub.publish(detections)


def main():
    rclpy.init()
    command_key = 'live-command-%d' % time.time_ns()
    peer = TestRosPeer(command_key)
    spin = threading.Thread(target=rclpy.spin, args=(peer,), daemon=True)
    spin.start()
    connection = connect()
    try:
        # Wait for DDS discovery instead of relying on a fixed delay. A busy
        # development domain may need a few seconds to discover the bridge.
        discovery_deadline = time.monotonic() + 6.0
        while time.monotonic() < discovery_deadline:
            if (
                peer.count_publishers('/fleet/command') > 0
                and peer.image_pub.get_subscription_count() > 0
                and all(pub.get_subscription_count() > 0 for pub in peer.cloud_pubs.values())
                and all(pub.get_subscription_count() > 0 for pub in peer.radar_pubs.values())
                and peer.fused_radar_pub.get_subscription_count() > 0
                and peer.detection_pub.get_subscription_count() > 0
            ):
                break
            time.sleep(0.1)

        peer.publish_sensors()
        send_json(connection, {
            'type': 'command',
            'commandKey': command_key,
            'commandType': 'USV_HOLD',
            'deviceCode': 'USV-01',
            'payload': {},
        })

        expected = {
            'camera_frame',
            'radar_frame',
            'pointcloud_frame',
            'visual_detection_frame',
            'command_ack',
        }
        seen = set()
        radar_devices = set()
        pointcloud_devices = set()
        expected_devices = {'usv_01', 'usv_02', 'usv_03'}
        connection.settimeout(0.5)
        deadline = time.monotonic() + 12.0
        next_sensor_publish = time.monotonic() + 0.5
        while time.monotonic() < deadline and (
            seen != expected
            or not expected_devices.issubset(radar_devices)
            or not expected_devices.issubset(pointcloud_devices)
        ):
            if time.monotonic() >= next_sensor_publish:
                peer.publish_sensors()
                next_sensor_publish = time.monotonic() + 0.5
            try:
                frame = json.loads(receive_text(connection))
            except TimeoutError:
                continue
            message_type = frame.get('type')
            if message_type == 'command_ack':
                if (
                    frame.get('commandKey') == command_key
                    and frame.get('status') == CommandAck.STATUS_SUCCEEDED
                ):
                    seen.add(message_type)
            elif message_type in expected:
                seen.add(message_type)
                if message_type == 'radar_frame':
                    radar_devices.add(frame.get('frame', {}).get('device_id'))
                elif message_type == 'pointcloud_frame':
                    pointcloud_devices.add(
                        frame.get('frame', {}).get('data', {}).get('vehicle_id')
                    )
        missing = expected - seen
        if missing:
            raise RuntimeError('missing live gateway frames: %s' % sorted(missing))
        if not peer.command_seen.is_set():
            raise RuntimeError('ROS peer did not receive the platform command')
        if not expected_devices.issubset(radar_devices):
            raise RuntimeError('missing per-device radar frames: %s' % sorted(radar_devices))
        if not expected_devices.issubset(pointcloud_devices):
            raise RuntimeError(
                'missing per-device pointcloud frames: %s'
                % sorted(pointcloud_devices)
            )

        escort_key = 'live-escort-%d' % time.time_ns()
        send_json(connection, {
            'type': 'command',
            'commandKey': escort_key,
            'commandType': 'START_MISSION',
            'payload': {
                'algorithmCode': 'ESCORT_GUARD',
                'targetId': 'friendly_ship',
            },
        })
        escort_ack = False
        escort_pose = False
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            try:
                frame = json.loads(receive_text(connection))
            except TimeoutError:
                continue
            if (
                frame.get('type') == 'command_ack'
                and frame.get('commandKey') == escort_key
                and frame.get('status') == CommandAck.STATUS_SUCCEEDED
            ):
                escort_ack = True
            mission = frame.get('mission', {})
            escort = mission.get('escort', {}) if isinstance(mission, dict) else {}
            if frame.get('type') == 'pose_frame' and escort.get('active') is True:
                escort_pose = True
            if escort_ack and escort_pose and len(peer.escort_vehicles) == 6:
                break
        if not escort_ack:
            raise RuntimeError('ROS gateway did not acknowledge escort mission')
        if not escort_pose:
            raise RuntimeError('escort state was not mirrored in pose frames')
        if len(peer.escort_vehicles) != 6:
            raise RuntimeError(
                'escort planner did not command 3+3 fleet: %s'
                % sorted(peer.escort_vehicles)
            )

        cancel_key = 'live-escort-cancel-%d' % time.time_ns()
        send_json(connection, {
            'type': 'command',
            'commandKey': cancel_key,
            'commandType': 'CANCEL_MISSION',
            'payload': {'algorithmCode': 'ESCORT_GUARD'},
        })
        print('platform gateway live round-trip: OK (%s)' % ', '.join(sorted(seen)))
        print('ROS escort 3+3 closed loop: OK (%s)' % ', '.join(sorted(peer.escort_vehicles)))
    finally:
        connection.close()
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=2.0)
        peer.destroy_node()


if __name__ == '__main__':
    main()
