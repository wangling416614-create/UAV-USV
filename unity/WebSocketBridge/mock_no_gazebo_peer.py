#!/usr/bin/env python3

"""Lightweight ROS/Gazebo-Transport peer for browser/Unity round-trip tests."""

import math
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzTransportNode
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
from uav_usv_interfaces.msg import VehicleState


INITIAL_POSES = {
    'uav_01': (-86.86, -222.43, 19.75),
    'uav_02': (-75.00, -215.00, 19.75),
    'uav_03': (-63.14, -207.57, 19.75),
    'usv_01': (-120.00, -305.00, 0.00),
    'usv_02': (-75.00, -320.00, 0.00),
    'usv_03': (-30.00, -305.00, 0.00),
    'friendly_ship': (-150.00, -355.00, 0.00),
    'enemy_ship': (-80.00, -315.00, 0.00),
}


class NoGazeboPeer(Node):
    def __init__(self):
        super().__init__('uav_usv_no_gazebo_peer')
        self.started = time.monotonic()
        self.vehicle_modes = {
            vehicle_id: ('GROUNDED' if vehicle_id.startswith('uav_') else 'MOORED')
            for vehicle_id in INITIAL_POSES
            if vehicle_id.startswith(('uav_', 'usv_'))
        }

        self.gz_node = GzTransportNode()
        self.pose_pub = self.gz_node.advertise(
            '/world/heterogeneous_332/pose/info', Pose_V
        )
        self.ack_pub = self.create_publisher(CommandAck, '/fleet/command_ack', 20)
        self.state_pub = self.create_publisher(VehicleState, '/fleet/state', 20)
        self.image_pub = self.create_publisher(
            Image, '/fleet/uplink/usv_01/camera', 2
        )
        self.cloud_pub = self.create_publisher(
            point_cloud2.PointCloud2, '/perception/usv_01/mid360/preview', 2
        )
        self.radar_pub = self.create_publisher(
            TrackedObjectArray, '/perception/lv_dot_ros2/tracks', 2
        )
        self.detection_pub = self.create_publisher(
            AffiliatedDetection2DArray,
            '/perception/usv_01/camera/affiliated_detections',
            2,
        )
        self.create_subscription(FleetCommand, '/fleet/command', self.on_command, 20)

        self.image_data = self._make_test_image(320, 180)
        self.create_timer(0.1, self.publish_poses)
        self.create_timer(0.5, self.publish_sensors)
        self.create_timer(1.0, self.publish_states)
        self.get_logger().info(
            'No-Gazebo peer ready: poses + camera/radar/point cloud; '
            'waiting for /fleet/command'
        )

    @staticmethod
    def _make_test_image(width, height):
        pixels = bytearray(width * height * 3)
        for y in range(height):
            for x in range(width):
                offset = (y * width + x) * 3
                pixels[offset:offset + 3] = bytes((
                    35 + 100 * y // height,
                    80 + 120 * x // width,
                    190,
                ))
        return bytes(pixels)

    def publish_poses(self):
        elapsed = time.monotonic() - self.started
        message = Pose_V()
        for name, initial in INITIAL_POSES.items():
            x, y, z = initial
            if name == 'enemy_ship':
                x += 5.0 * math.cos(elapsed * 0.12)
                y += 5.0 * math.sin(elapsed * 0.12)
            pose = message.pose.add()
            pose.name = name
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
        self.pose_pub.publish(message)

    def publish_states(self):
        stamp = self.get_clock().now().to_msg()
        for vehicle_id, mode in self.vehicle_modes.items():
            message = VehicleState()
            message.header.stamp = stamp
            message.header.frame_id = 'map'
            message.vehicle_id = vehicle_id
            message.vehicle_type = (
                VehicleState.TYPE_UAV
                if vehicle_id.startswith('uav_')
                else VehicleState.TYPE_USV
            )
            message.online = True
            message.armed = mode not in ('GROUNDED', 'MOORED', 'STOPPED')
            message.mode = mode
            x, y, z = INITIAL_POSES[vehicle_id]
            message.pose.position.x = x
            message.pose.position.y = y
            message.pose.position.z = z
            message.pose.orientation.w = 1.0
            message.battery_percent = 88.0
            message.status_text = 'NO_GAZEBO_TEST'
            self.state_pub.publish(message)

    def publish_sensors(self):
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'usv_01/camera'
        image.height = 180
        image.width = 320
        image.encoding = 'bgr8'
        image.step = 320 * 3
        image.data = self.image_data
        self.image_pub.publish(image)

        header = Header()
        header.stamp = stamp
        header.frame_id = 'map'
        points = []
        for angle_index in range(72):
            angle = angle_index * math.tau / 72.0
            for radius in (5.0, 10.0, 15.0, 20.0):
                points.append((
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    0.3 + 0.2 * math.sin(angle * 3.0),
                ))
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

        tracks = TrackedObjectArray()
        tracks.header = header
        track = TrackedObject()
        track.track_id = 'mock-enemy-ship'
        track.source_mask = TrackedObject.SOURCE_FUSED
        track.classification = TrackedObject.CLASS_VESSEL
        track.class_name = 'vessel'
        track.class_confidence = 0.97
        track.pose.pose.position.x = -80.0
        track.pose.pose.position.y = -315.0
        track.pose.pose.orientation.w = 1.0
        track.confidence = 0.97
        track.affiliation = TrackedObject.AFFILIATION_HOSTILE
        track.affiliation_confidence = 0.95
        tracks.objects.append(track)
        self.radar_pub.publish(tracks)

        detections = AffiliatedDetection2DArray()
        detections.header = header
        detection = AffiliatedDetection2D()
        detection.detection_id = 'mock-camera-enemy-ship'
        detection.center_x = 0.5
        detection.center_y = 0.48
        detection.size_x = 0.22
        detection.size_y = 0.18
        detection.class_name = 'vessel'
        detection.class_confidence = 0.96
        detection.affiliation = AffiliatedDetection2D.AFFILIATION_HOSTILE
        detection.affiliation_confidence = 0.94
        detections.detections.append(detection)
        self.detection_pub.publish(detections)

    def on_command(self, command):
        self.get_logger().info(
            'COMMAND %s: vehicle=%s type=%d'
            % (command.command_id, command.vehicle_id, command.command_type)
        )
        if command.vehicle_id in self.vehicle_modes:
            if command.command_type == FleetCommand.COMMAND_TAKEOFF:
                self.vehicle_modes[command.vehicle_id] = 'AIRBORNE'
            elif command.command_type == FleetCommand.COMMAND_HOLD:
                self.vehicle_modes[command.vehicle_id] = 'HOLDING'
            elif command.command_type == FleetCommand.COMMAND_NAVIGATE:
                self.vehicle_modes[command.vehicle_id] = (
                    'AIRBORNE'
                    if command.vehicle_id.startswith('uav_')
                    else 'SAILING'
                )
            elif command.command_type == FleetCommand.COMMAND_EMERGENCY_STOP:
                self.vehicle_modes[command.vehicle_id] = 'STOPPED'

        acknowledgement = CommandAck()
        acknowledgement.header.stamp = self.get_clock().now().to_msg()
        acknowledgement.command_id = command.command_id
        acknowledgement.vehicle_id = command.vehicle_id
        acknowledgement.status = CommandAck.STATUS_SUCCEEDED
        acknowledgement.progress = 1.0
        acknowledgement.message = 'No-Gazebo round-trip succeeded'
        self.ack_pub.publish(acknowledgement)


def main():
    rclpy.init()
    node = NoGazeboPeer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
