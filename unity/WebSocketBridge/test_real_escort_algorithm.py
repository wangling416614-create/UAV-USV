#!/usr/bin/env python3

"""Validate the supplied escort algorithm against the real ROS/Gazebo stack.

This test publishes only the operator action.  It does not create virtual UAV,
USV, pose, state, sensor, or acknowledgement nodes.
"""

import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from uav_usv_interfaces.msg import FleetCommand

from test_bridge_roundtrip import connect, receive_text
from test_scene_reset import collect


EXPECTED_VEHICLES = {
    'uav_01', 'uav_02', 'uav_03', 'usv_01', 'usv_02', 'usv_03'
}


class RealEscortObserver(Node):
    def __init__(self):
        super().__init__('real_escort_algorithm_validation')
        self.commands = {}
        self.action_pub = self.create_publisher(
            String, '/fleet/base/operator_action', 10
        )
        self.create_subscription(
            FleetCommand, '/fleet/command', self._on_command, 60
        )

    def _on_command(self, message):
        if message.command_id.startswith(('escort-', 'escort-takeoff-')):
            self.commands[message.vehicle_id] = message

    def publish_action(self, action):
        message = String()
        message.data = action
        self.action_pub.publish(message)


def main():
    rclpy.init()
    node = RealEscortObserver()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    connection = connect()
    details = None
    initial_poses = {}
    maximum_pose_delta = 0.0
    try:
        discovery_deadline = time.monotonic() + 12.0
        while time.monotonic() < discovery_deadline:
            if node.action_pub.get_subscription_count() >= 1:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError('real escort operator subscription was not discovered')

        # Capture the frozen home snapshot before the ROS-side start command.
        pose_deadline = time.monotonic() + 6.0
        while time.monotonic() < pose_deadline and not initial_poses:
            frame = json.loads(receive_text(connection))
            if frame.get('type') == 'pose_frame':
                initial_poses = collect(frame)
        if not EXPECTED_VEHICLES.issubset(initial_poses):
            raise RuntimeError('Unity bridge did not expose the initial 3+3 fleet poses')

        node.publish_action('ESCORT:friendly_ship')
        connection.settimeout(0.75)
        deadline = time.monotonic() + 35.0
        next_action = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_action and not node.commands:
                node.publish_action('ESCORT:friendly_ship')
                next_action = time.monotonic() + 3.0
            try:
                frame = json.loads(receive_text(connection))
            except TimeoutError:
                continue
            if frame.get('type') == 'pose_frame':
                current = collect(frame)
                for vehicle_id in EXPECTED_VEHICLES:
                    if vehicle_id in current:
                        maximum_pose_delta = max(
                            maximum_pose_delta,
                            math.dist(initial_poses[vehicle_id], current[vehicle_id]),
                        )
            if (
                frame.get('type') == 'mission_state'
                and frame.get('algorithmCode') == 'ESCORT_GUARD'
            ):
                candidate = frame.get('details')
                if (
                    isinstance(candidate, dict)
                    and candidate.get('algorithmMode') == 'REAL_GAZEBO_POSE'
                ):
                    details = candidate
            if (
                details is not None
                and EXPECTED_VEHICLES.issubset(node.commands)
                and maximum_pose_delta >= 0.5
            ):
                break

        if details is None:
            raise RuntimeError('no REAL_GAZEBO_POSE mission state was observed')
        roles = details.get('roles')
        if not isinstance(roles, dict):
            raise RuntimeError('real algorithm state did not contain role assignments')
        role_values = list(roles.values())
        expected_counts = {'core': 1, 'wing': 2, 'support': 3}
        actual_counts = {
            role: role_values.count(role) for role in expected_counts
        }
        if actual_counts != expected_counts:
            raise RuntimeError(
                'unexpected real role assignment: %s' % actual_counts
            )
        missing = EXPECTED_VEHICLES - set(node.commands)
        if missing:
            raise RuntimeError(
                'real fleet did not receive escort/takeoff commands: %s'
                % sorted(missing)
            )
        if maximum_pose_delta < 0.5:
            raise RuntimeError('ROS task started, but Unity pose frames did not move')
        print('real ROS/Gazebo escort algorithm: OK')
        print('Gazebo -> Unity pose synchronization: OK (delta %.2f)' % maximum_pose_delta)
        print('roles:', json.dumps(roles, ensure_ascii=False, sort_keys=True))
        print('blockerPoint:', details.get('blockerPoint'))
        print('threatDistance:', details.get('threatDistance'))
        print('vehicles commanded:', ', '.join(sorted(node.commands)))
    finally:
        node.publish_action('CANCEL_ESCORT')
        time.sleep(0.3)
        connection.close()
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
