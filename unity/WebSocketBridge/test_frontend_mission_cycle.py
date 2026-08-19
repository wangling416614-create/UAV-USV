#!/usr/bin/env python3

"""Exercise the real frontend mission sequence against ROS/Gazebo.

No virtual vehicle, sensor, state, or acknowledgement nodes are created.
The command envelopes match those forwarded by the platform backend.
"""

import json
import math
import socket
import time

from test_bridge_roundtrip import connect, receive_text, send_json
from test_scene_reset import EXPECTED, collect


EXPECTED_FLEET = {
    'uav_01', 'uav_02', 'uav_03', 'usv_01', 'usv_02', 'usv_03'
}


def wait_for(connection, predicate, timeout=20.0):
    deadline = time.monotonic() + timeout
    latest_pose = {}
    while time.monotonic() < deadline:
        try:
            frame = json.loads(receive_text(connection))
        except socket.timeout:
            continue
        if frame.get('type') == 'pose_frame':
            latest_pose = collect(frame)
        if predicate(frame):
            return frame, latest_pose
    raise RuntimeError('timed out waiting for ROS/bridge acknowledgement')


def send_mission_command(connection, command_type):
    command_key = 'frontend-cycle-%s-%d' % (
        command_type.lower(), time.time_ns()
    )
    send_json(connection, {
        'type': 'command',
        'commandKey': command_key,
        'commandType': command_type,
        'payload': {
            'missionSource': 'SYSTEM_OVERVIEW',
            'algorithmCode': 'GB_SFLA_CS',
            'targetId': 'enemy_ship',
        },
    })
    ack, _ = wait_for(
        connection,
        lambda item: (
            item.get('type') == 'command_ack'
            and item.get('commandKey') == command_key
        ),
    )
    if ack.get('status') not in (1, 3):
        raise RuntimeError(
            '%s failed: %s' % (command_type, ack.get('message', ack))
        )
    return ack


def wait_for_full_capture_allocation(connection):
    def is_full_allocation(frame):
        if frame.get('type') != 'pose_frame':
            return False
        mission = frame.get('mission', {})
        roles = mission.get('roles', {}) if isinstance(mission, dict) else {}
        assignments = roles.get('assignments', []) if isinstance(roles, dict) else []
        assigned = {
            item.get('vehicle_id') for item in assignments
            if isinstance(item, dict)
        }
        return EXPECTED_FLEET.issubset(assigned)

    wait_for(connection, is_full_allocation, timeout=20.0)


def reset_and_verify(connection):
    request_id = 'frontend-cycle-reset-%d' % time.time_ns()
    send_json(connection, {'type': 'reset_scene', 'request_id': request_id})
    acknowledged = False
    latest = {}
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        try:
            frame = json.loads(receive_text(connection))
        except socket.timeout:
            continue
        if frame.get('type') == 'pose_frame':
            latest = collect(frame)
        if (
            frame.get('type') == 'scene_reset_ack'
            and frame.get('request_id') == request_id
        ):
            if frame.get('success') is not True:
                raise RuntimeError(frame.get('status', 'scene reset failed'))
            acknowledged = True
            # Gazebo is paused before the reset ACK. Keep the final home-pose
            # frame received immediately before it; no later physics frame is
            # required (or expected) while the scene is held at home.
        if acknowledged and all(name in latest for name in EXPECTED):
            break
    if not acknowledged:
        raise RuntimeError('scene reset was not acknowledged')
    missing = set(EXPECTED) - set(latest)
    if missing:
        raise RuntimeError('reset pose frame is missing: ' + ', '.join(sorted(missing)))
    errors = []
    for name, expected in EXPECTED.items():
        tolerance = 1.4 if name.startswith('uav_') else 0.4
        distance = math.dist(latest[name], expected)
        if distance > tolerance:
            errors.append('%s %.3fm' % (name, distance))
    if errors:
        raise RuntimeError('reset pose mismatch: ' + ', '.join(errors))


def main():
    connection = connect()
    connection.settimeout(0.75)
    try:
        send_mission_command(connection, 'START_MISSION')
        wait_for_full_capture_allocation(connection)
        send_mission_command(connection, 'PAUSE_MISSION')
        send_mission_command(connection, 'RESUME_MISSION')
        send_mission_command(connection, 'STOP_MISSION')
        reset_and_verify(connection)
        print('frontend -> ROS start: OK')
        print('ROS capture allocation: OK (3 UAV + 3 USV)')
        print('frontend -> ROS pause: OK')
        print('frontend -> ROS resume: OK')
        print('frontend -> ROS stop: OK')
        print('frontend -> ROS/Gazebo reset: OK (8 Unity home poses)')
    finally:
        connection.close()


if __name__ == '__main__':
    main()
