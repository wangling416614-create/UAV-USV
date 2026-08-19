#!/usr/bin/env python3

import json
import math
import socket
import time

from test_bridge_roundtrip import connect
from test_bridge_roundtrip import receive_text
from test_bridge_roundtrip import send_json


SIMULATION_HOME = {
    'uav_01': (-15.6348, -40.0374, 3.53210004),
    'uav_02': (-13.5, -38.7, 3.53210004),
    'uav_03': (-11.3652, -37.3626, 3.53210004),
    'usv_01': (-21.6, -54.9, 0.0),
    'usv_02': (-13.5, -57.6, 0.0),
    'usv_03': (-5.4, -54.9, 0.0),
    'friendly_ship': (-27.0, -63.9, 0.0),
    'enemy_ship': (-14.4, -62.1, 0.0),
}
EXPECTED = {
    name: tuple(value / 0.18 for value in position)
    for name, position in SIMULATION_HOME.items()
}


def collect(frame):
    vehicles = {
        item['id']: tuple(item['position'])
        for item in frame.get('usvs', []) + frame.get('uavs', [])
    }
    for key in ('friendly_ship', 'target'):
        item = frame.get(key) or {}
        if item.get('id'):
            vehicles[item['id']] = tuple(item['position'])
    return vehicles


def main():
    connection = connect()
    request_id = 'scene-reset-%d' % time.time_ns()
    send_json(connection, {'type': 'reset_scene', 'request_id': request_id})
    acknowledged = False
    acknowledged_at = 0.0
    latest = {}
    deadline = time.monotonic() + 15.0
    try:
        while time.monotonic() < deadline:
            try:
                payload = json.loads(receive_text(connection))
            except socket.timeout:
                continue
            if (
                payload.get('type') == 'scene_reset_ack'
                and payload.get('request_id') == request_id
            ):
                if payload.get('success') is not True:
                    raise RuntimeError(payload.get('status', 'scene reset failed'))
                acknowledged = True
                acknowledged_at = time.monotonic()
                # Gazebo is paused before ACK, so the last pose frame may be
                # the final home snapshot rather than a later telemetry frame.
            if 'usvs' in payload:
                latest = collect(payload)
            if (
                acknowledged
                and time.monotonic() - acknowledged_at >= 6.0
                and all(name in latest for name in EXPECTED)
            ):
                break
    finally:
        connection.close()
    if not acknowledged:
        raise RuntimeError('scene reset acknowledgement was not received')
    errors = []
    for name, expected in EXPECTED.items():
        actual = latest[name]
        distance = math.dist(actual, expected)
        # Bridge frames expose logical coordinates (simulation / 0.18).
        # Allow normal physics settling at the water plane / landing pads.
        tolerance = 1.4 if name.startswith('uav_') else 0.4
        if distance > tolerance:
            errors.append('%s %.3fm' % (name, distance))
    if errors:
        raise RuntimeError('reset pose mismatch: ' + ', '.join(errors))
    print('scene reset: OK, 8 entities match local Unity initial poses')


if __name__ == '__main__':
    main()
