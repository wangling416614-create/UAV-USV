"""Real-pose adapter for the supplied 3-D single-target escort algorithm.

The source algorithm is ``护航守卫_三维单目标_Python39(1).py``.  Its
Matplotlib renderer and its internally simulated vehicle/target motion are
intentionally not used here.  This module keeps the decision core and applies
it to authoritative Gazebo poses:

* distance-triggered threat detection;
* strict interior blocker point;
* minimum-cost core/wing/support assignment;
* threat-facing wing arc and rear support arc;
* core/wing formation readiness.

The caller remains responsible for publishing the returned targets to the
real ROS fleet controllers.
"""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


EPS = 1e-12
Target = Tuple[float, float, float, float]


@dataclass(frozen=True)
class EscortGuardPlan:
    phase: str
    reason: str
    detected: bool
    targets: Dict[str, Target]
    roles: Dict[str, str]
    details: Dict[str, object]


def _xy(value: Sequence[float]) -> np.ndarray:
    return np.asarray(value[:2], dtype=float)


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return float(np.linalg.norm(_xy(a) - _xy(b)))


def _normalize(value: Sequence[float], fallback=(1.0, 0.0)) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    length = float(np.linalg.norm(vector))
    if length > EPS:
        return vector / length
    fallback_vector = np.asarray(fallback, dtype=float)
    fallback_length = float(np.linalg.norm(fallback_vector))
    return fallback_vector / fallback_length if fallback_length > EPS else np.zeros(2)


def _rotate90(value: Sequence[float]) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    return np.array([-vector[1], vector[0]], dtype=float)


class RealtimeEscortGuardPlanner:
    """Generate live ROS navigation goals from the supplied escort algorithm."""

    SOURCE = '护航守卫_三维单目标_Python39(1).py'

    def __init__(self, *, scale: float = 7.0, reserve_count: int = 0):
        if scale <= 0.0:
            raise ValueError('scale must be positive')
        if reserve_count not in (0, 2):
            raise ValueError('reserve_count must be 0 or 2')
        self.scale = float(scale)
        self.reserve_count = int(reserve_count)

        # Values below are the supplied algorithm defaults, converted from its
        # local plotting coordinates into the current Gazebo map scale.
        self.sensor_radius = 12.0 * self.scale
        self.normal_ring_radius = 4.0 * self.scale
        self.guard_arc_radius = 5.2 * self.scale
        self.support_guard_radius = 4.2 * self.scale
        self.support_arc_half_angle = math.radians(65.0)
        self.support_arc_rear_offset = math.pi
        self.guard_arc_half_angle = math.radians(32.0)
        self.max_guard_arc_half_angle = math.radians(75.0)
        self.minimum_guard_spacing = 1.05 * self.scale
        self.blocker_ratio = 0.38
        self.blocker_r_min = 2.2 * self.scale
        self.blocker_r_max = 4.0 * self.scale
        self.core_arrival_tolerance = 0.24 * self.scale
        self.wing_arrival_tolerance = 0.35 * self.scale
        self.wing_ready_ratio = 0.80

        self.detected = False
        self._assignment_signature: Tuple[str, ...] = ()
        self._assignments: Dict[str, Tuple[str, int]] = {}

    def reset(self) -> None:
        self.detected = False
        self._assignment_signature = ()
        self._assignments = {}

    @staticmethod
    def _minimum_cost_assignment(cost_matrix: np.ndarray) -> List[int]:
        """O(n^3) rectangular Hungarian assignment from the supplied code."""
        costs = np.asarray(cost_matrix, dtype=float)
        if costs.ndim != 2:
            raise ValueError('cost_matrix must be two-dimensional')
        rows, columns = costs.shape
        if rows == 0:
            return []
        if rows > columns:
            raise ValueError('there must be at least as many candidates as slots')
        if not np.all(np.isfinite(costs)):
            raise ValueError('cost_matrix must contain finite values')

        u = np.zeros(rows + 1, dtype=float)
        v = np.zeros(columns + 1, dtype=float)
        matched_row = np.zeros(columns + 1, dtype=int)
        predecessor = np.zeros(columns + 1, dtype=int)
        for row in range(1, rows + 1):
            matched_row[0] = row
            minimum = np.full(columns + 1, math.inf, dtype=float)
            used = np.zeros(columns + 1, dtype=bool)
            column0 = 0
            while True:
                used[column0] = True
                active_row = matched_row[column0]
                delta = math.inf
                column1 = 0
                for column in range(1, columns + 1):
                    if used[column]:
                        continue
                    reduced = costs[active_row - 1, column - 1] - u[active_row] - v[column]
                    if reduced < minimum[column] - EPS:
                        minimum[column] = reduced
                        predecessor[column] = column0
                    if minimum[column] < delta - EPS:
                        delta = minimum[column]
                        column1 = column
                for column in range(columns + 1):
                    if used[column]:
                        u[matched_row[column]] += delta
                        v[column] -= delta
                    else:
                        minimum[column] -= delta
                column0 = column1
                if matched_row[column0] == 0:
                    break
            while True:
                column1 = predecessor[column0]
                matched_row[column0] = matched_row[column1]
                column0 = column1
                if column0 == 0:
                    break

        assignment = [-1] * rows
        for column in range(1, columns + 1):
            row = matched_row[column]
            if row:
                assignment[row - 1] = column - 1
        if any(column < 0 for column in assignment):
            raise RuntimeError('minimum-cost assignment is incomplete')
        return assignment

    def _blocker_point(
        self, protected: Sequence[float], threat: Sequence[float]
    ) -> Tuple[np.ndarray, float]:
        own = _xy(protected)
        delta = _xy(threat) - own
        distance = float(np.linalg.norm(delta))
        direction = _normalize(delta)
        if distance <= EPS:
            distance = 1.0
        requested = float(np.clip(
            self.blocker_ratio * distance, self.blocker_r_min, self.blocker_r_max
        ))
        radius = min(max(distance * 1e-9, requested), distance * (1.0 - 1e-9))
        return own + direction * radius, radius / distance

    def _guard_arc_geometry(self, wing_count: int) -> Tuple[float, float]:
        if wing_count <= 1:
            return self.guard_arc_radius, 0.0
        half_angle = min(
            self.max_guard_arc_half_angle,
            max(self.guard_arc_half_angle, math.radians(8.0) * (wing_count - 1)),
        )
        angular_step = 2.0 * half_angle / max(wing_count - 1, 1)
        radius = max(
            self.guard_arc_radius,
            self.minimum_guard_spacing
            / (2.0 * max(math.sin(angular_step / 2.0), 1e-6)),
        )
        return radius, half_angle

    def _wing_goals(
        self, protected: Sequence[float], threat_direction: np.ndarray, count: int
    ) -> List[np.ndarray]:
        if count <= 0:
            return []
        radius, half_angle = self._guard_arc_geometry(count)
        angles: Iterable[float] = (
            [0.0] if count == 1 else np.linspace(-half_angle, half_angle, count)
        )
        lateral = _rotate90(threat_direction)
        own = _xy(protected)
        return [
            own + radius * (
                math.cos(phi) * threat_direction + math.sin(phi) * lateral
            )
            for phi in angles
        ]

    def _support_goals(
        self, protected: Sequence[float], threat_direction: np.ndarray, count: int
    ) -> List[np.ndarray]:
        if count <= 0:
            return []
        center = np.array([
            math.cos(self.support_arc_rear_offset) * threat_direction[0]
            - math.sin(self.support_arc_rear_offset) * threat_direction[1],
            math.sin(self.support_arc_rear_offset) * threat_direction[0]
            + math.cos(self.support_arc_rear_offset) * threat_direction[1],
        ])
        lateral = _rotate90(center)
        angles: Iterable[float] = (
            [0.0]
            if count == 1
            else np.linspace(-self.support_arc_half_angle, self.support_arc_half_angle, count)
        )
        own = _xy(protected)
        return [
            own + self.support_guard_radius * (
                math.cos(phi) * center + math.sin(phi) * lateral
            )
            for phi in angles
        ]

    @staticmethod
    def _mixed_order(vehicle_ids: Iterable[str]) -> List[str]:
        ids = set(vehicle_ids)
        uavs = sorted(value for value in ids if value.startswith('uav_'))
        usvs = sorted(value for value in ids if value.startswith('usv_'))
        result: List[str] = []
        for index in range(max(len(uavs), len(usvs))):
            if index < len(uavs):
                result.append(uavs[index])
            if index < len(usvs):
                result.append(usvs[index])
        result.extend(sorted(ids - set(result)))
        return result

    def _normal_goals(
        self, protected: Sequence[float], vehicle_ids: Iterable[str], yaw: float
    ) -> Dict[str, np.ndarray]:
        ordered = self._mixed_order(vehicle_ids)
        own = _xy(protected)
        count = len(ordered)
        return {
            vehicle_id: own + self.normal_ring_radius * np.array([
                math.cos(yaw + 2.0 * math.pi * index / count),
                math.sin(yaw + 2.0 * math.pi * index / count),
            ])
            for index, vehicle_id in enumerate(ordered)
        } if count else {}

    @staticmethod
    def _relative_speed(vehicle_id: str) -> float:
        return 0.28 if vehicle_id.startswith('uav_') else 0.15

    def _assign(
        self,
        protected: Sequence[float],
        blocker: np.ndarray,
        threat_direction: np.ndarray,
        vehicle_positions: Mapping[str, Sequence[float]],
    ) -> None:
        vehicle_ids = sorted(vehicle_positions)
        usable_count = max(0, len(vehicle_ids) - self.reserve_count)
        direct_quota = min(len(vehicle_ids), max(1, usable_count // 2))

        reserve_ids: List[str] = []
        if self.reserve_count:
            reserve_ids = sorted(
                vehicle_ids,
                key=lambda vehicle_id: _distance(vehicle_positions[vehicle_id], blocker)
                / (self._relative_speed(vehicle_id) + EPS),
                reverse=True,
            )[:self.reserve_count]
        candidates = [value for value in vehicle_ids if value not in reserve_ids]
        if not candidates:
            self._assignments = {
                vehicle_id: ('reserve', slot)
                for slot, vehicle_id in enumerate(reserve_ids)
            }
            return

        core_id = min(
            candidates,
            key=lambda vehicle_id: _distance(vehicle_positions[vehicle_id], blocker)
            / (self._relative_speed(vehicle_id) + EPS),
        )
        assignments: Dict[str, Tuple[str, int]] = {core_id: ('core', 0)}
        remaining = [value for value in candidates if value != core_id]

        wing_count = min(max(0, direct_quota - 1), len(remaining))
        wing_goals = self._wing_goals(protected, threat_direction, wing_count)
        if wing_goals:
            cost = np.zeros((len(wing_goals), len(remaining)), dtype=float)
            for row, goal in enumerate(wing_goals):
                for column, vehicle_id in enumerate(remaining):
                    cost[row, column] = _distance(vehicle_positions[vehicle_id], goal)
            columns = self._minimum_cost_assignment(cost)
            wing_ids = []
            for slot, column in enumerate(columns):
                vehicle_id = remaining[column]
                assignments[vehicle_id] = ('wing', slot)
                wing_ids.append(vehicle_id)
            remaining = [value for value in remaining if value not in wing_ids]

        support_goals = self._support_goals(protected, threat_direction, len(remaining))
        if support_goals:
            cost = np.zeros((len(support_goals), len(remaining)), dtype=float)
            for row, goal in enumerate(support_goals):
                for column, vehicle_id in enumerate(remaining):
                    cost[row, column] = _distance(vehicle_positions[vehicle_id], goal)
            columns = self._minimum_cost_assignment(cost)
            for slot, column in enumerate(columns):
                assignments[remaining[column]] = ('support', slot)

        for slot, vehicle_id in enumerate(reserve_ids):
            assignments[vehicle_id] = ('reserve', slot)
        self._assignments = assignments
        self._assignment_signature = tuple(vehicle_ids)

    def plan(
        self,
        *,
        protected_position: Sequence[float],
        threat_position: Optional[Sequence[float]],
        vehicle_positions: Mapping[str, Sequence[float]],
        protected_yaw: float,
        uav_altitude: float,
    ) -> EscortGuardPlan:
        protected = tuple(float(value) for value in protected_position[:3])
        vehicles = {
            str(vehicle_id): tuple(float(value) for value in position[:3])
            for vehicle_id, position in vehicle_positions.items()
            if position is not None and len(position) >= 3
        }
        threat_distance = (
            _distance(protected, threat_position) if threat_position is not None else math.inf
        )
        if threat_position is not None and threat_distance <= self.sensor_radius + EPS:
            self.detected = True

        heading = float(protected_yaw)
        roles: Dict[str, str]
        goal_xy: Dict[str, np.ndarray]
        blocker = None
        blocker_t = math.nan
        core_ready = False
        wing_ready_ratio = 0.0

        if not self.detected or threat_position is None:
            self._assignments = {}
            self._assignment_signature = ()
            roles = {vehicle_id: 'escort' for vehicle_id in vehicles}
            goal_xy = self._normal_goals(protected, vehicles, protected_yaw)
            if threat_position is None:
                reason = 'waiting for authoritative enemy pose; maintaining mixed escort ring'
            else:
                reason = (
                    'enemy outside sensor radius: %.1f m > %.1f m'
                    % (threat_distance, self.sensor_radius)
                )
            phase = 'NORMAL_ESCORT'
        else:
            threat_delta = _xy(threat_position) - _xy(protected)
            threat_direction = _normalize(threat_delta)
            heading = math.atan2(threat_direction[1], threat_direction[0])
            blocker, blocker_t = self._blocker_point(protected, threat_position)
            signature = tuple(sorted(vehicles))
            if signature != self._assignment_signature:
                self._assign(protected, blocker, threat_direction, vehicles)

            roles = {
                vehicle_id: self._assignments.get(vehicle_id, ('escort', 0))[0]
                for vehicle_id in vehicles
            }
            wing_ids = [
                vehicle_id for vehicle_id, value in self._assignments.items()
                if value[0] == 'wing'
            ]
            support_ids = [
                vehicle_id for vehicle_id, value in self._assignments.items()
                if value[0] == 'support'
            ]
            reserve_ids = [
                vehicle_id for vehicle_id, value in self._assignments.items()
                if value[0] == 'reserve'
            ]
            wing_goals = self._wing_goals(protected, threat_direction, len(wing_ids))
            support_goals = self._support_goals(protected, threat_direction, len(support_ids))
            normal_goals = self._normal_goals(protected, vehicles, protected_yaw)
            goal_xy = {}
            for vehicle_id, (role, slot) in self._assignments.items():
                if role == 'core':
                    goal_xy[vehicle_id] = blocker
                elif role == 'wing' and slot < len(wing_goals):
                    goal_xy[vehicle_id] = wing_goals[slot]
                elif role == 'support' and slot < len(support_goals):
                    goal_xy[vehicle_id] = support_goals[slot]
                elif role == 'reserve':
                    rear = -threat_direction
                    lateral = _rotate90(rear)
                    offsets = (
                        [self.normal_ring_radius * rear]
                        if len(reserve_ids) == 1
                        else [
                            self.normal_ring_radius * _normalize(rear + 0.55 * lateral),
                            self.normal_ring_radius * _normalize(rear - 0.55 * lateral),
                        ]
                    )
                    goal_xy[vehicle_id] = _xy(protected) + offsets[min(slot, len(offsets) - 1)]
                else:
                    goal_xy[vehicle_id] = normal_goals[vehicle_id]

            core_ids = [value for value, role in roles.items() if role == 'core']
            if core_ids:
                core_ready = _distance(vehicles[core_ids[0]], blocker) <= self.core_arrival_tolerance
            arrived = sum(
                _distance(vehicles[vehicle_id], goal_xy[vehicle_id])
                <= self.wing_arrival_tolerance
                for vehicle_id in wing_ids
            )
            wing_ready_ratio = arrived / len(wing_ids) if wing_ids else 1.0
            ready = core_ready and wing_ready_ratio + EPS >= self.wing_ready_ratio
            phase = 'GUARDING' if ready else 'FORMING'
            reason = (
                'new escort guard formation ready'
                if ready else
                'forming blocker/wing/support guard assignment from Gazebo poses'
            )

        targets: Dict[str, Target] = {}
        for vehicle_id, point in goal_xy.items():
            altitude = (
                float(uav_altitude)
                if vehicle_id.startswith('uav_')
                else float(protected[2])
            )
            targets[vehicle_id] = (
                float(point[0]), float(point[1]), altitude, heading
            )

        details: Dict[str, object] = {
            'algorithmSource': self.SOURCE,
            'algorithmMode': 'REAL_GAZEBO_POSE',
            'scale': self.scale,
            'detected': self.detected,
            'threatDistance': None if not math.isfinite(threat_distance) else threat_distance,
            'sensorRadius': self.sensor_radius,
            'roles': roles,
            'coreReady': core_ready,
            'wingReadyRatio': wing_ready_ratio,
        }
        if blocker is not None:
            details['blockerPoint'] = [float(blocker[0]), float(blocker[1]), float(protected[2])]
            details['blockerT'] = float(blocker_t)
        return EscortGuardPlan(
            phase=phase,
            reason=reason,
            detected=self.detected,
            targets=targets,
            roles=roles,
            details=details,
        )
