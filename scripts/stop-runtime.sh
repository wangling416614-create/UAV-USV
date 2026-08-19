#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

UAV_USV_WS="$PROJECT_ROOT/ros2_ws" \
UNITY_WS_DIR="$PROJECT_ROOT/unity" \
exec "$PROJECT_ROOT/platform/scripts/uav-usv-runtime.sh" stop
