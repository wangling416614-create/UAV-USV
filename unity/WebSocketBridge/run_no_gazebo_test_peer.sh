#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_WORKSPACE="${UAV_USV_WS:-$HOME/project/UAV_USV}"

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WORKSPACE/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
exec python3 "$SCRIPT_DIR/mock_no_gazebo_peer.py"
