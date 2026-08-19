#!/usr/bin/env bash
set -euo pipefail

RUNTIME_DIR="/tmp/uav_usv_platform"
ROS_LOG_DIR="$RUNTIME_DIR/logs"
DEMO_PID_FILE="$RUNTIME_DIR/demo.pid"
BRIDGE_PID_FILE="$RUNTIME_DIR/bridge.pid"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PLATFORM_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
UAV_USV_ROS_WS="${UAV_USV_WS:-$HOME/project/UAV_USV}"
UNITY_WS_DIR="${UNITY_WS_DIR:-$PLATFORM_ROOT/../unity_ws}"
FLEET_BRIDGE="$UNITY_WS_DIR/WebSocketBridge/run_fleet_bridge.sh"

mkdir -p "$ROS_LOG_DIR"

is_managed_running() {
    local pid_file="$1"
    [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null
}

is_external_running() {
    local pattern="$1"
    pgrep -f "$pattern" >/dev/null 2>&1
}

start_process() {
    local name="$1"
    local pid_file="$2"
    local command="$3"

    if is_managed_running "$pid_file"; then
        echo "$name=managed:$(cat "$pid_file")"
        return
    fi

    : >"$ROS_LOG_DIR/$name.log"
    nohup setsid bash -lc "echo '[platform] starting $name at '\"\$(date -Is)\"; $command" >"$ROS_LOG_DIR/$name.log" 2>&1 < /dev/null &
    local pid=$!
    echo "$pid" > "$pid_file"
    for _ in {1..10}; do
        if is_managed_running "$pid_file"; then
            echo "$name=started:$pid"
            return
        fi
        sleep 0.5
    done
    echo "$name=failed"
    tail -n 40 "$ROS_LOG_DIR/$name.log" || true
    return 1
}

start_runtime() {
    if [[ ! -f "$UAV_USV_ROS_WS/install/setup.bash" ]]; then
        echo "ROS workspace is not built: $UAV_USV_ROS_WS" >&2
        return 1
    fi
    if [[ ! -x "$FLEET_BRIDGE" ]]; then
        echo "Fleet WebSocket bridge is missing or not executable: $FLEET_BRIDGE" >&2
        return 1
    fi

    local setup="cd '$UAV_USV_ROS_WS' && source /opt/ros/humble/setup.bash && source '$UAV_USV_ROS_WS/install/setup.bash'"
    local px4_dir="${PX4_DIR:-$HOME/PX4-Autopilot}"
    local gazebo_gui="${GAZEBO_GUI:-false}"
    # Keep the full 3+3 simulation below the machine's memory/CPU saturation
    # point. Higher rates remain available through the environment overrides.
    local uav_camera_rate="${UAV_CAMERA_RATE:-4.0}"
    local mid360_update_rate="${MID360_UPDATE_RATE:-2.0}"
    local mid360_voxel_size="${MID360_VOXEL_SIZE:-0.35}"
    # Three RGL Mid-360 instances exhaust this demonstration GPU after a
    # Gazebo world reset. Keep one real point-cloud payload for the UI while
    # cameras and the lightweight safety lidars remain enabled on all boats.
    local mid360_vehicle_ids="${MID360_VEHICLE_IDS:-usv_01}"

    if is_external_running "fleet_dynamic_capture_live_perception.launch.py"; then
        echo "demo=external"
    else
        # PX4 SITL defaults SIM_BAT_DRAIN to 60 seconds. That test-oriented
        # default triggers low-battery RTL during a normal classroom demo and
        # can make a vehicle fight the capture setpoint, roll over, and fall.
        # Keep the real PX4 controller/failsafes, but disable synthetic battery
        # depletion for this persistent demonstration runtime.
        start_process "demo" "$DEMO_PID_FILE" "$setup && PX4_PARAM_SIM_BAT_DRAIN=0 ros2 launch uav_usv_bringup fleet_dynamic_capture_live_perception.launch.py start_rviz:=false gazebo_gui:='$gazebo_gui' enable_console:=false uav_camera_rate:='$uav_camera_rate' mid360_vehicle_ids:='$mid360_vehicle_ids' mid360_update_rate:='$mid360_update_rate' mid360_voxel_size:='$mid360_voxel_size' px4_dir:='$px4_dir'"
    fi

    if is_external_running "unity_websocket_bridge.py"; then
        echo "bridge=external"
    else
        start_process "bridge" "$BRIDGE_PID_FILE" "UAV_USV_WS='$UAV_USV_ROS_WS' '$FLEET_BRIDGE'"
    fi
}

cleanup_runtime_leftovers() {
    pkill -f "gz sim .*PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf" >/dev/null 2>&1 || true
    pkill -f "make px4_sitl gz_x500" >/dev/null 2>&1 || true
    pkill -f "PX4-Autopilot.*/bin/px4" >/dev/null 2>&1 || true
    pkill -f "parameter_bridge" >/dev/null 2>&1 || true
    pkill -f "rviz2" >/dev/null 2>&1 || true
    pkill -f "cooperative_lighthouse_mission" >/dev/null 2>&1 || true
}

stop_process() {
    local name="$1"
    local pid_file="$2"
    if ! is_managed_running "$pid_file"; then
        rm -f "$pid_file"
        echo "$name=not-managed"
        return
    fi

    local pid
    pid="$(cat "$pid_file")"
    # ros2 launch can leave re-parented Nav2 lifecycle processes alive after
    # the shell leader exits. Track the entire session/process group instead
    # of checking only the original leader PID.
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in {1..10}; do
        kill -0 -- "-$pid" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 -- "-$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
    for _ in {1..10}; do
        kill -0 -- "-$pid" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 -- "-$pid" 2>/dev/null; then
        kill -KILL -- "-$pid" 2>/dev/null || true
    fi
    rm -f "$pid_file"
    echo "$name=stopped"
}

status_runtime() {
    if is_managed_running "$DEMO_PID_FILE"; then echo "demo=managed:$(cat "$DEMO_PID_FILE")";
    elif is_external_running "fleet_dynamic_capture_live_perception.launch.py"; then echo "demo=external";
    else echo "demo=stopped"; fi

    if is_managed_running "$BRIDGE_PID_FILE"; then echo "bridge=managed:$(cat "$BRIDGE_PID_FILE")";
    elif is_external_running "unity_websocket_bridge.py"; then echo "bridge=external";
    else echo "bridge=stopped"; fi
}

case "${1:-status}" in
    start) start_runtime ;;
    stop)
        stop_process "bridge" "$BRIDGE_PID_FILE"
        stop_process "demo" "$DEMO_PID_FILE"
        cleanup_runtime_leftovers
        ;;
    status) status_runtime ;;
    *) echo "Usage: $0 {start|stop|status}" >&2; exit 2 ;;
esac
