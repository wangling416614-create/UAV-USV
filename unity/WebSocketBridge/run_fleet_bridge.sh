#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

exec "$SCRIPT_DIR/run_bridge.sh" --ros-args \
  -p gazebo_pose_topic:=/world/heterogeneous_332/pose/info \
  -p control_mode:=observe \
  -p friendly_ship_name:=friendly_ship \
  -p target_vessel_name:=enemy_ship \
  -p simulation_coordinate_scale:=0.18 \
  -p max_abs_coordinate:=600.0 \
  -p enable_camera_stream:=true \
  -p camera_publish_rate:=12.0 \
  -p camera_thumbnail_rate:=2.0 \
  -p camera_jpeg_quality:=55 \
  -p camera_max_width:=640 \
  -p camera_max_height:=360 \
  -p default_camera_id:=usv_01 \
  -p enable_sensor_stream:=true \
  -p radar_tracks_topic:=/perception/lv_dot_ros2/tracks \
  -p radar_device_ids:="['fleet_fused','usv_01','usv_02','usv_03']" \
  -p radar_tracks_topics:="['/perception/lv_dot_ros2/tracks','/perception/usv_01/radar/tracks','/perception/usv_02/radar/tracks','/perception/usv_03/radar/tracks']" \
  -p pointcloud_topic:=/perception/usv_01/mid360/preview \
  -p pointcloud_device_ids:="['usv_01','usv_02','usv_03']" \
  -p pointcloud_topics:="['/perception/usv_01/mid360/preview','/perception/usv_02/mid360/preview','/perception/usv_03/mid360/preview']" \
  -p visual_detections_topic:=/perception/usv_01/camera/affiliated_detections \
  -p sensor_publish_rate:=2.0 \
  -p pointcloud_max_points:=1800 \
  -p escort_algorithm_scale:=7.0 \
  -p escort_reserve_count:=0 \
  -p usv_names:="['usv_01','usv_02','usv_03']" \
  -p uav_names:="['uav_01','uav_02','uav_03']" \
  "$@"
