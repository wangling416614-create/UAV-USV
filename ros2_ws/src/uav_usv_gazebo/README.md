# uav_usv_gazebo

## Unity geometry parity

The active `heterogeneous_332` world uses the Unity product models as the
geometry reference:

- M3-F900 UAV visual/rotor envelope: `1.20 x 1.20 x 0.55 m`
- USV-M1500 visual envelope: `1.50 x 1.10 x 0.60 m`
- USV-M1500 collision envelope: `1.50 x 1.10 x 0.54 m`
- Friendly command ship: the same 10-part `13.5 m` Unity procedural model
- Enemy patrol ship: the same 17-part nominal `16.5 x 5.8 m` Unity model
- USV camera mount: `x=0.48, y=0.00, z=0.52 m`
- USV Mid-360 mount: `x=-0.03, y=0.13, z=0.48 m`

The Catalina mountain is not an approximation: both renderers use the same
`scene.gltf`, `scene.bin` and active land-cutout texture. The unchanged local
Unity reference uses its original `0.024` mesh scale and `0.18` presentation
scale; Gazebo independently uses an effective `0.018` mesh scale. Gazebo
additionally uses a generated 1,774-triangle terrain
collision OBJ with explicit vertex normals for DART/ODE; flat ocean triangles
are removed. The legacy mountain-shore box proxies were removed after the
terrain was resized; only the outer-ocean safety boundary remains. The
fleet launcher replaces only the visible PX4 x500 meshes with the Unity
M3-F900 primitives; PX4 mass, collisions, sensors, joints and motor plugins
remain active.

Local Unity and the embedded frontend WebGL retain the original `0.18`
presentation scale. Product vehicle meshes remain 1:1. Gazebo keeps world
coordinates in physical metres, while its mountain and UAV base are reduced
independently for clearer vehicle presentation. Vehicle geometry and sensor
extrinsics remain shared 1:1.

Ownership: simulation environment team.

This package owns the runnable maritime world, environmental models, weather,
coastline wrapper, and Gazebo system plugins. Control and mission nodes remain
in `uav_usv_sim`.

## Contents

- `worlds/default.sdf`: complete ocean environment with coastline, weather,
  fog, wind field, obstacles, moving vessels, and offshore facilities.
- `worlds/vrx_sydney_regatta_custom.sdf`: isolated VRX-style Sydney Regatta
  environment owned by this package, with the custom platform, reefs, harbor,
  lighthouses, buoys, and animated full-ocean wave surface.
- `plugins/BoatWaveFollower.cc`: wave-following motion for boats and floating objects.
- `plugins/DroneDeckFollower.cc`: parked-UAV deck attachment system.
- `config/sydney_coast.model.*`: local wrapper for the Sydney Regatta coastline.
- `models/simple_boat`: sensor-equipped USV and UAV landing deck.
- `models/waves`: animated Gerstner-wave surface.
- `models/medium_buoy` and `models/green_channel_buoy`: swaying channel marks.
- `models/target_vessel`: automatically moving maritime traffic vessel.

- `models/shore_platform`: collidable shoreline UAV helipad with an access pier.
- `models/rock_outcrop`: collidable marine rock cluster for obstacle courses.
- `models/green_channel_buoy`: illuminated starboard channel mark.
- `models/aquaculture_cage`: floating net pen with submerged net walls.
- `models/floating_barrel`: weathered oil-drum obstacle.
- `models/life_raft`: abandoned inflatable emergency raft.
- `models/driftwood`: floating logs and broken planks.
- `models/marina_pier`: illuminated T-head timber pier.
- `models/offshore_wind_turbine`: rotating offshore wind turbine with warning lights.
- `models/harbor_breakwater`: illuminated U-shaped harbor and concrete quay.
- `models/harbor_tug`: moving rescue tug with particle wake.
- `models/fishing_boat`: moving fishing vessel with outriggers and particle wake.
- `models/person_overboard`: floating casualty for rescue-perception tests.

## Run

This workspace currently targets ROS 2 Humble. If your team uses another ROS 2
distribution, make sure the Gazebo dependency versions in `CMakeLists.txt`
match your local installation.

```bash
cd ~/UAV_USV
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_usv_gazebo
source install/setup.bash
ros2 run uav_usv_gazebo run_gz_world.sh
```

Run the isolated VRX-style world without changing the default world:

```bash
ros2 run uav_usv_gazebo run_gz_world.sh vrx_sydney_regatta_custom
```

PX4 asset synchronization is also owned by this package:

```bash
export PX4_DIR=/path/to/PX4-Autopilot
ros2 run uav_usv_gazebo sync_to_px4.sh
```

The first run may download the Sydney Regatta coastline into
`/var/tmp/UAV_USV_gz_fuel` and generate local assets under
`/var/tmp/UAV_USV_assets`.

## 第一阶段任务

本包负责海面世界、动态目标船和标准测试场景。
