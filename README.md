# UAV-USV Unity WebSocket 运行说明

本仓库是 UAV-USV 系统的完整源码仓库，包含 ROS 2、Unity、WebSocket
Gateway、前端、后端、文档和统一运行脚本：

```text
UAV-USV/
├── ros2_ws/src/
├── unity/{Assets,Packages,ProjectSettings,WebSocketBridge}/
├── platform/{frontend,backend,scripts}/
├── docs/
└── scripts/
```

首次克隆后先构建 ROS 2 工作区：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

随后可从仓库根目录运行 `./scripts/start-runtime.sh`，停止时运行
`./scripts/stop-runtime.sh`。MySQL、PX4、Node.js、Java 和 Unity 编辑器等外部依赖
需要按本机环境另行安装。

## 当前默认：与 ROS 3+3 舰队场景一致

Unity 场景现在默认对应仓库的
`uav_usv_bringup/fleet_dynamic_capture.launch.py` 与
`uav_usv_gazebo/worlds/heterogeneous_332.sdf`。它同步
`usv_01..03`、`uav_01..03`、`friendly_ship`、`enemy_ship`、舰队在线状态和任务阶段，
并使用与 Gazebo 相同的 ENU 原点、初始位姿和水平尺寸
`30.8 × 10.5 m` 的舰队停机坪。

### Unity / Gazebo 严格视觉基准

本地 Unity 的运行时世界是唯一视觉基准。前端内嵌 Unity WebGL 直接由同一个
`UavUsvDemo.unity`、同一套脚本和 Resources 构建；Gazebo 按 Unity primitive 的
局部尺寸、形状、位姿和 RGBA 数值逐项映射：

- Catalina 两端共用原始 glTF，Gazebo mesh 缩放为 `0.024`、锚点为 `(0, 0, -0.8)`；
  Unity 再统一施加 `0.18` 展示坐标变换。
- 舰队停机坪恢复完整 `44 × 15 × 20 m` 结构，三块圆形坪半径均为 `5.5 m`，
  不再使用 `70%` 横向缩放。
- M3-F900 的物理/碰撞尺寸为 `1.20 × 1.20 × 0.55 m`，USV-M1500 为
  `1.50 × 1.10 × 0.60 m`；Gazebo 可见外壳统一放大 `1 / 0.18 = 5.5556`
  倍，以匹配 Unity 中载具相对友方船、敌方船的屏幕比例。碰撞、传感器和控制仍为
  真实 `1:1`，三艘 USV 与 Unity 一样使用同一红色识别盖板。
- 岸基指挥站、友方船、敌方船均按 `SimulationBootstrap` 中每个 box、cylinder、
  cone/ellipsoid 的数值构建，颜色使用相同线性 RGBA 参数。
- Gazebo 默认相机为 `(-190, -455, 48, 0, 0.217, 0.974)`；Unity 使用对应的
  `0.18` 展示坐标，因此构图保持一致。

Unity WebGL 更新后如果仍显示旧尺度，请在浏览器中按 `Ctrl + Shift + R`
强制刷新缓存。

推荐启动：

```bash
cd /home/wl/project/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_usv_bringup fleet_dynamic_capture.launch.py
```

另开终端：

```bash
cd /home/wl/UAV_USV_Unity_WebSocket_20260624/WebSocketBridge
./run_fleet_bridge.sh
```

最后用浏览器打开前端内嵌 Unity WebGL；正式运行不需要启动本地 Unity 编辑器。
左上角应显示
`ROS fleet v2`、`USV 3/3 · UAV 3/3 · 已就绪`。运动始终由后端/ROS
任务节点控制，Unity 只负责一致的三维显示。内嵌 WebGL 不再运行本地任务、路径规划或载具控制。

当前控制链路固定为：

```text
前端控制命令 → Spring Boot 后端 → ROS WebSocket 网关 → ROS 规划/控制
                                                ↓ /fleet/command_ack
前端实时状态 ← Spring Boot 后端 ← ROS WebSocket 网关

Unity WebGL ← ROS WebSocket 网关 ← ROS / Gazebo 真实位姿
```

Unity 不接收前端算法生成的本地位姿；系统总览、二维态势和内嵌 Unity 都以同一份
ROS/Gazebo 位姿为准。`GB_SFLA_CS` 使用 ROS `capture_manager` 围捕规划，
`ESCORT_GUARD` 使用网关节点中的 ROS 3+3 护航编队规划，并统一通过
`/fleet/control_lease`、`/fleet/command` 和 `/fleet/command_ack` 执行。

### 平台完整启动（推荐）

终端 1启动 ROS/Gazebo、3+3 任务节点和正式 WebSocket 网关：

```bash
cd /home/wl/UAV_USV_Unity_WebSocket_20260624/UnityProject/UAV_USV_Platform
./scripts/uav-usv-runtime.sh start
```

终端 2启动后端：

```bash
cd /home/wl/UAV_USV_Unity_WebSocket_20260624/UnityProject/UAV_USV_Platform/backend
MYSQL_USERNAME='uav_usv_platform_app' \
MYSQL_PASSWORD='<YOUR_MYSQL_PASSWORD>' \
BOOTSTRAP_ADMIN_PASSWORD='<YOUR_ADMIN_PASSWORD>' \
./mvnw spring-boot:run
```

其中 `MYSQL_PASSWORD` 是 MySQL 项目账号密码，
`BOOTSTRAP_ADMIN_PASSWORD` 是平台默认 `admin` 账号的登录密码，两者都必须设置。

终端 3启动前端：

```bash
cd /home/wl/UAV_USV_Unity_WebSocket_20260624/UnityProject/UAV_USV_Platform/frontend
npm run dev
```

浏览器打开 `http://127.0.0.1:5174`。正式网关实现只有一份，位于
`UnityProject/unity_ws/WebSocketBridge`；根目录 `WebSocketBridge/*.sh` 是兼容入口，
会自动转到这份实现，不再运行旧协议桥。

### 从 ROS 主动启动任务

ROS 启动围捕后，Gazebo 位姿会自动同步到前端和 Unity：

```bash
ros2 topic pub --once /fleet/base/operator_action std_msgs/msg/String \
  "{data: 'CAPTURE:enemy_ship'}"
```

ROS 启动 3+3 护航：

```bash
ros2 topic pub --once /fleet/base/operator_action std_msgs/msg/String \
  "{data: 'ESCORT:friendly_ship'}"
```

护航暂停、继续和取消：

```bash
ros2 topic pub --once /fleet/base/operator_action std_msgs/msg/String "{data: 'HOLD_ESCORT'}"
ros2 topic pub --once /fleet/base/operator_action std_msgs/msg/String "{data: 'ESCORT:friendly_ship'}"
ros2 topic pub --once /fleet/base/operator_action std_msgs/msg/String "{data: 'CANCEL_ESCORT'}"
```

平台前端点击开始/暂停/继续/终止时会生成同样的 ROS 动作，并等待任务权威状态确认；
单机起飞、悬停、返航、停止等命令会等待对应 agent 在
`/fleet/command_ack` 上返回真实执行确认。

### 任务 ACK 与 ROS 主动状态入库

任务命令不会在网关刚发布 `/fleet/base/operator_action` 时就报告成功。网关会保留命令，
等待下面的权威状态达到命令预期状态后才返回 `STATUS_SUCCEEDED (3)`：

- `GB_SFLA_CS` 围捕：监听真实 `/capture/state`，确认
  `RUNNING / PAUSED / COMPLETED / FAILED / CANCELLED`。
- `ESCORT_GUARD` 护航：由网关内 ROS 护航状态机确认相同的任务状态。
- 若 ROS 先进入不符合命令预期的 `COMPLETED / FAILED / CANCELLED` 终态，立即返回
  `STATUS_FAILED (5)`，并附带真实终态原因。
- 超过 `mission_ack_timeout`（默认 12 秒）仍未确认时返回
  `STATUS_FAILED (5)`，后端不会提前把任务改成成功状态。

网关同时发送 `mission_state` 帧，后端转发为浏览器实时主题 `mission.state`。
从 ROS 终端主动发布任务动作时，状态帧不包含平台命令关联键，后端会：

1. 优先推进同算法当前的 `PENDING / RUNNING / PAUSED` 批次；
2. 若 ROS 已是 `RUNNING` 且没有开放批次，从该算法优先级最高的 READY 任务创建
   `runtimeInstanceId=ros-external` 的批次；
3. 同步任务与批次状态并写入 `MissionEventType.ROS` 事件。

平台发起的状态帧带 `confirmedCommands`，只由原命令 ACK 协调器更新数据库，避免同一状态被
两套状态机重复写入。

### WebGL 完整四元数

位姿链路保留 ROS ENU 四元数，不再只向 WebGL 传偏航角。Unity 消息中每个 agent 包含：

```json
{
  "orientation": [0.0, 0.0, 0.0, 1.0],
  "yawDegrees": 0.0
}
```

`orientation` 顺序固定为 `[x, y, z, w]`，Unity `PlatformBridge` 完成 ROS ENU 到 Unity
坐标系的旋转转换；`yawDegrees` 只用于兼容旧 WebGL 构建。这样无人机的滚转、俯仰、偏航
都会随 ROS/Gazebo 位姿同步。

### 多设备雷达与点云

正式网关支持用成对数组配置任意数量的传感器，设备 ID 与话题必须一一对应：

```text
radar_device_ids       ↔ radar_tracks_topics
pointcloud_device_ids  ↔ pointcloud_topics
```

`run_fleet_bridge.sh` 默认订阅融合雷达 `fleet_fused`、`usv_01..03` 独立雷达，以及
`usv_01..03` 的 Mid360 点云。发出的 `radar_frame.frame.device_id`、
`pointcloud_frame.frame.data.vehicle_id` 和 `stream_id` 均按来源设备填写，不再固定为
`usv_01`。

ROS 场景现已默认给 `usv_01..03` 分别创建独立的 RGL Mid-360 实体传感器、Gazebo→ROS
点云桥、预处理器和 TF。三艘艇会分别发布：

```text
/perception/usv_01/mid360/preview
/perception/usv_02/mid360/preview
/perception/usv_03/mid360/preview
```

可用 `mid360_vehicle_ids:=usv_01,usv_02,usv_03` 调整载荷艇列表；默认已经包含三艘艇。

2026-08-17 实测三路 `/preview` 单帧分别为 2279、2368、2576 点；正式网关收到并下采样
为 1078、1169、1188 点的非空 `pointcloud_frame`，三帧 `vehicle_id` 分别为
`usv_01`、`usv_02`、`usv_03`。

### 新护航守卫算法（真实 Gazebo 位姿版）

`ESCORT_GUARD` 已由固定 USV/UAV 双环替换为
`护航守卫_三维单目标_Python39(1).py` 的实时算法适配版。正式运行时不启动原文件的
Matplotlib 和内部虚拟机艇，而是读取 Gazebo 的 `friendly_ship`、`enemy_ship`、
`uav_01..03`、`usv_01..03` 位姿，计算核心阻断、翼侧弧和后向支援目标，再通过真实
`/fleet/command` 下发。

默认 `escort_algorithm_scale=7.0`，把原算法局部坐标的感知半径 12、普通环半径 4、
翼侧半径 5.2 和支援半径 4.2 映射为当前 Gazebo 场景的 84、28、36.4、29.4 米；默认
`escort_reserve_count=0`，3+3 编队分为 1 个核心、2 个翼侧、3 个支援。参数位于
`UnityProject/unity_ws/WebSocketBridge/run_fleet_bridge.sh`。

软件任务汇报和验收项见：
`UnityProject/UAV_USV_Platform/docs/软件任务汇报_数据流传感器与算法验证.md`。

## 1. 当前目录

- ROS/Gazebo 项目：`/home/wl/project/UAV_USV`
- 更新前完整备份：`/home/wl/project/UAV_USV.before_zip2_20260705_211029`
- PX4：`/home/wl/PX4-Autopilot`
- Unity 项目：`/home/wl/UAV_USV_Unity_WebSocket_20260624/UnityProject/unity_ws`
- WebSocket 桥：`/home/wl/UAV_USV_Unity_WebSocket_20260624/WebSocketBridge`
- Unity 版本：`2022.3.57f1`
- Unity 场景：`Assets/Scenes/UavUsvDemo.unity`
- WebSocket：`ws://127.0.0.1:8765/uav_usv`

当前 3+3 舰队使用只读 Unity 数字孪生模式：

- Gazebo 向 Unity 同步无人艇、x500 无人机、灯塔、浮标和目标船的真实位姿。
- 路径规划在 ROS/Nav2 或 ROS 任务节点中完成，不在 Unity 中计算。
- 前端控制按钮只向 Spring Boot 后端下发命令，并等待 ROS ACK。
- Unity 只保留相机、设备跟随、轨迹叠加、传感器画面和位姿渲染。
- 即使 Unity 未连接，ROS 控制链路仍可工作；Unity 离线只影响三维可视化。

## 2. 首次安装依赖

新版完整 workspace 需要 Geographic Messages、Nav2、ros_gz_bridge、xacro 和 Vision Messages。执行：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-geographic-msgs \
  ros-humble-nav2-bringup \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-controller \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-msgs \
  ros-humble-nav2-planner \
  ros-humble-nav2-smoother \
  ros-humble-nav2-velocity-smoother \
  ros-humble-nav2-waypoint-follower \
  ros-humble-ros-gz-bridge \
  ros-humble-xacro \
  ros-humble-vision-msgs
```

安装后完整构建：

```bash
cd /home/wl/project/UAV_USV
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

当前核心 `uav_usv_sim` 已单独构建并完成联调；在安装完整依赖前，PX4/Gazebo/Unity 协同演示也可以运行。

Sydney官方资源已缓存到：

```text
/var/tmp/UAV_USV_gz_fuel
/var/tmp/UAV_USV_assets
```

如果清理了 `/var/tmp`，下一次启动会从 Gazebo Fuel 重新下载约137MB资源。

## 3. 本地 Unity（仅开发调试，正式运行跳过）

前端已内嵌构建好的 Unity WebGL，日常启动和任务演示不需要打开
Unity Hub 或点击本地场景 Play。只有修改 Unity 源码、场景或重新构建 WebGL
时才需要下面步骤：

1. 在 Unity Hub 选择“添加磁盘中的项目”。
2. 选择 `/home/wl/UAV_USV_Unity_WebSocket_20260624/UnityProject/unity_ws`。
3. 使用 Unity `2022.3.57f1` 打开。
4. 打开 `Assets/Scenes/UavUsvDemo.unity`。
5. 等待脚本编译结束。

## 4. 手动 ROS 分步启动（调试用）

正式平台优先使用前文 `scripts/uav-usv-runtime.sh start`。下面的方式仅用于
分别调试 Gazebo、PX4 或 WebSocket 网关，不要与统一运行脚本重复启动。

### 终端 1：Gazebo Server + PX4

```bash
cd /home/wl/project/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash
export PX4_DIR=/home/wl/PX4-Autopilot
export HEADLESS=1
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py
```

看到 `Ready for takeoff!` 表示启动成功。`HEADLESS=1`只启动Gazebo Server，避免GUI偶发启动失败。

### 终端 1.5：Gazebo GUI

```bash
gz sim -g
```

GUI 被关闭或卡住时，只需重新执行 `gz sim -g`，不要重启终端 1。

### 终端 2：WebSocket 桥（3+3 位姿与命令网关）

```bash
cd /home/wl/UAV_USV_Unity_WebSocket_20260624/WebSocketBridge
./run_fleet_bridge.sh
```

正常输出：

```text
Unity WebSocket bridge listening on ws://0.0.0.0:8765/uav_usv
```

### 前端 Unity WebGL：开始同步

启动后端和前端后，浏览器打开 `http://127.0.0.1:5174`。内嵌 WebGL
左上角出现下面的状态即表示正在接收 Gazebo 位姿：

```text
WebSocket pose seq ...
```

Unity 中不再选择目标、执行 A* 或发送停止命令。请在平台前端下达任务/载具命令；
后端把命令转发给 ROS，收到 ROS ACK 后更新页面状态，Gazebo 位姿随后同步到前端 WebGL。

### 终端 3：可选 ROS 协同任务

```bash
cd /home/wl/project/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_usv_sim cooperative_lighthouse_mission.launch.py
```

新版任务流程是：甲板释放、平滑起飞、船机协同前往灯塔、无人机实时追踪返船、下降、触地、重新锁定甲板并关闭电机。

同一载具只应由一个 ROS 控制节点负责，避免协同任务、键盘控制和多个 Nav2 实例同时发布速度。
WebSocket 桥保持运行即可持续向前端和 Unity 同步位姿。

## 5. 一条命令启动 ROS 演示

下面的 launch 同时启动 PX4/Gazebo server 和协同任务：

```bash
cd /home/wl/project/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash
export PX4_DIR=/home/wl/PX4-Autopilot
export HEADLESS=1
ros2 launch uav_usv_sim uav_usv_cooperation_demo.launch.py \
  px4_dir:=/home/wl/PX4-Autopilot \
  start_rviz:=false
```

随后另开终端运行 `gz sim -g`、启动 WebSocket 桥、后端和前端，然后在
浏览器打开内嵌 WebGL。若要完整看到起飞过程，优先使用第 4 节的分步方式。

## 6. 使用 ROS Nav2/MPPI 规划和执行路径

这一模式使用 ROS workspace 中的 Nav2 配置：ROS 侧全局规划器生成路径，MPPI
负责跟踪控制。Unity 不生成航点，只显示 Gazebo 返回的执行位姿。必须先完成第 2 节的依赖安装与完整构建。

终端 1先启动 PX4/Gazebo：

```bash
cd /home/wl/project/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash
export PX4_DIR=/home/wl/PX4-Autopilot
export HEADLESS=1
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py
```

终端 2启动 Nav2，不重复启动 Gazebo：

```bash
cd /home/wl/project/UAV_USV
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_usv_sim boat_nav2_navigation.launch.py start_rviz:=false
```

终端 3启动 3+3 WebSocket 网关：

```bash
cd /home/wl/UAV_USV_Unity_WebSocket_20260624/WebSocketBridge
./run_fleet_bridge.sh
```

最后从平台前端下达任务或目标命令。命令经后端和 WebSocket 网关进入 ROS/Nav2，
Nav2 规划并控制无人艇；Unity WebGL 只根据 ROS/Gazebo 回传位姿同步画面。

## 7. 其他新版功能

Nav2 无人船导航：

```bash
ros2 launch uav_usv_sim boat_nav2_navigation.launch.py
```

COLREGS 测试：

```bash
ros2 launch uav_usv_sim colregs_test_scenario.launch.py
```

这些功能需要先完成第 2 节的完整依赖安装和全 workspace 构建。

## 8. 常见问题

检查 Gazebo/PX4 进程：

```bash
ps -ef | grep -E 'px4|gz sim|ros2 launch' | grep -v grep
```

检查 WebSocket 端口：

```bash
ss -ltnp | grep 8765
```

如果 Unity 一直显示正在连接，先确认桥仍在运行。若 Windows Unity 无法访问 WSL 的 localhost，执行 `hostname -I` 获取 WSL 地址，并用下面的启动参数覆盖：

```text
--ros-ws --ros-ws-url=ws://WSL地址:8765/uav_usv
```

不要使用包含 `uav_usv` 的宽泛 `pkill -f`，它可能误杀 Unity、WebSocket 桥或其他相关进程。正常关闭时按第 9 节顺序操作。

如果右上角显示 `waiting_pose`，说明桥接器没有收到 Gazebo 无人艇位姿。检查终端 1是否仍在运行。

如果显示 `waiting_nav2`，说明选择了 `nav2` 模式，但 `NavigateToPose` 服务尚未启动。检查 Nav2 终端及第 2 节依赖。

如果无人艇收到目标后方向异常，先停止协同任务、键盘控制和其他 Nav2 实例，只保留一个控制源。

## 9. 正常关闭

如果使用平台脚本启动，先执行：

```bash
cd /home/wl/UAV_USV_Unity_WebSocket_20260624/UnityProject/UAV_USV_Platform
./scripts/uav-usv-runtime.sh stop
```

1. 停止 Unity Play。
2. 在任务终端按 `Ctrl+C`。
3. 在 WebSocket 桥终端按 `Ctrl+C`。
4. 关闭 Gazebo GUI。
5. 在 PX4/Gazebo server 终端按 `Ctrl+C`。
从
