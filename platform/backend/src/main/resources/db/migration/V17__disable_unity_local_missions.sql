UPDATE mission_task
SET execution_mode = 'ROS_GAZEBO',
    algorithm_code = 'GB_SFLA_CS',
    algorithm_version = '1.1.0-20260727'
WHERE algorithm_code = 'UNITY_SIMPLE_ENCIRCLEMENT';

UPDATE algorithm_definition
SET enabled = FALSE,
    default_for_type = FALSE,
    description = '已停用：Unity 仅显示 ROS/Gazebo 位姿，不再运行本地任务或路径规划。'
WHERE code = 'UNITY_SIMPLE_ENCIRCLEMENT';

UPDATE algorithm_definition
SET adapter_type = 'ROS_FLEET',
    description = 'ROS 3+3 舰队围捕规划；通过 /fleet/command 执行，Gazebo 位姿同步到前端与 Unity。'
WHERE code = 'GB_SFLA_CS';

UPDATE algorithm_definition
SET adapter_type = 'ROS_FLEET',
    description = 'ROS 3+3 护航编队规划；跟随保护目标 Gazebo 位姿并通过 /fleet/command 动态重规划。'
WHERE code = 'ESCORT_GUARD';
