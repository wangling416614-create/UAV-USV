UPDATE mission_task
SET execution_mode = 'ROS_GAZEBO'
WHERE execution_mode IN ('UNITY_STANDALONE', 'HYBRID_MIRROR');
