### 目录：move_cmd（运动指令/TF 广播）

**功能概述**
- 订阅轨迹或使用定时器，更新并广播主车/障碍物的 TF，实现“仿真前进”。

**节点**
- `car_move_cmd`（节点名：`car_move_cmd_node`）
  - 订阅：`/planning/local_trajectory`
  - 动作：寻找轨迹匹配点，更新主车位置与朝向，广播 TF。
  - 可选宏：`USE_ACTUAL_POS` 切换“实际位置赋值”或“匹配点赋值”。
- `obs_move_cmd`（节点名：`obs_move_cmd_node`）
  - 定时器：以配置初值（位置、速度、航向）推进三辆障碍物，广播 TF。

**实现方式**
- 使用 `tf2_ros::TransformBroadcaster` 发布 `geometry_msgs::TransformStamped`；帧名来源于 `ConfigReader` 的车辆配置。
- 以简化的速度/方向推进模型驱动位姿，便于演示规划产物在 RViz 中的动态效果。


