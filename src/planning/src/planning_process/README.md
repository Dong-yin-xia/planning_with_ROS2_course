### 目录：planning_process（规划总流程）

**功能概述**
- 作为系统调度中心：读取配置、生成车辆/障碍物、连接服务、组织参考线/决策/局部路径/轨迹合成管线并发布产物；负责 TF 监听与主循环定时。

**核心文件**
- `planning_process.h/.cpp`：节点逻辑与回调；创建发布器、客户端、定时器；主流程 `planning_callback`。
- `planning_process_main.cpp`：ROS2 入口，构造节点、调用 `process()` 完成初始化后进入 `spin`。

**流程说明**
1. 初始化
   - 读取 `planning_process` 配置（障碍物考虑半径）。
   - 生成主车与三辆障碍物；创建静态 TF 广播、TF Buffer/Listener。
   - 创建服务客户端：`/planning/pnc_map_service`、`/planning/global_path_service`。
   - 创建发布器：`reference_line`、`local_path`、`local_trajectory`。
2. 初始化阶段请求
   - 等待并连接地图/全局路径服务；请求 `PNCMap` 与 `Path` 并缓存。
3. 主回调（定时器）
   - 监听 TF 获取最新位姿；筛选距离内障碍物。
   - 参考线生成 + 发布；车辆与障碍物投影至参考线。
   - 决策点生成；局部路径规划 + 发布；合成轨迹 + 发布。
   - 统计耗时，超时保护（>1s 则 shutdown）。

**实现方式**
- 强模块化：参考线、决策、路径、轨迹均为可插拔类，便于教学和替换。
- 使用 `spin_until_future_complete` 等待服务响应，保证初始化阶段数据准备完整。


