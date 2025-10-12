### 目录：global_planner（全局路径）

**功能概述**
- 提供全局路径规划服务端与规划器实现，输入 PNC 地图输出 `nav_msgs/Path`。

**节点**
- `global_path_server`（可执行）：
  - 服务：`/planning/global_path_service`（请求：规划器类型 + PNCMap；响应：Path）。
  - 发布：`/planning/global_path`（一次性 Path）、`/planning/global_path_rviz`（Marker 可视化）。

**规划器接口**
- `GlobalPlannerBase`：定义 `search_global_path(const PNCMap&)`。
- `GlobalPlannerNormal`：对 `PNCMap` 的中线与右边界取均值生成 Path，便于演示。

**实现方式**
- 读取 `global_path` 配置以选择规划器类型；支持扩展（如 A*、D*、采样等）。
- RViz 可视化通过 `Marker LINE_STRIP` 并 `frame_locked=true` 固定显示。


