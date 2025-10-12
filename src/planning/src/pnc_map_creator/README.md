### 目录：pnc_map_creator（PNC 地图生成）

**功能概述**
- 根据 YAML 配置生成直线或 S 弯 PNC 地图，发布 MarkerArray 供 RViz 显示，并通过服务响应 `PNCMap` 供下游使用。

**节点**
- `pnc_map_server`（节点名：`pnc_map_server_node`）
  - 发布：`/planning/pnc_map`、`/planning/pnc_map_markerarray`
  - 服务：`/planning/pnc_map_service`（请求 `map_type`，响应 `PNCMap`）

**核心类**
- `PNCMapCreatorBase`：提供统一接口与成员（配置、MarkerArray、步长、角度等）。
- `PNCMapCreatorStraight`：沿 X 方向离散绘制中线与左右边界。
- `PNCMapCreatorSTurn`：先直线段，再两段 90° 弧线构成 S 弯；`midline.frame_locked = true` 便于 RViz 锁定。

**实现方式**
- 使用 `ConfigReader` 读取 `pnc_map.frame/road_length/road_half_width/segment_len` 等参数。
- 生成 `visualization_msgs/Marker`（`LINE_LIST`/`LINE_STRIP`）并组织为 `MarkerArray`；同时填充 `base_msgs::PNCMap`。
- 服务回调根据 `map_type` 多态创建具体 Creator，生成后同时发布地图与可视化。


