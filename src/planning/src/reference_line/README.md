### 目录：reference_line（参考线）

**功能概述**
- 在全局 Path 上根据当前位姿匹配到最近点，截取前后窗口生成参考线，平滑后计算投影参数，向下游提供 Frenet 几何信息。

**核心类**
- `ReferencelineCreator`
  - `create_reference_line(global_path, current_pose)`：
    - 通过 `Curve::find_match_point` 获取匹配点；根据 YAML 的 `front_size/back_size` 截取窗口；
    - 平滑整条参考线（`ReferenceLineSmoother`）；
    - 计算每个点的投影参数（`Curve::cal_projection_param`），后续 Frenet→笛卡尔转换会用到 `rs/rtheta/rkappa/rdkappa`。
  - `referline_to_rviz()`：转为 `nav_msgs/Path` 便于 RViz 显示。

**实现方式**
- 依赖 `Curve` 工具完成匹配与几何量计算；
- 与局部路径耦合点少，接口清晰，可替换为“拼接参考线”等高级实现。


