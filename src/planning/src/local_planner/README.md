### 目录：local_planner（局部规划）

**功能概述**
- 基于参考线与决策中心的 SL 点，生成局部路径、平滑、完成 Frenet→笛卡尔转换；合成轨迹并发布可视产物。

**子模块**
- `local_path/`
  - `LocalPathPlanner`：
    - 读取 `local_path` 配置（曲线类型、点数）。
    - 从车辆当前 s 起步，按帧前进，依据决策中心 SL 段采用五次多项式拼接侧向 `l(s)`，计算一/二阶导数。
    - 平滑（`LocalPathSmoother`），再转为笛卡尔坐标（调用 `Curve::frenet_to_cartesian`），补齐姿态与曲率信息。
    - 输出 `LocalPath` 与可视化 `Path`。
  - `LocalPathSmoother`：样板平滑接口，便于后续引入曲率/加速度约束的平滑策略。
- `local_speeds/`
  - `LocalSpeedsPlanner`：占位实现，预留纵向速度规划（如速度保持/跟驰/限速/舒适约束）。
  - `LocalSpeedsSmoother`：速度平滑器接口（占位）。
- `LocalTrajectoryCombiner`
  - 将 `LocalPath` 与 `LocalSpeeds` 合成为 `LocalTrajectory`；当前速度点为空位，重点演示路径到轨迹的打包流程。

**实现方式**
- 采用 Frenet 框架：参考线投影得到 `rs, rtheta, rkappa, rdkappa` 后，使用多项式产生侧向 `l(s)`；再通过几何关系回到笛卡尔坐标。
- 模块化设计：路径、速度、合成器相互解耦，便于分别迭代。


