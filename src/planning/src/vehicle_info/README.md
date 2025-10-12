### 目录：vehicle_info（车辆/障碍物信息）

**功能概述**
- 定义主车与障碍物的统一数据结构与接口，提供从定位点到参考线 Frenet 投影的能力，供决策与局部规划使用。

**核心类**
- `VehicleBase`
  - 存放车辆基本尺寸、帧名、定位点、姿态、速度/加速度等笛卡尔参数；
  - 保存向参考线投影后的 Frenet 参数（`s, l, ds_dt, ...`）；
  - 纯虚方法 `vehicle_cartesian_to_frenet` 由派生类实现。
- `MainCar` / `ObsCar`
  - 构造时从 `ConfigReader` 读取对应配置，初始化帧名、尺寸、初始位姿/速度等；
  - 通过 `Curve::find_projection_point` + `cartesian_to_frenet` 完成投影和坐标转换。

**实现方式**
- 与 `reference_line`/`Curve` 工具协作，避免车辆模块内部重复实现几何细节；
- 通过 `child_frame_` 与 TF 广播的帧名保持一致，便于 RViz 可视化。


