### 目录：common（通用工具）

**功能概述**
- 提供规划模块通用依赖：配置读取器与数学几何工具。

**主要子目录**
- `config_reader/`: 统一从安装后的 share 目录加载 YAML，集中管理车辆、地图、参考线、局部路径、速度、决策、流程、运动指令等配置。
- `math/`: 曲线与多项式工具，负责 Frenet/笛卡尔转换、投影点搜索、匹配点搜索、五次多项式系数求解等。

**关键类与职责**
- `ConfigReader`
  - 使用 `ament_index_cpp::get_package_share_directory("planning")` 定位配置路径；通过 `yaml-cpp` 解析；提供结构体访问接口（如 `pnc_map()`, `refer_line()` 等）。
  - 被各模块在构造期读取，避免运行中重复查找。
- `Curve`
  - `cartesian_to_frenet` / `frenet_to_cartesian`: 坐标转换核心；输入投影点参数与当前点状态，输出目标坐标系下的状态量。
  - `find_match_point` / `find_projection_point` / `cal_projection_param`: 在 Path/Referline/LocalPath 上查找匹配点、计算投影点参数及参考线/路径的导数信息。
- `PolynomialCurve`
  - `quintic_polynomial`: 基于边界条件（位置、一二阶导）求解 5 次多项式系数，供局部路径侧向曲线生成。

**实现方式**
- 纯头/源实现，提供无 ROS 话题/服务依赖的数学功能；通过 `ament_target_dependencies` 仅依赖 `rclcpp`、`yaml-cpp`、`ament_index_cpp`、`Eigen` 等。


