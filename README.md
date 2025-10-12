## 项目简介

本仓库是一个基于 ROS 2 Humble 的“自动驾驶规划（教学/演示）”项目，包含：

- 地图生成与发布（直道、S 弯）
- 全局路径规划服务（Normal）
- 参考线生成与平滑
- 决策中心（基于障碍物的简单变道/停车策略）
- 局部路径规划（Frenet 五次多项式 + 平滑 + 笛卡尔转换）
- 轨迹合成（路径 + 速度，当前速度为占位）
- 车辆/障碍物 TF 广播用于“仿真前进”
- RViz 可视化（地图、全局路径、参考线、局部路径、车辆模型与 TF）

阅读本 README 后，初学者可以快速完成构建、启动、观测数据流，并逐步修改配置和算法模块。


## 目录结构（关键）

```text
planning_with_ROS2_course/
├─ src/
│  ├─ base_msgs/                 # 自定义消息与服务定义
│  │  ├─ msg/                    # LocalPath/LocalTrajectory/PNCMap 等
│  │  └─ srv/                    # PNCMapService / GlobalPathService
│  ├─ data_plot/                 # 可选绘图工具（Python）
│  └─ planning/                  # 主功能包
│     ├─ config/                 # 统一配置 YAML（地图/车辆/参考线/局部路径/决策等）
│     ├─ launch/                 # 启动文件（主场景、运动指令）
│     ├─ rviz/                   # RViz 配置
│     ├─ urdf/                   # 主车与障碍物的 xacro 模型
│     └─ src/                    # 规划核心源码
│        ├─ common/              # 配置读取器、数学工具
│        ├─ decision_center/     # 决策中心（产出 SL 点）
│        ├─ global_planner/      # 全局路径服务器与规划器
│        ├─ local_planner/       # 局部路径/速度/轨迹合成
│        ├─ move_cmd/            # 主车/障碍物运动指令（TF 广播）
│        ├─ planning_process/    # 规划总流程（协调/发布各产物）
│        ├─ pnc_map_creator/     # PNC 地图生成（直线、S 弯）
│        ├─ reference_line/      # 参考线生成与平滑
│        ├─ vehicle_info/        # 车辆/障碍物模型与参数
│        └─ test/                # OSQP 测试样例
└─ README.md                     # 本文档
```


## 环境要求

- 系统：Ubuntu 22.04（建议）
- ROS 2：Humble（已安装基础开发工具链）
- 构建：colcon、CMake ≥ 3.22
- 依赖（通过 apt 安装，示例）：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-ament-cmake \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  libyaml-cpp-dev \
  libeigen3-dev
# 可选（如需 OSQP/OsqpEigen 测试）：
sudo apt install -y libosqp-dev
```

提示：工程引用的 Eigen 头路径在 CMake 中已设置到 `/usr/local/include/eigen-3.4.0`，若本机安装位置不同可修改 `planning/CMakeLists.txt` 的 include_directories。


## 构建与运行

1) 构建

```bash
cd /home/x/planning_with_ROS2_course
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select base_msgs planning data_plot
source install/setup.bash
```

2) 启动主场景（地图/全局路径/规划 + 模型 + RViz）

```bash
ros2 launch planning planning_launch.py
```

3) 启动车辆/障碍物运动指令（在新终端）

```bash
source /opt/ros/humble/setup.bash
source /home/x/planning_with_ROS2_course/install/setup.bash
ros2 launch planning move_cmd_launch.py
```

4) RViz 查看

- 本工程已提供预设 `rviz/planning.rviz`，启动脚本会自动加载。
- 默认 Fixed Frame 在配置中设置为 `base_footprint`。若希望以世界坐标观察，可切换为 `map`。


## ROS 图谱（包 / 节点 / 话题 / 服务 / TF）

- 包
  - `base_msgs`：定义消息与服务
  - `planning`：核心功能
  - `data_plot`：可选绘图（Python）

- 可执行/节点（由 `planning/launch/planning_launch.py` 启动，命名空间 `planning`）
  - `pnc_map_server`（节点名：`pnc_map_server_node`）
    - 发布：`/planning/pnc_map`、`/planning/pnc_map_markerarray`
    - 服务：`/planning/pnc_map_service`
  - `global_path_server`（节点名：`global_path_server_node`）
    - 发布：`/planning/global_path`、`/planning/global_path_rviz`
    - 服务：`/planning/global_path_service`
  - `planning_process`（节点名：`planning_process`）
    - 客户端：`/planning/pnc_map_service`、`/planning/global_path_service`
    - 发布：`/planning/reference_line`、`/planning/local_path`、`/planning/local_trajectory`

- 可执行/节点（由 `planning/launch/move_cmd_launch.py` 启动）
  - `car_move_cmd`（节点名：`car_move_cmd_node`）
    - 订阅：`/planning/local_trajectory`
    - 功能：依据轨迹广播主车 TF
  - `obs_move_cmd`（节点名：`obs_move_cmd_node`）
    - 定时：广播 3 辆障碍物 TF

- 服务接口
  - `/planning/pnc_map_service`（请求：`int32 map_type`，响应：`PNCMap`）
  - `/planning/global_path_service`（请求：`int32 global_planner_type + PNCMap`，响应：`nav_msgs/Path`）

- TF 框架
  - 规划侧：`planning_process` 在车辆生成时广播静态 TF；并用 TF2 Buffer/Listener 获取车辆与障碍物位姿。
  - 驱动侧：`car_move_cmd`、`obs_move_cmd` 分别广播主车/障碍物 TF 实现“仿真前进”。


## 配置文件说明（`planning/config/planning_static_obs_config.yaml`）

- `vehicle`: 主车与 3 辆障碍物的尺寸、初始位姿、初速度、TF 子帧名
- `pnc_map`: 全局坐标系名、类型（0 直线 / 1 S 弯）、道路长度/半宽、离散步长、限速
- `global_path`: 规划器类型（0 normal, 预留 astar）
- `reference_line`: 参考线窗口（前/后点数）
- `local_path`: 曲线类型（0/1/2 → 1/3/5 次多项式）、路径点数
- `local_speed`: 速度点数（预留）
- `decision`: 横/纵向安全距离
- `planning_process`: 障碍物考虑半径
- `move_cmd`: 预留

修改生效提示：配置文件由 `ConfigReader` 通过 `ament_index_cpp::get_package_share_directory("planning")` 定位到安装后的 share 目录加载，因此修改 YAML 后请重新构建或至少 `colcon build` + `source install/setup.bash`，确保运行时拿到最新配置。


## 模块与算法（入门视角）

1) PNC 地图生成（`pnc_map_creator`）
   - `PNCMapCreatorStraight`：按步长在 X 方向“画线”，生成中线、左右边界，并输出 `MarkerArray`（RViz 使用）与 `PNCMap`（下游使用）。
   - `PNCMapCreatorSTurn`：先直行，再两段 90° 弧线（逆/顺时针）生成 S 弯；`pnc_map_.midline.frame_locked = true` 便于 RViz 锁定显示。

2) 全局路径（`global_planner`）
   - `GlobalPlannerNormal`：以中线与右边界的坐标均值近似生成 `nav_msgs/Path`，用于演示。
   - 服务端 `global_path_server`：响应请求，发布 `global_path` 与 `global_path_rviz`。

3) 参考线（`reference_line`）
   - `ReferencelineCreator`：在全局路径上基于当前位姿寻找匹配点，截取前后窗口；对整条参考线平滑；计算每个点的投影参数（用于后续 Frenet 转换）。

4) 决策中心（`decision_center`）
   - 基于障碍物与车道边界，生成 SL 关键点（左绕/右绕/STOP/START/END）。横向可通过宽度作为判据；若两侧均不可行则生成停车点。

5) 局部路径（`local_planner/local_path`）
   - `LocalPathPlanner`：在 Frenet 坐标下，依 SL 断点区间，采用五次多项式生成侧向轨迹；再平滑；最后转换回笛卡尔并计算投影属性。
   - 发布可视用 `Path` 与消息 `LocalPath`。

6) 速度与轨迹（`local_speeds` / `local_trajectory_combiner`）
   - `LocalSpeedsPlanner`：目前为样板（未实际产出速度点），可作为练习实现纵向速度曲线。
   - `LocalTrajectoryCombiner`：将路径与速度合成 `LocalTrajectory`。现阶段以路径为主，速度为空位。

7) 规划总流程（`planning_process`）
   - 初始化：读取配置、创建车辆与障碍物、TF Broadcaster/Listener、服务客户端、各发布器/规划器/决策器。
   - 主循环：获取位姿 → 参考线 → 决策 → 局部路径 → 合成轨迹 → 发布 → 记录耗时并保护（>1s 触发 shutdown）。

8) 驱动广播（`move_cmd`）
   - `car_move_cmd`：订阅 `local_trajectory`，以匹配点作为参考，计算并广播主车 TF（可切换是否使用“实际位置”宏）。
   - `obs_move_cmd`：定时按速度与朝向更新三辆障碍物的 TF。


## 快速验证

```bash
# 终端1（主场景）
ros2 launch planning planning_launch.py

# 终端2（驱动 TF）
ros2 launch planning move_cmd_launch.py

# 查看话题
ros2 topic list | grep planning
ros2 topic echo /planning/local_path --qos-durability volatile --qos-reliability reliable

# 查看服务
ros2 service list | grep planning
```


## 常见问题（FAQ）

- 运行时报找不到配置文件？
  - 请确认已执行 `colcon build` 且 `source install/setup.bash`，配置通过 ament_index 在安装目录加载。

- RViz 看不到轨迹/参考线？
  - 检查命名空间：本工程在 `planning` 命名空间下发布，话题形如 `/planning/local_path`。
  - 检查 Fixed Frame：可尝试切换为 `map`。

- 缺少依赖（xacro、robot_state_publisher 等）？
  - 请参考“环境要求”中的 apt 安装命令。

- 编译报 Eigen/OsqpEigen 相关错误？
  - 请安装对应开发包或调整 CMake 中的 include 路径；`OsqpEigen` 仅在测试/示例中使用。


## 扩展练习与方向

- 实现 `LocalSpeedsPlanner` 的纵向速度规划（如速度保持、跟驰/制动约束）。
- 新增全局路径规划器（如 A*），在 `GlobalPlannerType` 中注册并选择。
- 丰富地图类型（曲线半径、换道段、减速带等），在 YAML 中参数化。
- 将轨迹发布为 Marker/Path 以增强可视化；为速度/加速度着色。
- 为 `planning_process` 增加诊断（统计耗时、失败原因）。


## 开发建议

- 统一使用 YAML 配置，避免硬编码；改动后重新构建并 source。
- 注意命名空间与话题名称一致性，便于 RViz 与节点互通。
- 逐步替换样板实现（速度规划）以构成完整的 L2 级规划栈最小闭环。


## 许可证

本项目用于学习与教学目的。上游源文件中包含的版权与声明需遵守原作者约束；请在正式产品或商业用途前完善 `package.xml` 的 license 字段并自查依赖条款。


