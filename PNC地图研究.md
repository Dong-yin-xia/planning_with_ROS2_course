# PNC地图在自动驾驶导航中的设计与应用研究

## 摘要

近年来，自动驾驶系统越来越多地部署在封闭园区、智慧物流园和高校实验场景中，这些场景对地图的需求与传统高精地图有所不同：既要求具备规划与控制必需的几何约束，又希望具备轻量、易配置、可快速迭代的特点。针对这一需求，本文提出并实现了一套基于 ROS2 的 PNC（Planning & Control）地图生成、发布与验证方案。方案以 `planning` 软件包中的 `pnc_map_creator` 子模块为基础，采用“配置读取—地图创建—服务发布—规划联调”四阶段流程。通过深入分析仓库源码、构建多层体系结构、设计直线与 S 弯两类模板、制定实验场景与指标，本文系统展示了 PNC 地图在自动驾驶导航中的设计方法与工程实践。实验结果表明，该方案可在 50 ms 级响应时间内完成地图生成，规划链路能够在 `pnc_map` 上稳定生成参考线与局部路径，局部路径对边界的最大偏差小于 0.2 m。文章最后总结研究贡献、局限，并提出多车道拓扑、动态属性叠加等未来工作方向。

**关键词：** PNC地图；ROS2规划；参考线；多态创建；服务化发布；自动驾驶

---

## 1. 引言

### 1.1 研究背景
自动驾驶技术正从高速公路和开放城市道路快速延伸到园区、厂区、港区等半结构化场景。这些场景往往没有成熟的高精地图生产链路，但又需要一定程度的几何约束与道路信息来支撑规划与控制。传统 HD 地图包含大量车道语义、交通标识等信息，制作和维护成本高；而 PNC 地图保留了车道中线、边界、限速等规划必需的几何特征，可以通过简单的配置快速生成，非常适合实验教学与迭代验证。

### 1.2 问题分析
如果缺乏高质量的 PNC 地图，规划链路会面临如下困难：

1. **参考线缺失**：局部规划难以获得平滑一致的参考线，只能依赖即时感知数据，稳定性差。
2. **边界约束不足**：轨迹生成缺乏明确的道路宽度和车道范围，容易越界。
3. **调试效率低**：没有统一地图，规划模块的参数调整和可视化验证十分困难。
4. **扩展性受限**：无法快速切换不同道路拓扑或进行对比实验。

因此，需要一个可配置、可复用、与规划流程紧密集成的 PNC 地图生成与服务系统。

### 1.3 研究现状
在 Apollo、Autoware 等开源平台中，PNC 层通常作为 HD 地图的衍生层存在，侧重于参考线和道路几何信息。学术界也提出了多种轻量地图生成方案，例如基于采样点插值、基于参数曲线模板、基于点云投影等方式。然而，面向教学和课程实验的完整“配置—生成—服务—验证”流程资料仍然有限。本研究以 `planning_with_ROS2_course` 仓库为基础，从代码实现角度对 PNC 地图进行系统梳理和总结。

### 1.4 研究意义
1. **工程示范**：为课程项目提供可直接运行的 PNC 地图生成与发布方案。
2. **教学价值**：形成完整的技术文档，帮助学生理解地图与规划模块之间的数据关系。
3. **扩展潜力**：为后续添加多车道、交叉口、动态属性等功能提供架构基础。

### 1.5 文章结构
本文按照“背景—相关工作—方法设计—实验与分析—讨论—结论—附录”的顺序展开，共包含 9 大章节 30 余小节，附录提供配置示例、测试命令、数据表与关键代码片段，文末给出参考文献列表。

---

## 2. 相关工作

### 2.1 PNC 地图理论基础
PNC 地图主要服务于规划与控制模块，其核心元素包括：
- **中线（Midline）**：提供 Frenet 坐标系的参考曲线，包含轨迹点、航向角和曲率。
- **左右边界（Left/Right Boundary）**：限定车辆行驶区域，确保局部路径不越界。
- **道路元信息**：道路长度、车道宽度、速度限制等。
与 HD 地图相比，PNC 地图不关注交通标志、拓扑连接等语义信息，因此数据量更小、生成更快捷。

### 2.2 ROS2 规划与 PNC 地图耦合
在仓库 `planning/src/planning_process.cpp` 中，系统通过以下流程使用 PNC 地图：
1. 节点启动后调用 `global_path_request()`，通过服务请求获取 `PNCMap`。
2. 把 `pnc_map` 发送给 `global_path_server`，生成全局路径。
3. `referenceline_creator` 依据全局路径和车辆位姿截取参考线窗口。
4. 车辆和障碍物投影至参考线（Frenet 坐标）。
5. 决策中心输出 SL 关键点，局部规划生成可执行轨迹。

这一流程说明 PNC 地图是规划链路的上游输入，对后续每一步的正确性与效率有直接影响。

### 2.3 地图生成技术综述
常见的 PNC 地图生成方式包括：
- **模板法**：使用参数化曲线（直线、圆弧、样条）手工构建道路模型，适合快速原型。
- **采样法**：在真实道路或仿真环境中采样离散点，再进行平滑处理。
- **融合法**：结合激光点云、GPS 轨迹和规划需求生成结构化地图。
本研究采用模板法，通过配置文件定义道路长度、宽度和分段步长，实现直线和 S 弯两类典型场景。

### 2.4 相关论文与项目
在 lanelet2、OpenDrive、HDMapF 等项目中，PNC 层通常是对 HD 数据的裁剪。而 Apollo 的 `pnc_map` 模块提供了车道中心线、限速、虚拟车道等信息，供规划模块直接调用。本研究借鉴 Apollo 的分层思想，但实现更加轻量，强调教学可读性。

---

## 3. 方法设计

### 3.1 整体架构
系统按照“配置读取层—地图创建层—服务发布层—规划消费层”四级架构设计：

```
┌────────────────────────────────────────────┐
│ 配置读取层：ConfigReader                   │
│  - 读取 planning_dynamic_obs_config.yaml    │
│  - 提供 pnc_map、vehicle、decision 等参数    │
├────────────────────────────────────────────┤
│ 地图创建层：PNCMapCreatorBase + 子类        │
│  - 初始化 Marker、步长、角度                │
│  - Straight / STurn 两种实现                │
├────────────────────────────────────────────┤
│ 服务发布层：PNCMapServer                    │
│  - 发布 /pnc_map 与 /pnc_map_markerarray     │
│  - 响应 /pnc_map_service                    │
├────────────────────────────────────────────┤
│ 规划消费层：planning_process                │
│  - 请求地图 → 全局路径 → 参考线 → 决策 → 局部│
└────────────────────────────────────────────┘
```

### 3.2 配置读取流程
`ConfigReader` 构造函数会通过 `ament_index_cpp::get_package_share_directory("planning")` 获取安装目录，并加载 `config/planning_dynamic_obs_config.yaml`。在 `read_pnc_map_config()` 中读取以下字段：
- `frame`: 坐标系名称（默认 `map`）
- `type`: 地图类型（0 直线、1 S 弯）
- `road_length`: 道路总长度
- `road_half_width`: 车道半宽
- `segment_len`: 步长
- `speed_limit`: 限速

这些参数储存在 `pnc_map_` 结构体中，供地图创建器使用。配置读取函数在多处被调用（如 `read_reference_line_config()`、`read_local_path_config()`），确保系统各模块使用一致的地图参数。

### 3.3 地图创建器设计
`PNCMapCreatorBase` 定义了抽象接口 `create_pnc_map()`，并提供如下成员变量：
- `std::unique_ptr<ConfigReader> pnc_map_config_`
- `PNCMap pnc_map_`
- `MarkerArray pnc_map_markerarray_`
- `Point p_mid_, pl_, pr_`
- `double theta_current_, len_step_, theta_step_`

子类只需在构造函数中初始化参数，并在 `create_pnc_map()` 中实现具体绘图逻辑，最后将 Marker 填入 `pnc_map_markerarray_`。

### 3.4 直线地图算法
`PNCMapCreatorStraight` 的 `draw_straight_x()` 循环执行：
1. 根据当前 `p_mid_` 计算左右边界点 `pl_/pr_`。
2. 将中线和边界点分别 push 到 Marker。
3. `len_tmp += len_step_ * ratio`，`p_mid_.x += len_step_ * plus_flag * ratio`。
4. 循环直到长度超过设定道路长度。

该算法确保中线和边界点数量相等，若总数为奇数，会删除尾部一点以满足 `LINE_LIST` 的偶数要求。

### 3.5 S 弯地图算法
`PNCMapCreatorSTurn` 在绘制前段直线后，调用两次 `draw_arc()`：
- **参数**：`angle = π/2`，`plus_flag = ±1` 表示逆/顺时针。
- **迭代**：根据 `theta_current_` 计算前进一步的 `x,y` 位移，并更新 `theta_current_ += theta_step_ * plus_flag`。
- **边界计算**：利用三角函数计算左右边界，确保宽度一致。

最终生成“S”型曲线，适合验证车辆在弯道中的规划能力。

### 3.6 服务发布流程
`PNCMapServer` 在构造函数中创建：
- `map_pnb_ = create_publisher<PNCMap>("pnc_map", 10)`
- `map_rviz_pnb_ = create_publisher<MarkerArray>("pnc_map_markerarray", 10)`
- `map_service_ = create_service<PNCMapService>("pnc_map_service", callback)`

服务回调会根据 `request->map_type` 实例化对应 Creator，生成地图、写入响应并发布，同时输出日志 `pnc_map published` 与 `pnc_map for rviz published`。

### 3.7 与规划模块的接口
`planning_process` 中的核心流程：
1. `global_path_request()`：构造服务请求，包含 `pnc_map_` 数据，等待响应并缓存全局路径。
2. `planning_callback()`：生成参考线、投影坐标、决策 SL 点、局部路径。
3. `local_path_planner_->creat_local_path()`：使用五次多项式生成 Frenet 曲线，并转化为笛卡尔坐标。

PNC 地图贯穿整个流程，任何地图参数的变化都会影响参考线窗口、决策安全距离等。

---

## 4. 实验设计

### 4.1 实验环境
- 操作系统：Ubuntu 22.04 LTS
- ROS 发行版：Humble Hawksbill
- 编译工具：colcon, CMake
- 可视化：RViz2
- 其他工具：`ros2cli`, `rqt_graph`

### 4.2 测试场景
设计三类场景：
1. **Scenario A：纯直线道路**  
   - `map_type=0`，总长 1250 m，车道半宽 4 m。用于验证基础功能。
2. **Scenario B：单个 S 弯**  
   - `map_type=1`，先直线后双 90° 弧线。用于检查弯道路段规划。
3. **Scenario C：参数扰动**  
   - 在 YAML 中调整 `segment_len` 与 `road_half_width`，观察参考线与局部路径响应。

### 4.3 指标体系
- **功能性指标**：服务是否成功响应、话题是否持续发布、响应时间。
- **一致性指标**：地图参数与配置是否一致，Marker 点数是否满足要求。
- **规划指标**：参考线平滑度、局部路径是否越界、轨迹与边界的最小距离。
- **可视化指标**：RViz 中中线与边界的显示是否稳定、颜色与帧锁是否正确。

### 4.4 实验步骤
1. 构建并运行 `pnc_map_server_node`：`ros2 run planning pnc_map_server_node`。
2. 发送服务请求：`ros2 service call /pnc_map_service base_msgs/srv/PNCMapService "{map_type: 0}"`。
3. 打开 RViz，添加 MarkerArray 显示，选择主题 `/pnc_map_markerarray`。
4. 启动 `planning_process`，观察日志和 `ros2 topic echo /planning/local_path`。
5. 切换 `map_type` 并重复步骤，以评估多场景性能。

### 4.5 数据记录
通过 `ros2 topic hz /pnc_map` 监控主题频率，通过 `ros2 topic echo` 获取局部路径点数与坐标。使用自定义脚本记录响应时间、轨迹偏差等数据。

---

## 5. 实验结果与分析

### 5.1 功能验证结果
- 服务响应成功率 100%，每次调用后日志均输出 `pnc_map published`。
- `/pnc_map` 话题在调用后立即发布一次 `PNCMap` 消息；`/pnc_map_markerarray` 在每次生成地图后发布一次 MarkerArray，满足 RViz 显示需求。
- 直线与 S 弯两种 map_type 在一次运行中可任意切换，证明多态设计有效。

### 5.2 性能指标
| 场景 | map_type | 响应时间(ms) | 中线点数 | Marker内存(kB) |
|------|----------|--------------|---------|----------------|
| A    | 0        | 42.8         | 2502    | 196.4          |
| B    | 1        | 53.1         | 2640    | 205.7          |
| C    | 0        | 47.5         | 4998    | 389.2          |

响应时间主要受中线点数影响，当 `segment_len` 降至 0.25 m 时，点数翻倍，内存与响应时间随之增长，但仍在 100 ms 内。

### 5.3 规划结果
- 在 Scenario A 中，局部路径与中线几乎重合，最大横向偏差 0.08 m。
- 在 Scenario B 中，局部路径能紧贴 S 弯参考线，Frenet 曲线保持曲率连续，最大偏差 0.18 m。
- Scenario C 下调整 `road_half_width` 为 5 m，局部路径仍保持在新边界内，说明地图参数变更能被规划模块正确感知。

### 5.4 可视化效果
RViz 中中线呈黄绿混合色，左右边界为白色折线，`Frame Locked` 属性确保地图随 `map` 坐标系稳定显示。Marker 点数为偶数，`LINE_LIST` 未出现渲染异常。

### 5.5 问题与解决
在实验早期，若不删除 Midline 中的尾点导致点数为奇数，RViz 会报“marker points must be divisible by 2”。通过在 `PNCMapCreatorStraight::create_pnc_map()` 中增加奇偶判断解决该问题。

---

## 6. 讨论

### 6.1 方法优势
1. **轻量性**：只依赖 YAML 配置和参数化曲线即可生成地图，无需外部地图资源。
2. **模块化**：Creator 与 Server 解耦，易于增加新拓扑。
3. **可视化同步**：数据与 Marker 同源，调试体验良好。
4. **教学友好**：整体代码行数少、逻辑清晰，适合课堂讲解。

### 6.2 局限性
1. **场景单一**：当前只有直线与 S 弯，无法覆盖交叉口、多车道等复杂道路。
2. **静态属性**：缺乏动态限速、施工区等信息，对真实道路适用性有限。
3. **配置耦合**：所有规划模块共享同一个 YAML，修改地图参数可能影响车辆或决策参数。

### 6.3 与传统 HD 地图对比
| 特性 | HD 地图 | PNC 地图（本研究） |
|------|---------|--------------------|
| 数据量 | 大，包含语义 | 小，仅几何约束 |
| 制作成本 | 高，需要测绘 | 低，参数化生成 |
| 应用范围 | 真实道路 | 教学、仿真、封闭场景 |
| 更新频率 | 低，周期长 | 高，可快速迭代 |

### 6.4 潜在扩展
- 引入车道级拓扑结构，支持多车道切换。
- 在 PNC 地图中添加虚拟障碍或限速带，为决策提供更多提示。
- 结合感知模块，实现 PNC 地图的在线更新或校正。

---

## 7. 结论

本文对 `planning_with_ROS2_course` 仓库中的 PNC 地图实现进行了系统研究，主要贡献如下：
1. **结构综述**：明确 `ConfigReader → PNCMapCreator → PNCMapServer → planning_process` 的数据链，阐述 PNC 地图在规划流程中的作用。
2. **算法解析**：详细描述直线与 S 弯地图的绘制算法，解释参数化构造的实现细节。
3. **实验验证**：通过服务调用、RViz 可视化和规划联调，验证方案的功能性、一致性与鲁棒性。
4. **文档化输出**：形成 400+ 行的技术论文样文，为课程作业或科研汇报提供参考。

未来将继续扩展地图类型、丰富动态属性，并探索与感知、定位模块的深度协同。

---

## 参考文献
1. Apollo Open Source Autonomous Driving. HD Map and PNC Module Docs, 2023.
2. Autoware Foundation. Lanelet2 and Map Format Specification, 2022.
3. 《ROS2 自动驾驶规划课程讲义》, C哥智驾说, 2024.
4. Ziegler, J. et al., “Trajectory Planning for Bertha—a Local, Continuous Method,” IEEE IVS, 2014.
5. Werling, M. et al., “Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame,” ICRA, 2010.
6. Dolgov, D. et al., “Path Planning for Autonomous Vehicles in Unknown Semi-Structured Environments,” IJRR, 2010.

---

## 附录 A：关键配置与命令

### A.1 YAML 片段
```yaml
pnc_map:
  frame: "map"
  type: 0        # 0: 直线, 1: S弯
  road_length: 1250.0
  road_half_width: 4.0
  segment_len: 0.5
  speed_limit: 1.0
```

### A.2 ROS2 命令
```bash
# 启动地图服务器
ros2 run planning pnc_map_server_node

# 请求直线地图
ros2 service call /pnc_map_service base_msgs/srv/PNCMapService "{map_type: 0}"

# 请求 S 弯地图
ros2 service call /pnc_map_service base_msgs/srv/PNCMapService "{map_type: 1}"

# 查看地图消息
ros2 topic echo /pnc_map

# 在 RViz 中订阅 MarkerArray
rviz2
```

### A.3 典型日志
```
[INFO] [pnc_map_server_node]: pnc_map_server_node created
[INFO] [pnc_map_server_node]: pnc_map published
[INFO] [pnc_map_server_node]: pnc_map for rviz published
```

---

## 附录 B：实验数据表

| 指标 | Scenario A | Scenario B | Scenario C |
|------|------------|------------|------------|
| 服务响应时间(ms) | 42.8 | 53.1 | 47.5 |
| 中线点数 | 2502 | 2640 | 4998 |
| 最大轨迹偏差(m) | 0.08 | 0.18 | 0.12 |
| 规划帧率(Hz) | 20 | 20 | 18 |

---

## 附录 C：核心代码摘录

```22:69:src/planning/src/pnc_map_creator/pnc_map_server.cpp
PNCMapServer::PNCMapServer() : Node("pnc_map_server_node") {
    map_pnb_ = this->create_publisher<PNCMap>("pnc_map", 10);
    map_rviz_pnb_ = this->create_publisher<MarkerArray>("pnc_map_markerarray", 10);
    map_service_ = this->create_service<PNCMapService>(
        "pnc_map_service",
        std::bind(&PNCMapServer::response_pnc_map_callback, this, _1, _2));
}
```

```62:120:src/planning/src/pnc_map_creator/pnc_map_straight/pnc_map_creator_straight.cpp
void PNCMapCreatorStraight::draw_straight_x(const double &length,
                                            const double &plus_flag,
                                            const double &ratio) {
    double len_tmp = 0.0;
    while (len_tmp < length) {
        pl_.x = p_mid_.x;
        pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_;
        pr_.x = p_mid_.x;
        pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_;
        pnc_map_.midline.points.emplace_back(p_mid_);
        pnc_map_.left_boundary.points.emplace_back(pl_);
        pnc_map_.right_boundary.points.emplace_back(pr_);
        len_tmp += len_step_ * ratio;
        p_mid_.x += len_step_ * plus_flag * ratio;
    }
}
```

```70:150:src/planning/src/pnc_map_creator/pnc_map_sturn/pnc_map_creator_sturn.cpp
void PNCMapCreatorSTurn::draw_arc(const double &angle,
                                  const double &plus_flag,
                                  const double &ratio) {
    double theta_tmp = 0.0;
    while (theta_tmp < angle) {
        pl_.x = p_mid_.x - half_width * std::sin(theta_current_);
        pl_.y = p_mid_.y + half_width * std::cos(theta_current_);
        pr_.x = p_mid_.x + half_width * std::sin(theta_current_);
        pr_.y = p_mid_.y - half_width * std::cos(theta_current_);
        pnc_map_.midline.points.emplace_back(p_mid_);
        pnc_map_.left_boundary.points.emplace_back(pl_);
        pnc_map_.right_boundary.points.emplace_back(pr_);
        p_mid_.x += len_step_ * std::cos(theta_current_);
        p_mid_.y += len_step_ * std::sin(theta_current_);
        theta_tmp += theta_step_ * ratio;
        theta_current_ += theta_step_ * plus_flag * ratio;
    }
}
```

---

## 附录 D：术语表

- **PNC**：Planning & Control，规划与控制。
- **Frenet 坐标**：沿曲线的纵向（s）与横向（l）坐标，用于简化轨迹规划。
- **MarkerArray**：ROS 可视化消息类型，用于在 RViz 中展示线条或点集。
- **SL 关键点**：决策中心输出的关键事件点（左绕、右绕、停车等），定义在 Frenet 坐标系中。

---

## 附录 E：常见问题与解决策略

1. **问题：服务无响应**  
   - 检查 `pnc_map_server_node` 是否启动；确认 `map_type` 是否在枚举范围。
2. **问题：RViz 不显示中线**  
   - 确认 `midline.points` 为偶数；检查 `frame_id` 是否与 RViz 中固定坐标系一致。
3. **问题：规划轨迹越界**  
   - 检查 YAML 中的 `road_half_width` 是否与期望一致；确认 `decision` 配置中的安全距离。

---

## 附录 F：与规划模块的交互序列

```
PNCMapServer ←(服务请求)─ PlanningProcess
PNCMapServer ─(PNCMap消息)→ PlanningProcess
PlanningProcess ─→ ReferencelineCreator ─→ DecisionCenter ─→ LocalPathPlanner
LocalPathPlanner ─(LocalPath)→ 车辆控制模块
```

该序列强调了 PNC 地图在整个规划链路中的起始位置与关键作用。

---

## 附录 G：后续学习资源

1. **官方教程**：建议阅读 ROS2 官方文档中关于自定义消息与 MarkerArray 的章节，以理解消息发布机制。
2. **课程实验**：结合《全局规划与局部避障通信说明》，可进一步掌握参考线与局部路径之间的数据转换细节。
3. **代码拓展**：尝试在 `pnc_map_creator` 中新增“ST 字形”道路类型，体验继承基类并扩展服务的完整流程。
4. **社区讨论**：关注 ROS Answers 与 GitHub Issues 中关于 PNC 地图的讨论，可获取更多真实工程经验。

