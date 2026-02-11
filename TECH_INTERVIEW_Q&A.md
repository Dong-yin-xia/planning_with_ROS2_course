# 自动驾驶规划项目 - 技术面试问答

## 项目概述

这是一个基于ROS2的自动驾驶局部路径规划项目，实现了从地图生成、全局路径、参考线平滑、决策中心、局部路径规划到轨迹合成的完整规划栈。

---

## 核心问题与回答

### 1. 架构设计

**Q: 为什么选择模块化设计？各模块之间的职责划分是什么？**

**A:** 采用分层模块化设计，遵循单一职责原则：

- **PNC地图生成器**: 负责静态地图数据（中线、边界）的生成和发布
- **参考线模块**: 从全局路径截取当前窗口，进行平滑处理，为Frenet坐标转换提供基准
- **决策中心**: 基于障碍物信息进行行为决策，生成SL关键点（变道/停止）
- **局部路径规划**: 在Frenet坐标下用五次多项式连接SL关键点，生成平滑的横向轨迹
- **轨迹合成**: 将路径与速度信息组合成完整的轨迹

这样的设计使得：
- 每个模块可以独立测试和优化
- 便于替换算法（如将决策从规则切换为学习）
- 代码复用性强，接口清晰

---

### 2. 模块间数据传递机制

**Q: 模块间如何传递数据（消息/服务）？为何用ROS2做通信中间件？**

**A:** 项目采用了**ROS2的分布式通信架构**，模块间通过多种通信机制传递数据：

#### 数据传递方式

**1. 话题（Topic）通信 - 发布/订阅模式**
用于**周期性、单向数据流**，特点是：
- **异步非阻塞**：发布者无需等待订阅者
- **多对多**：多个发布者/订阅者可以共享同一话题
- **适合**：实时性要求高的数据流

**实现示例：**
```cpp
// 发布器创建
refer_line_pnb_ = this->create_publisher<Path>("reference_line", 10);

// 发布数据
refer_line_pnb_->publish(refer_line_rviz);

// 订阅器创建
local_trajectory_sub_ = this->create_subscription<LocalTrajectory>(
    "planning/local_trajectory", 
    10, 
    std::bind(&CarMoveCmd::car_broadcast_tf, this, _1)
);
```

**项目中话题用途：**
- `/planning/reference_line`：参考线数据
- `/planning/local_path`：局部路径
- `/planning/local_trajectory`：最终轨迹（订阅者：控制层）
- `/planning/pnc_map`：静态地图数据

**2. 服务（Service）通信 - 请求/响应模式**
用于**按需、同步调用**，特点是：
- **同步阻塞**：客户端等待服务器响应
- **一对多**：多个客户端可以向同一服务器请求
- **适合**：初始化、配置获取等一次性操作

**实现示例：**
```cpp
// 服务端创建
map_service_ = this->create_service<PNCMapService>(
    "pnc_map_service", 
    std::bind(&PNCMapServer::response_pnc_map_callback, this, _1, _2)
);

// 客户端发送请求
auto request = std::make_shared<PNCMapService::Request>();
request->map_type = process_config_->pnc_map().type_;
auto result_future = map_client_->async_send_request(request);

// 等待响应
if (rclcpp::spin_until_future_complete(...) == SUCCESS) {
    pnc_map_ = result_future.get()->pnc_map;
}
```

**项目中服务用途：**
- `/planning/pnc_map_service`：地图请求（初始化时调用）
- `/planning/global_path_service`：全局路径请求（初始化时调用）

**3. TF（Transform）通信 - 坐标变换树**
用于**坐标系间的位姿变换**，特点是：
- **树状结构**：维护完整的坐标系层次关系
- **时间戳同步**：自动处理不同时间的变换
- **适合**：车辆位姿、障碍物位置等坐标相关数据

**实现示例：**
```cpp
// TF广播
broadcaster_ = std::make_shared<TransformBroadcaster>(this);
tf_stamped.header.frame_id = "map";
tf_stamped.child_frame_id = "base_footprint";
broadcaster_->sendTransform(tf_stamped);

// TF监听
buffer_ = std::make_unique<Buffer>(this->get_clock());
tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);

// 查询变换
geometry_msgs::msg::TransformStamped transform =
    buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
```

---

#### 为什么选择ROS2？

**1. 分布式架构优势**
```
┌─────────────────────────────────────────┐
│        规划模块（planning_process）       │
│  ├─ 参考线生成                            │
│  ├─ 决策中心                              │
│  └─ 局部路径规划                          │
└─────────────────────────────────────────┘
              ↓ Topic: local_trajectory
┌─────────────────────────────────────────┐
│      控制模块（car_move_cmd）            │
│      执行轨迹控制                         │
└─────────────────────────────────────────┘
```

- **解耦**：各模块独立编译、部署、测试
- **可扩展**：新增模块只需订阅/发布相应话题
- **容错性**：单个模块崩溃不影响其他模块

**2. 跨语言支持**
- 核心算法用C++（性能）
- 可视化用Python（开发效率）
- 通过ROS消息无缝通信

**3. 丰富的生态系统**
- **RViz2**：可视化调试
- **rosbag2**：数据录制与回放
- **Launch系统**：管理复杂启动流程
- **参数服务器**：运行时配置

**4. 实时性保证（DDS层）**
```
ROS2 DDS Middleware Layer:
├─ RTPS (Real-Time Publish Subscribe)
├─ 多种DDS实现可选：FastRTPS, Cyclone DDS
└─ 支持deterministic传输
```

- **QoS（Quality of Service）配置**：可根据需求配置可靠性、持久性
- **零拷贝**：避免数据不必要的内存拷贝
- **时间同步**：支持多机时间同步（用于分布式系统）

**5. 项目实际收益**

**代码对比：**
```cpp
// 传统方式：函数调用（紧耦合）
void planning() {
    Referline ref_line = generate_referline();
    Decision decision = make_decision(ref_line);
    LocalPath path = plan_path(ref_line, decision);
    return path;
}

// ROS2方式：话题通信（松耦合）
// 模块1：发布参考线
refer_line_pub->publish(ref_line);

// 模块2：订阅参考线，发布决策
void callback_ref(const Referline& ref_line) {
    Decision d = make_decision(ref_line);
    decision_pub->publish(d);
}

// 模块3：订阅决策，发布路径
void callback_decision(const Decision& d) {
    LocalPath path = plan_path(d);
    path_pub->publish(path);
}
```

**优势：**
- ✅ 模块2替换算法不影响模块1和3
- ✅ 可以独立启动/停止各模块
- ✅ 便于在RViz中观察中间产物（参考线、决策点）
- ✅ 支持多车协同（每辆车独立运行，通过话题通信）

**局限与改进：**
- ❌ ROS2有额外开销（序列化/反序列化、网络传输）
- ❌ 调试复杂度增加（需要理解话题数据流）
- ⚠️ 实际产品可能需要将关键模块合并（减少通信延迟）

---

### 3. 算法核心 - Frenet坐标转换

**Q: 你实现的Frenet到笛卡尔的转换原理是什么？为什么要用Frenet坐标？**

**A:** Frenet坐标的核心思想是将车辆的位姿分解为：
- **纵向s**: 沿参考线的累积弧长
- **横向l**: 垂直于参考线的偏移

**优势：**
- 在结构化道路（有明确参考线）中，横向和纵向约束可以独立处理
- 变道/跟车等行为在Frenet坐标下更直观
- 避免了笛卡尔坐标下复杂的非线性约束

**转换公式关键点：**
```cpp
// Frenet→Cartesian的核心公式
x = rx - sin(rtheta) * l  // rx, ry是参考线上的投影点
y = ry + cos(rtheta) * l

// 角度计算考虑曲率影响
tan(δθ) = dl_ds / (1 - rkappa * l)
theta = rtheta + δθ

// 曲率计算（考虑横向偏移对曲率的影响）
kappa = (ddl_ds + kappa_l_prime * tan(δθ)) * cos²(δθ) / (1-rkappa*l) 
      + rkappa * cos(δθ) / (1-rkappa*l)
```

**实现注意点：**
- 当`|s - rs|`过大时需要报错，因为投影点不准确
- 考虑参考线曲率`rkappa`对横向的影响（尤其是弯道）

---

### 4. 决策逻辑

**Q: 你的决策中心如何判断左绕、右绕还是停车？**

**A:** 决策基于**可通行带宽**判断：

```cpp
// 1. 计算障碍物左右两侧的可用宽度
left_width = left_bound_l - (obs_l + obs_width/2)
right_width = (obs_l - obs_width/2) - right_bound_l

// 2. 判断是否满足通行条件
if (left_width > vehicle_width + 2 * safe_distance) {
    // 左绕
    target_l = (left_bound + obs_left_bound) / 2
} else if (right_width > vehicle_width + 2 * safe_distance) {
    // 右绕
    target_l = (right_bound + obs_right_bound) / 2
} else {
    // 停车
    stop_s = obs_s - safe_distance
}
```

**关键参数：**
- `safe_dis_l` = 0.5m（横向安全距离）
- `safe_dis_s` = 10m（纵向安全距离）
- 最小变道距离：`max(v * dis_time, 30m)`（速度越快需要越早决策）

**局限和改进方向：**
- 当前是规则式决策，未来可引入**代价函数**（综合考虑速度、舒适性、风险）
- 多障碍物情况下存在优先级冲突（当前按S坐标排序处理）
- 可增加**博弈论**方法处理与动态障碍物的交互

---

### 5. 路径平滑

**Q: 为什么选择五次多项式？如何保证轨迹连续性？**

**A:** 五次多项式的形式：`l(s) = a₀ + a₁s + a₂s² + a₃s³ + a₄s⁴ + a₅s⁵`

**边界条件（6个）：**
- 起始点：位置l₀、速度dl_ds₀、加速度ddl_ds₀
- 终止点：位置l₁、速度dl_ds₁、加速度ddl_ds₁

**为什么是5次不是3次？**
- 3次多项式只能约束位置和速度（4个条件），**无法控制曲率**
- 5次多项式可以保证曲率连续，使车辆行驶更平滑

**实现：**
```cpp
Eigen::Vector<double, 6> coeffs = PolynomialCurve::quintic_polynomial(
    start_s, start_l, 0, 0,  // 起点状态
    end_s, end_l, 0, 0       // 终点状态
);
```

**平滑处理：**
- 在SL坐标下生成轨迹后，使用`LocalPathSmoother`进一步平滑
- 平滑可以在斜率突变处产生更舒适的轨迹

---

### 6. 性能与实时性

**Q: 规划循环的耗时如何？如何优化？**

**A:** 当前性能瓶颈分析：

**主要计算步骤：**
1. 参考线匹配点查找：O(n)，n为参考线点数量
2. Frenet转换（车辆/障碍物）：O(1) × N车辆
3. 决策计算：O(M)，M为障碍物数量
4. 路径生成：O(K)，K为路径点数量
5. 笛卡尔转换：O(K)

**优化方向：**
- **并行化**：障碍物的Frenet转换可以并行
- **缓存**：参考线匹配点利用上一帧的结果加速查找
- **降采样**：非关键路径点可以降低密度
- **提前终止**：局部路径超出参考线范围立即停止

**当前实现限制：**
- `path_size=80`意味着每次生成80个点
- 在实际车辆上需要根据速度动态调整预测距离

---

### 7. 安全性与鲁棒性

**Q: 如何保证生成轨迹的安全性？**

**A:** 多层安全保障：

**1. 决策层：**
- 安全距离约束（横向0.5m，纵向10m）
- 预测交汇点并提前生成变道点
- 若两侧均不可通行，强制停车

**2. 规划层：**
- 五次多项式保证加速度连续（避免急转）
- 笛卡尔转换后的曲率检查（避免曲率突变）
- 边界检查：路径点是否在道路范围内

**3. 验证层（待完善）：**
```cpp
// 应该增加碰撞检测
for (auto& path_point : local_path) {
    for (auto& obs : obstacles) {
        if (碰撞检测(path_point, obs)) {
            RCLCPP_ERROR("轨迹存在碰撞风险！");
        }
    }
}
```

**已知问题：**
- ❌ 当前没有显式的碰撞检测
- ❌ 没有考虑速度规划对安全性的影响（速度越快需要更大的安全距离）
- ❌ 没有处理传感器噪声导致的定位不确定性

---

### 8. 未完成功能与扩展

**Q: 速度规划部分还未实现，你打算如何实现？**

**A:** 速度规划可以考虑以下方法：

**方案1：基于规则的纵向规划**
```cpp
class LongitudinalPlanner {
    // 1. 根据前方障碍物距离计算目标速度
    double target_speed = compute_safe_speed(leading_vehicle);
    
    // 2. 考虑限速和舒适性
    target_speed = min(target_speed, speed_limit, max_comfortable_speed);
    
    // 3. 用二次规划生成平滑的速度曲线
    // s(t) = s0 + v0*t + 0.5*a*t²
};
```

**方案2：基于优化的纵向规划（更优）**
- 使用**OSQP**求解器，构建带约束的QP问题
- 目标函数：最小化加速度变化率（舒适性）
- 约束条件：速度限制、加速度限制、安全跟车距离
- 与横向路径规划联合优化（考虑曲率对速度的约束）

**实施步骤：**
1. 定义速度配置文件（每个路径点对应速度）
2. 根据路径曲率计算最大安全速度：`v_max = sqrt(a_max / kappa)`
3. 考虑跟车约束：`v <= v_leading if distance < safe_distance`
4. 合成最终轨迹

---

### 9. 工程实践

**Q: 配置文件如何管理的？便于调参吗？**

**A:** 采用YAML统一配置管理：

**配置结构：**
```yaml
vehicle:      # 车辆参数
pnc_map:      # 地图参数
local_path:   # 路径参数
decision:     # 决策参数
```

**优势：**
- 所有参数集中在`planning/config/planning_static_obs_config.yaml`
- 修改后重新编译自动加载到`install/share/planning/`目录
- 可以在线调参（读取新配置文件）但当前未实现

**改进建议：**
- 增加参数服务器（ROS2 Parameter）支持在线调参
- 增加参数校验（如安全距离不能为负）
- 提供参数说明文档

---

### 10. 可视化与调试

**Q: 如何调试和验证规划结果？**

**A:** 使用RViz2进行可视化：

**可视化的元素：**
- 全局路径（绿色线）
- 参考线（蓝色线）
- 局部路径（红色线）
- 车辆TF（白色模型）
- 障碍物TF（红色模型）

**调试方法：**
- 打印关键信息：`RCLCPP_INFO`输出SL坐标、曲率、路径点数
- 记录ROS bag：`ros2 bag record`保存所有话题数据
- 离线分析：用`data_plot`模块绘图分析轨迹质量

**当前局限：**
- 速度信息无法可视化（因为速度规划未实现）
- 缺少路径质量的量化指标（如舒适度评分）

---

### 11. 扩展方向

**Q: 如果要投入实际使用，还需要做哪些改进？**

**A:** 关键改进方向：

**1. 完善速度规划**
- 实现基于优化的纵向速度规划
- 考虑纵向动力学约束
- 实现紧急制动逻辑

**2. 增强安全机制**
- 添加碰撞检测
- 实现fallback轨迹（如果主轨迹不可行）
- 增加传感器不确定性处理（SLAM定位误差）

**3. 提升规划质量**
- 平滑性指标优化（减少jerk）
- 舒适性指标（考虑横向加速度）
- 增加轨迹优化器（用IPOPT/SQP求解器）

**4. 工程化**
- 增加单元测试（gtest）
- 增加集成测试（模拟各种场景）
- 性能profiling和优化
- 日志系统完善

**5. 高级功能**
- 换道决策（当前只有绕行）
- 多车道规划
- 与交通规则的结合（红绿灯、停车标志）

---

## 代码亮点

### 1. 清晰的模块划分
- 每个模块职责单一，接口清晰
- 使用ROS2服务/话题解耦模块间通信

### 2. Frenet坐标转换的准确实现
- 正确考虑了曲率对横向偏移的影响
- 实现了完整的导数计算（一阶、二阶）

### 3. 五次多项式轨迹生成
- 使用Eigen库高效求解系数
- 保证了轨迹的C²连续性

### 4. 配置驱动设计
- 所有关键参数都可以通过YAML配置
- 便于不同场景的调参

---

## 总结

这是一个**教学/演示性质**的规划项目，展示了自动驾驶规划的核心流程。在算法层面，Frenet坐标转换和五次多项式轨迹生成是**工程可用**的；但在**速度规划**、**安全验证**、**传感器融合**等方面还有很大改进空间。

**适合的场景：**
- 低速园区自动驾驶
- 停车场自动泊车
- 作为更复杂规划算法的教学案例

**不适合的场景：**
- 高速公路自动驾驶
- 强实时性要求
- 复杂交通环境（多车交互）
