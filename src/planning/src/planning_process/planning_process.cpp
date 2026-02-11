/*
文件名: 规划总流程
作者: C哥智驾说
完成时间: 2024.12

编译类型: 节点(planning_process)

依赖: ROS2内部库:
      rclcpp
      nav_msgs
      tf2
      tf2_ros
    外部库:
      base_msgs
      config_reader
      vehicle_info
      decision_center
      reference_line
      local_planner

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "planning_process.h"
#include "main_car_info.h"
#include "pnc_map_server.h"
#include "vehicle_info_base.h"
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(), "planning_process已创建");

        // 读取配置文件
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        obs_dis_ = process_config_->process().obs_dis_;
        // 读取参考线类型
        process_config_->read_reference_line_config();
        reference_line_type_ = process_config_->refer_line().type_;
        RCLCPP_INFO(this->get_logger(), "参考线配置 type = %d (%s)",
                    reference_line_type_, reference_line_type_ == 1 ? "stich(拼接)" : "normal(普通)");

        // 创建车辆和障碍物
        car_ = std::make_shared<MainCar>();
        for (int i = 0; i < 3; i++) 
        {
            auto obs_car_ = std::make_shared<ObsCar>(i + 1);   // 因为主车id为0，所以障碍物id从1开始
            obses_spawn_.emplace_back(obs_car_);
        
        }

        // 坐标广播器
        tf_broadcaster_ = std::make_shared<StaticTransformBroadcaster>(this);

        // 创建监听器，绑定主车缓存对象
        buffer_ = std::make_unique<Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<TransformListener>(*buffer_, this);

        // 创建地图和全局路径客户端
        map_client_ = this->create_client<PNCMapService>("pnc_map_service");
        global_path_client_ = this->create_client<GlobalPathService>("global_path_service");

        // 创建参考线和参考线的发布器
        refer_line_creator_ = std::make_shared<ReferencelineCreator>();
        refer_line_pnb_ = this->create_publisher<Path>("reference_line", 10);

        // 创建决策器
        decider_ = std::make_shared<DecisionCenter>();

        // 创建局部规划器和发布器
        local_path_planner_ = std::make_shared<LocalPathPlanner>();
        local_path_pnb_ = this->create_publisher<Path>("local_path", 10);
        local_speeds_planner_ = std::make_shared<LocalSpeedsPlanner>();

        // 创建轨迹合成器和发布器
        local_trajectory_combiner_ = std::make_shared<LocalTrajectoryCombiner>();
        local_trajectory_pnb_ = this->create_publisher<LocalTrajectory>("local_trajectory", 10);

        // 创建绘图信息发布器
        plot_info_pub_ = this->create_publisher<PlotInfo>("plot_info", 10);

    }

    bool PlanningProcess::process() // 总流程
    {
        // 阻塞1s，等待rviz和xacro模型先启动
        rclcpp::Rate rate(0.5);
        rate.sleep();
        
        // 规划初始化
        if (!planning_init())
        {
            RCLCPP_ERROR(this->get_logger(), "planning_init 失败");
            return false;
        }
        
        // 进入规划主流程
        timer_ = this->create_wall_timer(0.1s, std::bind(&PlanningProcess::planning_callback, this));

        return true;
    }


    bool PlanningProcess::planning_init() // 规划初始化
    {
        // 生成车辆
        vehicle_spawn(car_);
        for (const auto &obs_car : obses_spawn_) 
        {
            vehicle_spawn(obs_car);
        }

        // 连接地图服务器
        if (!connect_server(map_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "地图服务连接失败");
            return false;
        }

        // 获取地图
        if (!map_request())
        {
            RCLCPP_ERROR(this->get_logger(), "地图请求和响应失败");
            return false;
        }

        // 连接全局路径服务器
        if (!connect_server(global_path_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "全局路径服务连接失败");
            return false;
        }

        // 获取全局路径
        if (!global_path_request())
        {
            RCLCPP_ERROR(this->get_logger(), "全局路径请求和响应失败");
            return false;
        }

        return true;
    }

    void PlanningProcess::vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle) // 车辆生成
    {
        TransformStamped spawn;
        spawn.header.stamp = this->now();
        spawn.header.frame_id = process_config_->pnc_map().frame_;  // 地图坐标系
        spawn.child_frame_id = vehicle->child_frame(); //  地图坐标系的子坐标设为车辆坐标系

        spawn.transform.translation.x = vehicle->loc_point().pose.position.x;
        spawn.transform.translation.y = vehicle->loc_point().pose.position.y;
        spawn.transform.translation.z = vehicle->loc_point().pose.position.z;
        spawn.transform.rotation.x = vehicle->loc_point().pose.orientation.x;
        spawn.transform.rotation.y = vehicle->loc_point().pose.orientation.y;
        spawn.transform.rotation.z = vehicle->loc_point().pose.orientation.z;
        spawn.transform.rotation.w = vehicle->loc_point().pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "车辆 %s 生成, x = %.2f, y = %.2f", 
                                         spawn.child_frame_id.c_str(), 
                                         vehicle->loc_point().pose.position.x, 
                                         vehicle->loc_point().pose.position.y);
        tf_broadcaster_->sendTransform(spawn);
    }

    // 函数的职责是利用 ROS2 的 tf2 库，
    // 从 tf 坐标变换树中查询并获取指定车辆（主车或障碍物）在全局地图坐标系下的最新位置和姿态，然后更新车辆对象自身的内部状态。
    void PlanningProcess::get_location(const std::shared_ptr<VehicleBase> &vehicle) // 获取位置,监听定位点
    {
        try {
            PoseStamped point;
            auto ts = buffer_->lookupTransform(process_config_->pnc_map().frame_, vehicle->child_frame(), tf2::TimePointZero);
            point.header = ts.header;
            point.pose.position.x = ts.transform.translation.x;
            point.pose.position.y = ts.transform.translation.y;
            point.pose.position.z = ts.transform.translation.z;
            point.pose.orientation.x = ts.transform.rotation.x;
            point.pose.orientation.y = ts.transform.rotation.y;
            point.pose.orientation.z = ts.transform.rotation.z;
            point.pose.orientation.w = ts.transform.rotation.w;
            vehicle->update_location(point);

        } 
        catch (const tf2::LookupException &e) {
            RCLCPP_ERROR(this->get_logger(), "查找异常: %s", e.what());
        }
    }

   
    

    template<typename T>
    bool PlanningProcess::connect_server(const T &client) // 连接服务器
    {
        // 判断客户端类型
        std::string server_name;
        if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>)
        {
            server_name = "pnc_map";
        }
        else if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>) 
        {
            server_name = "global_path";
        }
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Wrong client type!");
            return false;
        }
        // 等待服务器
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())  // 对ctrl+c操作处理，防止进入死循环
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s server...", server_name.c_str());
                return false;
            }
            RCLCPP_ERROR(this->get_logger(), "%s server not available, waiting again...", server_name.c_str());
        }

        return true;
    }

    bool PlanningProcess::map_request() // 发送地图请求
    {
        RCLCPP_INFO(this->get_logger(),"----发送地图请求---");

        // 生成请求
        auto request = std::make_shared<PNCMapService::Request>();
        request->map_type = process_config_->pnc_map().type_;
        RCLCPP_INFO(this->get_logger(), "地图类型请求值: %d (0=直道STRAIGHT, 1=弯道STERN)", request->map_type);

        // 获取响应
        auto result_future = map_client_->async_send_request(request);

        // 判断响应是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) 
        {
            pnc_map_ = result_future.get()->pnc_map;
            RCLCPP_INFO(this->get_logger(),"---地图请求成功---");
            return true;
        }
        else 
        {
            RCLCPP_ERROR(this->get_logger(),"地图请求失败");
            return false;
        }
        return false;
    }

    bool PlanningProcess::global_path_request() // 请求全局路径
    {
        RCLCPP_INFO(this->get_logger(),"发送全局路径请求");

        // 生成请求
        auto request = std::make_shared<GlobalPathService::Request>();
        request->global_planner_type = process_config_->global_path().type_;
        RCLCPP_INFO(this->get_logger(), "全局路径配置 type = %d", request->global_planner_type);
        request->pnc_map = pnc_map_;

        // 获取响应
        auto result_future = global_path_client_->async_send_request(request);

        // 判断响应是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) 
        {
            global_path_ = result_future.get()->global_path;
            RCLCPP_INFO(this->get_logger(),"---全局路径响应成功 ---");
            return true;
        }
        else 
        {
            RCLCPP_ERROR(this->get_logger(),"全局路径响应失败");
            return false;
        }
        return false;
    }

    void PlanningProcess::planning_callback() // 总流程回调
    {
        const auto start_time = this->get_clock()->now();
        
        // TODO: 实现规划回调逻辑
        // 监听车辆定位
        get_location(car_);
        obses_.clear();
        for (const auto &obs : obses_spawn_) 
        {
            get_location(obs);
            if (std::hypot(car_->loc_point().pose.position.x - obs->loc_point().pose.position.x, 
                           car_->loc_point().pose.position.y - obs->loc_point().pose.position.y) > obs_dis_) // 距离太远的障碍物不考虑
            {
                continue;
            }
            obses_.emplace_back(obs);
        }

        // 参考线
        base_msgs::msg::Referline refer_line;
        if (reference_line_type_ == 1) {
            RCLCPP_INFO(this->get_logger(), "参考线类型: 拼接(stich)");
            // 拼接模式
            refer_line = refer_line_creator_->create_reference_line(prev_refer_line_, global_path_, car_->loc_point());
        } else {
            RCLCPP_INFO(this->get_logger(), "参考线类型: 普通(normal)");
            // 普通模式
            refer_line = refer_line_creator_->create_reference_line(global_path_, car_->loc_point());
        }
        if (refer_line.refer_line.empty()) {
            RCLCPP_ERROR(this->get_logger(), "参考线为空");
            return;
        }
        // 保存上一帧参考线
        prev_refer_line_ = refer_line;
        const auto refer_line_rviz = refer_line_creator_->referline_to_rviz();  // 生成rviz的参考线
        refer_line_pnb_->publish(refer_line_rviz);  // 发布参考线

        // 主车和障碍物向参考线投影
        car_->vehicle_cartesian_to_frenet(refer_line);
        for (const auto &obs : obses_) 
        {
            obs->vehicle_cartesian_to_frenet(refer_line);
        }

        // 障碍物按S值排序
        std::sort(obses_.begin(), obses_.end(), [](const std::shared_ptr<VehicleBase> &obs1, const std::shared_ptr<VehicleBase> &obs2) 
        {
            return obs1->s() < obs2->s();
        });

        // 路径决策
        decider_->make_path_decision(car_, obses_);

        // 路径规划
        const auto local_path = local_path_planner_->creat_local_path(refer_line, car_, decider_); // 生成局部路径
        if (local_path.local_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "局部路径为空");
            return;
        }
        const auto local_path_rviz = local_path_planner_->path_to_rviz(); // 生成rviz的局部路径
        local_path_pnb_->publish(local_path_rviz); // 发布局部路径


        // 障碍物向路径投影
        for (const auto &obs : obses_) 
        {
            obs->vehicle_cartesian_to_frenet_2path(local_path, refer_line, car_);
        }
        // 速度决策
        decider_->make_speed_decision(car_, obses_);
        
        // 速度规划
        const auto local_speeds = local_speeds_planner_->cal_speed(decider_);
        if (local_speeds.local_speeds.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "局部速度为空");
            return;
        }

        // 合成轨迹
        const auto local_trajectory = local_trajectory_combiner_->combine_trajectory(local_path, local_speeds);
        if (local_trajectory.local_trajectory.empty()) {
            RCLCPP_ERROR(this->get_logger(), "轨迹为空");
            return;
        }
        // const auto local_trajectory_rviz = local_trajectory_combiner_->trajectory_to_rviz(); // 生成rviz的轨迹
        local_trajectory_pnb_->publish(local_trajectory); // 发布轨迹

        // 更新绘图信息
        PlotInfo plot_info;
        plot_info.header.stamp = this->now();
        plot_info.header.frame_id = process_config_->pnc_map().frame_;
        plot_info.trajectory_info = local_trajectory;

        ObsInfo obs_info;
        for (const auto &obs : obses_)
        {
            obs_info.obs_length = obs->length();
            obs_info.obs_width = obs->width();
            obs_info.l = obs->l();
            obs_info.s = obs->s();
            obs_info.s_2path = obs->s_2path();
            obs_info.ds_dt_2path = obs->ds_dt_2path();
            obs_info.t_in = obs->t_in();
            obs_info.t_out = obs->t_out();
            plot_info.obs_info.emplace_back(obs_info);
        }

        plot_info_pub_->publish(plot_info);

        // 更新车辆信息
        
        car_->update_cartesian_info(local_trajectory.local_trajectory[0]);
        RCLCPP_INFO(this->get_logger(), "车辆状态: loc= ( %.2f, %.2f),speed= %.2f,a= %.2f,theta= %.2f,kappa= %.2f", 
                                         car_->loc_point().pose.position.x, 
                                         car_->loc_point().pose.position.y, 
                                         car_->speed(), 
                                         car_->acceleration(), 
                                         car_->theta(), 
                                         car_->kappa());

        const auto end_time = this->get_clock()->now();
        const double planner_total_time = end_time.seconds() - start_time.seconds();
        RCLCPP_INFO(this->get_logger(), "规划总时间: %f ms\n", planner_total_time * 1000);

        // 防止系统卡死
        if(planner_total_time > 1.0)
        {
            RCLCPP_ERROR(this->get_logger(), "规划总时间超过1秒，系统卡死");
            rclcpp::shutdown();
        }
    }

} // namespace Planning