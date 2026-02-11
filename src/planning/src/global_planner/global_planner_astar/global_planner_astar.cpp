/*
文件名: A星全局路径规划器
作者: Your Name
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
        nav_msgs
    外部库:
        base_msgs
        config_reader
*/

#include "global_planner_astar.h"
#include <limits>
#include <functional>
#include <queue>
#include <cmath>

namespace Planning
{
    GlobalPlannerAStar::GlobalPlannerAStar() // A星全局路径规划器
    {
        RCLCPP_INFO(rclcpp::get_logger("global_planner"), "全局路径_AStar 已创建");
    }

    Path GlobalPlannerAStar::search_global_path(const PNCMap &pnc_map) // 基于拓扑图搜索全局路径
    {
        RCLCPP_INFO(rclcpp::get_logger("global_planner"), "开始 基于拓扑图 的 A* 搜索...");

        // 1) 构建拓扑图（以中线点为节点，顺序相邻为边）
        build_topology_graph_from_midline(pnc_map);
        const int N = static_cast<int>(graph_nodes_.size());
        if (N < 2)
        {
            RCLCPP_ERROR(rclcpp::get_logger("global_planner"), "中线点过少，无法构建拓扑图");
            return global_path_;
        }

        // 2) 选择起终点：取“中线与右边界”的中点作为真实起点/终点，并在拓扑图上选取最近的节点索引
        int start_index = 0;
        int goal_index = N - 1;
        double start_mid_x = graph_nodes_.front().x;
        double start_mid_y = graph_nodes_.front().y;
        double goal_mid_x = graph_nodes_.back().x;
        double goal_mid_y = graph_nodes_.back().y;
        if (!pnc_map.right_boundary.points.empty())
        {
            const auto &mid0 = pnc_map.midline.points.front();
            const auto &rb0  = pnc_map.right_boundary.points.front();
            start_mid_x = (mid0.x + rb0.x) / 2.0;
            start_mid_y = (mid0.y + rb0.y) / 2.0;

            const auto &midN = pnc_map.midline.points.back();
            const auto &rbN  = pnc_map.right_boundary.points.back();
            goal_mid_x = (midN.x + rbN.x) / 2.0;
            goal_mid_y = (midN.y + rbN.y) / 2.0;

            // 在拓扑图上寻找与上述中点最近的节点索引
            auto nearest_index = [&](double tx, double ty) -> int {
                int idx = 0;
                double best = std::numeric_limits<double>::infinity();
                for (int i = 0; i < N; ++i)
                {
                    const double dx = graph_nodes_[i].x - tx;
                    const double dy = graph_nodes_[i].y - ty;
                    const double d2 = dx * dx + dy * dy;
                    if (d2 < best) { best = d2; idx = i; }
                }
                return idx;
            };
            start_index = nearest_index(start_mid_x, start_mid_y);
            goal_index  = nearest_index(goal_mid_x, goal_mid_y);
        }

        // 3) 在拓扑图上运行 A*
        const auto path_on_graph = search_on_topology(start_index, goal_index);

        // 4) 填充输出 Path
        global_path_.header.frame_id = pnc_map.header.frame_id;
        global_path_.header.stamp = rclcpp::Clock().now();
        global_path_.poses.clear();
        for (const auto &pose : path_on_graph.poses)
        {
            global_path_.poses.emplace_back(pose);
        }
        // 用真实起终点中点替换第一/最后一个点的坐标，使轨迹端点位于“中线-右边界”的中间位置
        if (!global_path_.poses.empty())
        {
            global_path_.poses.front().pose.position.x = start_mid_x;
            global_path_.poses.front().pose.position.y = start_mid_y;
            global_path_.poses.back().pose.position.x  = goal_mid_x;
            global_path_.poses.back().pose.position.y  = goal_mid_y;
        }
        if (global_path_.poses.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("global_planner"), "拓扑A*未找到路径，返回空路径");
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("global_planner"), "拓扑A*完成，点数: %zu", global_path_.poses.size());
        }
        return global_path_;
    }

    double GlobalPlannerAStar::heuristic(Node* a, Node* b)
    {
        return std::sqrt(std::pow(a->x - b->x, 2) + std::pow(a->y - b->y, 2));
    }

    void GlobalPlannerAStar::reconstruct_path(Node* goal_node)
    {
        global_path_.poses.clear();
        Node* current_node = goal_node;
        while (current_node != nullptr)
        {
            PoseStamped pose;
            pose.pose.position.x = current_node->x;
            pose.pose.position.y = current_node->y;
            pose.pose.position.z = 0;
            global_path_.poses.insert(global_path_.poses.begin(), pose);
            current_node = current_node->parent;
        }
        // header 在 search_global_path 中设置
    }

    // 从“中线-右边界”的中点构建拓扑图：顺序相邻连边
    void GlobalPlannerAStar::build_topology_graph_from_midline(const PNCMap &pnc_map)
    {
        graph_nodes_.clear();
        const auto &mid_pts = pnc_map.midline.points;
        const auto &rb_pts  = pnc_map.right_boundary.points;
        if (mid_pts.empty()) { return; }

        const bool has_rb = (!rb_pts.empty());
        graph_nodes_.resize(mid_pts.size());
        for (size_t i = 0; i < mid_pts.size(); ++i)
        {
            if (has_rb)
            {
                const size_t idx_rb = std::min(i, rb_pts.size() - 1);
                graph_nodes_[i].x = (mid_pts[i].x + rb_pts[idx_rb].x) / 2.0;
                graph_nodes_[i].y = (mid_pts[i].y + rb_pts[idx_rb].y) / 2.0;
            }
            else
            {
                graph_nodes_[i].x = mid_pts[i].x;
                graph_nodes_[i].y = mid_pts[i].y;
            }
        }
        for (size_t i = 0; i + 1 < mid_pts.size(); ++i)
        {
            graph_nodes_[i].neighbors.emplace_back(static_cast<int>(i + 1));
            graph_nodes_[i + 1].neighbors.emplace_back(static_cast<int>(i));
        }
    }

    // 在拓扑图上进行A*搜索
    Path GlobalPlannerAStar::search_on_topology(int start_index, int goal_index)
    {
        Path path;
        path.header.frame_id = ""; // 由调用者设置
        path.poses.clear();

        const int N = static_cast<int>(graph_nodes_.size());
        if (N == 0 || start_index < 0 || goal_index < 0 || start_index >= N || goal_index >= N)
        {
            return path;
        }

        auto dist_xy = [&](int i, int j) -> double {
            const double dx = graph_nodes_[i].x - graph_nodes_[j].x;
            const double dy = graph_nodes_[i].y - graph_nodes_[j].y;
            return std::sqrt(dx * dx + dy * dy);
        };

        std::vector<double> g(N, std::numeric_limits<double>::infinity());
        std::vector<double> f(N, std::numeric_limits<double>::infinity());
        std::vector<int> parent(N, -1);
        std::vector<char> closed(N, 0);

        using QItem = std::pair<double, int>; // f, index
        std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

        g[start_index] = 0.0;
        f[start_index] = dist_xy(start_index, goal_index);
        open.emplace(f[start_index], start_index);

        while (!open.empty())
        {
            const auto [f_cur, u] = open.top();
            (void)f_cur;
            open.pop();
            if (closed[u]) continue;
            closed[u] = 1;

            if (u == goal_index) { break; }

            for (int v : graph_nodes_[u].neighbors)
            {
                if (closed[v]) continue;
                const double w = dist_xy(u, v); // 边权为欧氏距离
                const double g_new = g[u] + w;
                if (g_new < g[v])
                {
                    g[v] = g_new;
                    parent[v] = u;
                    f[v] = g[v] + dist_xy(v, goal_index);
                    open.emplace(f[v], v);
                }
            }
        }

        // 无法到达
        if (parent[goal_index] == -1 && start_index != goal_index)
        {
            return path;
        }

        // 回溯路径
        std::vector<int> idx_path;
        for (int cur = goal_index; cur != -1; cur = parent[cur])
        {
            idx_path.emplace_back(cur);
        }
        std::reverse(idx_path.begin(), idx_path.end());

        PoseStamped p;
        p.header.stamp = rclcpp::Clock().now();
        for (int id : idx_path)
        {
            p.pose.position.x = graph_nodes_[id].x;
            p.pose.position.y = graph_nodes_[id].y;
            p.pose.position.z = 0.0;
            p.pose.orientation.w = 1.0;
            path.poses.emplace_back(p);
        }

        return path;
    }

} // namespace Planning
