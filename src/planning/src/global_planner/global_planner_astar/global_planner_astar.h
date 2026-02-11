#ifndef GLOBAL_PLANNER_ASTAR_H_
#define GLOBAL_PLANNER_ASTAR_H_

#include "global_planner_base.h"
#include <vector>

namespace Planning
{
    // A* 节点结构体
    struct Node
    {
        double x, y; // 节点坐标
        double g, h, f; // A* 代价: g = 起点到当前节点的代价, h = 启发式函数评估的代价, f = g + h
        Node* parent; // 指向父节点的指针，用于路径重建

        // 构造函数 用于初始化 Node 结构体的成员变量。
        Node(double x, double y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}

        // 优先队列（最小堆）的比较函数 重载>运算符
        bool operator>(const Node& other) const
        {
            return f > other.f;
        }
    };

    // 拓扑图节点（基于中线点的图）
    struct TopoNode
    {
        double x = 0.0;
        double y = 0.0;
        std::vector<int> neighbors; // 邻接节点索引
    };

    // 表示 GlobalPlannerAStar 类公开继承 (publicly inherits) 自 GlobalPlannerBase 类。
    class GlobalPlannerAStar : public GlobalPlannerBase // A星全局路径规划器
    {
    public:
        GlobalPlannerAStar();
        Path search_global_path(const PNCMap &pnc_map) override; // 搜索全局路径

    private:
        // TODO: 在这里添加A*特有的成员变量和辅助函数
        double heuristic(Node* a, Node* b); // 启发式函数 (欧几里得距离)
        void reconstruct_path(Node* goal_node); // 从目标节点重建路径

        // 基于拓扑图的实现
        void build_topology_graph_from_midline(const PNCMap &pnc_map);
        Path search_on_topology(int start_index, int goal_index);

    private:
        std::vector<TopoNode> graph_nodes_;
    };
} // namespace Planning

#endif // GLOBAL_PLANNER_ASTAR_H_
