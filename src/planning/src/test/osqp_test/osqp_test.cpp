#include "osqp_test.h"
#include <iostream>

namespace Planning
{
    OsqpTest::OsqpTest(): Node("osqp_test_node") // osqp测试
    {
        RCLCPP_INFO(this->get_logger(), "osqp_test_node 已创建");
        test_problem();
    }

    void OsqpTest::test_problem()   // 示例问题
    {
        // RCLCPP_INFO(this->get_logger(), "test_problem");
        Eigen::SparseMatrix<double> P(2, 2);  // P: 二次型矩阵
        Eigen::VectorXd Q(2);                         // Q: 一次项向量
        Eigen::SparseMatrix<double> A(2, 2);  // A： 单位阵
        Eigen::VectorXd lowerBound(2);                // 下边界向量
        Eigen::VectorXd upperBound(2);                // 上边界向量

        P.insert(0, 0) = 2.0;
        P.insert(1, 1) = 2.0;
        std::cout << "P: "<< std::endl
                  << P << std::endl;
        Q << -2, -2;
        std::cout << "Q: "<< std::endl
        << Q << std::endl;
        A.insert(0, 0) = 1.0;
        A.insert(1, 1) = 1.0;
        std::cout << "A: "<< std::endl 
                  << A << std::endl;

        lowerBound << 0, 0;
        upperBound << 1.5, 1.5;
        
        OsqpEigen::Solver solver;  // 创建求解器

        solver.settings()->setVerbosity(false);  // 设置
        solver.settings()->setWarmStart(true);  // 设置

        // 初始化
        solver.data()->setNumberOfVariables(2);  // 设置变量个数
        solver.data()->setNumberOfConstraints(2);  // 设置约束个数

        if (!solver.data()->setHessianMatrix(P)) {
            RCLCPP_ERROR(this->get_logger(), "设置二次型矩阵失败");
            return;
        }

        if (!solver.data()->setGradient(Q)) {
            RCLCPP_ERROR(this->get_logger(), "设置一次项向量失败");
            return;
        }

        if (!solver.data()->setLinearConstraintsMatrix(A)) {
            RCLCPP_ERROR(this->get_logger(), "设置线性约束矩阵失败");
            return;
        }

        if (!solver.data()->setLowerBound(lowerBound)) {
            RCLCPP_ERROR(this->get_logger(), "设置下界失败");
            return;
        }

        if (!solver.data()->setUpperBound(upperBound)) {
            RCLCPP_ERROR(this->get_logger(), "设置上界失败");
            return;
        }
        
        if (!solver.initSolver()) {
            RCLCPP_ERROR(this->get_logger(), "初始化求解器失败");
            return;
        }
        
        Eigen::VectorXd QPSolution;   // 待求解的变量

        // 求解
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            RCLCPP_ERROR(this->get_logger(), "求解失败");
            return;
        }
        
        QPSolution = solver.getSolution();
        std::cout << "QPSolution: "<< std::endl 
                  << QPSolution << std::endl;

    }
    
}   // namespace Planning

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::OsqpTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}