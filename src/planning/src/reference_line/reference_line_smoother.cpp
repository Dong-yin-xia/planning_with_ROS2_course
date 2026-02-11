/*
文件名: 参考线平滑器
作者: C哥智驾说
完成时间: 2024.12

编译类型: 动态库
依赖: ROS2内部库:
        rclcpp
        geometry_msgs
    外部库:
        base_msgs
        config_reader
        Eigen
        OsqpEigen

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/

#include "reference_line_smoother.h"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Constants.h"
#include "Eigen/src/SparseCore/SparseMatrix.h"

namespace Planning
{
    ReferenceLineSmoother::ReferenceLineSmoother() // 参考线平滑
    {
        RCLCPP_INFO(rclcpp::get_logger("reference_line"), "参考线平滑已创建");

        // 读取配置文件
        reference_line_config_ = std::make_unique<ConfigReader>();
        reference_line_config_->read_reference_line_config();
    }

    void ReferenceLineSmoother::smooth_reference_line(Referline &refer_line) // 平滑参考线
    {
        const int n = refer_line.refer_line.size();
        if (n < 3)
        {
            return;
        }

        // 构建矩阵P
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();  // 2x2 单位阵
        Eigen::Matrix2d W1 = 2.0 * w1 * I;  // 2x2 矩阵
        Eigen::Matrix2d W2 = 2.0 * w2 * I;  // 2x2 矩阵
        Eigen::Matrix2d W3 = 2.0 * w3 * I;  // 2x2 矩阵

        Eigen::Matrix2d block1 = W1 + W2 + W3;
        Eigen::Matrix2d block2 = -2.0 * W1 - W2;
        Eigen::Matrix2d block3 = -4.0 * W1 - W2;
        Eigen::Matrix2d block4 = 5.0 *W1 + 2.0 * W2 + W3;
        Eigen::Matrix2d block5 = 6.0 *W1 + 2.0 * W2 + W3;

        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(2 * n, 2 * n);   // 初始化为全0矩阵

        if (n == 3)
        {
            // 上三角部分
            // | W1+W2+W3  -2W1-W2       W1       |
            // |     0     4W1+2W2+W3   -2W1-W2   |
            // |                         W1+W2+W3 |

            // 只填充上三角部分
            P_tmp.block<2, 2>(0, 0) = block1;
            P_tmp.block<2, 2>(0, 2) = block2;
            P_tmp.block<2, 2>(0, 4) = W1;
            P_tmp.block<2, 2>(2, 2) = 4.0 * W1 + 2.0 * W2 + W3;
            P_tmp.block<2, 2>(2, 4) = block2;
            P_tmp.block<2, 2>(4, 4) = block1;
        }
        else 
        {
            // 上三角部分
            // | W1+W2+W3  -2W1-W2       W1              0            0     ...      0         |
            // |     0     5W1+2W2+W3   -4W1-W2          W1           0     ...      0         |  
            // |                         6W1+2W2+W3     -4W1-W2       W1    ...      0         |
            // |                                           .          .              .         |
            // |                                           .          .              .         |
            // |                                           .          .              .         |
            // |                                       6W1+2W2+W3   -4W1-W2         W1         |
            // |                                                    5W1+2W2+W3    -2W1-W2      |
            // |                                                                   W1+W2+W3    |

        // 只填充上三角部分
        for (int i = 0; i < n; ++i)
        {
            if (i == 0)  // 第0行
            {
                P_tmp.block<2, 2>(i * 2, i * 2) = block1;
                P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
                P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
            }
            else if (i == 1)  // 第1行
            {
                P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
            }
            else if (i == n - 2)  // 第n-2行
            {
                P_tmp.block<2, 2>(i * 2, i * 2) = block4;
                P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block2;
            }
            else if (i == n - 1)  // 第n-1行
            {
                P_tmp.block<2, 2>(i * 2, i * 2) = block1;
            }
            else 
            {
                P_tmp.block<2, 2>(i * 2, i * 2) = block5;
                P_tmp.block<2, 2>(i * 2, (i + 1) * 2) = block3;
                P_tmp.block<2, 2>(i * 2, (i + 2) * 2) = W1;
            }
            
        }
        }

        P_tmp = P_tmp.selfadjointView<Eigen::Upper>();  // 通过上三角矩阵构造对称阵
        Eigen::SparseMatrix<double> P = P_tmp.sparseView();  // 转换为稀疏矩阵

        Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(2 * n, 2 * n);
        Eigen::SparseMatrix<double> A = A_tmp.sparseView();  // 转换为稀疏矩阵

        // 原始点的坐标
        Eigen::VectorXd X(2 * n);
        for (int i = 0; i < n; ++i)
        {
            X(i * 2) = refer_line.refer_line[i].pose.pose.position.x;
            X(i * 2 + 1) = refer_line.refer_line[i].pose.pose.position.y;
        }

        Eigen::VectorXd Q = -2.0 * X;                                               // 一次项向量
        Eigen::VectorXd buff = Eigen::VectorXd::Constant(2 * n, 0.2);   // 偏差范围，动态列向量，2 * n行，值全为0.2
        buff(0) = buff(1) = buff(2 * n - 2) = buff(2 * n - 1) = 0.0;  // 第0、1、2n-2、2n-1行偏差范围为0

        Eigen::VectorXd lowerBound = X - buff;                    // 不等式约束的下边界
        Eigen::VectorXd upperBound = X + buff;                    // 不等式约束的上边界

        OsqpEigen::Solver solver;  // 创建求解器

        solver.settings()->setVerbosity(false);  // 设置
        solver.settings()->setWarmStart(true);  // 设置

        // 初始化
        solver.data()->setNumberOfVariables(2 * n);  // 设置变量个数
        solver.data()->setNumberOfConstraints(2 * n);  // 设置约束个数

        if (!solver.data()->setHessianMatrix(P)) {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "设置二次型矩阵失败");
            return;
        }

        if (!solver.data()->setGradient(Q)) {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "设置一次项向量失败");
            return;
        }

        if (!solver.data()->setLinearConstraintsMatrix(A)) {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "设置线性约束矩阵失败");
            return;
        }

        if (!solver.data()->setLowerBound(lowerBound)) {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "设置下界失败");
            return;
        }

        if (!solver.data()->setUpperBound(upperBound)) {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "设置上界失败");
            return;
        }
        
        if (!solver.initSolver()) {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "初始化求解器失败");
            return;
        }
        
        Eigen::VectorXd QPSolution;   // 待求解的变量

        // 求解
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
            RCLCPP_ERROR(rclcpp::get_logger("reference_line"), "求解失败");
            return;
        }
        
        QPSolution = solver.getSolution();

        // 把结果向量的数据更新到参考线中
        for (int i = 0; i < n; ++i)
        {
            refer_line.refer_line[i].pose.pose.position.x = QPSolution(i * 2);
            refer_line.refer_line[i].pose.pose.position.y = QPSolution(i * 2 + 1);
        }
        
    }

}// namespace Planning