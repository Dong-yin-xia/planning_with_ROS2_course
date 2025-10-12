#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include <Eigen/Dense>           // eigen
#include <OsqpEigen/OsqpEigen.h> // osqp-eigen
#include <cmath>

#include "config_reader.h"

namespace Planning
{
    using base_msgs::msg::Referline;

    class ReferenceLineSmoother // 参考线平滑
    {
    public:
        ReferenceLineSmoother();
        void smooth_reference_line(Referline &refer_line);  // 平滑参考线

    private:
        std::unique_ptr<ConfigReader> reference_line_config_;   // 配置文件读取器
        const double w1 = 100.0;
        const double w2 = 10.0;
        const double w3 = 1.0;
        
    };
} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_