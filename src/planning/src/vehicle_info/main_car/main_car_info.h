#ifndef MAIN_CAR_BASE_H_
#define MAIN_CAR_BASE_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class MainCar : public VehicleBase // 主车
    {
    public:
        MainCar();

        // 定位点转frenet
        void vehicle_cartesian_to_frenet(const Referline &refer_line) override;  // 定位点在参考线上的投影点参数
        void vehicle_cartesian_to_frenet_2path(const LocalPath &local_path,
                                               const Referline &refer_line,
                                               const std::shared_ptr<VehicleBase> &car) override; // 输出：定位点在局部路径上的投影点参数

                                               
    };
} // namespace Planning
#endif // MAIN_CAR_BASE_H_