#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorSTurn : public PNCMapCreatorBase // S弯道地图
    {
    public:
        PNCMapCreatorSTurn();
        PNCMap create_pnc_map() override;  // 创建S弯道地图

    private:
        void init_pnc_map(); // 初始化pnc地图
        void draw_straight_x(const double &length, const double &plus_flag, const double &ratio = 1.0); // 画直道x
        void draw_arc(const double &angle, const double &plus_flag, const double &ratio = 1.0); // 画弧线,逆时针为正方向，顺时针为负方向，angle为总角度
    };
} // namespace Planning
#endif // PNC_MAP_CREATOR_STURN_H_