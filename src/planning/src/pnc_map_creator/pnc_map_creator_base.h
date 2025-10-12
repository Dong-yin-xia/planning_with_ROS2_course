#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "base_msgs/msg/pnc_map.hpp"
#include "config_reader.h"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>

namespace Planning {
using base_msgs::msg::PNCMap;
using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

enum class PNCMapType // 地图类型,强枚举类
{
  STRAIGHT,
  STERN,
};

class PNCMapCreatorBase // pnc_map创建器基类
{
public:
  virtual PNCMap create_pnc_map() = 0; // 创建pnc地图
  inline PNCMap pnc_map() const {return pnc_map_;}  // 获取pnc地图
  inline MarkerArray pnc_map_markerarray() const {return pnc_map_markerarray_;}  // 获取rviz地图markerarray
  virtual ~PNCMapCreatorBase() {}

protected: // 这里用保护成员变量，子类可以继承并访问，但对象中不可访问
  std::unique_ptr<ConfigReader> pnc_map_config_; // 地图配置
  int map_type_ = 0;                             // 地图类型
  PNCMap pnc_map_;                               // pnc地图
  MarkerArray pnc_map_markerarray_;              // rviz地图markerarray

  Point p_mid_, pl_, pr_;      // 当前点，左点，右点
  double theta_current_ = 0.0; // 当前角度
  double len_step_ = 0.0;      // 长度步长
  double theta_step_ = 0.0;    // 角度步长
};
} // namespace Planning
#endif // PNC_MAP_CREATOR_BASE_H_
