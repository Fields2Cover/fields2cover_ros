//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include <fields2cover.h>
#include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <vector>

class ROS {
 public:
  static void to(const F2CPoint& _point, geometry_msgs::Point32& _p32);
  static void to(const F2CPoint& _point, geometry_msgs::Point& _p64);

  template <class T>
  static void to(const T& _curve, geometry_msgs::Polygon& _poly);

  static void to(const F2CCell& _poly,
    std::vector<geometry_msgs::Polygon>& _ros_poly);

  static void to(const F2CCells& _polys,
    std::vector<std::vector<geometry_msgs::Polygon>>& _ros_polys);

  static void to(const F2CLineString& _line, nav_msgs::Path& _path);

  // static void to(const F2CLineString& _line,
  //  visualization_msgs::MarkerArray& _markers);
  // static void to(const OGRMultiLineString& _line,
  //  visualization_msgs::MarkerArray& _markers);
};

template <class T>
inline void ROS::to(const T& _curve, geometry_msgs::Polygon& _poly) {
  geometry_msgs::Point32 p32;
  for (const auto& p : _curve) {
    to(p, p32);
    _poly.points.push_back(p32);
  }
}

