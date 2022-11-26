//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include <fields2cover.h>
#include <vector>

#ifdef IS_ROS2
  #include <nav_msgs/msg/path.hpp>
  #include <geometry_msgs/msg/polygon.hpp>
  #include <geometry_msgs/msg/pose.hpp>
  #include <geometry_msgs/msg/point32.hpp>
  #include <geometry_msgs/msg/point.h>
#else
  #include <nav_msgs/Path.h>
  // #include <visualization_msgs/Marker.h>
  // #include <visualization_msgs/MarkerArray.h>
  #include <geometry_msgs/Polygon.h>
  #include <geometry_msgs/Pose.h>
  #include <geometry_msgs/Point32.h>
  #include <geometry_msgs/Point.h>
#endif


namespace conversor {

#ifdef IS_ROS2
  namespace GeometryMsgs = geometry_msgs::msg;
  namespace NavMsgs = nav_msgs::msg;
#else
  namespace GeometryMsgs = geometry_msgs;
  namespace NavMsgs = nav_msgs;
#endif

class ROS {
 public:
  static void to(const F2CPoint& _point, GeometryMsgs::Point32& _p32);
  static void to(const F2CPoint& _point, GeometryMsgs::Point& _p64);

  template <class T>
  static void to(const T& _curve, GeometryMsgs::Polygon& _poly);

  static void to(const F2CCell& _poly,
    std::vector<GeometryMsgs::Polygon>& _ros_poly);

  static void to(const F2CCells& _polys,
    std::vector<std::vector<GeometryMsgs::Polygon>>& _ros_polys);

  static void to(const F2CLineString& _line, NavMsgs::Path& _path);

  // static void to(const F2CLineString& _line,
  //  visualization_msgs::MarkerArray& _markers);
  // static void to(const OGRMultiLineString& _line,
  //  visualization_msgs::MarkerArray& _markers);
};

template <class T>
inline void ROS::to(const T& _curve, GeometryMsgs::Polygon& _poly) {
  GeometryMsgs::Point32 p32;
  for (const auto& p : _curve) {
    to(p, p32);
    _poly.points.push_back(p32);
  }
}

}  // namespace conversor
