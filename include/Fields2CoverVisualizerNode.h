//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#ifndef FIELDS2COVER_ROS_VISUALIZER_NODE_H_
#define FIELDS2COVER_ROS_VISUALIZER_NODE_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <dynamic_reconfigure/server.h>
#include <fields2cover_ros/F2CConfig.h>
#include <fields2cover.h>

namespace fields2cover_ros {

  class VisualizerNode {
    public:
      void init_VisualizerNode();
      void publish_topics(void);
      void rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level);

    public:
      ros::NodeHandle private_node_handle_ { "~" };
      ros::NodeHandle public_node_handle_;

      ros::Publisher field_polygon_publisher_;
      ros::Publisher field_no_headlands_publisher_;
      ros::Publisher field_gps_publisher_;
      ros::Publisher field_swaths_publisher_;

      sensor_msgs::NavSatFix gps_;

      f2c::Transform transf_;

      F2CFields fields_;
      F2CRobot robot_{2.1, 2.5};
      F2COptim optim_;

      bool automatic_angle_ {false};
      int sg_objective_ {0};
      int opt_turn_type_ {0};
      int opt_route_type_ {0};
  };
}

#endif  // FIELDS2COVER_ROS_VISUALIZER_NODE_H_
