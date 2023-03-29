//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#ifndef FIELDS2COVER_ROS_VISUALIZER_NODE_H_
#define FIELDS2COVER_ROS_VISUALIZER_NODE_H_

#ifdef IS_ROS2
  #include "rclcpp/rclcpp.hpp"
  #include <visualization_msgs/msg/marker.hpp>
  #include <std_msgs/msg/color_rgba.hpp>
  #include <sensor_msgs/msg/nav_sat_fix.hpp>
  #include <geometry_msgs/msg/polygon_stamped.hpp>
#else
  #include <ros/ros.h>
  #include <visualization_msgs/Marker.h>
  #include <std_msgs/ColorRGBA.h>
  #include <geometry_msgs/PolygonStamped.h>
  #include <sensor_msgs/NavSatFix.h>
  #include <dynamic_reconfigure/server.h>
  #include <fields2cover_ros/F2CConfig.h>
#endif
#include <fields2cover.h>


namespace fields2cover_ros {

  #ifdef IS_ROS2
    namespace GeometryMsgs = geometry_msgs::msg;
    namespace SensorMsgs = sensor_msgs::msg;
    namespace StdMsgs = std_msgs::msg;
    namespace VisualizationMsgs = visualization_msgs::msg;

    typedef rclcpp::Publisher<GeometryMsgs::PolygonStamped>::SharedPtr Pub_PolygonStamped;
    typedef rclcpp::Publisher<SensorMsgs::NavSatFix>::SharedPtr Pub_NavSatFix;
    typedef rclcpp::Publisher<VisualizationMsgs::Marker>::SharedPtr Pub_Marker;
  #else
    namespace GeometryMsgs = geometry_msgs;
    namespace SensorMsgs = sensor_msgs;
    namespace StdMsgs = std_msgs;
    namespace VisualizationMsgs = visualization_msgs;

    typedef std::shared_ptr<ros::Publisher> Pub_PolygonStamped;
    typedef std::shared_ptr<ros::Publisher> Pub_NavSatFix;
    typedef std::shared_ptr<ros::Publisher> Pub_Marker;
  #endif


  #ifdef IS_ROS2
    class VisualizerNode : public rclcpp::Node {
  #else
    class VisualizerNode {
  #endif
     public:
      VisualizerNode();
      void publish_topics(void);
      auto timeNow() {
        #ifdef IS_ROS2
          return this->get_clock()->now();
        #else
          return ros::Time::now();
        #endif
      }


     public:
      #ifdef IS_ROS2
        rclcpp::TimerBase::SharedPtr timer_;
        void declareParams() {
          this->declare_parameter("field_file", "");
          this->declare_parameter("op_width", 3.);
          this->declare_parameter("turn_radius", 2.);
          this->declare_parameter("headland_width", 6.);
          this->declare_parameter("swath_angle", 0.01);
          this->declare_parameter("automatic_angle", false);
          this->declare_parameter("sg_objective", 0);
          this->declare_parameter("route_type", 0);
          this->declare_parameter("turn_type", 0);
        }
        void update() {
          robot_.op_width = this->get_parameter("op_width").get_parameter_value().get<double>();
          robot_.setMinRadius(this->get_parameter("turn_radius").get_parameter_value().get<double>());
          optim_.headland_width = this->get_parameter("headland_width").get_parameter_value().get<double>();
          optim_.best_angle = this->get_parameter("swath_angle").get_parameter_value().get<double>();
          automatic_angle_ = this->get_parameter("automatic_angle").get_parameter_value().get<bool>();
          sg_objective_ = this->get_parameter("sg_objective").get_parameter_value().get<int>();
          opt_route_type_ = this->get_parameter("route_type").get_parameter_value().get<int>();
          opt_turn_type_ = this->get_parameter("turn_type").get_parameter_value().get<int>();
          publish_topics();
        }
      #else
        void rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level);
        ros::NodeHandle private_node_handle_ { "~" };
        ros::NodeHandle public_node_handle_;
      #endif

      Pub_PolygonStamped field_polygon_publisher_;
      Pub_PolygonStamped field_no_headlands_publisher_;
      Pub_NavSatFix field_gps_publisher_;
      Pub_Marker field_swaths_publisher_;

      SensorMsgs::NavSatFix gps_;

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
