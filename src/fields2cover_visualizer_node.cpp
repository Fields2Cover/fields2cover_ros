//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#ifdef IS_ROS2
  #include <rclcpp/rclcpp.hpp>
#else
  #include <ros/ros.h>
  #include <dynamic_reconfigure/server.h>
  #include <fields2cover_ros/F2CConfig.h>
  #include <boost/bind.hpp>
#endif

#include <memory>
#include <functional>

#include "Fields2CoverVisualizerNode.h"

int main(int argc, char **argv) {
  #ifdef IS_ROS2
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fields2cover_ros::VisualizerNode>());
    rclcpp::shutdown();
  #else
    ros::init(argc, argv, "f2c_visualizer_node");
    fields2cover_ros::VisualizerNode visualizer_node;
    dynamic_reconfigure::Server<fields2cover_ros::F2CConfig> server;
    //dynamic_reconfigure::Server<fields2cover_ros::F2CConfig>::CallbackType f;
    auto f = boost::bind(
        &fields2cover_ros::VisualizerNode::rqt_callback,
        boost::ref(visualizer_node), _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
  #endif
  return 0;
}
