//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <fields2cover_ros/F2CConfig.h>
#include "Fields2CoverVisualizerNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fields2cover_visualizer_node");
    fields2cover_ros::VisualizerNode visualizer_node;
    visualizer_node.init_VisualizerNode();
    dynamic_reconfigure::Server<fields2cover_ros::F2CConfig> server;
    //dynamic_reconfigure::Server<fields2cover_ros::F2CConfig>::CallbackType f;
    auto f = boost::bind(&fields2cover_ros::VisualizerNode::rqt_callback, boost::ref(visualizer_node), _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(10);
    
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
