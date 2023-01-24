//=============================================================================
//    Copyright (C) 2021-2022 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================


#include "Fields2CoverVisualizerNode.h"
#include "ros/conversor.h"

#include <fstream>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <fields2cover_ros/F2CConfig.h>


using namespace std;


namespace fields2cover_ros {
    
    void VisualizerNode::init_VisualizerNode() {
      field_polygon_publisher_ = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/border", 1, true);
      field_no_headlands_publisher_ = public_node_handle_.advertise<geometry_msgs::PolygonStamped>("/field/no_headlands", 1, true);
      field_gps_publisher_ = public_node_handle_.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1, true);
      field_swaths_publisher_ = public_node_handle_.advertise<visualization_msgs::Marker>("/field/swaths", 1, true);


      std::string field_file;
      private_node_handle_.getParam(
          "field_file", field_file);

      f2c::Parser::importGml(field_file, fields_);
      
      f2c::Transform::transform(fields_[0], "EPSG:28992");
      robot_.cruise_speed = 2.0;
      robot_.setMinRadius(2.0);
      double headland_width = 3.0*robot_.op_width;
    }

    void VisualizerNode::publish_topics(void) {
      auto gps = transf_.getRefPointInGPS(fields_[0]);
      gps_.longitude = gps.getX();
      gps_.latitude = gps.getY();
      gps_.altitude = gps.getZ();
      gps_.header.stamp = ros::Time::now();
      gps_.header.frame_id = "base_link";
      field_gps_publisher_.publish(gps_);

      auto f = fields_[0].field.clone();
      f2c::hg::ConstHL hl_gen_;
      F2CCell no_headlands = hl_gen_.generateHeadlands(f, optim_.headland_width).getGeometry(0);

      geometry_msgs::PolygonStamped polygon_st;
      polygon_st.header.stamp = ros::Time::now();
      polygon_st.header.frame_id = "base_link";
      conversor::ROS::to(f.getCellBorder(0), polygon_st.polygon);
      field_polygon_publisher_.publish(polygon_st);
      polygon_st.polygon.points.clear();
      
      geometry_msgs::PolygonStamped polygon_st2;
      polygon_st2.header.stamp = ros::Time::now();
      polygon_st2.header.frame_id = "base_link";
      conversor::ROS::to(no_headlands.getGeometry(0), polygon_st2.polygon);
      field_no_headlands_publisher_.publish(polygon_st2);
      polygon_st2.polygon.points.clear();



      F2CSwaths swaths;
      f2c::sg::BruteForce swath_gen_;
      if (automatic_angle_) {
        switch (sg_objective_) {
          case 0 : {
            f2c::obj::SwathLength obj;
            swaths = swath_gen_.generateBestSwaths(obj, robot_.op_width, no_headlands);
            break;
	  }
          case 1 : {
            f2c::obj::NSwath obj;
            swaths = swath_gen_.generateBestSwaths(obj, robot_.op_width, no_headlands);
            break;
	  }
          case 2 : {
            f2c::obj::FieldCoverage obj;
            swaths = swath_gen_.generateBestSwaths(obj, robot_.op_width, no_headlands);
            break;
	  }
	}
      }
      else {
        swaths = swath_gen_.generateSwaths(optim_.best_angle,
            robot_.op_width, no_headlands);
      }

      F2CSwaths route;
      switch (opt_route_type_) {
        case 0 : {
          f2c::rp::BoustrophedonOrder swath_sorter;
          route = swath_sorter.genSortedSwaths(swaths);
          break;
        }
        case 1 : {
          f2c::rp::SnakeOrder swath_sorter;
          route = swath_sorter.genSortedSwaths(swaths);
          break;
        }
        case 2 : {
          f2c::rp::SpiralOrder swath_sorter(6);
          route = swath_sorter.genSortedSwaths(swaths);
          break;
        }
        case 3 : {
          f2c::rp::SpiralOrder swath_sorter(4);
          route = swath_sorter.genSortedSwaths(swaths);
          break;
        }
      }

      F2CPath path;
      f2c::pp::PathPlanning path_planner;

      switch(opt_turn_type_) {
        case 0 : {
          f2c::pp::DubinsCurves turn;
          path = path_planner.searchBestPath(robot_, route, turn);
          break;
        }
        case 1 : {
          f2c::pp::DubinsCurvesCC turn;
          path = path_planner.searchBestPath(robot_, route, turn);
          break;
        }
        case 2 : {
          f2c::pp::ReedsSheppCurves turn;
          path = path_planner.searchBestPath(robot_, route, turn);
          break;
        }
        case 3 : {
          f2c::pp::ReedsSheppCurvesHC turn;
          path = path_planner.searchBestPath(robot_, route, turn);
          break;
        }
      }


      visualization_msgs::Marker marker_swaths;
      marker_swaths.header.frame_id = "base_link";
      marker_swaths.header.stamp = ros::Time::now();
      marker_swaths.action = visualization_msgs::Marker::ADD;
      marker_swaths.pose.orientation.w = 1.0;
      marker_swaths.type = visualization_msgs::Marker::LINE_STRIP;
      marker_swaths.scale.x = 1.0;
      marker_swaths.scale.y = 1.0;
      marker_swaths.scale.z = 1.0;
      marker_swaths.color.b = 1.0;
      marker_swaths.color.a = 1.0;
      geometry_msgs::Point ros_p;

      for (auto&& s : path.states) {
	conversor::ROS::to(s.point, ros_p);
        marker_swaths.points.push_back(ros_p);
      }
      field_swaths_publisher_.publish(marker_swaths);
    }

    void VisualizerNode::rqt_callback(fields2cover_ros::F2CConfig &config, uint32_t level) {
      robot_.op_width = config.op_width;
      robot_.setMinRadius(config.turn_radius);
      optim_.best_angle = config.swath_angle;
      optim_.headland_width = config.headland_width;
      automatic_angle_ = config.automatic_angle;
      sg_objective_ = config.sg_objective;
      opt_turn_type_ = config.turn_type;
      opt_route_type_ = config.route_type;
      publish_topics();
    }
}


