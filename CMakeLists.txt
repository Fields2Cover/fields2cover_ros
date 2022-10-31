cmake_minimum_required(VERSION 3.0.2)
project(fields2cover_ros)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    dynamic_reconfigure
    )

generate_dynamic_reconfigure_options(
  cfg/Controls.cfg
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
)

find_package(Fields2Cover REQUIRED)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(visualizer_node 
    src/fields2cover_visualizer_node.cpp
    src/Fields2CoverVisualizerNode.cpp
    src/ros/conversor.cpp
)

target_link_libraries(visualizer_node
   ${catkin_LIBRARIES} Fields2Cover
)
add_dependencies(visualizer_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg )

