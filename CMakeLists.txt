cmake_minimum_required(VERSION 3.0.2)
project(fields2cover_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(Fields2Cover REQUIRED)

find_package(ament_cmake QUIET)
find_package(catkin QUIET)

if ( ament_cmake_FOUND )
    set(IS_ROS2 ON)
    message(STATUS "fields2cover_ros is being built using AMENT.")
    set(BUILD_TOOL_INCLUDE_DIRS ${ament_INCLUDE_DIRS})

    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)

    set(dependencies
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
    )

    include_directories(
        include
        ${BUILD_TOOL_INCLUDE_DIRS}
    )

    add_library(${PROJECT_NAME} SHARED src/ros/conversor.cpp)
	ament_target_dependencies(
      ${PROJECT_NAME} ${dependencies}
    )
    target_compile_definitions(${PROJECT_NAME} PUBLIC IS_ROS2)
    target_link_libraries(${PROJECT_NAME}
    	Fields2Cover
    )

    install(DIRECTORY include/
            DESTINATION include/
    )
    install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES DESTINATION include
    )
    if(BUILD_TESTING)
        add_subdirectory(test)
    endif()
    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME})
    ament_export_dependencies(${dependencies})
    ament_package()

elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
    message(STATUS "fields2cover_ros is being built using CATKIN.")
    find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        dynamic_reconfigure
    )

    # Avoid compile the controls with colcon, as it is not supported
    # https://answers.ros.org/question/290921/dynamic-reconfigure-in-ros2/
    if( NOT ament_cmake_FOUND )
      generate_dynamic_reconfigure_options(
        cfg/Controls.cfg
      )
    endif()

    catkin_package(
      CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
    )


    include_directories(
      include
      ${catkin_INCLUDE_DIRS}
    )
    add_library(${PROJECT_NAME} SHARED src/ros/conversor.cpp)
    target_link_libraries(${PROJECT_NAME} Fields2Cover)

    install(DIRECTORY include/
            DESTINATION include/
    )
    install(TARGETS ${PROJECT_NAME}
              ARCHIVE DESTINATION lib
              LIBRARY DESTINATION lib
              RUNTIME DESTINATION bin
              INCLUDES DESTINATION include
    )

    if (DEFINED ${RESOURCE_PATH})
      install(FILES package.xml DESTINATION ${RESOURCE_PATH})
    endif()

    # Avoid compile the controls with colcon, as it is not supported
    # https://answers.ros.org/question/290921/dynamic-reconfigure-in-ros2/
    if( NOT ament_cmake_FOUND )
      add_executable(visualizer_node
          src/fields2cover_visualizer_node.cpp
          src/Fields2CoverVisualizerNode.cpp
      )

      target_link_libraries(visualizer_node
         ${catkin_LIBRARIES} Fields2Cover ${PROJECT_NAME}
      )
      add_dependencies(visualizer_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg )
    endif()

    if(CATKIN_ENABLE_TESTING OR BUILD_TESTING)
      add_subdirectory(test)
    endif()
endif()

