
set( fields2cover_ros_tests
	test_ros_conversor
)

if ( ament_cmake_FOUND )
  find_package(ament_cmake_gtest REQUIRED)
endif()

foreach( f2c_test ${fields2cover_ros_tests})
  if ( ament_cmake_FOUND )
    ament_add_gtest_executable(
      ${f2c_test}
      ${f2c_test}.cpp
    )
    ament_target_dependencies(
      ${f2c_test}
      ${dependencies}
    )
  elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
    catkin_add_executable_with_gtest(
      ${f2c_test}
      ${f2c_test}.cpp
    )
    add_dependencies(
      ${f2c_test}
      ${catkin_EXPORTED_TARGETS} ${dependencies}
    )
  endif()
  target_include_directories(
    ${f2c_test}
    PUBLIC
    include
  )

  if ( ament_cmake_FOUND )
    target_link_libraries(
      ${f2c_test}
      ${PROJECT_NAME}
    )
  elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
    target_link_libraries(
      ${f2c_test}
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}
    )
  endif()
  target_include_directories(
    ${f2c_test}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endforeach()

