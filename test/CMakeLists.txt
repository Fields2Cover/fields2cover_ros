
set( fields2cover_ros_tests
	test_ros_conversor
)

foreach( f2c_test ${fields2cover_ros_tests})
  ament_add_gtest_executable(
    ${f2c_test}
    ${f2c_test}.cpp
  )
  ament_target_dependencies(
    ${f2c_test}
    ${dependencies}
  )
  target_include_directories(
    ${f2c_test}
    PUBLIC
    include
  )
  target_link_libraries(
    ${f2c_test}
    ${PROJECT_NAME}
  )
  target_include_directories(
    ${f2c_test}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endforeach()

