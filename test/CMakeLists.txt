

file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false unittests.cpp *.cpp)



if ( ament_cmake_FOUND )
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(
    unittests
    ${TEST_SOURCES}
  )
ament_target_dependencies(
  unittests
  ${dependencies}
)
elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
  find_package(rostest REQUIRED)
  catkin_add_executable_with_gtest(
    unittests
    ${TEST_SOURCES}
  )
  add_dependencies(
    unittests
    ${catkin_EXPORTED_TARGETS} ${dependencies}
  )
  add_dependencies(
    tests
    ${catkin_EXPORTED_TARGETS} ${dependencies}
  )
endif()
target_include_directories(
  unittests PUBLIC include
)

if ( ament_cmake_FOUND )
  target_link_libraries(
    unittests
    ${PROJECT_NAME}
    Fields2Cover
  )
elseif( CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE )
  target_link_libraries(
    unittests
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
  )

  find_package(rostest REQUIRED)
  add_rostest(unittests.test DEPENDENCIES unittests)
endif()
target_include_directories(
  unittests
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

