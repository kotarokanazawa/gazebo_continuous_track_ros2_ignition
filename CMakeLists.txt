cmake_minimum_required(VERSION 3.8)
project(gazebo_continuous_track_ros2_ignition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-gazebo6-all QUIET)
find_package(ignition-math6 QUIET)
find_package(ignition-plugin1 QUIET)
find_package(ignition-plugin1-register QUIET)

if(ignition-gazebo6-all_FOUND AND ignition-math6_FOUND AND ignition-plugin1_FOUND AND ignition-plugin1-register_FOUND)
  add_library(IgnitionContinuousTrackSimple SHARED
    src/ignition_continuous_track_simple.cpp
  )
  target_include_directories(IgnitionContinuousTrackSimple PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(IgnitionContinuousTrackSimple
    ignition-gazebo6::all
    ignition-math6::ignition-math6
    ignition-plugin1::ignition-plugin1
    ignition-plugin1::register
  )

  install(TARGETS IgnitionContinuousTrackSimple
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  ament_export_dependencies(ignition-gazebo6-all ignition-math6 ignition-plugin1 ignition-plugin1-register)
else()
  message(WARNING
    "Ignition Gazebo development packages were not found. "
    "Skipping libIgnitionContinuousTrackSimple.so. "
    "Install libignition-gazebo6-dev and libignition-plugin-dev to build the plugin.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  sdf
  urdf_xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  env-hooks/gazebo_continuous_track_ros2_ignition.dsv.in
  env-hooks/gazebo_continuous_track_ros2_ignition.sh.in
)

ament_export_include_directories(include)
ament_package()
