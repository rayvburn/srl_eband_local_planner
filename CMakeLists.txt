cmake_minimum_required(VERSION 2.8.3)
project(srl_eband_local_planner)
set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED
  COMPONENTS
    base_local_planner
    cmake_modules
    control_toolbox
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
    # dynamic_reconfigure
)

find_package(Eigen REQUIRED)
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    base_local_planner
    control_toolbox
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
  INCLUDE_DIRS include
  LIBRARIES srl_eband_local_planner
)

# build the library
add_library(srl_eband_local_planner
  src/srl_eband_local_planner_ros.cpp
  src/srl_eband_local_planner.cpp
  src/conversions_and_types.cpp
  src/srl_eband_visualization.cpp
  src/srl_eband_trajectory_controller.cpp
)

target_link_libraries(srl_eband_local_planner ${catkin_LIBRARIES})

install(
  TARGETS srl_eband_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES srl_eband_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
