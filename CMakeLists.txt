cmake_minimum_required(VERSION 2.8.3)
project(srl_eband_local_planner)

# Adding definitions
MESSAGE(STATUS "Compiling with maximum optimization, will only work on Core i7 processors! (-mavx -march=corei7-avx -msse4.2 -O3 -funroll-loops -fomit-frame-pointer -ftree-vectorize ...)")
ADD_DEFINITIONS(-march=corei7-avx)
ADD_DEFINITIONS(-mtune=corei7-avx)
ADD_DEFINITIONS(-msse)
ADD_DEFINITIONS(-msse2)
ADD_DEFINITIONS(-msse3)
ADD_DEFINITIONS(-mssse3)
ADD_DEFINITIONS(-msse4.1)
ADD_DEFINITIONS(-msse4.2)
IF($ENV{AVX})
  ADD_DEFINITIONS(-mavx)
  MESSAGE(STATUS "Compiling with AVX on")
ELSE()
  MESSAGE(WARNING "Compiling with AVX off")
  ADD_DEFINITIONS(-mno-avx)
ENDIF()
ADD_DEFINITIONS(-O3)
ADD_DEFINITIONS(-funroll-loops)
ADD_DEFINITIONS(-fomit-frame-pointer)
ADD_DEFINITIONS(-pipe)
ADD_DEFINITIONS(-ftree-vectorize)
ADD_DEFINITIONS(-DNDEBUG )

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(catkin REQUIRED
  COMPONENTS
    spencer_tracking_msgs
    hanp_prediction
    std_msgs
    base_local_planner
    cmake_modules
    control_toolbox
    costmap_2d
    geometry_msgs
    sensor_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf
    tf_conversions
    tf2
    tf2_ros
    dynamic_reconfigure
		global_planner
		cmake_modules
    message_generation
)

find_package(Eigen REQUIRED)
include_directories(
  include
	cfg
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/srl_eband_plannerRos.cfg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    EnableSocialLayer.srv
    EnableObstacleLayer.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    spencer_tracking_msgs
    hanp_prediction
    std_msgs
    base_local_planner
    control_toolbox
    costmap_2d
    geometry_msgs
    sensor_msgs
    nav_core
    nav_msgs
    pluginlib
		dynamic_reconfigure
    roscpp
    tf
    tf_conversions
    tf2
    tf2_ros
		global_planner
		cmake_modules
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES srl_eband_local_planner
)



if(NOT CMAKE_BUILD_TYPE MATCHES Debug)
# Disable debug and info-level logging statements in release mode
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_WARN)
else()
add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_DEBUG)
endif()


# build the library
add_library(srl_eband_local_planner
  src/srl_eband_local_planner_ros.cpp
  src/srl_eband_local_planner.cpp
  src/conversions_and_types.cpp
  src/srl_eband_visualization.cpp
  src/srl_eband_trajectory_controller.cpp
	src/context_cost_function.cpp
  src/costmap_layers_dyn_rec_handler.cpp
  src/check_points_on_path.cpp
)

# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
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
