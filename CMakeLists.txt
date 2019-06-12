cmake_minimum_required(VERSION 2.8.3)
project(stonefish_ros)

find_package(catkin REQUIRED COMPONENTS
  cola2_lib
  cola2_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  tf
  tf_conversions
  message_generation
)

find_package(Bullet REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Stonefish REQUIRED stonefish)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cola2_lib cola2_msgs geometry_msgs nav_msgs roscpp rospy sensor_msgs std_srvs tf tf_conversions
  DEPENDS Stonefish
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(${Stonefish_CFLAGS})

#add_executable(g500_auv_dynamics include/StonefishROSInterface.hpp src/g500_auv_dynamics.cpp src/G500AUVDynamicsManager.cpp)
#target_link_libraries(g500_auv_dynamics ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES})