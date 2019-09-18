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
  ${Stonefish_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(${Stonefish_CFLAGS})

add_library(stonefish_ros_lib
src/ROSInterface.cpp
src/ROSSimulationManager.cpp
src/ROSScenarioParser.cpp
)
target_link_libraries(stonefish_ros_lib ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES})

add_executable(girona500_auv_dynamics src/girona500_auv_dynamics.cpp src/G500AUVDynamicsManager.cpp)
add_dependencies(girona500_auv_dynamics cola2_msgs_gencpp)
target_link_libraries(girona500_auv_dynamics ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros_lib)

add_executable(girona500_auv_simulator src/girona500_auv_simulator.cpp src/G500AUVSimulationManager.cpp)
add_dependencies(girona500_auv_simulator cola2_msgs_gencpp)
target_link_libraries(girona500_auv_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros_lib)

add_executable(girona500_iauv_simulator src/girona500_iauv_simulator.cpp src/G500IAUVSimulationManager.cpp)
add_dependencies(girona500_iauv_simulator cola2_msgs_gencpp)
target_link_libraries(girona500_iauv_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros_lib)

add_executable(parsed_simulator src/parsed_simulator.cpp)
add_dependencies(parsed_simulator cola2_msgs_gencpp)
target_link_libraries(parsed_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros_lib)