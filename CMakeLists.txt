cmake_minimum_required(VERSION 2.8.3)
project(stonefish_ros)

find_package(catkin REQUIRED COMPONENTS
  cola2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
  roscpp
  roslib
  tf
  tf_conversions
)

find_package(Bullet REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Stonefish REQUIRED stonefish)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(${Stonefish_CFLAGS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stonefish_ros
  CATKIN_DEPENDS cola2_msgs 
                 geometry_msgs 
                 nav_msgs 
                 sensor_msgs 
                 std_srvs 
                 roscpp
                 roslib
                 tf 
                 tf_conversions
  DEPENDS BULLET Stonefish
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Stonefish_INCLUDE_DIRS}
)

#Library
add_library(stonefish_ros SHARED
src/ROSInterface.cpp
src/ROSSimulationManager.cpp
src/ROSScenarioParser.cpp
)
target_link_libraries(stonefish_ros ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES})

#Code examples
add_executable(girona500_auv_dynamics src/girona500_auv_dynamics.cpp src/G500AUVDynamicsManager.cpp)
target_link_libraries(girona500_auv_dynamics ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

add_executable(girona500_auv_simulator src/girona500_auv_simulator.cpp src/G500AUVSimulationManager.cpp)
target_link_libraries(girona500_auv_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

add_executable(girona500_iauv_simulator src/girona500_iauv_simulator.cpp src/G500IAUVSimulationManager.cpp)
target_link_libraries(girona500_iauv_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

#Universal simulator with XML parser
add_executable(parsed_simulator src/parsed_simulator.cpp)
target_link_libraries(parsed_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)