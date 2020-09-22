cmake_minimum_required(VERSION 3.10)
project(stonefish_ros)

find_package(catkin REQUIRED COMPONENTS
  cola2_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  roscpp
  roslib
  tf
  tf_conversions
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Stonefish REQUIRED stonefish>=1.1.0)

add_message_files(
  FILES
  ThrusterState.msg
)

add_service_files(
  FILES
  SonarSettings.srv
  SonarSettings2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(${Stonefish_CFLAGS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stonefish_ros
  CATKIN_DEPENDS cola2_msgs 
                 geometry_msgs 
                 nav_msgs 
                 sensor_msgs
                 std_msgs 
                 std_srvs 
                 roscpp
                 roslib
                 tf 
                 tf_conversions
  DEPENDS Stonefish
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
target_link_libraries(stonefish_ros ${catkin_LIBRARIES} ${Stonefish_LIBRARIES})

#Universal simulator with XML parser
add_executable(parsed_simulator src/parsed_simulator.cpp)
target_link_libraries(parsed_simulator ${catkin_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)