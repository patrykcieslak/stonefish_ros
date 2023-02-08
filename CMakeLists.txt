cmake_minimum_required(VERSION 3.10)
project(stonefish_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  roscpp
  roslib
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  image_transport
  controller_manager
)

find_package(Stonefish REQUIRED 1.3.0)

add_message_files(
  FILES
  DVLBeam.msg
  NEDPose.msg
  DVL.msg
  INS.msg
  Int32Stamped.msg
  ThrusterState.msg
  BeaconInfo.msg
)

add_service_files(
  FILES
  SonarSettings.srv
  SonarSettings2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

set(CMAKE_CXX_STANDARD 17)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stonefish_ros
  CATKIN_DEPENDS geometry_msgs
                 nav_msgs
                 sensor_msgs
                 std_msgs
                 std_srvs
                 roscpp
                 roslib
                 tf
                 tf_conversions
		             message_runtime
                 pcl_conversions
                 pcl_ros
                 image_transport
                 controller_manager
)

#Library
add_library(stonefish_ros SHARED
  src/ROSInterface.cpp
  src/ROSSimulationManager.cpp
  src/ROSScenarioParser.cpp
  src/ROSControlInterface.cpp
)
target_link_libraries(stonefish_ros PUBLIC ${catkin_LIBRARIES} Stonefish::Stonefish)
target_include_directories(stonefish_ros PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(stonefish_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})

#Universal simulators with XML parser
add_executable(parsed_simulator src/parsed_simulator.cpp)
target_link_libraries(parsed_simulator PRIVATE stonefish_ros)

add_executable(parsed_simulator_nogpu src/parsed_simulator_nogpu.cpp)
target_link_libraries(parsed_simulator_nogpu PRIVATE stonefish_ros)