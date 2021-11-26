cmake_minimum_required(VERSION 3.10)
project(stonefish_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
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
  pcl_ros
  pcl_conversions
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(Stonefish REQUIRED stonefish>=1.3.0)

add_message_files(
  FILES
  Int32Stamped.msg
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

set(CMAKE_CXX_STANDARD 17)
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
		             message_runtime
                 pcl_conversions
                 pcl_ros
  DEPENDS Stonefish
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Stonefish_INCLUDE_DIRS}
)

link_directories(
  ${Stonefish_LIBRARY_DIRS}
)

#Library
add_library(stonefish_ros SHARED
src/ROSInterface.cpp
src/ROSSimulationManager.cpp
src/ROSScenarioParser.cpp
)
add_dependencies(stonefish_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(stonefish_ros ${catkin_LIBRARIES} ${Stonefish_LIBRARIES})

#Universal simulators with XML parser
add_executable(parsed_simulator src/parsed_simulator.cpp)
target_link_libraries(parsed_simulator ${catkin_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

add_executable(parsed_simulator_nogpu src/parsed_simulator_nogpu.cpp)
target_link_libraries(parsed_simulator_nogpu ${catkin_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)
