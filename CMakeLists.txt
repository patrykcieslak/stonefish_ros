cmake_minimum_required(VERSION 2.8.3)
project(stonefish_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DBT_EULER_DEFAULT_ZYX -DBT_USE_DOUBLE_PRECISION -DBUILD_PYBULLET=OFF -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON -DSHADER_DIR_PATH=\\\"${CMAKE_CURRENT_SOURCE_DIR}/external/stonefish/Library/shaders/\\\"")

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

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Bullet REQUIRED)

include_directories(external/glm)
include_directories(${BULLET_INCLUDE_DIRS})
add_subdirectory(external)

set(Stonefish_INCLUDE_DIRS external/stonefish/Library/include)
set(Stonefish_LIBRARIES Stonefish)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(Stonefish REQUIRED stonefish)

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
                 #DEPENDS BULLET Stonefish
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

add_dependencies(stonefish_ros ${catkin_EXPORTED_TARGETS})

target_link_libraries(stonefish_ros ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES})

#Universal simulator with XML parser
add_executable(parsed_simulator src/parsed_simulator.cpp)
target_link_libraries(parsed_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)
