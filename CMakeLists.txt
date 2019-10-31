cmake_minimum_required(VERSION 2.8.3)
project(stonefish_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DBT_EULER_DEFAULT_ZYX -DBT_USE_DOUBLE_PRECISION -DBUILD_PYBULLET=OFF -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON")

find_package(catkin REQUIRED COMPONENTS
    #cola2_lib_ros
  cola2_msgs
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  std_srvs
  tf
  tf_conversions
  message_generation
)

add_subdirectory(external)

#set(BULLET_ROOT /home/nbore/Stonefish/bullet3_install)
#link_directories(/home/nbore/Stonefish/bullet3_install/lib)

#find_package(Bullet REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Stonefish REQUIRED stonefish)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(${Stonefish_CFLAGS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stonefish_ros
  CATKIN_DEPENDS #cola2_lib_ros
                 cola2_msgs 
                 geometry_msgs 
                 nav_msgs 
                 roscpp 
                 rospy 
                 sensor_msgs 
                 std_srvs 
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

add_dependencies(stonefish_ros ${catkin_EXPORTED_TARGETS})

target_link_libraries(stonefish_ros ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES})

#Code examples
#add_executable(girona500_auv_dynamics src/girona500_auv_dynamics.cpp src/G500AUVDynamicsManager.cpp)
#add_dependencies(girona500_auv_dynamics cola2_msgs_gencpp)
#target_link_libraries(girona500_auv_dynamics ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

#add_executable(girona500_auv_simulator src/girona500_auv_simulator.cpp src/G500AUVSimulationManager.cpp)
#add_dependencies(girona500_auv_simulator cola2_msgs_gencpp)
#target_link_libraries(girona500_auv_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

#add_executable(girona500_iauv_simulator src/girona500_iauv_simulator.cpp src/G500IAUVSimulationManager.cpp)
#add_dependencies(girona500_iauv_simulator cola2_msgs_gencpp)
#target_link_libraries(girona500_iauv_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)

#Universal simulator with XML parser
add_executable(parsed_simulator src/parsed_simulator.cpp)
add_dependencies(parsed_simulator cola2_msgs_gencpp)
target_link_libraries(parsed_simulator ${catkin_LIBRARIES} ${BULLET_LIBRARIES} ${Stonefish_LIBRARIES} stonefish_ros)
