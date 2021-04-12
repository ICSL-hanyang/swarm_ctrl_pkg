cmake_minimum_required(VERSION 2.8.3)
project(swarm_ctrl_pkg)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_msgs
  mavros
  mavros_msgs
  mavros_extras
  sensor_msgs
  geometry_msgs
  geographic_msgs
  message_generation
)

##add_message_files(FILES)

add_service_files(
   FILES
   srvMultiSetpointLocal.srv
   srvMultiSetpointGlobal.srv
   srvMultiSetVelLocal.srv
   srvGoToVehicle.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   tf2_msgs
   mavros_msgs
   geometry_msgs  
   geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swarm_ctrl_pkg
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs
 ## DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(swarm_node src/main.cpp src/vehicle.cpp)
##add_dependencies(swarm_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(swarm_node ${catkin_LIBRARIES})