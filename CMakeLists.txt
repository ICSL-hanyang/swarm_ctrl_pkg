cmake_minimum_required(VERSION 2.8.3)
project(test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_extras
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   mavros_msgs
   geometry_msgs  
   std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control_node
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(multi_control_node src/control.cpp)
add_dependencies(multi_control_node Drone_control_cpp)
target_link_libraries(multi_control_node ${catkin_LIBRARIES})


