cmake_minimum_required(VERSION 2.8.3)
project(selfie_drone)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_extras
  roscpp
  std_msgs
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
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


add_executable(control_node src/selfie.cpp)
add_dependencies(control_node Selfie_drone_selfie_cpp)
target_link_libraries(control_node ${catkin_LIBRARIES})


