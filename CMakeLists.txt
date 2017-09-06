cmake_minimum_required(VERSION 2.8.3)
project(swarm_ctrl_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  mavros_extras
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

add_service_files(FILES srvMultiArming.srv)
add_service_files(FILES srvMultiMode.srv)
add_service_files(FILES srvMultiSetPosLocal.srv)
add_service_files(FILES srvMultiSetVelLocal.srv)
add_service_files(FILES srvMultiSetRawLocal.srv)
add_service_files(FILES srvMultiSetHome.srv)
add_service_files(FILES srvMultiLanding.srv)
add_message_files(FILES msgState.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   mavros_msgs
   geometry_msgs  
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swarm_ctrl_pkg
  CATKIN_DEPENDS geometry_msgs mavros mavros_extras roscpp std_msgs
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(control_node src/control_node.cpp)
add_dependencies(control_node swarm_ctrl_node_generate_messages_cpp)
target_link_libraries(control_node ${catkin_LIBRARIES})

add_executable(arming_node src/arming.cpp)
add_dependencies(arming_node swarm_ctrl_node_generate_messages_cpp)
target_link_libraries(arming_node ${catkin_LIBRARIES})

add_executable(state_node src/state.cpp)
add_dependencies(state_node swarm_ctrl_node_generate_messages_cpp)
target_link_libraries(state_node ${catkin_LIBRARIES})

add_executable(set_point_node src/set_point.cpp)
add_dependencies(set_point_node swarm_ctrl_node_generate_messages_cpp)
target_link_libraries(set_point_node ${catkin_LIBRARIES})

add_executable(set_home_node src/set_home.cpp)
add_dependencies(set_home_node swarm_ctrl_node_generate_messages_cpp)
target_link_libraries(set_home_node ${catkin_LIBRARIES})

add_executable(landing_node src/landing.cpp)
add_dependencies(landing_node swarm_ctrl_node_generate_messages_cpp)
target_link_libraries(landing_node ${catkin_LIBRARIES})