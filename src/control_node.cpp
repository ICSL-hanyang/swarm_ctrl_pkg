#include <ros/ros.h>
#include "swarm_ctrl_pkg/msgState.h"   //multi_state msg
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
#include "swarm_ctrl_pkg/srvMultiArming.h" 
#include "swarm_ctrl_pkg/srvMultiMode.h"
#include "swarm_ctrl_pkg/srvMultiSetHome.h"
#include "swarm_ctrl_pkg/srvMultiLanding.h" 
#define NUM_DRONE 4

swarm_ctrl_pkg::msgState m_state;
void multiStateCB(const swarm_ctrl_pkg::msgState::ConstPtr& msg){
  m_state = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_node");

  ros::NodeHandle nh;
  ros::Subscriber multi_state_sub = nh.subscribe("multi_state", 50, multiStateCB);
  ros::ServiceClient arming_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiArming>("multi_arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiMode>("multi_mode");
  ros::ServiceClient set_pos_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetPosLocal>("multi_set_pos_local");
  ros::ServiceClient set_vel_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetVelLocal>("multi_set_vel_local");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0); // period 0.01 s
  while(ros::ok() && m_state.connected){
    ros::spinOnce();
    rate.sleep();    
  }
  swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
  swarm_ctrl_pkg::srvMultiMode mode;
  p_msg.request.pos_flag = true;
  p_msg.request.x = 0;
  p_msg.request.y = 0;
  p_msg.request.z = -1;
  mode.request.mode = "OFFBOARD";


  // wait for FCU connection
  ROS_INFO("Control node executed");
  if(set_pos_client.call(p_msg) && p_msg.response.success){
    ;
  }
  ros::Time last_req = ros::Time::now();
  while(ros::ok()){
    if(m_state.mode != true && (ros::Time::now() - last_req > ros::Duration(3.0))){
      if(set_mode_client.call(mode) && mode.response.success){
      }

      last_req = ros::Time::now();
    }


  }

  return 0;
}
