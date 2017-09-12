#include <ros/ros.h>
#include "swarm_ctrl_pkg/msgState.h"   //multi_state msg
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
#include "swarm_ctrl_pkg/srvMultiArming.h" 
#include "swarm_ctrl_pkg/srvMultiMode.h"
#include "swarm_ctrl_pkg/srvMultiSetHome.h"
#include "swarm_ctrl_pkg/srvMultiLanding.h" 
#define NUM_DRONE 3

//int land = 0;
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
  ros::ServiceClient set_home_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetHome>("multi_set_home");
  ros::ServiceClient set_pos_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetPosLocal>("multi_set_pos_local");
  ros::ServiceClient set_vel_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetVelLocal>("multi_set_vel_local");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0); // period 0.01 s
  //nh.setParam("control_node/mode", 0);
  while(ros::ok() && m_state.connected){
    ros::spinOnce();
    rate.sleep();    
  }
  swarm_ctrl_pkg::srvMultiMode mode;
  mode.request.mode = "OFFBOARD";
  if(set_mode_client.call(mode) && mode.response.success){
  }

  // wait for FCU connection
  ROS_INFO("Control node executed");
 /* swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
  swarm_ctrl_pkg::srvMultiSetVelLocal v_msg;
  while(ros::ok()){
    nh.getParam("control_node/land", land);
    if(land == 1){
      p_msg.request.pos_flag = true;
      p_msg.request.x = 0;
      p_msg.request.y = 0;
      p_msg.request.z = -10;
      v_msg.request.vel_flag = true;
      v_msg.request.vel_x = 0;
      v_msg.request.vel_y = 0;
      v_msg.request.vel_z = -10;
        set_pos_client.call(p_msg);
        set_vel_client.call(v_msg);

    }
    else{
      v_msg.request.vel_flag = false;
      set_vel_client.call(v_msg);
    }
    ros::spinOnce();
    rate.sleep();
  }*/
  ros::spin();

  return 0;
}
