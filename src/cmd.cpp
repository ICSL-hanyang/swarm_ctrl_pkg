#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>   //arm용
#include <mavros_msgs/SetMode.h>       //OFFBOARD 모드 설정용
#include <geometry_msgs/PoseStamped.h> //local_position 용
#include <mavros_msgs/CommandHome.h>   //set_home
#include <sensor_msgs/NavSatFix.h>     //recieve global pos
#include "swarm_ctrl_pkg/srvMultiArming.h"
#include "swarm_ctrl_pkg/srvMultiMode.h"
#include "swarm_ctrl_pkg/msgState.h" //multi_state msg
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetHome.h"
#include "swarm_ctrl_pkg/srvMultiLanding.h"

#define NUM_DRONE 1

ros::ServiceClient arming_client[5];
ros::ServiceClient set_mode_client[5];
ros::ServiceClient multi_set_pos_local_client;
ros::ServiceClient multi_set_vel_local_client;
ros::ServiceClient multi_set_home_client;

ros::ServiceClient set_home_client[5];
geometry_msgs::PoseStamped l_pos[5];
sensor_msgs::NavSatFix g_pos[5];
swarm_ctrl_pkg::msgState multi_state;
std::string group_name = "camila";
double takeoff_alt = 2.5;


bool multiArming(swarm_ctrl_pkg::srvMultiArming::Request& req,
	swarm_ctrl_pkg::srvMultiArming::Response& res)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = req.arming;

	if (arming_client[NUM_DRONE].call(arm_cmd) && arm_cmd.response.success){
		res.success = true;
	}
	else
	{
		ROS_WARN("Arming fail. Camila%d are disarmed", NUM_DRONE);
		res.success = false;
	}
	return true;
}

bool multiMode(swarm_ctrl_pkg::srvMultiMode::Request& req,
	swarm_ctrl_pkg::srvMultiMode::Response& res)
{
	swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
	mavros_msgs::SetMode set_mode;
	p_msg.request.pos_flag = true;
	p_msg.request.x = 0;
	p_msg.request.y = 0;
	p_msg.request.z = takeoff_alt;

	set_mode.request.custom_mode = req.mode;
	if (set_mode_client[NUM_DRONE].call(set_mode) && set_mode.response.success)
	{
		res.success = true;
	}
	else
	{
		res.success = false;
	}
	multi_set_pos_local_client.call(p_msg);
	return true;
}


void LocalPosCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	l_pos[NUM_DRONE] = *msg;
}
void globalPosCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	g_pos[NUM_DRONE] = *msg;
}

void multiStateCB(const swarm_ctrl_pkg::msgState::ConstPtr& msg)
{
	multi_state = *msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cmd_node");
	ros::NodeHandle nh;

	ros::Subscriber local_pos_sub[5];
	ros::Subscriber global_pos_sub[5];
	ros::ServiceServer multi_arming_server =
	nh.advertiseService("multi_arming", multiArming);
	ros::ServiceServer multi_mode_server =
	nh.advertiseService("multi_mode", multiMode);
	ros::Subscriber multi_state_sub =
	nh.subscribe("multi_state", 50, multiStateCB);

	multi_set_pos_local_client =
	nh.serviceClient<swarm_ctrl_pkg::srvMultiSetPosLocal>(
		"multi_set_pos_local");
	multi_set_vel_local_client =
	nh.serviceClient<swarm_ctrl_pkg::srvMultiSetVelLocal>(
		"multi_set_vel_local");

	std::stringstream stream;
	std::string d_mavros_arm = "/mavros/cmd/arming";
	std::string d_mavros_mode = "/mavros/set_mode";

	std::string d_mavros_l_pos = "/mavros/local_position/pose";
	std::string d_mavros_g_pos = "/mavros/global_position/global";

	stream << NUM_DRONE;
	arming_client[NUM_DRONE] = nh.serviceClient<mavros_msgs::CommandBool>(
		group_name + stream.str() + d_mavros_arm);
	set_mode_client[NUM_DRONE] = nh.serviceClient<mavros_msgs::SetMode>(
		group_name + stream.str() + d_mavros_mode);
	local_pos_sub[NUM_DRONE] = nh.subscribe(group_name + stream.str() + d_mavros_l_pos,
		10, LocalPosCB);
	global_pos_sub[NUM_DRONE] = nh.subscribe(group_name + stream.str() + d_mavros_g_pos,
		10, globalPosCB);
	stream.str("");

  ros::Rate rate(10.0); // period 0.01 s
  ROS_INFO("Command node started");

  while (ros::ok()){

  	nh.getParam("cmd_node/takeoff_alt", takeoff_alt);

  	ros::spinOnce();
  	rate.sleep();
  }



  ros::spin();
  return 0;
}
