#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>   //arm용
#include <mavros_msgs/SetMode.h>       //OFFBOARD 모드 설정용
#include "swarm_ctrl_pkg/srvMultiArming.h" 
#include "swarm_ctrl_pkg/srvMultiMode.h"
#include "swarm_ctrl_pkg/msgState.h"   //multi_state msg
#define NUM_DRONE 4

ros::ServiceClient arming_client[NUM_DRONE];
ros::ServiceClient set_mode_client[NUM_DRONE];
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode set_mode;
swarm_ctrl_pkg::msgState multi_state;

bool multiArming(swarm_ctrl_pkg::srvMultiArming::Request &req, swarm_ctrl_pkg::srvMultiArming::Response &res){
	arm_cmd.request.value = req.arming;
	for (int i = 0; i < NUM_DRONE; i++){
		if (arming_client[i].call(arm_cmd) && arm_cmd.response.success){
			res.success = true;
		}
		else{
			res.success = false;
		}
	}
	return true;
}

bool multiMode(swarm_ctrl_pkg::srvMultiMode::Request &req, swarm_ctrl_pkg::srvMultiMode::Response &res){
	set_mode.request.custom_mode = req.mode;
	for (int i = 0; i < NUM_DRONE; i++){
		if (set_mode_client[i].call(set_mode) && set_mode.response.success){
			res.success = true;
		}
		else{
			res.success = false;
		}
	}
	return true;
}

void multiStateCB(const swarm_ctrl_pkg::msgState::ConstPtr& msg){
	multi_state = *msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "arming_node");

	ros::NodeHandle nh;
	ros::ServiceServer multi_arming_server = nh.advertiseService("multi_arming", multiArming);
	ros::ServiceServer multi_mode_server = nh.advertiseService("multi_mode", multiMode);
	ros::Subscriber multi_state_sub = nh.subscribe("multi_state", 50,multiStateCB);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_arm = "/mavros/cmd/arming";
	std::string d_mavros_mode = "/mavros/set_mode";
	
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		arming_client[i] = nh.serviceClient<mavros_msgs::CommandBool>(
			group_name + stream.str() + d_mavros_arm);		
		set_mode_client[i] = nh.serviceClient<mavros_msgs::SetMode>(
			group_name + stream.str() + d_mavros_mode); 
		stream.str("");
	}
	ros::Rate rate(10.0); // period 0.01 s

	ROS_INFO("Ready to arming");
	ros::spin();

	return 0;
}
