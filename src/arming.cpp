#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>   //arm용
#include <mavros_msgs/SetMode.h>       //OFFBOARD 모드 설정용
#include <mavros_msgs/State.h>         //mavros 메세지 활용용
#include "swarm_ctrl_pkg/srvMultiArming.h"
#include "swarm_ctrl_pkg/msgState.h"   //multi_state msg
#define NUM_DRONE 5

bool b_armig = false;
swarm_ctrl_pkg::msgState multi_state;

bool multiArming(swarm_ctrl_pkg::srvMultiArming::Request &req, swarm_ctrl_pkg::srvMultiArming::Response &res){
	if(multi_state.armed == false){
		(req.arming == true) ? b_armig = true : b_armig = false;
		res.success = true;
	}
	return true;
}

void multiStateCB(const swarm_ctrl_pkg::msgState::ConstPtr& msg){
	multi_state = *msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "arming_node");

	ros::NodeHandle nh;
	ros::ServiceClient arming_client[NUM_DRONE];
	ros::ServiceClient set_mode_client[NUM_DRONE];
	ros::ServiceServer multi_arming_server = nh.advertiseService("multi_arming", multiArming);
	ros::Subscriber multi_state_sub = nh.subscribe<swarm_ctrl_pkg::msgState>("multi_state", 50,multiStateCB);

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

	// switch mode to OFFBOARD
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = "OFFBOARD";

	// arming_client
	mavros_msgs::CommandBool arm_cmd;

	ROS_INFO("Ready to arming");
	ros::Time last_request = ros::Time::now();
	while(ros::ok()){
		int cnt_mode = 0;
		int cnt_armed = 0;
		arm_cmd.request.value = b_armig;
		if ( (cnt_mode != NUM_DRONE) && (ros::Time::now() - last_request > ros::Duration(3.0))){
			for (int i = 0; i < NUM_DRONE; i++){
				if (set_mode_client[i].call(set_mode) && set_mode.response.success){
					ROS_INFO("Camila%d OFFBOARD enabled", i);
				}
			}
			last_request = ros::Time::now();
		}
		else{
			if ( (cnt_armed != NUM_DRONE) && (ros::Time::now() - last_request > ros::Duration(3.0))){
				for (int i = 0; i < NUM_DRONE; i++){
					if (arming_client[i].call(arm_cmd) && arm_cmd.response.success){
						if(b_armig == true){
							ROS_INFO("Camila%d armed", i);
						}
						else{
							ROS_INFO("Camila%d disarmed", i);
						}
					}
				}
				last_request = ros::Time::now();
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
	

	return 0;
}
