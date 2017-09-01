#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>   //arm용
#include <mavros_msgs/SetMode.h>       //OFFBOARD 모드 설정용
#include <mavros_msgs/State.h>         //mavros 메세지 활용용
#include "swarm_ctrl_pkg/srvMultiArming.h"
#define NUM_DRONE 5

typedef void (*FuncPtr)(const mavros_msgs::State::ConstPtr&);
mavros_msgs::State current_state[NUM_DRONE];
bool b_armig = false;

bool multi_arming(swarm_ctrl_pkg::srvMultiArming::Request &req, swarm_ctrl_pkg::srvMultiArming::Response &res){
	(req.arming == true) ? b_armig = true : b_armig = false;
	res.success = true;
	return true;
}

void state_cb0(const mavros_msgs::State::ConstPtr& msg){
	current_state[0] = *msg;
}
void state_cb1(const mavros_msgs::State::ConstPtr& msg){
	current_state[1] = *msg;
}
void state_cb2(const mavros_msgs::State::ConstPtr& msg){
	current_state[2] = *msg;
}
void state_cb3(const mavros_msgs::State::ConstPtr& msg){
	current_state[3] = *msg;
}
void state_cb4(const mavros_msgs::State::ConstPtr& msg){
	current_state[4] = *msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "arming_node");

	ros::NodeHandle nh;
	ros::Subscriber state_sub[NUM_DRONE];
	ros::ServiceClient arming_client[NUM_DRONE];
	ros::ServiceClient set_mode_client[NUM_DRONE];
	ros::ServiceServer multi_arming_server = nh.advertiseService("multi_arming", multi_arming);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_state = "/mavros/state";
	std::string d_mavros_arm = "/mavros/cmd/arming";
	std::string d_mavros_mode = "/mavros/set_mode";

	FuncPtr stateFP[NUM_DRONE] = {state_cb0, state_cb1, state_cb2, state_cb3, state_cb4};
	
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		state_sub[i] = nh.subscribe<mavros_msgs::State>(
			group_name + stream.str() + d_mavros_state, 10, stateFP[i]);
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
		for(int i=0 ; i< NUM_DRONE; i++){
			(current_state[i].mode == "OFFBOARD") ? cnt_mode++ : cnt_mode = 0;
			(current_state[i].armed == true) ? cnt_armed++ : cnt_armed = 0;
		}
		cnt_mode = (cnt_mode < NUM_DRONE) ? 0 : cnt_mode;
		cnt_armed = (cnt_armed < NUM_DRONE) ? 0 : cnt_armed;
		if(cnt_mode == NUM_DRONE){
			ROS_INFO("All drones OFFBOARD enabled");
		}
		if(cnt_armed == NUM_DRONE){
			ROS_INFO("All drones armed");
		}
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
