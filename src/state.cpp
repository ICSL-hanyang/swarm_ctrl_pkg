#include <ros/ros.h>
#include <mavros_msgs/State.h>         //state check 메세지 활용용
#include "swarm_ctrl_pkg/msgState.h"   //multi_state msg
#define NUM_DRONE 5

typedef void (*FuncPtr)(const mavros_msgs::State::ConstPtr&);
mavros_msgs::State current_state[NUM_DRONE];

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
	ros::init(argc, argv, "state_node");

	ros::NodeHandle nh;
	ros::Subscriber state_sub[NUM_DRONE];
	ros::Publisher multi_state_pub = nh.advertise<swarm_ctrl_pkg::msgState>("multi_state", 50);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_state = "/mavros/state";

	FuncPtr stateFP[NUM_DRONE] = {state_cb0, state_cb1, state_cb2, state_cb3, state_cb4};

	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		state_sub[i] = nh.subscribe<mavros_msgs::State>(
			group_name + stream.str() + d_mavros_state, 10, stateFP[i]);
		stream.str("");
	}

	ros::Rate rate(20.0); // period 0.01 s

	ROS_INFO("State check start");
	ros::Time last_request = ros::Time::now();
	while(ros::ok()){
		int cnt_connected = 0;
		int cnt_mode = 0;
		int cnt_armed = 0;
		swarm_ctrl_pkg::msgState msg;
		for(int i=0 ; i< NUM_DRONE; i++){
			(current_state[i].connected == true) ? cnt_connected++ : cnt_connected = 0;
			if(cnt_connected == 0 && (ros::Time::now() - last_request > ros::Duration(3.0))){
				ROS_WARN("Camila%d is not connected", i);
				ros::Time last_request = ros::Time::now();
			}
			(current_state[i].mode == "OFFBOARD") ? cnt_mode++ : cnt_mode = 0;
			if(cnt_mode == 0 && (ros::Time::now() - last_request > ros::Duration(3.0))){
				ROS_INFO("Camila%d is not OFFBOARD mode", i);	
				ros::Time last_request = ros::Time::now();
			}
			(current_state[i].armed == true) ? cnt_armed++ : cnt_armed = 0;
			if(cnt_armed == 0 && (ros::Time::now() - last_request > ros::Duration(3.0))){
				ROS_INFO("Camila%d is not armed", i);
				ros::Time last_request = ros::Time::now();			
			}
		}
		(cnt_connected == NUM_DRONE) ? msg.connected = true : msg.connected = false;
		(cnt_mode == NUM_DRONE) ? msg.mode = true : msg.mode = false;
		(cnt_armed == NUM_DRONE) ? msg.armed = true : msg.armed = false;

		ros::spinOnce();
		rate.sleep();
	}


	return 0;
}