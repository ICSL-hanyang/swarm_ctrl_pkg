#include <ros/ros.h>
#include <mavros_msgs/State.h>         //state check 메세지 활용용
#include "swarm_ctrl_pkg/msgState.h"   //multi_state msg
#define NUM_DRONE 5

typedef void (*FuncPtr)(const mavros_msgs::State::ConstPtr&);
mavros_msgs::State current_state[NUM_DRONE];
std::string mode[NUM_DRONE];

void stateCB0(const mavros_msgs::State::ConstPtr& msg){
	current_state[0] = *msg;
	mode[0]=current_state[0].mode;
}
void stateCB1(const mavros_msgs::State::ConstPtr& msg){
	current_state[1] = *msg;
	mode[1]=current_state[1].mode;
}
void stateCB2(const mavros_msgs::State::ConstPtr& msg){
	current_state[2] = *msg;
	mode[2]=current_state[2].mode;
}
void stateCB3(const mavros_msgs::State::ConstPtr& msg){
	current_state[3] = *msg;
	mode[3]=current_state[3].mode;
}
void stateCB4(const mavros_msgs::State::ConstPtr& msg){
	current_state[4] = *msg;
	mode[4]=current_state[4].mode;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "state_node");

	ros::NodeHandle nh;
	ros::Subscriber state_sub[NUM_DRONE];
	ros::Publisher multi_state_pub = nh.advertise<swarm_ctrl_pkg::msgState>("multi_state", 50);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_state = "/mavros/state";

	FuncPtr stateFP[NUM_DRONE] = {stateCB0, stateCB1, stateCB2, stateCB3, stateCB4};
	mavros_msgs::State pre_state[NUM_DRONE];

	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		state_sub[i] = nh.subscribe<mavros_msgs::State>(
			group_name + stream.str() + d_mavros_state, 10, stateFP[i]);
		stream.str("");
	}

	ros::Rate rate(20.0); // period 0.005
	for(int i=0 ; i< NUM_DRONE; i++){
		pre_state[i].connected = true;
		pre_state[i].mode = "OFFBOARD";
		pre_state[i].armed = true;
	}

	ROS_INFO("State check start");
	while(ros::ok()){
		int cnt_connected = 0;
		int cnt_mode = 0;
		int cnt_armed = 0;
		swarm_ctrl_pkg::msgState msg;
		for(int i=0 ; i< NUM_DRONE; i++){
			(current_state[i].connected == true) ? cnt_connected++ : cnt_connected = 0;
			if(current_state[i].connected != pre_state[i].connected){
				if(current_state[i].connected == true){
					ROS_WARN("Camila%d is connected", i);	
				}
				else{
					ROS_WARN("Camila%d is not connected", i);	
				}	
			}
			(current_state[i].mode == "OFFBOARD") ? cnt_mode++ : cnt_mode = 0;
			if(current_state[i].mode != pre_state[i].mode){
				ROS_INFO("Camila%d is %s mode", i, mode[i].c_str());			
			}
			(current_state[i].armed == true) ? cnt_armed++ : cnt_armed = 0;
			if(current_state[i].armed != pre_state[i].armed){
				if(current_state[i].armed == true){
					ROS_INFO("Camila%d is armed", i);	
				}
				else{
					ROS_INFO("Camila%d is disarmed", i);
				}
				
			}
			pre_state[i] = current_state[i];
		}
		(cnt_connected == NUM_DRONE) ? msg.connected = true : msg.connected = false;
		(cnt_mode == NUM_DRONE) ? msg.mode = true : msg.mode = false;
		(cnt_armed == NUM_DRONE) ? msg.armed = true : msg.armed = false;
		
		multi_state_pub.publish(msg);

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}