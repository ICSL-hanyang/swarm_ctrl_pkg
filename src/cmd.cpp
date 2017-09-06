#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>   //arm용
#include <mavros_msgs/SetMode.h>       //OFFBOARD 모드 설정용
#include <geometry_msgs/PoseStamped.h> //local_position 용
#include "swarm_ctrl_pkg/srvMultiArming.h" 
#include "swarm_ctrl_pkg/srvMultiMode.h"
#include "swarm_ctrl_pkg/msgState.h"   //multi_state msg
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
#include "swarm_ctrl_pkg/srvMultiLanding.h" 
#define NUM_DRONE 4

ros::ServiceClient arming_client[NUM_DRONE];
ros::ServiceClient set_mode_client[NUM_DRONE];
ros::ServiceClient multi_set_pos_local_client;
ros::ServiceClient multi_set_vel_local_client;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode set_mode;
geometry_msgs::PoseStamped l_pos[NUM_DRONE];
swarm_ctrl_pkg::msgState multi_state;
std::string group_name = "camila";
bool b_home_landing = false;

bool multiArming(swarm_ctrl_pkg::srvMultiArming::Request &req, swarm_ctrl_pkg::srvMultiArming::Response &res){
	arm_cmd.request.value = req.arming;
	swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
	swarm_ctrl_pkg::srvMultiSetVelLocal v_msg;
	if(req.arming){
		p_msg.request.pos_flag = true;
		p_msg.request.x = l_pos[0].pose.position.x;
		p_msg.request.y = l_pos[0].pose.position.y;
		p_msg.request.z = 2;
		v_msg.request.vel_flag = false;
		if(multi_set_pos_local_client.call(p_msg) && p_msg.response.success){
			if(multi_set_vel_local_client.call(v_msg) && v_msg.response.success){
				for (int i = 0; i < NUM_DRONE; i++){
					if (arming_client[i].call(arm_cmd) && arm_cmd.response.success){
						res.success = true;
					}
					else{
						res.success = false;
					}
				}
			}
			else{
				res.success = false;
			}
		}
		else{
			res.success = false;
		}

	}
	else{
		for (int i = 0; i < NUM_DRONE; i++){
			if (arming_client[i].call(arm_cmd) && arm_cmd.response.success){
				res.success = true;
			}
			else{
				res.success = false;
			}
		}
	}
	if(res.success == false){
		for (int i = 0; i < NUM_DRONE; i++){
			if (arming_client[i].call(arm_cmd) && arm_cmd.response.success){
				ROS_WARN("Arming fail. All drones are disarmed");
			}
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

bool multiLanding(swarm_ctrl_pkg::srvMultiLanding::Request &req, swarm_ctrl_pkg::srvMultiLanding::Response &res){
	swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
	swarm_ctrl_pkg::srvMultiSetVelLocal v_msg;
	if(req.where == "here" || req.where == "HERE" | req.where == "Here"){
		p_msg.request.pos_flag = false;
		v_msg.request.vel_flag = true;
		v_msg.request.vel_z = -0.7;
		if(multi_set_pos_local_client.call(p_msg) && p_msg.response.success){
			if(multi_set_vel_local_client.call(v_msg) && v_msg.response.success){
				res.success = true;
			}
			else{
				res.success = false;	
			}
		}
		else{
			res.success = false;	
		}
	}
	else if(req.where == "home" || req.where == "HOME" | req.where == "Home"){
		p_msg.request.pos_flag = true;
		p_msg.request.x = 0;
		p_msg.request.y = 0;
		p_msg.request.z = l_pos[0].pose.position.z;
		if(multi_set_pos_local_client.call(p_msg) && p_msg.response.success){
			b_home_landing = true;
			res.success = true;
		}
		else{
			res.success = false;	
		}
	}
	return true;
}

void LocalPosCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{	
	std::stringstream stream;
	for(int i = 0; i < NUM_DRONE; i++){
		stream << i;
		if(msg->header.frame_id == group_name + stream.str())
			l_pos[i] = *msg;
		stream.str("");
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "cmd_node");

	ros::NodeHandle nh;
	ros::Subscriber local_pos_sub[NUM_DRONE];
	ros::ServiceServer multi_arming_server = nh.advertiseService("multi_arming", multiArming);
	ros::ServiceServer multi_mode_server = nh.advertiseService("multi_mode", multiMode);
	ros::ServiceServer multi_landing_server = nh.advertiseService("multi_landing", multiLanding);
	ros::Subscriber multi_state_sub = nh.subscribe("multi_state", 50, multiStateCB);
	multi_set_pos_local_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetPosLocal>("multi_set_pos_local");
	multi_set_vel_local_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetVelLocal>("multi_set_vel_local");

	std::stringstream stream;  
	
	std::string d_mavros_arm = "/mavros/cmd/arming";
	std::string d_mavros_mode = "/mavros/set_mode";
	std::string d_mavros_l_pos = "/mavros/local_position/pose";
	
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		arming_client[i] = nh.serviceClient<mavros_msgs::CommandBool>(
			group_name + stream.str() + d_mavros_arm);		
		set_mode_client[i] = nh.serviceClient<mavros_msgs::SetMode>(
			group_name + stream.str() + d_mavros_mode); 
		local_pos_sub[i] = nh.subscribe(
			group_name + stream.str() + d_mavros_l_pos, 10, LocalPosCB);
		stream.str("");
	}
	ros::Rate rate(10.0); // period 0.01 s

	ROS_INFO("Command node started");
	ros::Time set_timer;
	while(ros::ok()){
		if(b_home_landing && (l_pos[0].pose.position.x < 0.5 || l_pos[0].pose.position.x > -0.5)){
			set_timer = ros::Time::now() + ros::Duration(3.0);
			b_home_landing = false;
		}
		if( (set_timer - ros::Time::now()) < ros::Duration(0.2)){
			swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
			swarm_ctrl_pkg::srvMultiSetVelLocal v_msg;
			p_msg.request.pos_flag = false;
			v_msg.request.vel_flag = true;
			v_msg.request.vel_z = -0.7;
			multi_set_pos_local_client.call(p_msg);
			multi_set_vel_local_client.call(v_msg);
		}
		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}
