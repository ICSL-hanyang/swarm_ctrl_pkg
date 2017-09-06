#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //local_position 용
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
#include "swarm_ctrl_pkg/srvMultiLanding.h" 
#define NUM_DRONE 4

ros::ServiceClient multi_set_pos_local_client;
ros::ServiceClient multi_set_vel_local_client;
geometry_msgs::PoseStamped l_pos[NUM_DRONE];
std::string group_name = "camila";
	
bool multiLanding(swarm_ctrl_pkg::srvMultiLanding::Request &req, swarm_ctrl_pkg::srvMultiLanding::Response &res){
	swarm_ctrl_pkg::srvMultiSetPosLocal p_msg;
	swarm_ctrl_pkg::srvMultiSetVelLocal v_msg;
	if(req.where == "here" || req.where == "HERE" | req.where == "Here"){
		p_msg.request.pos_flag = true;
		p_msg.request.x = l_pos[0].pose.position.x;
		p_msg.request.y = l_pos[0].pose.position.y;
		p_msg.request.z = 0;
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

		res.success = true;
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
	ros::init(argc, argv, "landing_node");

	ros::NodeHandle nh;
	ros::Subscriber local_pos_sub[NUM_DRONE];
	ros::ServiceServer multi_landing_server = nh.advertiseService("multi_landing", multiLanding);
	multi_set_pos_local_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetPosLocal>("multi_set_pos_local");
	multi_set_vel_local_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetVelLocal>("multi_set_vel_local");

	std::stringstream stream;  
	std::string d_mavros_l_pos = "/mavros/local_position/pose";
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		local_pos_sub[i] = nh.subscribe(
			group_name + stream.str() + d_mavros_l_pos, 10, LocalPosCB);
		stream.str("");
	}
	ros::Rate rate(20.0); // period 0.005 s
	ROS_INFO("Ready to landing");
	ros::spin();

	return 0;
}