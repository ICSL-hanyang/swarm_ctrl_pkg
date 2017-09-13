#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <geometry_msgs/TwistStamped.h> //set velocity 용
#include <mavros_msgs/PositionTarget.h> //set raw 용
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
//#include "swarm_ctrl_pkg/srvMultiSetRawLocal.h"
#define NUM_DRONE 5

double offset = 2;
double pre_offset;
double pre_req_pos[3] = {0.0, 0.0, 2.0}; //x, y, z
double pre_req_vel[3] = {0.0, 0.0, 0.0}; //vel_x, vel_y, vel_z
//double pre_req_raw[3][3] = {0.0,}; //x, y, z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z
bool b_pos_flag = false;
bool b_vel_flag = false;
//bool b_acc_flag = false;

std::string formation = "diamond";
geometry_msgs::PoseStamped l_pos[NUM_DRONE];
geometry_msgs::TwistStamped l_vel[NUM_DRONE];
//mavros_msgs::PositionTarget l_raw[NUM_DRONE];
void mkFomation(std::string formation, double _offset);

bool multiSetPosLocal(swarm_ctrl_pkg::srvMultiSetPosLocal::Request &req, 
	swarm_ctrl_pkg::srvMultiSetPosLocal::Response &res){
	b_pos_flag = req.pos_flag;
	//nh.getParam("set_point_node/offset", offset);
	if(req.pos_flag){
		if (req.x != pre_req_pos[0] | req.y != pre_req_pos[1] | req.z != pre_req_pos[2] | offset != pre_offset){
			l_pos[0].pose.position.x = req.x;
			l_pos[0].pose.position.y = req.y;
			l_pos[0].pose.position.z = req.z;
			pre_req_pos[0] = req.x;
			pre_req_pos[1] = req.y;
			pre_req_pos[2] = req.z;
			pre_offset = offset;
      mkFomation(formation, offset);
			ROS_INFO("move(%lf, %lf, %lf) offset : %lf", req.x, req.y, req.z, offset);
			res.success = true;
		}
		else{
			res.success = false;
		}
	}
	else{
		res.success = false;
	}
	return true;
}

bool multiSetVelLocal(swarm_ctrl_pkg::srvMultiSetVelLocal::Request &req, 
	swarm_ctrl_pkg::srvMultiSetVelLocal::Response &res){
	b_vel_flag = req.vel_flag;
	if(req.vel_flag){
		if (req.vel_x != pre_req_vel[0] | req.vel_y != pre_req_vel[1] | req.vel_z != pre_req_vel[2]){
			for (int i = 0; i < NUM_DRONE; i++){
				l_vel[i].twist.linear.x = req.vel_x;
				l_vel[i].twist.linear.y = req.vel_y;
				l_vel[i].twist.linear.z = req.vel_z;
			}
			ROS_INFO("vel(%lf, %lf, %lf)", req.vel_x, req.vel_y, req.vel_z);
			pre_req_vel[0] = req.vel_x;
			pre_req_vel[1] = req.vel_y;
			pre_req_vel[2] = req.vel_z;

			res.success = true;
		}
		else{
			res.success = false;
		}
	}
	else{
		res.success = false;
	}
	return true;
}
/*
bool multiSetRawLocal(swarm_ctrl_pkg::srvMultiSetRawLocal::Request &req, 
	swarm_ctrl_pkg::srvMultiSetRawLocal::Response &res){
	b_pos_flag = req.pos_flag;
	b_vel_flag = req.vel_flag;
	b_acc_flag = req.acc_flag;
	if(req.pos_flag){
		if (req.x != pre_req_raw[0] | req.y != pre_req_raw[1] | req.z != pre_req_raw[2] | req.offset != pre_req_raw[9]){
			l_raw[0].position.x = req.x;
			l_raw[0].position.y = req.y;
			l_raw[0].position.z = req.z;
			pre_req_raw[0] = req.x;
			pre_req_raw[1] = req.y;
			pre_req_raw[2] = req.z;
			pre_req_raw[9] = req.offset;
			for (int i = 1; i < NUM_DRONE; i++){
				l_raw[i] = l_raw[0];
				if(i < 3)
					l_raw[i].position.x += req.offset;
				else if(i < 5){
					l_raw[i].position.y += req.offset;
				}
				req.offset *= -1;
			}
			req.offset = (req.offset > 0) ? req.offset : (-1)*req.offset;
			ROS_INFO("move(%lf, %lf, %lf) offset : %lf", req.x, req.y, req.z, req.offset);
			res.success = true;
			else{
				res.success = false;
			}
		}	
	}
	else{
		res.success = false;
	}
	if(req.vel_flag){
		if (req.vel_x != pre_req_raw[3] | req.vel_y != pre_req_raw[4] | req.vel_z != pre_req_raw[5]){
			for (int i = 0; i < NUM_DRONE; i++){
				l_raw[i].velocity.x = req.vel_x;
				l_raw[i].velocity.y = req.vel_y;
				l_raw[i].velocity.z = req.vel_z;
			}
			ROS_INFO("vel(%lf, %lf, %lf)", req.vel_x, req.vel_y, req.vel_z);
			pre_req_raw[3] = req.vel_x;
			pre_req_raw[4] = req.vel_y;
			pre_req_raw[5] = req.vel_z;
			res.success = true;
		}
		else{
			res.success = false;
		}
	}
	else{
		res.success = false;
	}
	if(req.acc_flag){
		if (req.acc_x != pre_req_raw[6] | req.acc_y != pre_req_raw[7] | req.acc_z != pre_req_raw[8]){
			for (int i = 1; i < NUM_DRONE; i++){
				l_raw[i].acceleration_or_force.x = req.acc_x;
				l_raw[i].acceleration_or_force.y = req.acc_y;
				l_raw[i].acceleration_or_force.z = req.acc_z;
			}
			ROS_INFO("acc(%lf, %lf, %lf)", req.acc_x, req.acc_y, req.acc_z);
			pre_req_raw[6] = req.acc_x;
			pre_req_raw[7] = req.acc_y;
			pre_req_raw[8] = req.acc_z;
			res.success = true;
		}
		else{
			res.success = false;
		}
	}
	else{
		res.success = false;
	}
	return true;
}*/

void mkFomation(std::string formation, double _offset){
	if(formation == "diamond" || formation == "Diamond" || formation == "DIAMOND"){
		for (int i = 1; i < NUM_DRONE; i++){
			l_pos[i] = l_pos[0];
			if(i < 3)
				l_pos[i].pose.position.x += _offset;
			else if(i < 5){
				l_pos[i].pose.position.y += _offset;
			}
			_offset *= -1;
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "set_point_node");

	ros::NodeHandle nh;
	ros::Publisher local_pos_pub[NUM_DRONE];
	ros::Publisher local_vel_pub[NUM_DRONE];
	//ros::Publisher local_raw_pub[NUM_DRONE];
	ros::ServiceServer multi_set_pos_local_server = nh.advertiseService("multi_set_pos_local", multiSetPosLocal);
	ros::ServiceServer multi_set_vel_local_server = nh.advertiseService("multi_set_vel_local", multiSetVelLocal);
	//ros::ServiceServer multi_set_raw_local_server = nh.advertiseService("multi_set_raw_local", multiSetRawLocal);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_l_pos = "/mavros/setpoint_position/local";
	std::string d_mavros_l_vel = "/mavros/setpoint_velocity/cmd_vel";
	//std::string d_mavros_l_raw = "/mavros/setpoint_raw/local";
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		local_pos_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(
			group_name + stream.str() + d_mavros_l_pos, 10);
		local_vel_pub[i] = nh.advertise<geometry_msgs::TwistStamped>(
			group_name + stream.str() + d_mavros_l_vel, 10);
	//	local_raw_pub[i] = nh.advertise<geometry_msgs::TwistStamped>(
	//		group_name + stream.str() + d_mavros_l_raw, 10);
		stream.str("");
	}

	nh.setParam("set_point_node/offset", 2.0);
  nh.setParam("set_point_node/formation", "diamond");
  ros::Rate rate(20.0); // period 0.05 s
	ROS_INFO("Local_position publish start");

	while (ros::ok()){
		nh.getParam("set_point_node/offset", offset);
    nh.getParam("set_point_node/formation", formation);

		if(b_pos_flag){
			for (int i = 0; i < NUM_DRONE; i++){
				l_pos[i].header.stamp = ros::Time::now();
				local_pos_pub[i].publish(l_pos[i]);
			}
		}

		if(b_vel_flag){
			for (int i = 0; i < NUM_DRONE; i++){
				l_vel[i].header.stamp = ros::Time::now();
				local_vel_pub[i].publish(l_vel[i]);

			}
		}

		ros::spinOnce();
		rate.sleep();
	}
	return 0; 
}
