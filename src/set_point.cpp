#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <geometry_msgs/TwistStamped.h> //set velocity 용
#include <mavros_msgs/PositionTarget.h> //set raw 용
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetVelLocal.h"
#include "swarm_ctrl_pkg/srvMultiSetRawLocal.h"
//#include "swarm_ctrl_pkg/multi_header.h"

#define NUM_DRONE 4

double offset = 2;
double pre_offset;
double pre_req_pos[3] = {0.0, 0.0, 2.0}; //x, y, z
double pre_req_vel[3] = {0.0, 0.0, 0.0}; //vel_x, vel_y, vel_z
double pre_req_raw[3][3] = {0.0,}; //x, y, z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z
bool b_pos_flag = false;
bool b_vel_flag = false;
bool b_raw_flag = false;

std::string formation = "diamond";
geometry_msgs::PoseStamped l_pos[NUM_DRONE];
geometry_msgs::TwistStamped l_vel[NUM_DRONE];
mavros_msgs::PositionTarget l_raw[NUM_DRONE];
void mkFomation(std::string formation, double _offset);

bool multiSetPosLocal(swarm_ctrl_pkg::srvMultiSetPosLocal::Request &req, 
	swarm_ctrl_pkg::srvMultiSetPosLocal::Response &res){
	b_pos_flag = req.pos_flag;
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

bool multiSetRawLocal(swarm_ctrl_pkg::srvMultiSetRawLocal::Request &req, 
	swarm_ctrl_pkg::srvMultiSetRawLocal::Response &res){
	if((req.flag_mask&0x01) == 0x01){
		b_raw_flag = true;
		b_pos_flag = false;
		b_vel_flag = false; 
		for(int i = 0; i < NUM_DRONE; i++){
			l_raw[i].coordinate_frame = req.coordinate_frame;
			l_raw[i].type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
			mavros_msgs::PositionTarget::IGNORE_PY | 
			mavros_msgs::PositionTarget::IGNORE_PZ | 
			mavros_msgs::PositionTarget::IGNORE_VX | 
			mavros_msgs::PositionTarget::IGNORE_VY | 
			mavros_msgs::PositionTarget::IGNORE_VZ | 
			mavros_msgs::PositionTarget::IGNORE_AFX | 
			mavros_msgs::PositionTarget::IGNORE_AFY | 
			mavros_msgs::PositionTarget::IGNORE_AFZ | 
			mavros_msgs::PositionTarget::IGNORE_YAW | 
			mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
		}
	}
	else{
		b_raw_flag = false;	
	}

	if((req.flag_mask&0x02) == 0x02){
		if (req.x != pre_req_raw[0][0] | req.y != pre_req_raw[0][1] | req.z != pre_req_raw[0][2] | offset != pre_offset){
			l_raw[0].position.x = req.x;
			l_raw[0].position.y = req.y;
			l_raw[0].position.z = req.z;
			pre_req_raw[0][0] = req.x;
			pre_req_raw[0][1] = req.y;
			pre_req_raw[0][2] = req.z;
			pre_offset = offset;
			mkFomation(formation, offset);
			for (int i = 0; i < NUM_DRONE; i++){
				l_raw[i].type_mask |= !(mavros_msgs::PositionTarget::IGNORE_PX | 
					mavros_msgs::PositionTarget::IGNORE_PY | 
					mavros_msgs::PositionTarget::IGNORE_PZ);
			}
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
	if((req.flag_mask& 0x04) == 0x04){
		if (req.vel_x != pre_req_raw[1][0] | req.vel_y != pre_req_raw[1][1] | req.vel_z != pre_req_raw[1][2]){
			for (int i = 0; i < NUM_DRONE; i++){
				l_raw[i].velocity.x = req.vel_x;
				l_raw[i].velocity.y = req.vel_y;
				l_raw[i].velocity.z = req.vel_z;
				l_raw[i].type_mask |= !(mavros_msgs::PositionTarget::IGNORE_VX | 
					mavros_msgs::PositionTarget::IGNORE_VY | 
					mavros_msgs::PositionTarget::IGNORE_VZ);
			}
			ROS_INFO("vel(%lf, %lf, %lf)", req.vel_x, req.vel_y, req.vel_z);
			pre_req_raw[1][0] = req.vel_x;
			pre_req_raw[1][1] = req.vel_y;
			pre_req_raw[1][2] = req.vel_z;
			res.success = true;
		}
		else{
			res.success = false;
		}
	}
	else{
		res.success = false;
	}
	if((req.flag_mask&0x08) == 0x08){
		if (req.acc_x != pre_req_raw[2][0] | req.acc_y != pre_req_raw[2][1] | req.acc_z != pre_req_raw[2][2]){
			for (int i = 1; i < NUM_DRONE; i++){
				l_raw[i].acceleration_or_force.x = req.acc_x;
				l_raw[i].acceleration_or_force.y = req.acc_y;
				l_raw[i].acceleration_or_force.z = req.acc_z;
				l_raw[i].type_mask |= !(mavros_msgs::PositionTarget::IGNORE_AFX | 
					mavros_msgs::PositionTarget::IGNORE_AFY | 
					mavros_msgs::PositionTarget::IGNORE_AFZ);
			}
			ROS_INFO("acc(%lf, %lf, %lf)", req.acc_x, req.acc_y, req.acc_z);
			pre_req_raw[2][0] = req.acc_x;
			pre_req_raw[2][1] = req.acc_y;
			pre_req_raw[2][2] = req.acc_z;
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

void mkFomation(std::string formation, double _offset){
	if(formation == "diamond" || formation == "Diamond" || formation == "DIAMOND"){
		for (int i = 1; i < NUM_DRONE; i++){
			l_pos[i] = l_pos[0];
			if(i > 1)
				l_pos[i].pose.position.z += 2;
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "set_point_node");

	ros::NodeHandle nh;
	ros::Publisher local_pos_pub[NUM_DRONE];
	ros::Publisher local_vel_pub[NUM_DRONE];
	ros::Publisher local_raw_pub[NUM_DRONE];
	ros::ServiceServer multi_set_pos_local_server = nh.advertiseService("multi_set_pos_local", multiSetPosLocal);
	ros::ServiceServer multi_set_vel_local_server = nh.advertiseService("multi_set_vel_local", multiSetVelLocal);
	ros::ServiceServer multi_set_raw_local_server = nh.advertiseService("multi_set_raw_local", multiSetRawLocal);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_l_pos = "/mavros/setpoint_position/local";
	std::string d_mavros_l_vel = "/mavros/setpoint_velocity/cmd_vel";
	std::string d_mavros_l_raw = "/mavros/setpoint_raw/local";
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		local_pos_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(
			group_name + stream.str() + d_mavros_l_pos, 10);
		local_vel_pub[i] = nh.advertise<geometry_msgs::TwistStamped>(
			group_name + stream.str() + d_mavros_l_vel, 10);
		local_raw_pub[i] = nh.advertise<mavros_msgs::PositionTarget>(
			group_name + stream.str() + d_mavros_l_raw, 10);
		stream.str("");
	}

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

  	if(b_raw_flag){
  		for (int i = 0; i < NUM_DRONE; i++){
  			l_raw[i].header.stamp = ros::Time::now();
  			local_raw_pub[i].publish(l_raw[i]);

  		}
  	}

  	ros::spinOnce();
  	rate.sleep();
  }
  return 0; 
}
