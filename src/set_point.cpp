#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h> //set raw 용
#include "swarm_ctrl_pkg/srvMultiSetRawLocal.h"

#define NUM_DRONE 5
#define ONLY_POS 1
#define ONLY_VEL 2
#define ONLY_ACC 4
#define POS_WITH_VEL 3
#define POS_WITH_ACC 5
#define VEL_WITH_ACC 6
#define POS_WITH_VEL_ACC 7
#define POS_ON 1
#define VEL_ON 2
#define ACC_ON 4


double offset = 2;
double pre_offset;
double pre_req_pos[3] = {0.0, 0.0, 2.0}; //x, y, z
double pre_req_vel[3] = {0.0, 0.0, 0.0}; //vel_x, vel_y, vel_z
double pre_req_raw[3][3] = {0.0,}; //x, y, z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z
bool b_pos_flag = false;
bool b_vel_flag = false;
bool b_raw_flag = false;

std::string formation = "diamond";
mavros_msgs::PositionTarget l_raw[NUM_DRONE];
void mkFomation(std::string formation, double _offset);

bool multiSetRawLocal(swarm_ctrl_pkg::srvMultiSetRawLocal::Request &req, 
	swarm_ctrl_pkg::srvMultiSetRawLocal::Response &res){
	uint16_t type_mask = 0;
	(req.flag_mask > 0) ? b_raw_flag = true : b_raw_flag = false;
	switch(req.flag_mask){
		case ONLY_POS : type_mask = (3 << 10) | (7 << 6) | (7 << 3); break;
		case ONLY_VEL : type_mask = (3 << 10) | (7 << 6) | (7 << 0); break;
		case ONLY_ACC : type_mask = (3 << 10) | (7 << 3) | (7 << 0); break;
		case POS_WITH_VEL : type_mask = (3 << 10) | (7 << 6); break;
		case POS_WITH_ACC : type_mask = (3 << 10) | (7 << 3); break;
		case VEL_WITH_ACC : type_mask = (3 << 10) | (7 << 0);break;
		case POS_WITH_VEL_ACC : type_mask = (3 << 10); break;
		default : type_mask = 0; break;
	}
	if((req.flag_mask&POS_ON) == POS_ON){
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
				l_raw[i].type_mask = type_mask;
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
	if((req.flag_mask&VEL_ON) == VEL_ON){
		if (req.vel_x != pre_req_raw[1][0] | req.vel_y != pre_req_raw[1][1] | req.vel_z != pre_req_raw[1][2]){
			for (int i = 0; i < NUM_DRONE; i++){
				l_raw[i].velocity.x = req.vel_x;
				l_raw[i].velocity.y = req.vel_y;
				l_raw[i].velocity.z = req.vel_z;
				l_raw[i].type_mask = type_mask;
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
	if((req.flag_mask&ACC_ON) == ACC_ON){
		if (req.acc_x != pre_req_raw[2][0] | req.acc_y != pre_req_raw[2][1] | req.acc_z != pre_req_raw[2][2]){
			for (int i = 1; i < NUM_DRONE; i++){
				l_raw[i].acceleration_or_force.x = req.acc_x;
				l_raw[i].acceleration_or_force.y = req.acc_y;
				l_raw[i].acceleration_or_force.z = req.acc_z;
				l_raw[i].type_mask = type_mask;
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
			l_raw[i].position = l_raw[0].position;
			if(i < 3)
				l_raw[i].position.x += _offset;
			else if(i < 5){
				l_raw[i].position.y += _offset;
			}
			_offset *= -1;
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "set_point_node");

	ros::NodeHandle nh;
	ros::Publisher local_raw_pub[NUM_DRONE];
	ros::ServiceServer multi_set_raw_local_server = nh.advertiseService("multi_set_raw_local", multiSetRawLocal);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_l_raw = "/mavros/setpoint_raw/local";
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		local_raw_pub[i] = nh.advertise<mavros_msgs::PositionTarget>(
			group_name + stream.str() + d_mavros_l_raw, 10);
		stream.str("");
	}

  ros::Rate rate(20.0); // period 0.05 s
  ROS_INFO("Local_position publish start");

  while (ros::ok()){
  	nh.getParam("set_point_node/offset", offset);
  	nh.getParam("set_point_node/formation", formation);

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
