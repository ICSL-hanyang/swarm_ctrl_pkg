#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include "swarm_ctrl_pkg/srvMultiSetPosLocal.h"
#define NUM_DRONE 4

double pre_req[4] = {0.0, 0.0, 0.0, 2.0};
bool b_sending = false;
geometry_msgs::PoseStamped l_pos[NUM_DRONE];

bool multiSetPosLocal(swarm_ctrl_pkg::srvMultiSetPosLocal::Request &req, 
	swarm_ctrl_pkg::srvMultiSetPosLocal::Response &res){
	b_sending = req.sending;
	if(req.sending){
		if (req.x != pre_req[0] | req.y != pre_req[1] | req.z != pre_req[2] | req.offset != pre_req[3]){
			l_pos[0].pose.position.x = req.x;
			l_pos[0].pose.position.y = req.y;
			l_pos[0].pose.position.z = req.z;
			pre_req[0] = req.x;
			pre_req[1] = req.y;
			pre_req[2] = req.z;
			pre_req[3] = req.offset;

			for (int i = 1; i < NUM_DRONE; i++){
				l_pos[i] = l_pos[0];
				if(i < 3)
					l_pos[i].pose.position.x += req.offset;
				else if(i < 5){
					l_pos[i].pose.position.y += req.offset;
				}
				req.offset *= -1;
			}
			ROS_INFO("(%lf, %lf, %lf), offset : %lf", req.x, req.y, req.z, req.offset);
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

int main(int argc, char** argv){
	ros::init(argc, argv, "set_pos_local_node");

	ros::NodeHandle nh;
	ros::Publisher local_pos_pub[NUM_DRONE];
	ros::ServiceServer multi_set_pos_local_server = nh.advertiseService("multi_set_pos_local", multiSetPosLocal);

	std::stringstream stream;  
	std::string group_name = "camila";
	std::string d_mavros_l_pos = "/mavros/setpoint_position/local";
	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		local_pos_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(
			group_name + stream.str() + d_mavros_l_pos, 10);
		stream.str("");
	}
	ros::Rate rate(20.0); // period 0.005 s
	ROS_INFO("Local_position publish start");

	while (ros::ok()){
		if(b_sending){
			for (int i = 0; i < NUM_DRONE; i++){
				local_pos_pub[i].publish(l_pos[i]);
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0; 
}
