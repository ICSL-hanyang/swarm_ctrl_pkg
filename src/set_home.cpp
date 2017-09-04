#include <ros/ros.h>
#include <mavros_msgs/CommandHome.h>   //set_home
#include <sensor_msgs/NavSatFix.h>     //set_home
#include "swarm_ctrl_pkg/srvMultiSetHome.h"
#define NUM_DRONE 5

ros::ServiceClient set_home_client[NUM_DRONE];
sensor_msgs::NavSatFix g_pos[NUM_DRONE];
std::string group_name = "camila";

bool multiSetHome(swarm_ctrl_pkg::srvMultiSetHome::Request &req, 
	swarm_ctrl_pkg::srvMultiSetHome::Response &res){
	int cnt_home = 0;
	mavros_msgs::CommandHome home_gps;
	home_gps.request.current_gps = false;
    home_gps.request.latitude = g_pos[req.drone_num].latitude;
    home_gps.request.longitude = g_pos[req.drone_num].longitude;
    home_gps.request.altitude = g_pos[req.drone_num].altitude;
    for (int i = 0; i < NUM_DRONE; i++){
      if (set_home_client[i].call(home_gps) && home_gps.response.success){
        ROS_INFO("Camila%d reset home position", i);
        cnt_home++;
      }
      else{
      	cnt_home = 0;	
      }
    }
 	(cnt_home == NUM_DRONE) ? res.success = true : res.success = false;    
	return true;
}

void globalPosCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
{	
	std::stringstream stream;
	for(int i = 0; i < NUM_DRONE; i++){
		stream << i;
		if(msg->header.frame_id == group_name + stream.str())
			g_pos[i] = *msg;
		stream.str("");
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "set_home_node");

	ros::NodeHandle nh;
	ros::Subscriber global_pos_sub[NUM_DRONE];
	ros::ServiceServer multi_set_home_server = nh.advertiseService("multi_set_home", multiSetHome);

	std::stringstream stream;  
	std::string d_mavros_home = "/mavros/cmd/set_home";
	std::string d_mavros_g_pos	= "/mavros/global_position/global";

	for(int i=0 ; i < NUM_DRONE ; i++){
		stream << i;
		global_pos_sub[i] = nh.subscribe<sensor_msgs::NavSatFix>(
			group_name + stream.str() + d_mavros_g_pos, 10, globalPosCB);	
		set_home_client[i] = nh.serviceClient<mavros_msgs::CommandHome>(
			group_name + stream.str() + d_mavros_home);
		stream.str("");
	}
	ros::Rate rate(10.0); // period 0.005 s
	ROS_INFO("Ready to set_home");

	ros::spin();
	return 0; 
}
