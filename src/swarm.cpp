#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh("~");

	SwarmVehicle *camila;
	camila = new SwarmVehicle("camila", 6);

	ROS_INFO("swarm_node start");
	ros::spin();

	delete camila;
	return 0;
}