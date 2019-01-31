#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(20);

	double num_drone;
	nh.getParam("num_drone", num_drone);
	SwarmVehicle *camila;
	camila = new SwarmVehicle("camila", (int) num_drone);

	ROS_INFO("swarm_node start");
	
	camila->init();

	while (ros::ok())
	{
		camila->run();

		ros::spinOnce();
		rate.sleep();
	}

	delete camila;
	return 0;
}