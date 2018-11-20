#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(10);

	SwarmVehicle *camila;
	camila = new SwarmVehicle("camila", 2);

	ROS_INFO("swarm_node start");

	mavros_msgs::State state;
	camila->setSwarmMap();
	camila->offsetPublisher();

	while (ros::ok())
	{

		camila->formationGenerater();

		ros::spinOnce();
		rate.sleep();
	}

	delete camila;
	return 0;
}