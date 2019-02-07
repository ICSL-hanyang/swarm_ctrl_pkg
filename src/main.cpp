#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(20);

<<<<<<< HEAD:src/swarm.cpp
	SwarmVehicle *camila;
	camila = new SwarmVehicle("camila", 40);
=======
	double num_drone;
	nh.getParam("num_drone", num_drone);
	std::unique_ptr<SwarmVehicle> camila(new SwarmVehicle(nh, "camila", num_drone));
>>>>>>> 4ea3e04952fb3f5d048db1379b067a42f9bd9a2a:src/main.cpp

	ROS_INFO("swarm_node start");
	
	camila->init();

	while (ros::ok())
	{
		camila->run();

		ros::spinOnce();
		rate.sleep();
	}

	camila.release();
	return 0;
}