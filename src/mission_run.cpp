#include <ros/ros.h>
#include <vehicle.h>
#include <vector>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(5);

    std::vector<tf2::Vector3> waypoints;

    waypoints.push_back(tf2::Vector3(0, 0, 2));
    waypoints.push_back(tf2::Vector3(2, 0, 2));
    waypoints.push_back(tf2::Vector3(2, 2, 2));
    waypoints.push_back(tf2::Vector3(2, -2, 2));

	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}