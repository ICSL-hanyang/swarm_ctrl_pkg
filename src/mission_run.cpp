#include <ros/ros.h>
#include <vehicle.h>
#include <vector>
#include <mission.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_node");
	ros::NodeHandle nh("~");
	ros::Rate rate(5);

    Mission mission;
	mission.pushWaypoint(tf2::Vector3(0, 0, 1.5));
	mission.pushWaypoint(tf2::Vector3(4.5, 3.1, 1.5));
	mission.pushWaypoint(tf2::Vector3(9, -3.1, 1.5));
	mission.pushWaypoint(tf2::Vector3(15, 3.1, 1.5));
	mission.pushWaypoint(tf2::Vector3(21, -3.1, 1.5));
	mission.pushWaypoint(tf2::Vector3(27, 3.1, 1.5));
	mission.pushWaypoint(tf2::Vector3(27, -13, 1.5));
	mission.pushWaypoint(tf2::Vector3(21, -6.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(15.5, -13, 1.5));
	mission.pushWaypoint(tf2::Vector3(10, -6.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(5, -9.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(0, -9.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(0, -9.5, -5));

	ros::Time now = ros::Time::now();
	while (ros::Time::now() < now + ros::Duration(20)){
		ros::spinOnce();
		rate.sleep();	
	}

	while (ros::ok())
	{
		mission.run();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}