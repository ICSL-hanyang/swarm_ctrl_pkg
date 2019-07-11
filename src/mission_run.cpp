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
	mission.pushWaypoint(tf2::Vector3(5.6, 3.1, 1.5));
	mission.pushWaypoint(tf2::Vector3(9, 0, 1.5));
	mission.pushWaypoint(tf2::Vector3(15, -2, 1.5));
	mission.pushWaypoint(tf2::Vector3(21., 0, 1.5));
	mission.pushWaypoint(tf2::Vector3(26.5, 3.1, 1.5));
	mission.pushWaypoint(tf2::Vector3(26.5, -13, 1.5));
	mission.pushWaypoint(tf2::Vector3(11, -11.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(14, -10.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(8.5, -9.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(8.5, -9.5, 0.9));
	mission.pushWaypoint(tf2::Vector3(6.2, -9.5, 0.9));
	mission.pushWaypoint(tf2::Vector3(6.2, -9.5, 2));
	mission.pushWaypoint(tf2::Vector3(5, -9.5, 2.5));
	mission.pushWaypoint(tf2::Vector3(0, -9.5, 1.5));
	mission.pushWaypoint(tf2::Vector3(0, -9.5, -5));

	ros::Time now = ros::Time::now();
	while (ros::Time::now() < now + ros::Duration(15)){
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