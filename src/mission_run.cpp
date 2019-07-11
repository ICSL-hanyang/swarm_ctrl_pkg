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
	mission.pushWaypoint(tf2::Vector3(0, 0, 1.3));
	mission.pushWaypoint(tf2::Vector3(4, 0, 1.3));
	// mission.pushWaypoint(tf2::Vector3(0, 0, 1.5));
	// mission.pushWaypoint(tf2::Vector3(6, -4, 1.5));
	// mission.pushWaypoint(tf2::Vector3(11, 4, 1.5));
	// mission.pushWaypoint(tf2::Vector3(19, -4, 1.5));
	// mission.pushWaypoint(tf2::Vector3(28., 4, 1.5));
	// mission.pushWaypoint(tf2::Vector3(28., -14, 1.5));
	// mission.pushWaypoint(tf2::Vector3(21., -5, 1.5));
	// mission.pushWaypoint(tf2::Vector3(15, -14, 1.5));
	// mission.pushWaypoint(tf2::Vector3(9, -5, 1.5));
	// mission.pushWaypoint(tf2::Vector3(0, -10, 1.5));
	// mission.pushWaypoint(tf2::Vector3(0, -10, -5));

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