#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh;

	ros::Rate rate(10);

	ros::Publisher multi_arm_pub = nh.advertise<std_msgs::Bool>("multi/arming", 10);
	ros::Publisher multi_mode_pub = nh.advertise<std_msgs::String>("multi/set_mode", 10);
	ros::ServiceClient goto_client = nh.serviceClient<swarm_ctrl_pkg::srvMultiSetpointLocal>("multi_setpoint_local");
	ros::ServiceClient goto_vehicle_client = nh.serviceClient<swarm_ctrl_pkg::srvGoToVehicle>("goto_vehicle");

	SwarmVehicle *camila;
	camila = new SwarmVehicle("camila", 6);

	ROS_INFO("swarm_node start");
	/* int i = 0;
	while (i != 30)
	{
		i++;
		ros::spinOnce();
		rate.sleep();
	}

	std_msgs::Bool arm;
	arm.data = true;
	i = 0;
	while (i != 4)
	{
		multi_arm_pub.publish(arm);
		i++;
		ROS_INFO("Drones are arming...");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}
	ROS_INFO("1");

	swarm_ctrl_pkg::srvMultiSetpointLocal set_local;
	set_local.request.x = 0;
	set_local.request.y = 0;
	set_local.request.z = 5;
	ROS_INFO("2");
	if(goto_client.call(set_local))
	ROS_INFO("3");
	std_msgs::String mode;
	mode.data = "offboard";
	i = 0;
	while (i != 4)
	{
		multi_mode_pub.publish(mode);
		i++;
		ROS_INFO("Drones are taking off...");
		ros::spinOnce();
		ros::Duration(1.0).sleep();
	}

	double r = 4.0, wn = 1.2, theta;
	int count = 0; */

	while (ros::ok())
	{
		/* theta = wn * count * 0.05;
		set_local.request.x = r * sin(theta);
		set_local.request.y = r * cos(theta);
		set_local.request.x = 5;

		goto_client.call(set_local);
		count++; */
		ros::spinOnce();
		rate.sleep();
	}

	delete camila;
	return 0;
}