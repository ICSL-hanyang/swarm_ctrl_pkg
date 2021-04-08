#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh("~");
	// ros::NodeHandle nh_mul("multi");
	// ros::NodeHandle nh_global("");
	ros::Rate rate(15);

	// ros::Publisher multi_arming = nh_mul.advertise<std_msgs::Bool>("arming", 10);
	// ros::Publisher multi_mode = nh_mul.advertise<std_msgs::String>("set_mode", 10);
	// ros::ServiceClient multi_setpoint = nh_global.serviceClient<swarm_ctrl_pkg::srvMultiSetpointLocal>("multi_setpoint_local");

	// std_msgs::Bool arm_cmd;
	// arm_cmd.data = true;
	// std_msgs::String mode;
	// mode.data = "offboard";
	// swarm_ctrl_pkg::srvMultiSetpointLocal setpoint_cmd;
	// setpoint_cmd.request.formation = "SCEN5";
	// setpoint_cmd.request.x = 0;
	// setpoint_cmd.request.y = 0;
	// setpoint_cmd.request.z = 10;

	double num_drone;
	nh.getParam("num_drone", num_drone);
	std::unique_ptr<SwarmVehicle> camila(new SwarmVehicle(nh, "camila", num_drone));

	// ROS_INFO("swarm_node start");

	// ros::Time now = ros::Time::now();
	// while(ros::ok())
	// {
	// 	if( ros::Time::now() > now + ros::Duration(20))
	// 		break;
	// 	ros::Duration term = now + ros::Duration(20) - ros::Time::now() ;
	// 	ROS_INFO("Start in %lf second", term.toSec());
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }

	// while(ros::ok()){
	// 	camila->run();
	// 	if(multi_setpoint.call(setpoint_cmd) && setpoint_cmd.response.success ){
	// 		ROS_INFO("Scenario : SCEN5 , (0, 0, 10)");
	// 		break;
	// 	}
	// 	else
	// 		ROS_WARN("setpoint call fail");
	// }
	// now = ros::Time::now();
	// while(ros::ok())
	// {
	// 	if(ros::Time::now() > now + ros::Duration(3)){
	// 		break;
	// 	}
	// 	multi_arming.publish(arm_cmd);
	// 	ROS_INFO("Multi arming...");
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }
	// now = ros::Time::now();
	// while(1)
	// {
	// 	if(ros::Time::now() > now + ros::Duration(3)){
	// 		break;
	// 	}
	// 	multi_mode.publish(mode);
	// 	ROS_INFO("Multi set mode to 'offboard'...");
	// 	ros::spinOnce();
	// 	rate.sleep();
	// }	

	while (ros::ok())
	{
		camila->run();

		ros::spinOnce();
		rate.sleep();
	}

	camila.release();
	return 0;
}