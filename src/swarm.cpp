#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	/*VehicleInfo camila0_info, camila1_info;
	camila0_info.vehicle_name = "camila0";
	camila1_info.vehicle_name = "camila1";*/

	//Vehicle camila0(camila0_info), camila1(camila1_info);
	SwarmVehicle *camila;
	camila = new SwarmVehicle("camila", 1);

	ROS_INFO("swarm_node start");
	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = 0;
	msg.pose.position.y = 0;
	msg.pose.position.z = 2;

	while(ros::ok()){
		//camila.camila[0].gotoLocal(msg);

		ros::spinOnce();
		rate.sleep();	
	}

	return 0;
}