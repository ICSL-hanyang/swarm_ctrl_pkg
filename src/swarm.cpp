#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	VehicleInfo camila0_info, camila1_info;
	camila0_info.vehicle_name = "camila0";
	camila1_info.vehicle_name = "camila1";
	VehicleInfo mav_info;
	mav_info.system_id = 2;
	mav_info.vehicle_name = "mav2";

	ROS_INFO_STREAM(camila0_info.system_id << " "<< camila1_info.system_id);

	/* Vehicle camila0, camila1(camila1_info);
	Vehicle camila2(camila0_info); */
	//camila2 = camila0;
	SwarmVehicle *camila;
	camila = new SwarmVehicle("camila", 5);
	camila->setSwarmInfo("mav", 5);

	camila->addVehicle(camila0_info);
	camila->showVehicleList();
	camila->deleteVehicle(camila0_info);
	camila->showVehicleList();
	
	camila->deleteVehicle(mav_info);
	camila->showVehicleList();

	/* std::vector<Vehicle> camila;
	camila.push_back(camila0); */

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

	//delete camila;
	return 0;
}