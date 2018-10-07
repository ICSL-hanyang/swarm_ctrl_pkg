#include <ros/ros.h>
#include <vehicle.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "swarm_node");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	VehicleInfo camila0_info, camila1_info;
	camila0_info.vehicle_name = "mavros";
	camila1_info.vehicle_name = "mavros1";
	camila0_info.system_id=1;	
	camila1_info.system_id=1;
	VehicleInfo mav_info;
	mav_info.system_id = 2;
	mav_info.vehicle_name = "mav2";

	//Vehicle camila0, camila1(camila1_info);
	// Vehicle camila2(camila0_info);
	//camila2 = camila0;
	SwarmVehicle *camila;
	camila = new SwarmVehicle("mavros", 1);
	camila->addVehicle(camila0_info);
	camila->deleteVehicle(camila1_info);
	/* camila->setSwarmInfo("mav", 5);

	camila->showVehicleList();
	camila->showVehicleList();
	camila->deleteVehicle(mav_info);
	camila->showVehicleList(); */

	/* std::vector<Vehicle> camila;
	camila.push_back(camila0); */

	ROS_INFO("swarm_node start");
	geometry_msgs::PoseStamped msg;
	msg.pose.position.x = 0;
	msg.pose.position.y = 10;
	msg.pose.position.z = 2;

	mavros_msgs::State state;
	while(ros::ok()){
		/* state=camila->
		if(state.connected==true && state.armed != true){
			camila0.arming(true);
			camila0.setMode("auto.takeoff");
		}
		else{
			camila0.gotoLocal(msg);
			camila0.setMode("offboard");
		} */

		
		//camila.camila[0].gotoLocal(msg);
		camila->run();

		ros::spinOnce();
		rate.sleep();	
	}

	delete camila;
	return 0;
}