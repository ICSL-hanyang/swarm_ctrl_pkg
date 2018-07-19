#include <ros/ros.h>
#include <stdio.h>
#include <vehicle.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>  
#include <mavros_msgs/GlobalPositionTarget.h>


Vehicle::Vehicle(){
	vehicle_info.system_id = 1;
	vehicle_info.vehicle_name = "mavros";

	vehicleInit();
}

Vehicle::Vehicle(VehicleInfo _vehicle_info){
	vehicle_info.system_id = _vehicle_info.system_id;
	vehicle_info.vehicle_name = _vehicle_info.vehicle_name;

	vehicleInit();
}

void Vehicle::vehicleInit(){
	nh = ros::NodeHandle(vehicle_info.vehicle_name);

	setpoint_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("setpoint_position/global", 10);
	setpoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 10);

	state_sub = nh.subscribe("state", 10, &Vehicle::stateCB, this);
	battery_sub = nh.subscribe("battery", 10, &Vehicle::batteryCB, this);
	local_pos_sub = nh.subscribe("local_position/pose", 10, &Vehicle::localPositionCB, this);
	global_pos_sub = nh.subscribe("global_position/global", 10, &Vehicle::globalPositionCB, this);

	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("set_mode");
	set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("cmd/set_home");

	ROS_INFO("%s instance generated", vehicle_info.vehicle_name.c_str());

	setpoint_publish_flag = false;
}

bool Vehicle::arming(bool _arm_state){
	mavros_msgs::CommandBool msg;
	msg.request.value = _arm_state;

	if(arming_client.call(msg) && msg.response.success){
		ROS_INFO("%d", msg.response.result);
		return true;
	}
	else{
		ROS_ERROR("%s failed to call arming service. %d", vehicle_info.vehicle_name.c_str(), msg.response.result);
		return false;
	}
}

bool Vehicle::setMode(std::string _mode){
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = _mode;
	if( ( (_mode == "offboard") || (_mode == "OFFBOARD") ) && (setpoint_publish_flag == false)){
		ROS_WARN("Please publish setpoint first");
		return false;
	}
	else{
		if(set_mode_client.call(mode) && mode.response.mode_sent)
			return true;
		else{
			ROS_ERROR("Failed to call set_mode service. %d", mode.response.mode_sent);		
			return false;
		}
	}

}

void Vehicle::gotoGlobal(sensor_msgs::NavSatFix _tar_global){
	bool changed = false;
	
	setpoint_publish_flag = true;

	if((tar_global.latitude != _tar_global.latitude) || (tar_global.longitude != _tar_global.longitude) || (tar_global.altitude != _tar_global.altitude))
		changed = true;

	tar_global = _tar_global;
	tar_global.header.seq += 1;
	tar_global.header.stamp = ros::Time::now();
	tar_global.header.frame_id = vehicle_info.vehicle_name;
	setpoint_global_pub.publish(tar_global);
	
	if(changed)
		ROS_INFO("%s set target_global_pos(long : %lf, lati : %lf, alti : %lf)", vehicle_info.vehicle_name.c_str(), tar_global.longitude, tar_global.latitude, tar_global.altitude);
}

void Vehicle::gotoLocal(geometry_msgs::PoseStamped _tar_local){
	bool changed = false;

	setpoint_publish_flag = true;

	if((tar_local.pose.position.x != _tar_local.pose.position.x) || (tar_local.pose.position.y != _tar_local.pose.position.y) || (tar_local.pose.position.z != _tar_local.pose.position.z))
		changed = true;	

	tar_local = _tar_local;
	tar_local.header.seq += 1;
	tar_local.header.stamp = ros::Time::now();
	tar_local.header.frame_id = vehicle_info.vehicle_name;
	setpoint_local_pub.publish(tar_local);

	if(changed)
		ROS_INFO("%s set target_local_pos(x : %lf, y : %lf, z : %lf)", vehicle_info.vehicle_name.c_str(), tar_local.pose.position.x, tar_local.pose.position.y, tar_local.pose.position.z);
}

void Vehicle::setVehicleInfo(VehicleInfo new_vehicle_info){
	vehicle_info.system_id = new_vehicle_info.system_id;
	vehicle_info.vehicle_name = new_vehicle_info.vehicle_name;

	vehicleInit();
}

VehicleInfo Vehicle::getInfo(){
	return vehicle_info;
}

void Vehicle::stateCB(const mavros_msgs::State::ConstPtr& msg){
	if(cur_state.connected != msg->connected){
		if(msg->connected == true)
			ROS_INFO("%s is connected", vehicle_info.vehicle_name.c_str());
		else
			ROS_WARN("%s is not connected", vehicle_info.vehicle_name.c_str());
	}
	if(cur_state.armed != msg->armed){
		if(msg->armed == true)
			ROS_INFO("%s is armed", vehicle_info.vehicle_name.c_str());
		else
			ROS_INFO("%s is disarmed", vehicle_info.vehicle_name.c_str());
	}
	if(cur_state.mode != msg->mode){
		ROS_INFO("%s is %s mode", vehicle_info.vehicle_name.c_str(), msg->mode.c_str());
	}
	cur_state = *msg;
}

mavros_msgs::State Vehicle::getState(){
	return cur_state;
}

void Vehicle::batteryCB(const sensor_msgs::BatteryState::ConstPtr& msg){
	cur_battery = *msg;
}

sensor_msgs::BatteryState Vehicle::getBattery(){
	return cur_battery;
}

bool Vehicle::setHomeGlobal(){
	mavros_msgs::CommandHome msg;
	
	msg.request.current_gps = false;
	msg.request.latitude = cur_global.latitude;
	msg.request.longitude = cur_global.longitude;
	msg.request.altitude = cur_global.altitude;

	home_global.latitude = cur_global.latitude;
	home_global.longitude = cur_global.longitude;
	home_global.altitude = cur_global.altitude;

	if(set_home_client.call(msg) && msg.response.success){
		ROS_INFO("%s Global home position is set. %d", vehicle_info.vehicle_name.c_str(), msg.response.result);
		ROS_INFO("%s new global home is (long : %lf, lang : %lf, alti : %lf)", vehicle_info.vehicle_name.c_str(), 
			home_global.longitude, home_global.latitude, home_global.altitude);
		return true;
	}
	else{
		ROS_ERROR("%s Failed to set global home position. %d", vehicle_info.vehicle_name.c_str(), msg.response.result);
		return false;
	}
}

sensor_msgs::NavSatFix Vehicle::getHomeGlobal(){
	return home_global;
}

void Vehicle::globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr& msg){
	cur_global = *msg;
}

sensor_msgs::NavSatFix Vehicle::getGlobalPosition(){
	return cur_global;	
}

sensor_msgs::NavSatFix Vehicle::getTargetGlobal(){
	return tar_global;
}


void Vehicle::setHomeLocal(){
	home_local = cur_local;

	ROS_INFO("%s Local home position is set.", vehicle_info.vehicle_name.c_str());
	ROS_INFO("%s new local home is (x : %lf, y : %lf, z : %lf)", vehicle_info.vehicle_name.c_str(), 
		home_local.pose.position.x, home_local.pose.position.y, home_local.pose.position.z);
}

geometry_msgs::PoseStamped Vehicle::getHomeLocal(){
	return home_local;
}

void Vehicle::localPositionCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
	cur_local = *msg;
}

geometry_msgs::PoseStamped Vehicle::getLocalPosition(){
	return cur_local;
}

geometry_msgs::PoseStamped Vehicle::getTargetLocal(){
	return tar_local;
}

bool Vehicle::isPublish(){
	return setpoint_publish_flag;
}


//default value : default name = camila, _num_of_vehicle = 1;
SwarmVehicle::SwarmVehicle(std::string _swarm_name, int _num_of_vehicle){  
	swarm_name = _swarm_name;
	num_of_vehicle = _num_of_vehicle;
	multi_setpoint_publish_flag = false;

	VehicleInfo vehicle_info[num_of_vehicle];
	camila = new Vehicle[num_of_vehicle];

	for(int i=0 ; i < num_of_vehicle ; i++){
		std::stringstream stream;
		stream << i;
		vehicle_info[i].system_id = i+1;
		vehicle_info[i].vehicle_name = swarm_name + stream.str();
		camila[i].setVehicleInfo(vehicle_info[i]);
	}

	nh = ros::NodeHandle(swarm_name);
	multi_arming_server = nh.advertiseService("multi_arming", &SwarmVehicle::multiArming, this);
	multi_mode_server = nh.advertiseService("multi_mode", &SwarmVehicle::multiMode, this);
	multi_sethome_server = nh.advertiseService("multi_sethome", &SwarmVehicle::multiSetHome, this);
	multi_setpoint_local_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server = nh.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
}

void SwarmVehicle::setSwarmInfo(std::string _swarm_name, int _num_of_vehicle){
	delete[] camila;

	swarm_name = _swarm_name;
	num_of_vehicle = _num_of_vehicle;

	VehicleInfo vehicle_info[num_of_vehicle];
	camila = new Vehicle[num_of_vehicle];

	for(int i=0 ; i < num_of_vehicle ; i++){
		std::stringstream stream;
		stream << i;
		vehicle_info[i].system_id = i+1;
		vehicle_info[i].vehicle_name = swarm_name + stream.str();
		camila[i].setVehicleInfo(vehicle_info[i]);
	}

	nh = ros::NodeHandle(swarm_name);
	multi_arming_server = nh.advertiseService("multi_arming", &SwarmVehicle::multiArming, this);
	multi_mode_server = nh.advertiseService("multi_mode", &SwarmVehicle::multiMode, this);
	multi_sethome_server = nh.advertiseService("multi_sethome", &SwarmVehicle::multiSetHome, this);
	multi_setpoint_local_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server = nh.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);

}
std::string SwarmVehicle::getSwarmInfo(){
	return swarm_name;
}

bool SwarmVehicle::multiArming(mavros_msgs::CommandBool::Request& req,
	mavros_msgs::CommandBool::Response& res)
{
	int arm_fail = -1;
	VehicleInfo fail_vehicle_info;

	for(int i=0 ; i < num_of_vehicle ; i++){
		if((!camila[i].arming(req.value)) && (req.value == true)){ //arm fail 
			arm_fail = i;
			fail_vehicle_info = camila[i].getInfo();
			ROS_WARN("Multi arm fail. please check %s", fail_vehicle_info.vehicle_name.c_str());
			break;
		}	
	}

	if(arm_fail == -1){
		res.success = true;
		return true;
	}
	else{
		req.value = false;
		for(int i=0; i < arm_fail ; i++)
			camila[i].arming(req.value);

		res.success = false;
		res.result = arm_fail;
		return false;
	}
}

bool SwarmVehicle::multiMode(swarm_ctrl_pkg::srvMultiMode::Request& req,
	swarm_ctrl_pkg::srvMultiMode::Response& res)
{
	int mode_fail = 0; 
	for(int i=0 ; i < num_of_vehicle ; i++){
		if(camila[i].setMode(req.mode));
		else
			mode_fail++;
	}

	if(mode_fail == 0){
		res.success = true;
		return true;
	}
	else{
		res.success = false;
		return false;	
	}
}

bool SwarmVehicle::multiSetHome(swarm_ctrl_pkg::srvMultiSetHome::Request& req,
	swarm_ctrl_pkg::srvMultiSetHome::Response& res)
{
	int sethome_fail = 0;
	for(int i=0; i < num_of_vehicle ; i++){
		if(camila[i].setHomeGlobal()){
			if(!req.global_only)
				camila[i].setHomeLocal();
		}
		else
			sethome_fail++;
	}

	if(sethome_fail == 0){
		res.success = true;
		return true;
	}
	else{
		res.success = false;
		return false;
	}
}
bool SwarmVehicle::multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request& req,
	swarm_ctrl_pkg::srvMultiSetpointLocal::Response& res)
{
	

	return true;
}

bool SwarmVehicle::multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request& req,
	swarm_ctrl_pkg::srvMultiSetpointGlobal::Response& res)
{
	return true;
}