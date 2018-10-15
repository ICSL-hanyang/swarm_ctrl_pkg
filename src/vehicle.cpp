#include <ros/ros.h>
#include <vehicle.h>

Vehicle::Vehicle() : vehicle_info({1,"mavros"}),
	nh(ros::NodeHandle(vehicle_info.vehicle_name)),
	nh_mul(ros::NodeHandle("multi")),
	setpoint_publish_flag(false)
{
	vehicleInit();
}

Vehicle::Vehicle(VehicleInfo _vehicle_info) : vehicle_info(_vehicle_info),
	nh(ros::NodeHandle(vehicle_info.vehicle_name)),
	nh_mul(ros::NodeHandle("multi")),
	setpoint_publish_flag(false)
{
	vehicleInit();
}

Vehicle::Vehicle(const Vehicle &rhs) : vehicle_info(rhs.vehicle_info),
	nh(ros::NodeHandle(vehicle_info.vehicle_name)),
	nh_mul(ros::NodeHandle("multi")),
	setpoint_publish_flag(rhs.setpoint_publish_flag)
{
	vehicleInit();
	*this = rhs;
}

const Vehicle& Vehicle::operator=(const Vehicle &rhs){
	if(this == &rhs){
		return *this;
	}
	vehicle_info = rhs.vehicle_info;
	setpoint_publish_flag = rhs.setpoint_publish_flag;
	nh = ros::NodeHandle(vehicle_info.vehicle_name);
	nh_mul = ros::NodeHandle("multi");
	vehicleInit();
	return *this;
}

void Vehicle::vehicleInit(){
	// nh.setParam("kp", 1.0);
	
	/* home_global.latitude = 47.397742;
	home_global.longitude = 8.5455937;
	home_global.altitude = 535.323; */
	setpoint_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("setpoint_position/global", 10);
	setpoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 10);
	setpoint_vel_pub = nh.advertise<geometry_msgs::Twist>("setpoint_velocity/cmd_vel_unstamped", 10);

	state_sub = nh.subscribe("state", 10, &Vehicle::stateCB, this);
	battery_sub = nh.subscribe("battery", 10, &Vehicle::batteryCB, this);
	local_pos_sub = nh.subscribe("local_position/pose", 10, &Vehicle::localPositionCB, this);
	global_pos_sub = nh.subscribe("global_position/global", 10, &Vehicle::globalPositionCB, this);
	// global_pos_sub = nh.subscribe("global_position/raw/fix", 10, &Vehicle::globalPositionCB, this);

	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("set_mode");
	set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("cmd/set_home");
	
	multi_arming_sub = nh_mul.subscribe("arming", 10, &Vehicle::multiArming, this);
	multi_set_mode_sub = nh_mul.subscribe("set_mode", 10, &Vehicle::multiSetMode, this);
	multi_set_home_sub = nh_mul.subscribe("set_home", 10, &Vehicle::multiSetHome, this);

	ROS_INFO_STREAM(vehicle_info.vehicle_name <<" instance generated");
}

bool Vehicle::arming(bool _arm_state){
	mavros_msgs::CommandBool msg;
	msg.request.value = _arm_state;

	if(arming_client.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info.vehicle_name << " failed to call arming service. " << msg.response.result);
	return msg.response.success;
}

bool Vehicle::setMode(std::string _mode){
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = _mode;
	if( ( (_mode == "offboard") || (_mode == "OFFBOARD") ) && (!setpoint_publish_flag) ){
		ROS_WARN("Please publish setpoint first");
		return false;
	}
	else{
		if(set_mode_client.call(mode) && mode.response.mode_sent);
		else
			ROS_ERROR_STREAM("Failed to call set_mode service. " << mode.response.mode_sent);		
		return mode.response.mode_sent;
	}
}

void Vehicle::gotoGlobal(sensor_msgs::NavSatFix _tar_global){
	setpoint_publish_flag = true;

	if((tar_global.latitude != _tar_global.latitude) || 
		(tar_global.longitude != _tar_global.longitude) || 
		(tar_global.altitude != _tar_global.altitude))
		ROS_INFO("%s set target_global_pos(long : %lf, lati : %lf, alti : %lf)", vehicle_info.vehicle_name.c_str(), 
			tar_global.longitude, tar_global.latitude, tar_global.altitude);

	tar_global = _tar_global;
	tar_global.header.seq += 1;
	tar_global.header.stamp = ros::Time::now();
	tar_global.header.frame_id = vehicle_info.vehicle_name;

	mavros_msgs::GlobalPositionTarget msg;
	msg.latitude = tar_global.latitude;
	msg.longitude = tar_global.longitude;
	msg.altitude = tar_global.altitude;
	msg.header.seq += 1;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = vehicle_info.vehicle_name;
	msg.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
		 mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
		 mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
		 mavros_msgs::GlobalPositionTarget::IGNORE_VX |
		 mavros_msgs::GlobalPositionTarget::IGNORE_VY |
		 mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
		 mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
		 mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
	
	setpoint_global_pub.publish(msg);
}

void Vehicle::setLocalTarget(geometry_msgs::PoseStamped _tar_local){
	setpoint_publish_flag = true;

	if((tar_local.pose.position.x != _tar_local.pose.position.x) || 
		(tar_local.pose.position.y != _tar_local.pose.position.y) || 
		(tar_local.pose.position.z != _tar_local.pose.position.z) )
	{
		tar_local = _tar_local;
		ROS_INFO("%s set target_local_pos(x : %lf, y : %lf, z : %lf)", vehicle_info.vehicle_name.c_str(), 
			tar_local.pose.position.x, tar_local.pose.position.y, tar_local.pose.position.z);
		
	}
}

void Vehicle::gotoLocal(){
	tar_local.header.seq += 1;
	tar_local.header.stamp = ros::Time::now();
	tar_local.header.frame_id = vehicle_info.vehicle_name;
	setpoint_local_pub.publish(tar_local);	
}

void Vehicle::gotoVel(){
	nh.getParam("kp", kp);
	geometry_msgs::Twist vel;
	
	vel.linear.x = (tar_local.pose.position.x-cur_local.pose.position.x) * kp;
	vel.linear.y = (tar_local.pose.position.y-cur_local.pose.position.y) * kp;
	vel.linear.z = (tar_local.pose.position.z-cur_local.pose.position.z) * kp;

	setpoint_vel_pub.publish(vel);
}

void Vehicle::multiArming(const std_msgs::Bool::ConstPtr& msg){
	mavros_msgs::CommandBool arm;
	arm.request.value = msg->data;

	if(arming_client.call(arm) && arm.response.success){
	}
	else{
		ROS_ERROR_STREAM(vehicle_info.vehicle_name << " failed to call arming service. " << arm.response.result);
	}
}

void Vehicle::multiSetMode(const std_msgs::String::ConstPtr& msg){
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = msg->data;
	if( ( (mode.request.custom_mode == "offboard") || (mode.request.custom_mode == "OFFBOARD") ) 
		&& (!setpoint_publish_flag) ){
		ROS_WARN("Please publish setpoint first");
	}
	else{
		if(set_mode_client.call(mode) && mode.response.mode_sent){
		}
		else{
			ROS_ERROR_STREAM(vehicle_info.vehicle_name <<" failed to call set_mode service. " << mode.response.mode_sent);
		}
	}
}

void Vehicle::multiSetHome(const std_msgs::Empty::ConstPtr& trigger){
	if(setHomeGlobal()){
		setHomeLocal();
	}
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
			ROS_INFO_STREAM(vehicle_info.vehicle_name << " is connected");
		else
			ROS_WARN_STREAM(vehicle_info.vehicle_name << " is not connected");
	}
	if(cur_state.armed != msg->armed){
		if(msg->armed == true)
			ROS_INFO_STREAM(vehicle_info.vehicle_name << " is armed");
		else
			ROS_INFO_STREAM(vehicle_info.vehicle_name << " is disarmed");
	}
	if(cur_state.mode != msg->mode){
		ROS_INFO_STREAM(vehicle_info.vehicle_name << " is " << msg->mode <<" mode");
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


	if(set_home_client.call(msg) && msg.response.success){
		home_global = cur_global;

		ROS_INFO_STREAM(vehicle_info.vehicle_name << " Global home position is set. " << msg.response.result);
		ROS_INFO("%s new global home is (long : %lf, lang : %lf, alti : %lf)", vehicle_info.vehicle_name.c_str(), 
			home_global.longitude, home_global.latitude, home_global.altitude);
	}
	else
		ROS_ERROR_STREAM(vehicle_info.vehicle_name << " Failed to set global home position. " << msg.response.result);
		
	return msg.response.success;
}

sensor_msgs::NavSatFix Vehicle::getHomeGlobal(){
	return home_global;
}

void Vehicle::globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr& msg){
	cur_global = *msg;

	/* geometry_msgs::Vector3 pose = convertGeoToENU(cur_global.latitude, cur_global.longitude,
		cur_global.altitude, home_global.latitude, home_global.longitude, home_global.altitude);
	cur_local.pose.position.x = pose.x;
	cur_local.pose.position.y = pose.y;
	cur_local.pose.position.z = pose.z; */
	/* ROS_INFO_STREAM(cur_local.pose.position.x << " " << cur_local.pose.position.y 
		<<" "<< cur_local.pose.position.z); */
}

sensor_msgs::NavSatFix Vehicle::getGlobalPosition(){
	return cur_global;	
}

sensor_msgs::NavSatFix Vehicle::getTargetGlobal(){
	return tar_global;
}


void Vehicle::setHomeLocal(){
	home_local = cur_local;

	ROS_INFO_STREAM(vehicle_info.vehicle_name << " Local home position is set.");
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

geometry_msgs::Vector3 Vehicle::convertGeoToENU(double coord_lat, double coord_long, 
	double coord_alt, double home_lat, double home_long, double home_alt) 
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

    double lat_rad = coord_lat * M_DEG_TO_RAD;
    double lon_rad = coord_long* M_DEG_TO_RAD;

    double ref_lon_rad = home_long * M_DEG_TO_RAD;
    double ref_lat_rad = home_lat * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    geometry_msgs::Vector3 offset;

    offset.x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
    offset.y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    offset.z = coord_alt - home_alt;

    return offset;
}

bool Vehicle::isPublish(){
	return setpoint_publish_flag;
}


//default value : default name = camila, _num_of_vehicle = 1;
SwarmVehicle::SwarmVehicle(std::string _swarm_name, int _num_of_vehicle) : swarm_name(_swarm_name),
 	num_of_vehicle(_num_of_vehicle),
	nh(ros::NodeHandle()),
	min_length(3.0),
	multi_setpoint_publish_flag(false)
{  	
	VehicleInfo vehicle_info[num_of_vehicle];
	camila.reserve(num_of_vehicle);

	for(int i = 0 ; i < num_of_vehicle ; i++){
		vehicle_info[i].system_id = i+1;
		vehicle_info[i].vehicle_name = swarm_name + std::to_string(i+1);
		camila.push_back(Vehicle(vehicle_info[i]));
	}

	// nh.setParam("setVel", true);
	multi_setpoint_local_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server = nh.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server = nh.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);
}

SwarmVehicle::SwarmVehicle(const SwarmVehicle& rhs): swarm_name(rhs.swarm_name),
	num_of_vehicle(rhs.num_of_vehicle), 
	nh(ros::NodeHandle()),
	min_length(3.0),
	multi_setpoint_publish_flag(rhs.multi_setpoint_publish_flag)
{
	VehicleInfo vehicle_info[num_of_vehicle];
	camila.reserve(num_of_vehicle);

	std::vector<Vehicle>::const_iterator it;
	for(it = rhs.camila.begin() ; it != rhs.camila.end() ; it++){
		camila.push_back(*it);
	}
	// nh.setParam("setVel", true);
	multi_setpoint_local_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server = nh.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server = nh.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);

	*this = rhs;
}

const SwarmVehicle& SwarmVehicle::operator=(const SwarmVehicle &rhs){
	if(this == &rhs){
		return *this;
	}

	swarm_name = rhs.swarm_name;
	num_of_vehicle = rhs.num_of_vehicle;
	
	std::vector<Vehicle>().swap(camila);
	camila.reserve(num_of_vehicle);
	VehicleInfo vehicle_info[num_of_vehicle];

	std::vector<Vehicle>::const_iterator it;
	for(it = rhs.camila.begin() ; it != rhs.camila.end() ; it++){
		camila.push_back(*it);
	}

	nh = ros::NodeHandle();
	// nh.setParam("setVel", true);
	multi_setpoint_local_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server = nh.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server = nh.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);
	
	return *this;
}

SwarmVehicle::~SwarmVehicle(){
	std::vector<Vehicle>().swap(camila);
}

void SwarmVehicle::setSwarmInfo(std::string _swarm_name, int _num_of_vehicle){
	swarm_name = _swarm_name;
	num_of_vehicle = _num_of_vehicle;

	std::vector<Vehicle>().swap(camila);
	camila.reserve(num_of_vehicle);
	VehicleInfo vehicle_info[num_of_vehicle];

	for(int i=0 ; i < num_of_vehicle ; i++){
		vehicle_info[i].system_id = i+1;
		vehicle_info[i].vehicle_name = swarm_name + std::to_string(i+1);
		camila.push_back(Vehicle(vehicle_info[i]));
	}

	nh = ros::NodeHandle();
	// nh.setParam("setVel", true);
	multi_setpoint_local_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server = nh.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server = nh.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);

}
std::string SwarmVehicle::getSwarmInfo(){
	return swarm_name;
}

void SwarmVehicle::addVehicle(VehicleInfo _vehicle_info){
	camila.push_back(Vehicle(_vehicle_info));
	num_of_vehicle++;
}

void SwarmVehicle::deleteVehicle(VehicleInfo _vehicle_info){
	for(iter = camila.begin(); iter != camila.end() ; iter++){
		VehicleInfo temp = iter->getInfo();
		if((temp.system_id == _vehicle_info.system_id) && (temp.vehicle_name == _vehicle_info.vehicle_name)){
			camila.erase(iter);
			ROS_INFO_STREAM("Delete vehicle : sys_id : " << temp.system_id<< ", name : "<<temp.vehicle_name);
			num_of_vehicle--;
			break;
		}
	}
}

void SwarmVehicle::showVehicleList(){
	for(iter = camila.begin(); iter != camila.end(); iter++){
		VehicleInfo temp = iter->getInfo();
		ROS_INFO_STREAM(temp.system_id <<" "<< temp.vehicle_name);
	}
}

bool SwarmVehicle::multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request& req,
	swarm_ctrl_pkg::srvMultiSetpointLocal::Response& res)
{
	geometry_msgs::PoseStamped msg, msg_f;
	double angle = 0;
	msg.pose.position.x = req.x;	
	msg.pose.position.y = req.y;	
	msg.pose.position.z = req.z;	
	
	sensor_msgs::NavSatFix leader = camila.front().getGlobalPosition();
	if(num_of_vehicle != 1)
		angle = 2.0 * M_PI / (num_of_vehicle - 1);

	for(iter = camila.begin();iter != camila.end(); iter++){
		if(iter != camila.begin()){
			sensor_msgs::NavSatFix follower = iter->getGlobalPosition();
			geometry_msgs::Vector3 offset;
			offset = convertGeoToENU(leader.latitude, leader.longitude, leader.altitude,
				follower.latitude, follower.longitude, follower.altitude);
			msg_f.pose.position.x = msg.pose.position.x + offset.x + SPACING * cos(angle);
			msg_f.pose.position.y = msg.pose.position.y + offset.y + SPACING * sin(angle);
			msg_f.pose.position.z = msg.pose.position.z + offset.z;

			iter->setLocalTarget(msg_f);
			angle += angle;
		}
		else
			iter->setLocalTarget(msg);
	}

	res.success = true;

	return res.success;
}

bool SwarmVehicle::multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request& req,
	swarm_ctrl_pkg::srvMultiSetpointGlobal::Response& res)
{
	return true;
}
bool SwarmVehicle::gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request& req,
	swarm_ctrl_pkg::srvGoToVehicle::Response& res)
{
	geometry_msgs::PoseStamped msg;
	sensor_msgs::NavSatFix leader = camila.front().getGlobalPosition();
	iter = camila.begin() + req.num_drone - 1;
	
	if(num_of_vehicle != 1){
		sensor_msgs::NavSatFix follower = iter->getGlobalPosition();
		geometry_msgs::Vector3 offset;
		offset = convertGeoToENU(leader.latitude, leader.longitude, leader.altitude,
			follower.latitude, follower.longitude, follower.altitude);
		msg.pose.position.x = req.x + offset.x;
		msg.pose.position.y = req.y + offset.y;
		msg.pose.position.z = req.z + offset.z;		
	}
	else{
		msg.pose.position.x = req.x;
		msg.pose.position.y = req.y;
		msg.pose.position.z = req.z;		
	}

	iter->setLocalTarget(msg);

	return true;
}

geometry_msgs::Vector3 SwarmVehicle::convertGeoToENU(double coord_lat, double coord_long, 
	double coord_alt, double home_lat, double home_long, double home_alt) 
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

    double lat_rad = coord_lat * M_DEG_TO_RAD;
    double lon_rad = coord_long* M_DEG_TO_RAD;

    double ref_lon_rad = home_long * M_DEG_TO_RAD;
    double ref_lat_rad = home_lat * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    geometry_msgs::Vector3 offset;

    offset.x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
    offset.y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    offset.z = coord_alt - home_alt;

    return offset;
}

geometry_msgs::Vector3 SwarmVehicle::convertENUToGeo(double x, double y, double z,
	double home_lat, double home_long, double home_altitude) 
{
    static const float epsilon = std::numeric_limits<double>::epsilon();

	double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
    double sin_c = sin(c);
    double cos_c = cos(c);

    double ref_lon_rad = home_long * M_DEG_TO_RAD;
    double ref_lat_rad = home_lat * M_DEG_TO_RAD;

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double lat_rad;
    double lon_rad;

    if (fabs(c) > epsilon) {
        lat_rad = asin(cos_c * ref_sin_lat + (y_rad * sin_c * ref_cos_lat) / c);
        lon_rad = (ref_lon_rad + atan2(x_rad * sin_c, c * ref_cos_lat * cos_c - y_rad * ref_sin_lat * sin_c));

    } else {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    geometry_msgs::Vector3 geoPoint;

    double latitude = lat_rad * M_RAD_TO_DEG;
    double longitude = lon_rad * M_RAD_TO_DEG;
    double altitude = z + home_altitude;

    geoPoint.x  = latitude;
    geoPoint.y = longitude;
    geoPoint.z = altitude;

    return geoPoint;
}

bool SwarmVehicle::isPublish(){
	multi_setpoint_publish_flag = true;
	for(iter = camila.begin() ; iter != camila.end() ; iter++){
		if(iter->isPublish() != true){
			multi_setpoint_publish_flag = false;
			break;
		}
	}
	return multi_setpoint_publish_flag;
}

void SwarmVehicle::run(){
	if(isPublish()){
		nh.getParam("use_vel", control_method);
		for(iter = camila.begin() ; iter != camila.end() ; iter++){
			if(control_method)
				iter->gotoVel();
			else
				iter->gotoLocal();
		}
	}
}