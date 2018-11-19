#include <ros/ros.h>
#include <vehicle.h>

const sensor_msgs::NavSatFix &operator+(const sensor_msgs::NavSatFix &a, const sensor_msgs::NavSatFix &b)
{
	sensor_msgs::NavSatFix result;
	result.latitude = a.latitude + b.latitude;
	result.longitude = a.longitude + b.longitude;
	result.altitude = a.altitude + b.altitude;

	return result;
}

sensor_msgs::NavSatFix &operator+=(sensor_msgs::NavSatFix &a, const sensor_msgs::NavSatFix &b)
{
	a.latitude += b.latitude;
	a.longitude += b.longitude;
	a.altitude += b.altitude;

	return a;
}

Vehicle::Vehicle() : vehicle_info({1, "mavros"}),
					 nh(ros::NodeHandle(vehicle_info.vehicle_name)),
					 nh_mul(ros::NodeHandle("multi")),
					 nh_global(ros::NodeHandle("~")),
					 setpoint_publish_flag(false)
{
	vehicleInit();
}

Vehicle::Vehicle(VehicleInfo _vehicle_info) : vehicle_info(_vehicle_info),
											  nh(ros::NodeHandle(vehicle_info.vehicle_name)),
											  nh_mul(ros::NodeHandle("multi")),
											  nh_global(ros::NodeHandle("~")),
											  setpoint_publish_flag(false)
{
	vehicleInit();
}

Vehicle::Vehicle(const Vehicle &rhs) : vehicle_info(rhs.vehicle_info),
									   nh(ros::NodeHandle(vehicle_info.vehicle_name)),
									   nh_mul(ros::NodeHandle("multi")),
									   nh_global(ros::NodeHandle("~")),
									   setpoint_publish_flag(rhs.setpoint_publish_flag)
{
	vehicleInit();
	*this = rhs;
}

const Vehicle &Vehicle::operator=(const Vehicle &rhs)
{
	if (this == &rhs)
	{
		return *this;
	}
	vehicle_info = rhs.vehicle_info;
	setpoint_publish_flag = rhs.setpoint_publish_flag;
	nh = ros::NodeHandle(vehicle_info.vehicle_name);
	nh_mul = ros::NodeHandle("multi");
	nh_global = ros::NodeHandle("~");
	vehicleInit();
	return *this;
}

void Vehicle::vehicleInit()
{
	//camila1's_target frame을 publish 하게
	// setpoint_global_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("setpoint_position/global", 10);
	// setpoint_local_pub = nh.advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 10);
	setpoint_vel_pub = nh.advertise<geometry_msgs::Twist>("setpoint_velocity/cmd_vel_unstamped", 10);

	state_sub = nh.subscribe("state", 10, &Vehicle::stateCB, this);
	battery_sub = nh.subscribe("battery", 10, &Vehicle::batteryCB, this);
	//lookup 으로 확인하기
	// local_pos_sub = nh.subscribe("local_position/pose", 10, &Vehicle::localPositionCB, this);
	// global_pos_sub = nh.subscribe("global_position/global", 10, &Vehicle::globalPositionCB, this);

	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("set_mode");
	set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("cmd/set_home");
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("cmd/takeoff");
	land_client = nh.serviceClient<mavros_msgs::CommandTOL>("cmd/land");

	multi_arming_sub = nh_mul.subscribe("arming", 10, &Vehicle::multiArming, this);
	multi_set_mode_sub = nh_mul.subscribe("set_mode", 10, &Vehicle::multiSetMode, this);
	multi_takeoff_sub = nh_mul.subscribe("takeoff", 10, &Vehicle::multiTakeoff, this);
	multi_land_sub = nh_mul.subscribe("land", 10, &Vehicle::multiLand, this);

	ROS_INFO_STREAM(vehicle_info.vehicle_name << " instance generated");
}

void Vehicle::setVehicleInfo(VehicleInfo new_vehicle_info)
{
	vehicle_info.system_id = new_vehicle_info.system_id;
	vehicle_info.vehicle_name = new_vehicle_info.vehicle_name;

	vehicleInit();
}

VehicleInfo Vehicle::getInfo()
{
	return vehicle_info;
}

void Vehicle::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
	if (cur_state.connected != msg->connected)
	{
		if (msg->connected == true)
			ROS_INFO_STREAM(vehicle_info.vehicle_name << " is connected");
		else
			ROS_WARN_STREAM(vehicle_info.vehicle_name << " is NOT connected");
	}
	if (cur_state.armed != msg->armed)
	{
		if (msg->armed == true)
			ROS_INFO_STREAM(vehicle_info.vehicle_name << " is armed");
		else
			ROS_INFO_STREAM(vehicle_info.vehicle_name << " is disarmed");
	}
	if (cur_state.mode != msg->mode)
	{
		ROS_INFO_STREAM(vehicle_info.vehicle_name << " is in " << msg->mode << " mode");
	}
	cur_state = *msg;
}

mavros_msgs::State Vehicle::getState()
{
	return cur_state;
}

void Vehicle::batteryCB(const sensor_msgs::BatteryState::ConstPtr &msg)
{
	cur_battery = *msg;
}

sensor_msgs::BatteryState Vehicle::getBattery()
{
	return cur_battery;
}

bool Vehicle::arming(bool _arm_state)
{
	mavros_msgs::CommandBool msg;
	msg.request.value = _arm_state;

	if (arming_client.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info.vehicle_name << " failed to call arming service. " << msg.response.result);
	return msg.response.success;
}

bool Vehicle::setMode(std::string _mode)
{
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = _mode;
	if (((_mode == "offboard") || (_mode == "OFFBOARD")) && (!setpoint_publish_flag))
	{
		ROS_WARN("Please publish setpoint first");
		return false;
	}
	else
	{
		if (set_mode_client.call(mode) && mode.response.mode_sent)
			;
		else
			ROS_ERROR_STREAM("Failed to call set_mode service. " << mode.response.mode_sent);
		return mode.response.mode_sent;
	}
}

bool Vehicle::takeoff(double _takeoff_alt)
{
	mavros_msgs::CommandTOL msg;
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_global.latitude;
	msg.request.longitude = cur_global.longitude;
	msg.request.altitude = home_global.altitude + _takeoff_alt;

	if (takeoff_client.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info.vehicle_name << " failed to call takeoff service. " << msg.response.result);
	return msg.response.success;
}

bool Vehicle::land()
{
	mavros_msgs::CommandTOL msg;
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_global.latitude;
	msg.request.longitude = cur_global.longitude;
	msg.request.altitude = 0;

	if (land_client.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info.vehicle_name << " failed to call land service. " << msg.response.result);
	return msg.response.success;
}

/*multi란? -> 커멘드로 날라온 토픽을 서비스로 날리는 놈, 객체마다 있기에 동시에 서비스를 날리는 효과*/

void Vehicle::multiArming(const std_msgs::Bool::ConstPtr &msg)
{
	mavros_msgs::CommandBool arm;
	arm.request.value = msg->data;

	if (arming_client.call(arm) && arm.response.success)
	{
	}
	else
	{
		ROS_ERROR_STREAM(vehicle_info.vehicle_name << " failed to call arming service. " << arm.response.result);
	}
}

void Vehicle::multiSetMode(const std_msgs::String::ConstPtr &msg)
{
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = msg->data;
	if (((mode.request.custom_mode == "offboard") || (mode.request.custom_mode == "OFFBOARD")) && (!setpoint_publish_flag))
	{
		ROS_WARN("Please publish setpoint first");
	}
	else
	{
		if (set_mode_client.call(mode) && mode.response.mode_sent)
		{
		}
		else
		{
			ROS_ERROR_STREAM(vehicle_info.vehicle_name << " failed to call set_mode service. " << mode.response.mode_sent);
		}
	}
}

void Vehicle::multiTakeoff(const std_msgs::Empty::ConstPtr &trigger)
{
	int _takeoff_alt;
	nh_global.getParam("takeoff_alt", _takeoff_alt);
	takeoff(_takeoff_alt);
}

void Vehicle::multiLand(const std_msgs::Empty::ConstPtr &trigger)
{
	land();
}

void Vehicle::globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	cur_global = *msg;

	/* geometry_msgs::Vector3 pose = convertGeoToENU(cur_global.latitude, cur_global.longitude,
		cur_global.altitude, home_global.latitude, home_global.longitude, home_global.altitude);
	cur_local.pose.position.x = pose.x;
	cur_local.pose.position.y = pose.y;
	cur_local.pose.position.z = pose.z; */
	/* ROS_INFO_STREAM(cur_local.pose.position.x << " " << cur_local.pose.position.y 
		<<" "<< cur_local.pose.position.z); */
}

sensor_msgs::NavSatFix Vehicle::getGlobalPosition()
{
	return cur_global;
}

bool Vehicle::isPublish()
{
	return setpoint_publish_flag;
}

//default value : default name = camila, _num_of_vehicle = 1;
SwarmVehicle::SwarmVehicle(std::string _swarm_name, int _num_of_vehicle) : swarm_name(_swarm_name),
																		   num_of_vehicle(_num_of_vehicle),
																		   nh(ros::NodeHandle()),
																		   nh_global(ros::NodeHandle("~")),
																		   multi_setpoint_publish_flag(false)
{
	VehicleInfo vehicle_info[num_of_vehicle];
	camila.reserve(num_of_vehicle);

	for (int i = 0; i < num_of_vehicle; i++)
	{
		vehicle_info[i].system_id = i + 1;
		vehicle_info[i].vehicle_name = swarm_name + std::to_string(i + 1) + "/mavros";
		camila.push_back(Vehicle(vehicle_info[i]));
	}
	swarm_target_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
}

SwarmVehicle::SwarmVehicle(const SwarmVehicle &rhs) : swarm_name(rhs.swarm_name),
													  num_of_vehicle(rhs.num_of_vehicle),
													  nh(ros::NodeHandle()),
													  nh_global(ros::NodeHandle("~")),
													  multi_setpoint_publish_flag(rhs.multi_setpoint_publish_flag)
{
	VehicleInfo vehicle_info[num_of_vehicle];
	camila.reserve(num_of_vehicle);

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila.begin(); it != rhs.camila.end(); it++)
	{
		camila.push_back(*it);
	}
	swarm_target_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
	*this = rhs;
}

const SwarmVehicle &SwarmVehicle::operator=(const SwarmVehicle &rhs)
{
	if (this == &rhs)
	{
		return *this;
	}

	swarm_name = rhs.swarm_name;
	num_of_vehicle = rhs.num_of_vehicle;

	std::vector<Vehicle>().swap(camila);
	camila.reserve(num_of_vehicle);
	VehicleInfo vehicle_info[num_of_vehicle];

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila.begin(); it != rhs.camila.end(); it++)
	{
		camila.push_back(*it);
	}

	nh = ros::NodeHandle();
	nh_global = ros::NodeHandle("~");

	swarm_target_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
	return *this;
}

SwarmVehicle::~SwarmVehicle()
{
	std::vector<Vehicle>().swap(camila);
}

void SwarmVehicle::setSwarmInfo(std::string _swarm_name, int _num_of_vehicle)
{
	swarm_name = _swarm_name;
	num_of_vehicle = _num_of_vehicle;

	std::vector<Vehicle>().swap(camila);
	camila.reserve(num_of_vehicle);
	VehicleInfo vehicle_info[num_of_vehicle];

	for (int i = 0; i < num_of_vehicle; i++)
	{
		vehicle_info[i].system_id = i + 1;
		vehicle_info[i].vehicle_name = swarm_name + std::to_string(i + 1) + "/mavros";
		camila.push_back(Vehicle(vehicle_info[i]));
	}

	nh = ros::NodeHandle();
	nh_global = ros::NodeHandle("~");

	swarm_target_server = nh.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
}

std::string SwarmVehicle::getSwarmInfo()
{
	return swarm_name;
}

void SwarmVehicle::addVehicle(VehicleInfo _vehicle_info)
{
	camila.push_back(Vehicle(_vehicle_info));
	num_of_vehicle++;
}

void SwarmVehicle::deleteVehicle(VehicleInfo _vehicle_info)
{
	for (iter = camila.begin(); iter != camila.end(); iter++)
	{
		VehicleInfo temp = iter->getInfo();
		if ((temp.system_id == _vehicle_info.system_id) && (temp.vehicle_name == _vehicle_info.vehicle_name))
		{
			camila.erase(iter);
			ROS_INFO_STREAM("Delete vehicle : sys_id : " << temp.system_id << ", name : " << temp.vehicle_name);
			num_of_vehicle--;
			break;
		}
	}
}

void SwarmVehicle::showVehicleList()
{
	for (iter = camila.begin(); iter != camila.end(); iter++)
	{
		VehicleInfo temp = iter->getInfo();
		ROS_INFO_STREAM(temp.system_id << " " << temp.vehicle_name);
	}
}
void SwarmVehicle::setSwarmMap()
{
	swarm_map.latitude = 0;
	swarm_map.longitude = 0;
	swarm_map.altitude = 0;

	for (iter = camila.begin(); iter != camila.end(); iter++)
	{
		sensor_msgs::NavSatFix gps_pos = iter->getGlobalPosition();
		swarm_map += gps_pos;
	}
	swarm_map.latitude /= camila.size();
	swarm_map.longitude /= camila.size();
	swarm_map.altitude /= camila.size();
}

void SwarmVehicle::offsetPublisher()
{
	int i = 0;
	for (iter = camila.begin(); iter != camila.end(); iter++)
	{
		//geometry_msgs::TransformStamped tf_stamped;
		sensor_msgs::NavSatFix gps_pos = iter->getGlobalPosition();
		geometry_msgs::Vector3 _offset = convertGeoToENU(gps_pos, swarm_map);
		offset.push_back(_offset);
		ROS_INFO_STREAM("offset[" << i << "] = " << offset[i]);

		// tf_stamped.header.stamp = ros::Time::now();
		// tf_stamped.header.frame_id = "swarm_map";
		// tf_stamped.child_frame_id = "camila" + std::to_string(i + 1) + "_map";
		// tf_stamped.transform.translation.x = _offset.x;
		// tf_stamped.transform.translation.y = _offset.y;
		// tf_stamped.transform.translation.z = _offset.z;
		// tf2::Quaternion quat;
		// quat.setRPY(0, 0, 0); //처음 비틀림 계속반영? ENU라서 상관 없으려나
		// tf_stamped.transform.rotation.x = quat.x();
		// tf_stamped.transform.rotation.y = quat.y();
		// tf_stamped.transform.rotation.z = quat.z();
		// tf_stamped.transform.rotation.w = quat.w();
		// static_offset_bc.sendTransform(tf_stamped);

		std::string vehicle_map = "camila" + std::to_string(i + 1) + "_map";
		transformSender(_offset.x, _offset.y, _offset.z, 0, 0, 0, ros::Time::now(), "swarm_map", vehicle_map);
		i++;
	}
}

void SwarmVehicle::transformSender(double x, double y, double z, double roll, double pitch, double yaw, ros::Time call_time, const std::string &frame_id, const std::string &child_frame_id)
{
	tf2_ros::TransformBroadcaster tf_br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = call_time;
	transformStamped.header.frame_id = frame_id;
	transformStamped.child_frame_id = child_frame_id;
	transformStamped.transform.translation.x = x;
	transformStamped.transform.translation.y = y;
	transformStamped.transform.translation.z = z;
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
	tf_br.sendTransform(transformStamped);
}

bool SwarmVehicle::setSwarmTarget(swarm_ctrl_pkg::srvSetSwarmTarget::Request &req,
								  swarm_ctrl_pkg::srvSetSwarmTarget::Response &res)
{
	swarm_target_TF.header.stamp = ros::Time::now();
	swarm_target_TF.header.frame_id = "swarm_map";
	swarm_target_TF.child_frame_id = "swarm_target";
	swarm_target_TF.transform.translation.x = req.x;
	swarm_target_TF.transform.translation.y = req.y;
	swarm_target_TF.transform.translation.z = req.z;
	tf2::Quaternion q;
	q.setRPY(0, 0, req.yaw);
	swarm_target_TF.transform.rotation.x = q.x();
	swarm_target_TF.transform.rotation.y = q.y();
	swarm_target_TF.transform.rotation.z = q.z();
	swarm_target_TF.transform.rotation.w = q.w();
	swarm_target_bc.sendTransform(swarm_target_TF);

	formation = req.formation;

	res.success = true;

	return res.success;
}

void SwarmVehicle::formationGenerater()
{
	geometry_msgs::TransformStamped vehicle_target_TF;

	//if(formation == POINT)
	vehicle_target_TF = swarm_target_TF;
	int i = 0;
	for (iter = camila.begin(); iter != camila.end(); iter++)
	{
		std::string vehicle_target = "camila" + std::to_string(i + 1) + "_target";
		transformSender(vehicle_target_TF.transform.translation.x, vehicle_target_TF.transform.translation.y, vehicle_target_TF.transform.translation.z, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		i++;
	}
}
geometry_msgs::Vector3 SwarmVehicle::convertGeoToENU(sensor_msgs::NavSatFix _coord,
													 sensor_msgs::NavSatFix _home)
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

	double lat_rad = _coord.latitude * M_DEG_TO_RAD;
	double lon_rad = _coord.longitude * M_DEG_TO_RAD;

	double ref_lon_rad = _home.longitude * M_DEG_TO_RAD;
	double ref_lat_rad = _home.latitude * M_DEG_TO_RAD;

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
	offset.z = _coord.altitude - _home.altitude;

	return offset;
}

geographic_msgs::GeoPoint SwarmVehicle::convertENUToGeo(geometry_msgs::PoseStamped _local,
														sensor_msgs::NavSatFix _home_global)
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

	double x_rad = _local.pose.position.x / CONSTANTS_RADIUS_OF_EARTH;
	double y_rad = _local.pose.position.y / CONSTANTS_RADIUS_OF_EARTH;
	double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
	double sin_c = sin(c);
	double cos_c = cos(c);

	double ref_lon_rad = _home_global.longitude * M_DEG_TO_RAD;
	double ref_lat_rad = _home_global.latitude * M_DEG_TO_RAD;

	double ref_sin_lat = sin(ref_lat_rad);
	double ref_cos_lat = cos(ref_lat_rad);

	double lat_rad;
	double lon_rad;

	if (fabs(c) > epsilon)
	{
		lat_rad = asin(cos_c * ref_sin_lat + (y_rad * sin_c * ref_cos_lat) / c);
		lon_rad = (ref_lon_rad + atan2(x_rad * sin_c, c * ref_cos_lat * cos_c - y_rad * ref_sin_lat * sin_c));
	}
	else
	{
		lat_rad = ref_lat_rad;
		lon_rad = ref_lon_rad;
	}

	geographic_msgs::GeoPoint geo_point;

	geo_point.latitude = lat_rad * M_RAD_TO_DEG;
	geo_point.longitude = lon_rad * M_RAD_TO_DEG;
	geo_point.altitude = _local.pose.position.z + _home_global.altitude;

	return geo_point;
}

bool SwarmVehicle::isPublish()
{
	multi_setpoint_publish_flag = true;
	for (iter = camila.begin(); iter != camila.end(); iter++)
	{
		if (iter->isPublish() != true)
		{
			multi_setpoint_publish_flag = false;
			break;
		}
	}
	return multi_setpoint_publish_flag;
}