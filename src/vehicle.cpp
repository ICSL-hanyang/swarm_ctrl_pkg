#include <ros/ros.h>
#include <vehicle.h>

double Vehicle::kp_;

Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global) 
			: vehicle_info_({1, "mavros"}),
					 nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
					 nh_mul_(nh_mul),
					 nh_global_(nh_global),
					 setpoint_publish_flag_(false)
{
	vehicleInit();
}

Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global, const VehicleInfo &vehicle_info) 
: vehicle_info_(vehicle_info),
											  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
											  nh_mul_(nh_mul),
											  nh_global_(nh_global),
											  setpoint_publish_flag_(false)
{
	vehicleInit();
}

Vehicle::Vehicle(const Vehicle &rhs) 
: vehicle_info_(rhs.vehicle_info_),
									   nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
									   nh_mul_(rhs.nh_mul_),
									   nh_global_(rhs.nh_global_),
									   setpoint_publish_flag_(rhs.setpoint_publish_flag_)
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
	vehicle_info_ = rhs.vehicle_info_;
	setpoint_publish_flag_ = rhs.setpoint_publish_flag_;
	nh_ = ros::NodeHandle(vehicle_info_.vehicle_name_);
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;
	vehicleInit();
	return *this;
}

void Vehicle::vehicleInit()
{
	setpoint_global_pub_ = nh_.advertise<mavros_msgs::GlobalPositionTarget>("setpoint_position/global", 10);
	setpoint_local_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("setpoint_position/local", 10);
	setpoint_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("setpoint_velocity/cmd_vel_unstamped", 10);

	state_sub_ = nh_.subscribe("state", 10, &Vehicle::stateCB, this);
	battery_sub_ = nh_.subscribe("battery", 10, &Vehicle::batteryCB, this);
	home_sub_ = nh_.subscribe("home_position/home", 10, &Vehicle::homeCB, this);
	local_pos_sub_ = nh_.subscribe("local_position/pose", 10, &Vehicle::localPositionCB, this);
	global_pos_sub_ = nh_.subscribe("global_position/global", 10, &Vehicle::globalPositionCB, this);

	arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("set_mode");
	set_home_client_ = nh_.serviceClient<mavros_msgs::CommandHome>("cmd/set_home");
	takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("cmd/takeoff");
	land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("cmd/land");

	multi_arming_sub_ = nh_mul_.subscribe("arming", 10, &Vehicle::multiArming, this);
	multi_set_mode_sub_ = nh_mul_.subscribe("set_mode", 10, &Vehicle::multiSetMode, this);
	multi_set_home_sub_ = nh_mul_.subscribe("set_home", 10, &Vehicle::multiSetHome, this);
	multi_takeoff_sub_ = nh_mul_.subscribe("takeoff", 10, &Vehicle::multiTakeoff, this);
	multi_land_sub_ = nh_mul_.subscribe("land", 10, &Vehicle::multiLand, this);

	ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " instance generated");
}

void Vehicle::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
	if (cur_state_.connected != msg->connected)
	{
		if (msg->connected == true)
			ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is connected");
		else
			ROS_WARN_STREAM(vehicle_info_.vehicle_name_ << " is not connected");
	}
	if (cur_state_.armed != msg->armed)
	{
		if (msg->armed == true)
			ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is armed");
		else
			ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is disarmed");
	}
	if (cur_state_.mode != msg->mode)
	{
		ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " is " << msg->mode << " mode");
	}
	cur_state_ = *msg;
}

void Vehicle::batteryCB(const sensor_msgs::BatteryState::ConstPtr &msg)
{
	cur_battery_ = *msg;
}

void Vehicle::homeCB(const mavros_msgs::HomePosition::ConstPtr &msg)
{
	home_global_.latitude = msg->geo.latitude;
	home_global_.longitude = msg->geo.longitude;
	home_global_.altitude = msg->geo.altitude;
	home_local_.pose.position.x = msg->position.x;
	home_local_.pose.position.y = msg->position.y;
	home_local_.pose.position.z = msg->position.z;
}

void Vehicle::globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	cur_global_ = *msg;

	/* geometry_msgs::Vector3 pose = convertGeoToENU(cur_global_.latitude, cur_global_.longitude,
		cur_global_.altitude, home_global_.latitude, home_global_.longitude, home_global_.altitude);
	cur_local_.pose.position.x = pose.x;
	cur_local_.pose.position.y = pose.y;
	cur_local_.pose.position.z = pose.z; */
	/* ROS_INFO_STREAM(cur_local_.pose.position.x << " " << cur_local_.pose.position.y 
		<<" "<< cur_local_.pose.position.z); */
}

void Vehicle::localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	cur_local_ = *msg;
}

void Vehicle::multiArming(const std_msgs::Bool::ConstPtr &msg)
{
	mavros_msgs::CommandBool arm;
	arm.request.value = msg->data;

	if (arming_client_.call(arm) && arm.response.success)
	{
	}
	else
	{
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call arming service. " << arm.response.result);
	}
}

void Vehicle::multiSetMode(const std_msgs::String::ConstPtr &msg)
{
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = msg->data;
	if (((mode.request.custom_mode == "offboard") || (mode.request.custom_mode == "OFFBOARD")) && (!setpoint_publish_flag_))
	{
		ROS_WARN("Please publish setpoint first");
	}
	else
	{
		if (set_mode_client_.call(mode) && mode.response.mode_sent)
		{
		}
		else
		{
			ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call set_mode service. " << mode.response.mode_sent);
		}
	}
}

void Vehicle::multiSetHome(const std_msgs::Empty::ConstPtr &trigger)
{
	if (setHomeGlobal())
	{
		setHomeLocal();
	}
}

void Vehicle::multiTakeoff(const std_msgs::Empty::ConstPtr &trigger)
{
	int takeoff_alt;
	nh_global_.getParam("takeoff_alt", takeoff_alt);
	takeoff(takeoff_alt);
}

void Vehicle::multiLand(const std_msgs::Empty::ConstPtr &trigger)
{
	land();
}

void Vehicle::setVehicleInfo(const VehicleInfo &new_vehicle_info)
{
	vehicle_info_.vehicle_id_ = new_vehicle_info.vehicle_id_;
	vehicle_info_.vehicle_name_ = new_vehicle_info.vehicle_name_;

	vehicleInit();
}

VehicleInfo Vehicle::getInfo() const
{
	return vehicle_info_;
}

mavros_msgs::State Vehicle::getState() const
{
	return cur_state_;
}

sensor_msgs::BatteryState Vehicle::getBattery() const
{
	return cur_battery_;
}

bool Vehicle::arming(const bool &arm_state)
{
	mavros_msgs::CommandBool msg;
	msg.request.value = arm_state;

	if (arming_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call arming service. " << msg.response.result);
	return msg.response.success;
}

bool Vehicle::setMode(const std::string &current_mode)
{
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = current_mode;
	if (((current_mode == "offboard") || (current_mode == "OFFBOARD")) && (!setpoint_publish_flag_))
	{
		ROS_WARN("Please publish setpoint first");
		return false;
	}
	else
	{
		if (set_mode_client_.call(mode) && mode.response.mode_sent)
			;
		else
			ROS_ERROR_STREAM("Failed to call set_mode service. " << mode.response.mode_sent);
		return mode.response.mode_sent;
	}
}

bool Vehicle::takeoff(const double &takeoff_alt)
{
	mavros_msgs::CommandTOL msg;
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_global_.latitude;
	msg.request.longitude = cur_global_.longitude;
	msg.request.altitude = home_global_.altitude + takeoff_alt;

	if (takeoff_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call takeoff service. " << msg.response.result);
	return msg.response.success;
}

bool Vehicle::land()
{
	mavros_msgs::CommandTOL msg;
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_global_.latitude;
	msg.request.longitude = cur_global_.longitude;
	msg.request.altitude = 0;

	if (land_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call land service. " << msg.response.result);
	return msg.response.success;
}

void Vehicle::gotoGlobal(const sensor_msgs::NavSatFix &tar_global)
{
	setpoint_publish_flag_ = true;

	if ((tar_global_.latitude != tar_global.latitude) ||
		(tar_global_.longitude != tar_global.longitude) ||
		(tar_global_.altitude != tar_global.altitude))
		ROS_INFO("%s set target_global_pos(long : %lf, lati : %lf, alti : %lf)", vehicle_info_.vehicle_name_.c_str(),
				 tar_global_.longitude, tar_global_.latitude, tar_global_.altitude);

	tar_global_ = tar_global;
	tar_global_.header.seq += 1;
	tar_global_.header.stamp = ros::Time::now();
	tar_global_.header.frame_id = vehicle_info_.vehicle_name_;

	mavros_msgs::GlobalPositionTarget msg;
	msg.latitude = tar_global_.latitude;
	msg.longitude = tar_global_.longitude;
	msg.altitude = tar_global_.altitude;
	msg.header.seq += 1;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = vehicle_info_.vehicle_name_;
	msg.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
	msg.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
					mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
					mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
					mavros_msgs::GlobalPositionTarget::IGNORE_VX |
					mavros_msgs::GlobalPositionTarget::IGNORE_VY |
					mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
					mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
					mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;

	setpoint_global_pub_.publish(msg);
}

void Vehicle::setLocalTarget(const geometry_msgs::PoseStamped &tar_local)
{
	setpoint_publish_flag_ = true;

	if ((tar_local_.pose.position.x != tar_local.pose.position.x) ||
		(tar_local_.pose.position.y != tar_local.pose.position.y) ||
		(tar_local_.pose.position.z != tar_local.pose.position.z))
	{
		tar_local_ = tar_local;
		ROS_INFO("%s set target_local_pos(x : %lf, y : %lf, z : %lf)", vehicle_info_.vehicle_name_.c_str(),
				 tar_local_.pose.position.x, tar_local_.pose.position.y, tar_local_.pose.position.z);
	}
}

void Vehicle::gotoLocal()
{
	tar_local_.header.seq += 1;
	tar_local_.header.stamp = ros::Time::now();
	tar_local_.header.frame_id = vehicle_info_.vehicle_name_;

	setpoint_local_pub_.publish(tar_local_);
}

void Vehicle::gotoVel()
{
	nh_global_.getParam("pid/kp", kp_);
	geometry_msgs::Twist vel;

	vel.linear.x = (tar_local_.pose.position.x - cur_local_.pose.position.x) * kp_;
	vel.linear.y = (tar_local_.pose.position.y - cur_local_.pose.position.y) * kp_;
	vel.linear.z = (tar_local_.pose.position.z - cur_local_.pose.position.z) * kp_;

	setpoint_vel_pub_.publish(vel);
}

bool Vehicle::setHomeGlobal()
{
	mavros_msgs::CommandHome msg;

	msg.request.current_gps = false;
	msg.request.latitude = cur_global_.latitude;
	msg.request.longitude = cur_global_.longitude;
	msg.request.altitude = cur_global_.altitude;

	if (set_home_client_.call(msg) && msg.response.success)
	{
		home_global_ = cur_global_;

		ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " Global home position is set. " << msg.response.result);
		ROS_INFO("%s new global home is (long : %lf, lang : %lf, alti : %lf)", vehicle_info_.vehicle_name_.c_str(),
				 home_global_.longitude, home_global_.latitude, home_global_.altitude);
	}
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " Failed to set global home position. " << msg.response.result);

	return msg.response.success;
}

sensor_msgs::NavSatFix Vehicle::getHomeGlobal() const
{
	return home_global_;
}

sensor_msgs::NavSatFix Vehicle::getGlobalPosition() const
{
	return cur_global_;
}

sensor_msgs::NavSatFix Vehicle::getTargetGlobal() const
{
	return tar_global_;
}

void Vehicle::setHomeLocal()
{
	home_local_ = cur_local_;

	ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " Local home position is set.");
	ROS_INFO("%s new local home is (x : %lf, y : %lf, z : %lf)", vehicle_info_.vehicle_name_.c_str(),
			 home_local_.pose.position.x, home_local_.pose.position.y, home_local_.pose.position.z);
}

geometry_msgs::PoseStamped Vehicle::getHomeLocal() const
{
	return home_local_;
}

geometry_msgs::PoseStamped Vehicle::getLocalPosition() const
{
	return cur_local_;
}

geometry_msgs::PoseStamped Vehicle::getTargetLocal() const
{
	return tar_local_;
}

geometry_msgs::Vector3 Vehicle::convertGeoToENU(const double &coord_lat, const double &coord_long, 
const double &coord_alt, const double &home_lat, const double &home_long, const double &home_alt)
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

	double lat_rad = coord_lat * M_DEG_TO_RAD;
	double lon_rad = coord_long * M_DEG_TO_RAD;

	double ref_lon_rad = home_long * M_DEG_TO_RAD;
	double ref_lat_rad = home_lat * M_DEG_TO_RAD;

	double sin_lat = sin(lat_rad);
	double cos_lat = cos(lat_rad);
	double cos_d_lon = cos(lon_rad - ref_lon_rad);

	double ref_sin_lat = sin(ref_lat_rad);
	double ref_cos_lat = cos(ref_lat_rad);

	double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
	double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

	geometry_msgs::Vector3 offset_;

	offset_.x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
	offset_.y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
	offset_.z = coord_alt - home_alt;

	return offset_;
}

bool Vehicle::isPublish() const
{
	return setpoint_publish_flag_;
}

//default value : default name = camila, num_of_vehicle = 1;
SwarmVehicle::SwarmVehicle(ros::NodeHandle &nh_global, const std::string &swarm_name, const int &num_of_vehicle)
 : swarm_name_(swarm_name),
																		   num_of_vehicle_(num_of_vehicle),
																		   nh_(ros::NodeHandle()),
																		   nh_mul_("multi"),
																		   nh_global_(nh_global),
																		   multi_setpoint_publish_flag_(false)
{
	VehicleInfo vehicle_info_[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1) + "/mavros";
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}
	multi_setpoint_local_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server_ = nh_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);
}

SwarmVehicle::SwarmVehicle(const SwarmVehicle &rhs) 
: swarm_name_(rhs.swarm_name_),
													  num_of_vehicle_(rhs.num_of_vehicle_),
													  nh_(ros::NodeHandle()),
													  nh_global_(rhs.nh_global_),
													  multi_setpoint_publish_flag_(rhs.multi_setpoint_publish_flag_)
{
	VehicleInfo vehicle_info_[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
	{
		camila_.push_back(*it);
	}
	multi_setpoint_local_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server_ = nh_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);

	*this = rhs;
}

const SwarmVehicle &SwarmVehicle::operator=(const SwarmVehicle &rhs)
{
	if (this == &rhs)
	{
		return *this;
	}

	swarm_name_ = rhs.swarm_name_;
	num_of_vehicle_ = rhs.num_of_vehicle_;

	std::vector<Vehicle>().swap(camila_);
	camila_.reserve(num_of_vehicle_);
	VehicleInfo vehicle_info_[num_of_vehicle_];

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
	{
		camila_.push_back(*it);
	}

	nh_ = ros::NodeHandle();
	nh_global_ = ros::NodeHandle("~");

	multi_setpoint_local_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server_ = nh_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);

	return *this;
}

SwarmVehicle::~SwarmVehicle()
{
	std::vector<Vehicle>().swap(camila_);
}

void SwarmVehicle::updateOffset()
{
	sensor_msgs::NavSatFix leader = camila_.front().getGlobalPosition();

	int i = 0;
	if (offset_.size() == 0)
	{
		if (num_of_vehicle_ != 1)
			angle_ = 2.0 * M_PI / (num_of_vehicle_ - 1);
		for (iter_ = camila_.begin(); iter_ != camila_.end(); iter_++)
		{
			sensor_msgs::NavSatFix follower = iter_->getGlobalPosition();
			geometry_msgs::Vector3 _offset = convertGeoToENU(leader, follower);
			offset_.push_back(_offset);
			ROS_INFO_STREAM("offset_[" << i << "] = " << offset_[i]);
			i++;
		}
	}
}

bool SwarmVehicle::multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request &req,
									  swarm_ctrl_pkg::srvMultiSetpointLocal::Response &res)
{
	geometry_msgs::PoseStamped msg, msg_f;
	double angle;

	msg.pose.position.x = req.x;
	msg.pose.position.y = req.y;
	msg.pose.position.z = req.z;

	updateOffset();

	double spacing;
	nh_global_.getParam("spacing", spacing);

	int i = 0;
	for (iter_ = camila_.begin(); iter_ != camila_.end(); iter_++)
	{
		if (iter_ != camila_.begin())
		{
			angle = i * angle_;

			msg_f.pose.position.x = msg.pose.position.x + offset_[i].x + spacing * cos(angle);
			msg_f.pose.position.y = msg.pose.position.y + offset_[i].y + spacing * sin(angle);
			msg_f.pose.position.z = msg.pose.position.z;
			iter_->setLocalTarget(msg_f);
		}
		else
			iter_->setLocalTarget(msg);
		i++;
	}
	res.success = true;

	return res.success;
}

bool SwarmVehicle::multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request &req,
									   swarm_ctrl_pkg::srvMultiSetpointGlobal::Response &res)
{
	res.success = true;
	return res.success;
}

bool SwarmVehicle::gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request &req,
							   swarm_ctrl_pkg::srvGoToVehicle::Response &res)
{
	geometry_msgs::PoseStamped msg;
	iter_ = camila_.begin() + req.num_drone - 1;

	updateOffset();

	if (num_of_vehicle_ != 1)
	{
		msg.pose.position.x = req.x + offset_[req.num_drone - 1].x;
		msg.pose.position.y = req.y + offset_[req.num_drone - 1].y;
		msg.pose.position.z = req.z;
	}
	else
	{
		msg.pose.position.x = req.x;
		msg.pose.position.y = req.y;
		msg.pose.position.z = req.z;
	}
	iter_->setLocalTarget(msg);

	return true;
}

geometry_msgs::Vector3 SwarmVehicle::convertGeoToENU(const sensor_msgs::NavSatFix &coord,
													 const sensor_msgs::NavSatFix &home)
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

	double lat_rad = coord.latitude * M_DEG_TO_RAD;
	double lon_rad = coord.longitude * M_DEG_TO_RAD;

	double ref_lon_rad = home.longitude * M_DEG_TO_RAD;
	double ref_lat_rad = home.latitude * M_DEG_TO_RAD;

	double sin_lat = sin(lat_rad);
	double cos_lat = cos(lat_rad);
	double cos_d_lon = cos(lon_rad - ref_lon_rad);

	double ref_sin_lat = sin(ref_lat_rad);
	double ref_cos_lat = cos(ref_lat_rad);

	double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
	double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

	geometry_msgs::Vector3 offset_;

	offset_.x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
	offset_.y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
	offset_.z = coord.altitude - home.altitude;

	return offset_;
}

geographic_msgs::GeoPoint SwarmVehicle::convertENUToGeo(const geometry_msgs::PoseStamped &local,
														const sensor_msgs::NavSatFix &home_global)
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

	double x_rad = local.pose.position.x / CONSTANTS_RADIUS_OF_EARTH;
	double y_rad = local.pose.position.y / CONSTANTS_RADIUS_OF_EARTH;
	double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
	double sin_c = sin(c);
	double cos_c = cos(c);

	double ref_lon_rad = home_global.longitude * M_DEG_TO_RAD;
	double ref_lat_rad = home_global.latitude * M_DEG_TO_RAD;

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
	geo_point.altitude = local.pose.position.z + home_global.altitude;

	return geo_point;
}

bool SwarmVehicle::isPublish()
{
	multi_setpoint_publish_flag_ = true;
	for (iter_ = camila_.begin(); iter_ != camila_.end(); iter_++)
	{
		if (iter_->isPublish() != true)
		{
			multi_setpoint_publish_flag_ = false;
			break;
		}
	}
	return multi_setpoint_publish_flag_;
}

void SwarmVehicle::setSwarmInfo(const std::string &swarm_name, const int &num_of_vehicle)
{
	swarm_name_ = swarm_name;
	num_of_vehicle_ = num_of_vehicle;

	std::vector<Vehicle>().swap(camila_);
	camila_.reserve(num_of_vehicle_);
	VehicleInfo vehicle_info_[num_of_vehicle_];

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1) + "/mavros";
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}

	nh_ = ros::NodeHandle();
	nh_global_ = ros::NodeHandle("~");

	multi_setpoint_local_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server_ = nh_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);
}

std::string SwarmVehicle::getSwarmInfo() const
{
	return swarm_name_;
}

void SwarmVehicle::addVehicle(const VehicleInfo &vehicle_info)
{
	camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info));
	num_of_vehicle_++;
}

void SwarmVehicle::deleteVehicle(const VehicleInfo &vehicle_info)
{
	for (iter_ = camila_.begin(); iter_ != camila_.end(); iter_++)
	{
		VehicleInfo temp = iter_->getInfo();
		if ((temp.vehicle_id_ == vehicle_info.vehicle_id_) && (temp.vehicle_name_ == vehicle_info.vehicle_name_))
		{
			camila_.erase(iter_);
			ROS_INFO_STREAM("Delete vehicle : sys_id : " << temp.vehicle_id_ << ", name : " << temp.vehicle_name_);
			num_of_vehicle_--;
			break;
		}
	}
}

void SwarmVehicle::showVehicleList() const
{
	for (auto &vehicle : camila_)
	{
		VehicleInfo info = vehicle.getInfo();
		ROS_INFO_STREAM(info.vehicle_id_ << " " << info.vehicle_name_);
	}
}

void SwarmVehicle::run()
{
	if (isPublish())
	{
		bool control_method;
		nh_global_.getParam("use_vel", control_method);
		for (iter_ = camila_.begin(); iter_ != camila_.end(); iter_++)
		{
			if (control_method)
				iter_->gotoVel();
			else
				iter_->gotoLocal();
		}
	}
}