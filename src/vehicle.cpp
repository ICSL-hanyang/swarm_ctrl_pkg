#include <ros/ros.h>
#include <vehicle.h>
#include <ros_msg_extension.h>

Vehicle::Vehicle(ros::NodeHandle &rNH_mul, ros::NodeHandle &rNH_global)
	: vehicle_info({1, "mavros"}),
	  nh_("camila"),
	  rNH_mul_(rNH_mul),
	  rNH_global_(rNH_global),
	  setpoint_publish_flag_(true)
{
	vehicleInit();
}

Vehicle::Vehicle(ros::NodeHandle &rNH_mul, ros::NodeHandle &rNH_global, const VehicleInfo &vehicle_info)
	: vehicle_info(vehicle_info),
	  nh_(vehicle_info.vehicle_name_),
	  rNH_mul_(rNH_mul),
	  rNH_global_(rNH_global),
	  setpoint_publish_flag_(true)
{
	vehicleInit();
}

Vehicle::Vehicle(const Vehicle &rhs)
	: vehicle_info(rhs.vehicle_info),
	  nh_(rhs.nh_),
	  rNH_mul_(rhs.rNH_mul_),
	  rNH_global_(rhs.rNH_global_),
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
	vehicle_info = rhs.vehicle_info;
	setpoint_publish_flag_ = rhs.setpoint_publish_flag_;
	nh_ = rhs.nh_;
	rNH_mul_ = rhs.rNH_mul_;
	rNH_global_ = rhs.rNH_global_;
	vehicleInit();
	return *this;
}

Vehicle::~Vehicle()
{
	state_sub_.shutdown();
	battery_sub_.shutdown();
	global_pos_sub_.shutdown();
	multi_arming_sub_.shutdown();
	multi_set_mode_sub_.shutdown();
	multi_takeoff_sub_.shutdown();
	multi_land_sub_.shutdown();

	setpoint_vel_pub_.shutdown();

	arming_client_.shutdown();
	set_mode_client_.shutdown();
	set_home_client_.shutdown();
	takeoff_client_.shutdown();
	land_client_.shutdown();
}

void Vehicle::vehicleInit()
{
	cur_global_.latitude = 0;
	cur_global_.longitude = 0;
	cur_global_.altitude = 0;

	setpoint_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("setpoint_velocity/cmd_vel_unstamped", 10);

	state_sub_ = nh_.subscribe("state", 10, &Vehicle::stateCB, this);
	battery_sub_ = nh_.subscribe("battery", 10, &Vehicle::batteryCB, this);
	global_pos_sub_ = nh_.subscribe("global_position/global", 10, &Vehicle::globalPositionCB, this);

	arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("cmd/arming");
	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("set_mode");
	set_home_client_ = nh_.serviceClient<mavros_msgs::CommandHome>("cmd/set_home");
	takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("cmd/takeoff");
	land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("cmd/land");

	multi_arming_sub_ = rNH_mul_.subscribe("arming", 10, &Vehicle::multiArming, this);
	multi_set_mode_sub_ = rNH_mul_.subscribe("set_mode", 10, &Vehicle::multiSetMode, this);
	multi_takeoff_sub_ = rNH_mul_.subscribe("takeoff", 10, &Vehicle::multiTakeoff, this);
	multi_land_sub_ = rNH_mul_.subscribe("land", 10, &Vehicle::multiLand, this);

	ROS_INFO_STREAM(vehicle_info.vehicle_name_ << " instance generated");
}

void Vehicle::stateCB(const mavros_msgs::State::ConstPtr &msg)
{
	if (cur_state_.connected != msg->connected)
	{
		if (msg->connected == true)
			ROS_INFO_STREAM(vehicle_info.vehicle_name_ << " is connected");
		else
			ROS_WARN_STREAM(vehicle_info.vehicle_name_ << " is NOT connected");
	}
	if (cur_state_.armed != msg->armed)
	{
		if (msg->armed == true)
			ROS_INFO_STREAM(vehicle_info.vehicle_name_ << " is armed");
		else
			ROS_INFO_STREAM(vehicle_info.vehicle_name_ << " is disarmed");
	}
	if (cur_state_.mode != msg->mode)
	{
		ROS_INFO_STREAM(vehicle_info.vehicle_name_ << " is in " << msg->mode << " mode");
	}
	cur_state_ = *msg;
}

void Vehicle::batteryCB(const sensor_msgs::BatteryState::ConstPtr &msg)
{
	cur_battery_ = *msg;
}

void Vehicle::globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	cur_global_ = *msg;
}

void Vehicle::setVehicleInfo(const VehicleInfo &new_vehicle_info)
{
	vehicle_info.system_id_ = new_vehicle_info.system_id_;
	vehicle_info.vehicle_name_ = new_vehicle_info.vehicle_name_;

	nh_ = ros::NodeHandle(vehicle_info.vehicle_name_);
	vehicleInit();
}

VehicleInfo Vehicle::getInfo() const
{
	return vehicle_info;
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
		ROS_ERROR_STREAM(vehicle_info.vehicle_name_ << " failed to call arming service. " << msg.response.result);
	return msg.response.success;
}

bool Vehicle::setMode(const std::string &input_mode)
{
	mavros_msgs::SetMode mode;
	mode.request.custom_mode = input_mode;
	if (((input_mode == "offboard") || (input_mode == "OFFBOARD")) && (!setpoint_publish_flag_))
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
		ROS_ERROR_STREAM(vehicle_info.vehicle_name_ << " failed to call takeoff service. " << msg.response.result);
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
		ROS_ERROR_STREAM(vehicle_info.vehicle_name_ << " failed to call land service. " << msg.response.result);
	return msg.response.success;
}

/*multi란? -> 커멘드로 날라온 단발 토픽을 mavros 서비스로 날리는 놈, 객체마다 있기에 동시에 서비스를 날리는 효과*/

void Vehicle::multiArming(const std_msgs::Bool::ConstPtr &msg)
{
	mavros_msgs::CommandBool arm;
	arm.request.value = msg->data;

	if (arming_client_.call(arm) && arm.response.success)
	{
	}
	else
	{
		ROS_ERROR_STREAM(vehicle_info.vehicle_name_ << " failed to call arming service. " << arm.response.result);
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
			ROS_ERROR_STREAM(vehicle_info.vehicle_name_ << " failed to call set_mode service. " << mode.response.mode_sent);
		}
	}
}

void Vehicle::multiTakeoff(const std_msgs::Empty::ConstPtr &trigger)
{
	int takeoff_alt;
	rNH_global_.getParam("takeoff_alt", takeoff_alt);
	takeoff(takeoff_alt);
}

void Vehicle::multiLand(const std_msgs::Empty::ConstPtr &trigger)
{
	land();
}

sensor_msgs::NavSatFix Vehicle::getGlobalPosition() const
{
	return cur_global_;
}

bool Vehicle::isReceivedGlobalPos() const
{
	sensor_msgs::NavSatFixConstPtr msg = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(vehicle_info.vehicle_name_ + "/global_position/global");
	if (msg && (cur_state_.mode == "AUTO.LOITER" || cur_state_.mode == "auto.loiter"))
		return true;
	else
		return false;
}

bool Vehicle::isPublish() const
{
	return setpoint_publish_flag_;
}

//default value : default name = camila, num_of_vehicle = 1;
SwarmVehicle::SwarmVehicle(ros::NodeHandle &rNH, const std::string &swarm_name,const int &num_of_vehicle)
	: swarm_name_(swarm_name),
	  num_of_vehicle_(num_of_vehicle),
	  nh_(ros::NodeHandle()),
	  nh_mul_("multi"),
	  rNH_global_(rNH),
	  swarm_map_(sensor_msgs::NavSatFix()),
	  multi_setpoint_publish_flag_(false)
{
	VehicleInfo vehicle_info[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info[i].system_id_ = i + 1;
		vehicle_info[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1) + "/mavros";
		camila_.push_back(Vehicle(nh_mul_, rNH_global_, vehicle_info[i]));
	}
	swarm_target_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
}

SwarmVehicle::SwarmVehicle(const SwarmVehicle &rhs)
	: swarm_name_(rhs.swarm_name_),
	  num_of_vehicle_(rhs.num_of_vehicle_),
	  nh_(ros::NodeHandle()),
	  nh_mul_("multi"),
	  rNH_global_(rhs.rNH_global_),
	  multi_setpoint_publish_flag_(rhs.multi_setpoint_publish_flag_)
{
	VehicleInfo vehicle_info[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
	{
		camila_.push_back(*it);
	}
	swarm_target_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
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

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
	{
		camila_.push_back(*it);
	}

	nh_ = rhs.nh_;
	nh_mul_ = rhs.nh_mul_;
	rNH_global_ = rhs.rNH_global_;

	swarm_target_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
	return *this;
}

SwarmVehicle::~SwarmVehicle()
{
	swarm_target_server_.shutdown();
	std::vector<Vehicle>().swap(camila_);
}

void SwarmVehicle::setSwarmMap()
{
	unsigned int cnt = 0;
	unsigned int sec = 1;
	ros::Time start = ros::Time::now();

	while (cnt != camila_.size())
	{
		ros::spinOnce();
		ros::Time now = ros::Time::now();

		cnt = 0;
		for (auto &vehicle : camila_)
		{
			if (vehicle.isReceivedGlobalPos())
				cnt++;
		}

		if (now - start > ros::Duration(sec))
		{
			ROS_INFO_STREAM("Waiting for GPS signal ..." << cnt);
			sec++;
		}

		if (now - start > ros::Duration(60))
		{
			ROS_WARN_STREAM("GPS time out! only " << cnt << " vehilces get GPS signal.");
			break;
		}
	}

	for (auto &vehicle : camila_)
	{
		sensor_msgs::NavSatFix gps_pos = vehicle.getGlobalPosition();
		swarm_map_ += gps_pos;
	}
	swarm_map_.latitude /= camila_.size();
	swarm_map_.longitude /= camila_.size();
	swarm_map_.altitude /= camila_.size();
}

void SwarmVehicle::offsetPublisher()
{
	int i = 0;
	for (auto &vehicle : camila_)
	{
		geometry_msgs::TransformStamped tf_stamped;
		sensor_msgs::NavSatFix gps_pos = vehicle.getGlobalPosition();
		geometry_msgs::Vector3 _offset = convertGeoToENU(gps_pos, swarm_map_);
		offset_.push_back(_offset);
		ROS_INFO_STREAM("offset[" << i << "] = " << _offset);

		tf_stamped.header.stamp = ros::Time::now();
		tf_stamped.header.frame_id = "swarm_map";
		tf_stamped.child_frame_id = "camila" + std::to_string(i + 1) + "_map";
		tf_stamped.transform.translation.x = _offset.x;
		tf_stamped.transform.translation.y = _offset.y;
		tf_stamped.transform.translation.z = _offset.z;
		tf2::Quaternion quat;
		quat.setRPY(0, 0, 0);
		tf_stamped.transform.rotation.x = quat.x();
		tf_stamped.transform.rotation.y = quat.y();
		tf_stamped.transform.rotation.z = quat.z();
		tf_stamped.transform.rotation.w = quat.w();
		static_offset_bc_.sendTransform(tf_stamped);
		i++;
	}
}

void SwarmVehicle::formationGenerater()
{
	if (multi_setpoint_publish_flag_)
	{ //요거 약간 병신같음
		swarm_target_TF_.header.stamp = ros::Time::now();
		swarm_target_bc_.sendTransform(swarm_target_TF_);

		geometry_msgs::TransformStamped vehicle_target_TF;
		if (formation_ == "POINT")
		{
			vehicle_target_TF = swarm_target_TF_;
			for (auto &vehicle : camila_)
			{
				std::string vehicle_target = "camila" + std::to_string(vehicle.getInfo().system_id_) + "_target";
				transformSender(0, 0, 0, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			}
		}
		else if (formation_ == "SCEN1")
		{
			scenario1();
		}
		else if(formation == "SCEN2"){
			scenario2();
		}
		else if(formation == "SCEN3"){
			scenario3();
		}
	}
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

bool SwarmVehicle::setSwarmTarget(swarm_ctrl_pkg::srvSetSwarmTarget::Request &req,
								  swarm_ctrl_pkg::srvSetSwarmTarget::Response &res)
{
	swarm_target_TF_.header.stamp = ros::Time::now();
	swarm_target_TF_.header.frame_id = "swarm_map";
	swarm_target_TF_.child_frame_id = "swarm_target";
	swarm_target_TF_.transform.translation.x = req.x;
	swarm_target_TF_.transform.translation.y = req.y;
	swarm_target_TF_.transform.translation.z = req.z;
	tf2::Quaternion q;
	q.setRPY(0, 0, req.yaw);
	swarm_target_TF_.transform.rotation.x = q.x();
	swarm_target_TF_.transform.rotation.y = q.y();
	swarm_target_TF_.transform.rotation.z = q.z();
	swarm_target_TF_.transform.rotation.w = q.w();

	formation_ = req.formation;
	multi_setpoint_publish_flag_ = true;

	res.success = true;

	return res.success;
}

bool SwarmVehicle::isPublish()
{
	multi_setpoint_publish_flag_ = true;
	for (auto &vehicle : camila_)
	{
		if (vehicle.isPublish() != true)
		{
			multi_setpoint_publish_flag_ = false;
			break;
		}
		//set_point publish_flag랑 is publish 정리
		//vehicle 의 isPublish flag를 라즈단에서 결정
	}
	return multi_setpoint_publish_flag_;
}

void SwarmVehicle::scenario1() const
{
	double MSec = ros::Time::now().toNSec() / 1000000;
	double x = 0.0002 * MSec; // MSec가 초당 1000씩 증가하므로 0.2 rad/s
	double cosx = cos(x);
	double sinx = sin(x);
	std::string vehicle_target = "camila1_target";
	transformSender(-45 * cosx, -45 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila2_target";
	transformSender(-40 * cosx, -40 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila3_target";
	transformSender(-35 * cosx, -35 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila4_target";
	transformSender(-40 * cosx, -40 * sinx, 20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila5_target";
	transformSender(-40 * cosx, -40 * sinx, 10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila6_target";
	transformSender(-40 * cosx, -40 * sinx, 0, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila7_target";
	transformSender(-40 * cosx, -40 * sinx, -10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila8_target";
	transformSender(-45 * cosx, -45 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila9_target";
	transformSender(-40 * cosx, -40 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila10_target";
	transformSender(-35 * cosx, -35 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila11_target";
	transformSender(-20 * cosx, -20 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila12_target";
	transformSender(-15 * cosx, -15 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila13_target";
	transformSender(-25 * cosx, -25 * sinx, 20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila14_target";
	transformSender(-10 * cosx, -10 * sinx, 20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila15_target";
	transformSender(-25 * cosx, -25 * sinx, 10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila16_target";
	transformSender(-25 * cosx, -25 * sinx, 0, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila17_target";
	transformSender(-25 * cosx, -25 * sinx, -10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila18_target";
	transformSender(-10 * cosx, -10 * sinx, -10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila19_target";
	transformSender(-20 * cosx, -20 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila20_target";
	transformSender(-15 * cosx, -15 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila21_target";
	transformSender(5 * cosx, 5 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila22_target";
	transformSender(10 * cosx, 10 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila23_target";
	transformSender(0, 0, 20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila24_target";
	transformSender(15 * cosx, 15 * sinx, 20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila2;5_target";
	transformSender(5 * cosx, 5 * sinx, 10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila26_target";
	transformSender(10 * cosx, 10 * sinx, 0, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila27_target";
	transformSender(15 * cosx, 15 * sinx, -10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila28_target";
	transformSender(0, 0, -10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila29_target";
	transformSender(5 * cosx, 5 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila30_target";
	transformSender(10 * cosx, 10 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila31_target";
	transformSender(25 * cosx, 25 * sinx, 30, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila32_target";
	transformSender(25 * cosx, 25 * sinx, 20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila33_target";
	transformSender(25 * cosx, 25 * sinx, 10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila34_target";
	transformSender(25 * cosx, 25 * sinx, 0, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila35_target";
	transformSender(25 * cosx, 25 * sinx, -10, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila36_target";
	transformSender(25 * cosx, 25 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila37_target";
	transformSender(30 * cosx, 30 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila38_target";
	transformSender(35 * cosx, 35 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila39_target";
	transformSender(40 * cosx, 40 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
	vehicle_target = "camila40_target";
	transformSender(50 * cosx, 50 * sinx, -20, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
}
void SwarmVehicle::scenario2()
{
	geometry_msgs::TransformStamped vehicle_target_TF;
	vehicle_target_TF = swarm_target_TF;
	double MSec = ros::Time::now().toNSec() / 1000000;
	//double x = 0.001 * MSec; // MSec가 초당 1000씩 증가하므로 1 rad/s
	double x = 0;
	double scale = 2;
	for (auto &vehicle : camila)
	{
		int id = vehicle.getInfo().system_id;
		std::string vehicle_target = "camila" + std::to_string(id) + "_target";
		
		if (id <= 3)
			transformSender((2*(id%3)-2)*scale, 0, 6*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 6)
			transformSender((2*(id%3)-2)*scale, 0, -6*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 9)
			transformSender(-4*scale, 0, (3*(id%3)-3)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 12)
			transformSender(4*scale, 0, (3*(id%3)-3)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 14)
			transformSender((-2*(id%2)-5)*scale, 0, (2*(id%2)+7)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 16)
			transformSender((2*(id%2)+5)*scale, 0, (2*(id%2)+7)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 18)
			transformSender((2*(id%2)-7)*scale, 0, (2*(id%2)-9)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 20)
			transformSender((-2*(id%2)+7)*scale, 0, (2*(id%2)-9)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 25){
			if (id%5 == 2)
				transformSender((-9)*scale, 5, (11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((-9+2*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((-9+2*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((-9+4*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((-9+4*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
		else if (id <= 30){
			if (id%5 == 2)
				transformSender((9)*scale, 5, (11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((9+2*sqrt(2)*cos(x-M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((9+2*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((9+4*sqrt(2)*cos(x-M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((9+4*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
		else if (id <= 35){
			if (id%5 == 2)
				transformSender((-9)*scale, 5, (-11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((-9+2*sqrt(2)*cos(x-M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((-9+2*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((-9+4*sqrt(2)*cos(x-M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((-9+4*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
		else if (id <= 40){
			if (id%5 == 2)
				transformSender((9)*scale, 5, (-11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((9+2*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((9+2*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((9+4*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((9+4*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
	}
}
void SwarmVehicle::scenario3()
{
	geometry_msgs::TransformStamped vehicle_target_TF;
	vehicle_target_TF = swarm_target_TF;
	double MSec = ros::Time::now().toNSec() / 1000000;
	double x = 0.0005 * MSec; // MSec가 초당 1000씩 증가하므로 1 rad/s
	//double x = 0;
	double scale = 2;
	for (auto &vehicle : camila)
	{
		int id = vehicle.getInfo().system_id;
		std::string vehicle_target = "camila" + std::to_string(id) + "_target";
		
		if (id <= 3)
			transformSender((2*(id%3)-2)*scale, 0, 6*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 6)
			transformSender((2*(id%3)-2)*scale, 0, -6*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 9)
			transformSender(-4*scale, 0, (3*(id%3)-3)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 12)
			transformSender(4*scale, 0, (3*(id%3)-3)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 14)
			transformSender((-2*(id%2)-5)*scale, 0, (2*(id%2)+7)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 16)
			transformSender((2*(id%2)+5)*scale, 0, (2*(id%2)+7)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 18)
			transformSender((2*(id%2)-7)*scale, 0, (2*(id%2)-9)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 20)
			transformSender((-2*(id%2)+7)*scale, 0, (2*(id%2)-9)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		else if (id <= 25){
			if (id%5 == 2)
				transformSender((-9)*scale, 5, (11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((-9+2*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((-9+2*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((-9+4*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((-9+4*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
		else if (id <= 30){
			if (id%5 == 2)
				transformSender((9)*scale, 5, (11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((9+2*sqrt(2)*cos(x-M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((9+2*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (11+2*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((9+4*sqrt(2)*cos(x-M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((9+4*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (11+4*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
		else if (id <= 35){
			if (id%5 == 2)
				transformSender((-9)*scale, 5, (-11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((-9+2*sqrt(2)*cos(x-M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((-9+2*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((-9+4*sqrt(2)*cos(x-M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(x-M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((-9+4*sqrt(2)*cos(x+3*M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(x+3*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
		else if (id <= 40){
			if (id%5 == 2)
				transformSender((9)*scale, 5, (-11)*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 1)
				transformSender((9+2*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 3)
				transformSender((9+2*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (-11+2*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else if (id%5 == 0)
				transformSender((9+4*sqrt(2)*cos(-x+M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(-x+M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
			else
				transformSender((9+4*sqrt(2)*cos(-x+5*M_PI_4))*scale, 5, (-11+4*sqrt(2)*sin(-x+5*M_PI_4))*scale, 0, 0, 0, ros::Time::now(), "swarm_target", vehicle_target);
		}
	}
}
void SwarmVehicle::setSwarmInfo(const std::string &swarm_name, const int &num_of_vehicle)
{
	swarm_name_ = swarm_name;
	num_of_vehicle_ = num_of_vehicle;

	std::vector<Vehicle>().swap(camila_);
	camila_.reserve(num_of_vehicle_);
	VehicleInfo vehicle_info[num_of_vehicle_];

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info[i].system_id_ = i + 1;
		vehicle_info[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1) + "/mavros";
		camila_.push_back(Vehicle(nh_mul_, rNH_global_, vehicle_info[i]));
	}

	nh_ = ros::NodeHandle();

	swarm_target_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::setSwarmTarget, this);
}

std::string SwarmVehicle::getSwarmInfo() const
{
	return swarm_name_;
}

void SwarmVehicle::addVehicle(const VehicleInfo &vehicle_info)
{
	camila_.push_back(Vehicle(nh_mul_, rNH_global_, vehicle_info));
	num_of_vehicle_++;
}

void SwarmVehicle::deleteVehicle(const VehicleInfo &vehicle_info)
{
	for (iter_ = camila_.begin(); iter_ != camila_.end(); iter_++)
	{
		VehicleInfo temp = iter_->getInfo();
		if ((temp.system_id_ == vehicle_info.system_id_) && (temp.vehicle_name_ == vehicle_info.vehicle_name_))
		{
			iter_ = camila_.erase(iter_);
			ROS_INFO_STREAM("Delete vehicle : sys_id : " << temp.system_id_ << ", name : " << temp.vehicle_name_);
			num_of_vehicle_--;
			break;
		}
	}
}

void SwarmVehicle::showVehicleList() const
{
	for (auto &vehicle : camila_)
	{
		VehicleInfo temp = vehicle.getInfo();
		ROS_INFO_STREAM(temp.system_id_ << " " << temp.vehicle_name_);
	}
}

void SwarmVehicle::init()
{
	setSwarmMap();
	offsetPublisher();
}

void SwarmVehicle::run()
{
	formationGenerater();
}