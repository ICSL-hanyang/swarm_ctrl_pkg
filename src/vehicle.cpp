#include <ros/ros.h>
#include <vehicle.h>
#include <scenario.h>
#include <scenario2.h>

AttractiveOnly* AttractiveOnly::instance_ = nullptr;
PotentialField* PotentialField::instance_ = nullptr;

AttractiveOnly::AttractiveOnly(){}
PotentialField::PotentialField(){}

tf2::Vector3 AttractiveOnly::generate(LocalPlanner &lp){
	tf2::Vector3 local_plan = lp.getErr() * lp.getKpAtt();
	return local_plan;
}

tf2::Vector3 PotentialField::generate(LocalPlanner &lp){
	tf2::Vector3 local_plan = lp.getErr() * lp.getKpAtt() + lp.getSumOfRepulsive() * lp.getKpRep();
	return local_plan;
}

LocalPlanner::LocalPlanner()
{
	plan_ = PotentialField::getInstance();
}

LocalPlanner::LocalPlanner(const LocalPlanner &rhs) :
	cur_global_pose_(rhs.cur_global_pose_),
	kp_attractive_(rhs.kp_attractive_),
	kp_repulsive_(rhs.kp_repulsive_),
	err_(rhs.err_),
	sum_repulsive_(rhs.sum_repulsive_),
	local_plan_(rhs.local_plan_)
{
	*this = rhs;
}

const LocalPlanner &LocalPlanner::operator=(const LocalPlanner &rhs)
{
	if (this == &rhs)
		return *this;
	cur_global_pose_ = rhs.cur_global_pose_;
	kp_attractive_ = rhs.kp_attractive_;
	kp_repulsive_ = rhs.kp_repulsive_;
	err_ = rhs.err_;
	sum_repulsive_ = rhs.sum_repulsive_;
	local_plan_ = rhs.local_plan_;
	return *this;
}

tf2::Vector3 LocalPlanner::generate(){
	return plan_->generate(*this);
}

template <typename T>
PoseController<T>::PoseController(ros::NodeHandle &nh, ros::NodeHandle &nh_global, std::string &vehicle_name) :
	nh_(nh),
	nh_global_(nh_global),
	vehicle_name_(vehicle_name)
{}

GeoPoseController::GeoPoseController(ros::NodeHandle &nh, ros::NodeHandle &nh_global, std::string &vehicle_name) :
	PoseController(nh, nh_global, vehicle_name)
{
	home_sub_ = nh_.subscribe("mavros/home_position/home", 10, &GeoPoseController::homeCB, this);
	cur_pose_sub_ = nh_.subscribe("mavros/global_position/global", 10, &GeoPoseController::curPoseCB, this);
	setpoint_pub_ = nh_.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
	setpoint_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
}

GeoPoseController::~GeoPoseController(){
	home_sub_.shutdown();
	cur_pose_sub_.shutdown();
	setpoint_pub_.shutdown();
	setpoint_vel_pub_.shutdown();
}

void GeoPoseController::homeCB(const mavros_msgs::HomePosition::ConstPtr &msg){
	home_.longitude = msg->geo.longitude;
	home_.latitude = msg->geo.latitude;
	home_.altitude = msg->geo.altitude;
}

void GeoPoseController::curPoseCB(const sensor_msgs::NavSatFix::ConstPtr &msg){
	cur_pose_ = *msg;
}

void GeoPoseController::setTarget(const sensor_msgs::NavSatFix &target){
	setpoint_publish_flag_ = true;

	if ((target_.latitude != target.latitude) ||
		(target_.longitude != target.longitude) ||
		(target_.altitude != target.altitude))
	{
		target_ = target;
		ROS_INFO("%s set target_global_pos(long : %lf, lati : %lf, alti : %lf)", getName().c_str(),
				 target_.longitude, target_.latitude, target_.altitude);
	}
}

void GeoPoseController::goTo(){
	target_.header.seq += 1;
	target_.header.stamp = ros::Time::now();
	target_.header.frame_id = getName();

	setpoint_pub_.publish(target_);
}

void GeoPoseController::goToVel(){

	setpoint_vel_pub_.publish(target_);
}

bool GeoPoseController::setHome(){
	mavros_msgs::CommandHome msg;

	msg.request.current_gps = false;
	msg.request.latitude = cur_pose_.latitude;
	msg.request.longitude = cur_pose_.longitude;
	msg.request.altitude = cur_pose_.altitude;

	if (set_home_client_.call(msg) && msg.response.success)
	{
		home_ = cur_pose_;

		ROS_INFO_STREAM(getName() << " Global home position is set. " << msg.response.result);
		ROS_INFO("%s new global home is (long : %lf, lang : %lf, alti : %lf)", getName().c_str(),
				 home_.longitude, home_.latitude, home_.altitude);
	}
	else
		ROS_ERROR_STREAM(getName() << " Failed to set global home position. " << msg.response.result);

	return msg.response.success;
}

LocalPoseController::LocalPoseController(ros::NodeHandle &nh, ros::NodeHandle &nh_global, std::string &vehicle_name) : 	
	PoseController(nh, nh_global, vehicle_name)
{
	setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	setpoint_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	cur_pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &LocalPoseController::curPoseCB, this);
}

LocalPoseController::~LocalPoseController(){
	setpoint_pub_.shutdown();
	setpoint_vel_pub_.shutdown();
	cur_pose_sub_.shutdown();
}

void LocalPoseController::goTo(){
	local_path_.header.seq += 1;
	local_path_.header.stamp = ros::Time::now();
	local_path_.header.frame_id = getName();

	setpoint_pub_.publish(local_path_);
}

void LocalPoseController::goToVel(){
	double kp;
	geometry_msgs::Twist vel;
	nh_global_.getParamCached("pid/kp", kp);
	vel.linear.x = (local_path_.pose.position.x - cur_pose_.pose.position.x) * kp;
	vel.linear.y = (local_path_.pose.position.y - cur_pose_.pose.position.y) * kp;
	vel.linear.z = (local_path_.pose.position.z - cur_pose_.pose.position.z) * kp;

	setpoint_vel_pub_.publish(vel);
}

void LocalPoseController::setTarget(const geometry_msgs::PoseStamped &target){
	setpoint_publish_flag_ = true;

	if ((target_.pose.position.x != target.pose.position.x) ||
		(target_.pose.position.y != target.pose.position.y) ||
		(target_.pose.position.z != target.pose.position.z))
	{
		target_ = target;
		ROS_INFO_STREAM(getName()<< " set target : " << target_.pose.position.x << ", " << target_.pose.position.y << ", " << target_.pose.position.z);
	}
}

Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global)
	: vehicle_info_({1, "mavros"}),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(nh_mul),
	  nh_global_(nh_global),
	  scen_pos_(std::pair<int, int>(0, 0)),
	  gp_controller_(nh_, nh_global, vehicle_info_.vehicle_name_),
	  lp_controller_(nh_, nh_global, vehicle_info_.vehicle_name_)
{
	vehicleInit();
}

Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global, const VehicleInfo &vehicle_info)
	: vehicle_info_(vehicle_info),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(nh_mul),
	  nh_global_(nh_global),
	  scen_pos_(std::pair<int, int>(0, 0)),
	  gp_controller_(nh_, nh_global, vehicle_info_.vehicle_name_),
	  lp_controller_(nh_, nh_global, vehicle_info_.vehicle_name_)
{
	vehicleInit();
}

Vehicle::Vehicle(const Vehicle &rhs)
	: vehicle_info_(rhs.vehicle_info_),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(rhs.nh_mul_),
	  nh_global_(rhs.nh_global_),
	  scen_pos_(std::pair<int, int>(0, 0)),
	  gp_controller_(nh_, rhs.nh_global_, vehicle_info_.vehicle_name_),
	  lp_controller_(nh_, rhs.nh_global_, vehicle_info_.vehicle_name_)
{
	vehicleInit();
	*this = rhs;
}

const Vehicle &Vehicle::operator=(const Vehicle &rhs)
{
	if (this == &rhs)
		return *this;

	vehicle_info_ = rhs.vehicle_info_;

	nh_ = ros::NodeHandle(vehicle_info_.vehicle_name_);
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;
	scen_pos_ = rhs.scen_pos_;

	vehicleInit();
	return *this;
}

Vehicle::~Vehicle()
{
	release();
}

void Vehicle::vehicleInit()
{
	state_sub_ = nh_.subscribe("mavros/state", 10, &Vehicle::stateCB, this);
	battery_sub_ = nh_.subscribe("mavros/battery", 10, &Vehicle::batteryCB, this);

	arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	set_home_client_ = nh_.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");
	takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

	multi_arming_sub_ = nh_mul_.subscribe("arming", 10, &Vehicle::multiArming, this);
	multi_set_mode_sub_ = nh_mul_.subscribe("set_mode", 10, &Vehicle::multiSetMode, this);
	multi_set_home_sub_ = nh_mul_.subscribe("set_home", 10, &Vehicle::multiSetHome, this);
	multi_takeoff_sub_ = nh_mul_.subscribe("takeoff", 10, &Vehicle::multiTakeoff, this);
	multi_land_sub_ = nh_mul_.subscribe("land", 10, &Vehicle::multiLand, this);

	ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " instance generated");
}

void Vehicle::release()
{
	
	state_sub_.shutdown();
	battery_sub_.shutdown();

	arming_client_.shutdown();
	set_mode_client_.shutdown();
	set_home_client_.shutdown();
	takeoff_client_.shutdown();
	land_client_.shutdown();

	multi_arming_sub_.shutdown();
	multi_set_mode_sub_.shutdown();
	multi_set_home_sub_.shutdown();
	multi_takeoff_sub_.shutdown();
	multi_land_sub_.shutdown();
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

void Vehicle::multiArming(const std_msgs::Bool::ConstPtr &msg)
{
	if (arming(msg->data))
		;
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call multi arming service. ");
}

void Vehicle::multiSetMode(const std_msgs::String::ConstPtr &msg)
{
	if (setMode(msg->data))
		;
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call multi set_mode service. ");
}

void Vehicle::multiSetHome(const std_msgs::Empty::ConstPtr &trigger)
{
	setHomeGlobal();
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

	release();

	nh_ = ros::NodeHandle(vehicle_info_.vehicle_name_);

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
	if (((current_mode == "offboard") || (current_mode == "OFFBOARD")) && (!lp_controller_.isPublished()))
	{
		ROS_WARN("Please publish setpoint first");
		return false;
	}
	else
	{
		if (set_mode_client_.call(mode) && mode.response.mode_sent)
			;
		else
			ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call set_mode service. " << mode.response.mode_sent);
		return mode.response.mode_sent;
	}
}

bool Vehicle::takeoff(const double &takeoff_alt)
{
	mavros_msgs::CommandTOL msg;
	sensor_msgs::NavSatFix cur_pose = getGeoPose();
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_pose.latitude;
	msg.request.longitude = cur_pose.longitude;
	msg.request.altitude = gp_controller_.getHome().altitude + takeoff_alt;

	if (takeoff_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call takeoff service. " << msg.response.result);
	return msg.response.success;
}

bool Vehicle::land()
{
	mavros_msgs::CommandTOL msg;
	sensor_msgs::NavSatFix cur_pose = getGeoPose();
	msg.request.min_pitch = 0;
	msg.request.yaw = 0;
	msg.request.latitude = cur_pose.latitude;
	msg.request.longitude = cur_pose.longitude;
	msg.request.altitude = 0;

	if (land_client_.call(msg) && msg.response.success)
		ROS_INFO_STREAM(msg.response.result);
	else
		ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call land service. " << msg.response.result);
	return msg.response.success;
}

void Vehicle::setGeoTarget(const sensor_msgs::NavSatFix &target)
{
	gp_controller_.setTarget(target);
}

void Vehicle::setLocalTarget(const geometry_msgs::PoseStamped &target)
{
	lp_controller_.setTarget(target);
}

void Vehicle::goTo(){
	bool use_velocity_controller;
	double kp_att, kp_rep, kp;
	nh_global_.getParamCached("use_velocity_controller", use_velocity_controller);
	nh_global_.getParamCached("local_plan/kp_attractive", kp_att);
	nh_global_.getParamCached("local_plan/kp_attractive", kp_rep);
	local_planner_.setKpAtt(kp_att);
	local_planner_.setKpRep(kp_rep);

	tf2::Vector3 local_plan = local_planner_.generate();
	geometry_msgs::PoseStamped local_path;
	geometry_msgs::PoseStamped cur_pose = lp_controller_.getCuPose();
	local_path.pose.position.x = cur_pose.pose.position.x + local_plan.getX();
	local_path.pose.position.y = cur_pose.pose.position.y + local_plan.getY();
	local_path.pose.position.z = cur_pose.pose.position.z + local_plan.getZ();
	lp_controller_.setLocalPath(local_path);
	
	if(isPublish()){
		if (use_velocity_controller)
			lp_controller_.goToVel();
		else
			lp_controller_.goTo();
	}
}

void Vehicle::setScenPos(const std::pair<int, int> &scen_pos)
{
	scen_pos_ = scen_pos;
}

std::pair<int, int> Vehicle::getScenPos() const
{
	return scen_pos_;
}

bool Vehicle::setHomeGlobal()
{
	gp_controller_.setHome();
}

void Vehicle::setLocalPlanner(Plans *plan){
	local_planner_.setPlanner(plan);
}

void Vehicle::setGlobalPose(const tf2::Vector3 &global_pose){
	local_planner_.setGlobalPose(global_pose);
}

void Vehicle::setErr(const tf2::Vector3 &err){
	local_planner_.setErr(err);
}

void Vehicle::setSumOfRepulsive(const tf2::Vector3 &sum_of_repulsive){
	local_planner_.setSumOfRepulsive(sum_of_repulsive);
}

bool Vehicle::isPublish() const
{
	return lp_controller_.isPublished();
}

double SwarmVehicle::repulsive_range_;
double SwarmVehicle::max_speed_;
int SwarmVehicle::scen_num_;
std::string SwarmVehicle::scen_str_ = "";

//default value : default name = camila, num_of_vehicle = 1;
SwarmVehicle::SwarmVehicle(ros::NodeHandle &nh_global, const std::string &swarm_name, const int &num_of_vehicle)
	: swarm_name_(swarm_name),
	  num_of_vehicle_(num_of_vehicle),
	  nh_(ros::NodeHandle()),
	  nh_mul_("multi"),
	  nh_global_(nh_global),
	  swarm_target_local_(tf2::Vector3(0, 0, 0)),
	  target_changed_flag_(false)
{
	VehicleInfo vehicle_info_[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1);
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}

	swarmServiceInit();
}

SwarmVehicle::SwarmVehicle(const SwarmVehicle &rhs)
	: swarm_name_(rhs.swarm_name_),
	  num_of_vehicle_(rhs.num_of_vehicle_),
	  nh_(rhs.nh_),
	  nh_mul_(rhs.nh_mul_),
	  nh_global_(rhs.nh_global_),
	  swarm_target_local_(tf2::Vector3(0, 0, 0)),
	  target_changed_flag_(rhs.target_changed_flag_)
{
	VehicleInfo vehicle_info_[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
		camila_.push_back(*it);

	nh_ = rhs.nh_;
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;

	swarmServiceInit();

	*this = rhs;
}

const SwarmVehicle &SwarmVehicle::operator=(const SwarmVehicle &rhs)
{
	if (this == &rhs)
		return *this;

	swarm_name_ = rhs.swarm_name_;
	num_of_vehicle_ = rhs.num_of_vehicle_;

	std::vector<Vehicle>().swap(camila_);
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);
	scen_hex_.reserve(num_of_vehicle_);

	std::vector<Vehicle>::const_iterator it;
	for (it = rhs.camila_.begin(); it != rhs.camila_.end(); it++)
		camila_.push_back(*it);

	nh_ = rhs.nh_;
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;

	swarmServiceInit();

	return *this;
}

void SwarmVehicle::swarmServiceInit()
{
	multi_setpoint_local_server_ = nh_.advertiseService("multi_setpoint_local", &SwarmVehicle::multiSetpointLocal, this);
	multi_setpoint_global_server_ = nh_.advertiseService("multi_setpoint_global", &SwarmVehicle::multiSetpointGlobal, this);
	goto_vehicle_server_ = nh_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);
}

void SwarmVehicle::release()
{
	std::vector<Vehicle>().swap(camila_);
	std::vector<tf2::Vector3>().swap(offset_);
	std::vector<uint8_t>().swap(scen_hex_);
	multi_setpoint_global_server_.shutdown();
	multi_setpoint_local_server_.shutdown();
	goto_vehicle_server_.shutdown();
}

SwarmVehicle::~SwarmVehicle()
{
	release();
}

void SwarmVehicle::updateOffset()
{
	sensor_msgs::NavSatFix leader = camila_.front().getHomeGeo();

	int i = 0;
	if (offset_.size() == 0)
	{
		if (num_of_vehicle_ != 1)
			angle_ = 2.0 * M_PI / (num_of_vehicle_ - 1);
		for (auto &vehicle : camila_)
		{
			sensor_msgs::NavSatFix follower = vehicle.getGeoPose();
			tf2::Vector3 _offset = convertGeoToENU(leader, follower);
			offset_.push_back(_offset);
			ROS_INFO_STREAM("offset_[" << i << "] = " << offset_[i].getX() << ", " << offset_[i].getY() << ", " << offset_[i].getZ());
			i++;
		}
	}
}

void SwarmVehicle::limit(tf2::Vector3 &v, const double &limit)
{
	if (v.length() > limit)
		v = v.normalize() * limit;
}

void SwarmVehicle::setVehicleGlobalPose()
{
	sensor_msgs::NavSatFix origin = camila_.front().getHomeGeo();
	for (auto &vehicle : camila_)
	{
		tf2::Vector3 vehicle_pose = convertGeoToENU(vehicle.getGeoPose(), origin);
		vehicle.setGlobalPose(vehicle_pose);
	}
}

void SwarmVehicle::calRepulsive(Vehicle &vehicle)
{
	tf2::Vector3 sum(0, 0, 0);
	unsigned int cnt = 0;

	for (auto &another_vehicle : camila_)
	{
		if (&vehicle != &another_vehicle)
		{
			tf2::Vector3 diff = vehicle.getGlobalPose() - another_vehicle.getGlobalPose();
			double dist = diff.length();
			if (dist < repulsive_range_)
			{
				if (diff.length() != 0)
					diff = diff.normalize();
				diff *= (repulsive_range_ / dist);
				sum += diff;
				cnt++;
			}
		}
	}
	if (cnt > 0)
	{
		sum /= cnt;
		limit(sum, max_speed_);
		vehicle.setSumOfRepulsive(sum);
	}
	else
	{
		sum.setZero();
		vehicle.setSumOfRepulsive(sum);
	}
}

void SwarmVehicle::calAttractive(Vehicle &vehicle)
{
	tf2::Vector3 err(0, 0, 0);
	geometry_msgs::PoseStamped target_pos = vehicle.getTargetLocal();
	geometry_msgs::PoseStamped current_pos = vehicle.getLocalPose();
	err.setX(target_pos.pose.position.x - current_pos.pose.position.x);
	err.setY(target_pos.pose.position.y - current_pos.pose.position.y);
	err.setZ(target_pos.pose.position.z - current_pos.pose.position.z);

	limit(err, max_speed_);
	vehicle.setErr(err);
}

void SwarmVehicle::formationGenerator()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.2 rad/s
	double angle, spacing;
	nh_global_.getParamCached("spacing", spacing);

	msg.pose.position.x = swarm_target_local_.getX();
	msg.pose.position.y = swarm_target_local_.getY();
	msg.pose.position.z = swarm_target_local_.getZ();

	if (formation_ == "POINT")
	{
		int i = 0;
		for (auto &vehicle : camila_)
		{
			if (vehicle.getInfo().vehicle_id_ != 1)
			{
				msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX();
				msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY();
				msg_f.pose.position.z = msg.pose.position.z;
				vehicle.setLocalTarget(msg_f);
			}
			else
				vehicle.setLocalTarget(msg);
			i++;
		}
	}
	else if (formation_ == "SINGLE"){
		;
	}
	else if (formation_ == "IDLE")
	{
		int i = 0;
		int x = 0 , y = -3;
		for (auto &vehicle : camila_)
		{
			if(y < 16)
				y += 3;
			else{
				x += 3;
				y = 0;
			}
			msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() + x;
			msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() + y;
			msg_f.pose.position.z = msg.pose.position.z;
			vehicle.setLocalTarget(msg_f);
			i++;
		}
	}
	else if (formation_ == "SCEN1")
	{
		int i = 0;
		for (auto &vehicle : camila_)
		{
			if (vehicle.getInfo().vehicle_id_ != 1)
			{
				angle = i * angle_;

				msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() + spacing * cos(angle + x);
				msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() + spacing * sin(angle + x);
				msg_f.pose.position.z = msg.pose.position.z;
				vehicle.setLocalTarget(msg_f);
			}
			else
				vehicle.setLocalTarget(msg);
			i++;
		}
	}
	else if (formation_ == "SCEN2")
	{
		scenario2();
	}
	else if (formation_ == "SCEN3")
	{
		scenario3();
	}
	else if (formation_ == "SCEN4")
	{
		scenario4();
	}
	else if (formation_ == "SCEN5")
	{
		scenario5();
	}
	else if (formation_ == "SCEN6")
	{
		scenario6();
	}
}

void SwarmVehicle::scenario2()
{
	geometry_msgs::PoseStamped temp;
	std::vector<geometry_msgs::PoseStamped> scen;
	scen.reserve(num_of_vehicle_);

	double scale = 1;
	int id = 1;
	ros::Time time = ros::Time::now();

	if (scen.size() == 0)
	{
		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 3) - 2) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + 6 * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 3) - 2) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + 6 * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 3) - 2) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + 6 * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 3) - 2) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + -6 * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 3) - 2) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + -6 * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 3) - 2) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + -6 * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //7
		temp.pose.position.x = swarm_target_local_.getX() + -4 * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (3 * (id % 3) - 3) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + -4 * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (3 * (id % 3) - 3) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + -4 * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (3 * (id % 3) - 3) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //10
		temp.pose.position.x = swarm_target_local_.getX() + 4 * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (3 * (id % 3) - 3) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + 4 * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (3 * (id % 3) - 3) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + 4 * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (3 * (id % 3) - 3) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //13
		temp.pose.position.x = swarm_target_local_.getX() + (-2 * (id % 2) - 5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) + 7) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //14
		temp.pose.position.x = swarm_target_local_.getX() + (-2 * (id % 2) - 5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) + 7) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 2) + 5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) + 7) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //16
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 2) + 5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) + 7) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 2) - 7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) - 9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //18
		temp.pose.position.x = swarm_target_local_.getX() + (2 * (id % 2) - 7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) - 9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-2 * (id % 2) + 7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) - 9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //20
		temp.pose.position.x = swarm_target_local_.getX() + (-2 * (id % 2) + 7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 0;
		temp.pose.position.z = swarm_target_local_.getZ() + (2 * (id % 2) - 9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-13) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (7) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-11) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-9) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (11) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (13) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //25
		temp.pose.position.x = swarm_target_local_.getX() + (-5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (15) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (13) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (7) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (11) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (9) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (11) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (13) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time; //30
		temp.pose.position.x = swarm_target_local_.getX() + (5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (15) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-13) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-7) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-11) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-9) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-11) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-13) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (-5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-15) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (5) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-15) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (7) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-13) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (9) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-11) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (11) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-9) * scale;
		scen.push_back(temp);
		id++;

		temp.header.stamp = time;
		temp.pose.position.x = swarm_target_local_.getX() + (13) * scale;
		temp.pose.position.y = swarm_target_local_.getY() + 5;
		temp.pose.position.z = swarm_target_local_.getZ() + (-7) * scale;
		scen.push_back(temp);
		id++;
	}
	int i = 0;
	for (auto &vehicle : camila_)
	{
		scen[i].pose.position.x += offset_[i].getX();
		scen[i].pose.position.y += offset_[i].getY();
		scen[i].pose.position.z += offset_[i].getZ();
		vehicle.setLocalTarget(scen[i]);
		i++;
	}
}

void SwarmVehicle::scenario3()
{
	// double spacing;
	// nh_global_.getParamCached("spacing", spacing);
	// nh_global_.getParamCached("scen_num", scen_num_);
	// std::vector<std::pair<int, int>> scen;
	// std::vector<std::pair<int, int>>::iterator iter;
	// scen.reserve(num_of_vehicle_);

	// int i = 0;
	// for (auto line : FONT[scen_num_])
	// {
	// 	uint8_t left_bits, right_bits;
	// 	left_bits = line >> 4;
	// 	right_bits = 0x0f & line;

	// 	hexToCoord(scen, left_bits, 7 - i, true);
	// 	hexToCoord(scen, right_bits, 7 - i, false);
	// 	i++;
	// 	if(scen.size() > num_of_vehicle_)
	// 		break;
	// }
	// while(scen.size() > num_of_vehicle_)
	// 	scen.pop_back();true;
	// if (scen_size > num_of_vehicle_)
	// {
	// 	int i = 0;
	// 	for (auto &vehicle : camila_)
	// 	{
	// 		int min_num = 0;
	// 		int min_dist = 150;
	// 		int k = 0;
	// 		std::pair<int, int> prev_scen = vehicle.getScenPos();
	// 		for (iter = scen.begin(); iter != scen.end(); iter++)
	// 		{
	// 			int a, b, length;
	// 			a = prev_scen.first - iter->first;
	// 			b = prev_scen.second - iter->second;
	// 			length = a * a + b * b;
	// 			if (length < min_dist)
	// 			{
	// 				min_dist = length;
	// 				min_num = k;
	// 			}
	// 			k++;
	// 		}
	// 		iter = scen.begin() + min_num;
	// 		temp.header.stamp = ros::Time::now();
	// 		temp.pose.position.x = swarm_target_local_.getX() + offset_[i].getX() + iter->first * spacing;
	// 		temp.pose.position.y = swarm_target_local_.getY() + offset_[i].getY();
	// 		temp.pose.position.z = swarm_target_local_.getZ() + offset_[i].getZ() + iter->second * spacing;
	// 		vehicle.setScenPos(*iter);
	// 		vehicle.setLocalTarget(temp);
	// 		scen.erase(iter);
	// 		i++;
	// 	}
	// }
	// else
	// {
	// 	int i = 0;
	// 	for (auto &vehicle : camila_)
	// 	{
	// 		if (scen.size() == 0)
	// 			break;
	// 		int min_num = 0;
	// 		int min_dist = 150;
	// 		int k = 0;
	// 		std::pair<int, int> prev_scen = vehicle.getScenPos();
	// 		for (iter = scen.begin(); iter != scen.end(); iter++)
	// 		{
	// 			int a, b, length;
	// 			a = prev_scen.first - iter->first;
	// 			b = prev_scen.second - iter->second;
	// 			length = a * a + b * b;
	// 			if (length < min_dist)
	// 			{
	// 				min_dist = length;
	// 				min_num = k;
	// 			}
	// 			k++;
	// 		}
	// 		iter = scen.begin() + min_num;
	// 		temp.header.stamp = ros::Time::now();
	// 		temp.pose.position.x = swarm_target_local_.getX() + offset_[i].getX() + iter->first * spacing;
	// 		temp.pose.position.y = swarm_target_local_.getY() + offset_[i].getY();
	// 		temp.pose.position.z = swarm_target_local_.getZ() + offset_[i].getZ() + iter->second * spacing;
	// 		vehicle.setScenPos(*iter);
	// 		vehicle.setLocalTarget(temp);
	// 		scen.erase(iter);
	// 		i++;
	// 	}
	// 	for (int j = scen_size; j < num_of_vehicle_; j++)
	// 	{
	// 		temp.header.stamp = ros::Time::now();
	// 		temp.pose.position.x = swarm_target_local_.getX() + offset_[j].getX() + (j - scen_size) * spacing;
	// 		temp.pose.position.y = swarm_target_local_.getY() + offset_[j].getY() - 10;
	// 		temp.pose.position.z = swarm_target_local_.getZ() + offset_[j].getZ() - 2;
	// 		camila_[j].setLocalTarget(temp);
	// 	}
	// }
	std::string scen_str;
	double spacing;
	nh_global_.getParamCached("scen", scen_str);
	nh_global_.getParamCached("spacing", spacing);
	std::vector<std::pair<int, int>> scen;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	scen.reserve(num_of_vehicle_);

	if(scen_hex_.size() == 0 || scen_str_ != scen_str)
	{
		scen_hex_.clear();
		scen_str_ = scen_str;
		for(auto &character : scen_str_){
			int scen_num = static_cast<int>(character);
			for(auto &line : FONT[scen_num]){
				scen_hex_.push_back(line);
			}
		}
		prev_ = ros::Time::now();
	}
	else{
		if(ros::Time::now() > prev_ + ros::Duration(35.0)){
			prev_ = ros::Time::now();
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
		}
	}
	
	int t = 0;
	for (auto & hex : scen_hex_)
	{
		uint8_t left_bits, right_bits;
		left_bits = hex >> 4;
		right_bits = 0x0f & hex;

		hexToCoord(scen, left_bits, 8 - t, true);
		hexToCoord(scen, right_bits, 8 - t, false);
		t++;
	}

	while(scen.size() > num_of_vehicle_)
		scen.pop_back();

	geometry_msgs::PoseStamped temp;
	int scen_size = scen.size();
	int i = 0, j=0;
	for (auto &vehicle : camila_)
	{
		if (scen.size() == 0){
			hexToCoord(scen, 0x0f, -3-j, true);
			hexToCoord(scen, 0x0f, -3-j, false);
			j++;
		}
		int min_num = 0;
		int min_dist = 350;
		int k = 0;
		std::pair<int, int> prev_scen = vehicle.getScenPos();
		for (iter = scen.begin(); iter != scen.end(); iter++)
		{
			int a, b, length;
			a = prev_scen.first - iter->first;
			b = prev_scen.second - iter->second;
			length = a * a + b * b;
			if (length < min_dist)
			{
				min_dist = length;
				min_num = k;
			}
			k++;
		}
		iter = scen.begin() + min_num;
		temp.header.stamp = ros::Time::now();
		temp.pose.position.x = swarm_target_local_.getX() + offset_[i].getX() + iter->first * spacing;
		temp.pose.position.y = swarm_target_local_.getY() + offset_[i].getY();
		temp.pose.position.z = swarm_target_local_.getZ() + offset_[i].getZ() + iter->second * spacing;
		vehicle.setScenPos(*iter);
		vehicle.setLocalTarget(temp);
		scen.erase(iter);
		i++;
	}
}

void SwarmVehicle::scenario4()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double m_sec = ros::Time::now().toNSec() / 1000000;
	double x = 0.00006 * m_sec; // 0.2 rad/s
	double angle, spacing;
	nh_global_.getParamCached("spacing", spacing);

	msg.pose.position.x = swarm_target_local_.getX();
	msg.pose.position.y = swarm_target_local_.getY();
	msg.pose.position.z = swarm_target_local_.getZ();

	int i = 0;
	for (auto &vehicle : camila_)
	{
		if (vehicle.getInfo().vehicle_id_ != 1)
		{
			angle = i * angle_;
			msg_f.header.stamp = ros::Time::now();
			msg_f.pose.position.x = msg.pose.position.x + offset_[i].getX() + spacing * cos(angle + x);
			msg_f.pose.position.y = msg.pose.position.y + offset_[i].getY() + spacing * sin(angle + x);
			msg_f.pose.position.z = msg.pose.position.z + offset_[i].getZ() + spacing * cos(angle + x) * x;
			vehicle.setLocalTarget(msg_f);
		}
		else
		{
			msg.header.stamp = ros::Time::now();
			vehicle.setLocalTarget(msg);
		}
		i++;
	}
}

void SwarmVehicle::scenario5()
{
	std::string scen_str;
	double spacing;
	nh_global_.getParamCached("scen", scen_str);
	nh_global_.getParamCached("spacing", spacing);
	std::vector<std::pair<int, int>> scen;
	std::vector<std::pair<int, int>>::iterator iter;
	std::vector<uint8_t>::iterator iter_uint8;
	scen.reserve(num_of_vehicle_);

	if(scen_hex_.size() == 0 || scen_str_ != scen_str)
	{
		scen_hex_.clear();
		scen_str_ = scen_str;
		for(auto &character : scen_str_){
			int scen_num = static_cast<int>(character);
			for(auto &line : FONT2[scen_num]){
				scen_hex_.push_back(line);
			}
		}
		prev_ = ros::Time::now();
	}
	else{
		if(ros::Time::now() > prev_ + ros::Duration(20.0)){
			prev_ = ros::Time::now();
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
			iter_uint8 = scen_hex_.begin();
			scen_hex_.erase(iter_uint8);
		}
	}
	int t = 0;
	for (auto & hex : scen_hex_)
	{
		uint8_t left_bits, right_bits;
		left_bits = hex >> 4;
		right_bits = 0x0f & hex;

		hexToCoord(scen, left_bits, 5 - t, false);
		hexToCoord(scen, right_bits, 5 - t, true);
		t++;
	}

	while(scen.size() > num_of_vehicle_)
		scen.pop_back();

	geometry_msgs::PoseStamped temp;
	int scen_size = scen.size();
	int i = 0, j=0;
	for (auto &vehicle : camila_)
	{
		if (scen.size() == 0){
			hexToCoord(scen, 0x0f, -3-j, true);
			hexToCoord(scen, 0x0f, -3-j, false);
			j++;
		}
		int min_num = 0;
		int min_dist = 350;
		int k = 0;
		std::pair<int, int> prev_scen = vehicle.getScenPos();
		for (iter = scen.begin(); iter != scen.end(); iter++)
		{
			int a, b, length;
			a = prev_scen.first - iter->first;
			b = prev_scen.second - iter->second;
			length = a * a + b * b;
			if (length < min_dist)
			{
				min_dist = length;
				min_num = k;
			}
			k++;
		}
		iter = scen.begin() + min_num;
		temp.header.stamp = ros::Time::now();
		temp.pose.position.x = swarm_target_local_.getX() + offset_[i].getX() + iter->first * spacing;
		temp.pose.position.y = swarm_target_local_.getY() + offset_[i].getY();
		temp.pose.position.z = swarm_target_local_.getZ() + offset_[i].getZ() + iter->second * spacing;
		vehicle.setScenPos(*iter);
		vehicle.setLocalTarget(temp);
		scen.erase(iter);
		i++;
	}
}

void SwarmVehicle::scenario6()
{
	geometry_msgs::PoseStamped msg, msg_f;
	double sec = ros::Time::now().toSec();
	double spacing, distance_x, distance_y;
	bool start;
	nh_global_.getParamCached("spacing", spacing);
	nh_global_.getParamCached("distance_x", distance_x);
	nh_global_.getParamCached("distance_y", distance_y);
	nh_global_.getParamCached("scen6", start);
	msg.pose.position.x = swarm_target_local_.getX();
	msg.pose.position.y = swarm_target_local_.getY();
	msg.pose.position.z = swarm_target_local_.getZ();
	int j = 0;
	for (auto &vehicle : camila_)
	{
		if (vehicle.getInfo().vehicle_id_ != 1)
		{
			int i = vehicle.getInfo().vehicle_id_;
			
			msg_f.header.stamp = ros::Time::now();
			msg_f.pose.position.x = swarm_target_local_.getX() + offset_[j].getX() + (i%2 - 0.5) * distance_x;
			msg_f.pose.position.y = swarm_target_local_.getY() + offset_[j].getY() + int(i/2) * distance_y;
			msg_f.pose.position.z = swarm_target_local_.getZ() + offset_[j].getZ();
			vehicle.setLocalTarget(msg_f);
		}
		else
		{
			msg.header.stamp = ros::Time::now();
			if(start == true){
				if(msg.pose.position.y <= 20)
					msg.pose.position.y += int(sec) * 3;
				else{
					msg.pose.position.y -= int(sec) * 3;
				}
			}
			vehicle.setLocalTarget(msg);
		}
		j++;
	}
}

void SwarmVehicle::hexToCoord(std::vector<std::pair<int, int>> &scen, const uint8_t &hex, const int &x_value, const bool &is_left_bits)
{
	int offset, a, b, c, d;
	if (is_left_bits)
		offset = 0;
	else
		offset = 4;

	if(formation_ == "SCEN3"){
		a=3;
		b=2;
		c=1;
		d=0;
	}
	else if(formation_ == "SCEN5"){
		a=0;
		b=1;
		c=2;
		d=3;
	}

	switch (hex)
	{
	case 1:
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 2:
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 3:
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 4:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		break;
	case 5:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 6:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 7:
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 8:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		break;
	case 9:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 10:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 11:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 12:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		break;
	case 13:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;
	case 14:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		break;
	case 15:
		scen.push_back(std::pair<int, int>(x_value, d + offset));
		scen.push_back(std::pair<int, int>(x_value, c + offset));
		scen.push_back(std::pair<int, int>(x_value, b + offset));
		scen.push_back(std::pair<int, int>(x_value, a + offset));
		break;

	default:
		break;
	}
}

bool SwarmVehicle::multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request &req,
									  swarm_ctrl_pkg::srvMultiSetpointLocal::Response &res)
{
	updateOffset();

	formation_ = req.formation;
	swarm_target_local_.setX(req.x);
	swarm_target_local_.setY(req.y);
	swarm_target_local_.setZ(req.z);

	target_changed_flag_ = true;

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
	int id = req.num_drone -1;
	geometry_msgs::PoseStamped msg;
	updateOffset();

	if (req.num_drone > 0 && req.num_drone <= num_of_vehicle_)
	{
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = req.x + offset_[id].getX();
		msg.pose.position.y = req.y + offset_[id].getY();
		msg.pose.position.z = req.z;
	}

	camila_[id].setLocalTarget(msg);
	res.success = true;
	return res.success;
}

tf2::Vector3 SwarmVehicle::convertGeoToENU(const sensor_msgs::NavSatFix &coord,
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

	tf2::Vector3 offset;

	offset.setX(k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH);
	offset.setY(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH);
	offset.setZ(coord.altitude - home.altitude);

	return offset;
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

void SwarmVehicle::setSwarmInfo(const std::string &swarm_name, const int &num_of_vehicle)
{
	swarm_name_ = swarm_name;
	num_of_vehicle_ = num_of_vehicle;

	release(); // service 및 camila vector 초기화

	camila_.reserve(num_of_vehicle_);
	VehicleInfo vehicle_info_[num_of_vehicle_];

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1);
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}

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
			iter_ = camila_.erase(iter_);
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
	bool use_repulsive_force;
	nh_global_.getParamCached("local_plan/max_speed", max_speed_);
	nh_global_.getParamCached("local_plan/repulsive_range", repulsive_range_);
	nh_global_.getParamCached("local_plan/use_repulsive_force", use_repulsive_force);
	setVehicleGlobalPose();
	for (auto &vehicle : camila_)
	{
		tf2::Vector3 setpoint;
		calAttractive(vehicle);
		if (use_repulsive_force)
		{
			vehicle.setLocalPlanner(PotentialField::getInstance());
			calRepulsive(vehicle);
		}
		else
			vehicle.setLocalPlanner(AttractiveOnly::getInstance());
		vehicle.goTo();
	}
	formationGenerator();
}