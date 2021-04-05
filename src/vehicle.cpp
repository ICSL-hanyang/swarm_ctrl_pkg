#include <ros/ros.h>
#include <vehicle.h>
#include <Font7x5.h>
#include <Font8x8.h>

PoseController::PoseController(ros::NodeHandle &nh, ros::NodeHandle &nh_global,VehicleInfo &vehicle_info) : nh_(nh), nh_global_(nh_global), vehicle_info_(vehicle_info), setpoint_publish_flag_(false)
{}

GeoPoseController::GeoPoseController(ros::NodeHandle &nh, ros::NodeHandle &nh_global, VehicleInfo &vehicle_info) : PoseController(nh, nh_global, vehicle_info)
{
	setpoint_geo_pub_ = nh_.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
	home_sub_ = nh_.subscribe("mavros/home_position/home", 10, &GeoPoseController::homeCB, this);
	cur_pose_sub_ = nh_.subscribe("mavros/global_position/global", 10, &GeoPoseController::curPoseCB, this);
	set_home_client_ = nh_.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");
}

GeoPoseController::~GeoPoseController(){
	setpoint_geo_pub_.shutdown();
	home_sub_.shutdown();
	cur_pose_sub_.shutdown();
	set_home_client_.shutdown();
}

void GeoPoseController::homeCB(const mavros_msgs::HomePosition::ConstPtr &msg)
{
	home_.pose.position.latitude = msg->geo.latitude;
	home_.pose.position.longitude = msg->geo.longitude;
	home_.pose.position.altitude = msg->geo.altitude;
	home_.pose.orientation.x = msg->orientation.x;
	home_.pose.orientation.y = msg->orientation.y;
	home_.pose.orientation.z = msg->orientation.z;
	home_.pose.orientation.w = msg->orientation.w;
}

void GeoPoseController::curPoseCB(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
	cur_pose_.pose.position.latitude = msg->latitude;
	cur_pose_.pose.position.longitude = msg->latitude;
	cur_pose_.pose.position.altitude = msg->altitude;
}

bool GeoPoseController::setHome(){
	mavros_msgs::CommandHome msg;
	msg.request.current_gps = false;
	msg.request.latitude = cur_pose_.pose.position.latitude;
	msg.request.longitude = cur_pose_.pose.position.longitude;
	msg.request.altitude = cur_pose_.pose.position.altitude;

	if (set_home_client_.call(msg) && msg.response.success)
	{
		home_ = cur_pose_;

		ROS_INFO_STREAM(getName() << " Global home position is set. " << msg.response.result);
		ROS_INFO("%s new global home is (long : %lf, lang : %lf, alti : %lf)", getName().c_str(),
				 home_.pose.position.longitude, home_.pose.position.latitude, home_.pose.position.altitude);
	}
	else
		ROS_ERROR_STREAM(getName() << " Failed to set global home position. " << msg.response.result);

	return msg.response.success;
}

void GeoPoseController::setTatget(const geographic_msgs::GeoPoseStamped &target){
	//현재 Pose 중에서 Position에 대한 변화가 있을 때만 타겟을 변경하므로 Position은 그대로인 상태에서 Orientation만 바뀌는 경우 업데이트가 안되는 문제가 있음 
	//다양한 문제 해결 방법중에 메세지에서 pose의 내용만 비교하는 함수를 만들어 볼까 함
	setpoint_publish_flag_ = true;

	if ((target_.pose.position.latitude != target.pose.position.latitude) ||
		(target_.pose.position.longitude != target.pose.position.longitude) ||
		(target_.pose.position.altitude != target.pose.position.altitude))
	{
		ROS_INFO("%s set target_global_pos(long : %lf, lati : %lf, alti : %lf)", getName().c_str(),
				 target_.pose.position.longitude, target_.pose.position.latitude, target_.pose.position.altitude);
		target_ = target;
	}
}

void GeoPoseController::goTo(){
	target_.header.seq += 1;
	target_.header.stamp = ros::Time::now();
	target_.header.frame_id = getName();

	setpoint_geo_pub_.publish(target_);
}

LocalPoseController::LocalPoseController(ros::NodeHandle &nh, ros::NodeHandle &nh_global, VehicleInfo &vehicle_info) : PoseController(nh, nh_global, vehicle_info)
{
	setpoint_local_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	cur_pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &LocalPoseController::curPoseCB, this);
}

LocalPoseController::~LocalPoseController(){
	setpoint_local_pub_.shutdown();
	cur_pose_sub_.shutdown();
}

void LocalPoseController::setTatget(const geometry_msgs::PoseStamped &target){
	//현재 Pose 중에서 Position에 대한 변화가 있을 때만 타겟을 변경하므로 Position은 그대로인 상태에서 Orientation만 바뀌는 경우 업데이트가 안되는 문제가 있음 
	setpoint_publish_flag_ = true;

	if ((target_.pose.position.x != target.pose.position.x) ||
		(target_.pose.position.y != target.pose.position.y) ||
		(target_.pose.position.z != target.pose.position.z))
		target_ = target;
}

void LocalPoseController::goTo(){
	target_.header.seq += 1;
	target_.header.stamp = ros::Time::now();
	target_.header.frame_id = getName();

	setpoint_local_pub_.publish(target_);	
}

LocalVelocityController::LocalVelocityController(ros::NodeHandle &nh, ros::NodeHandle &nh_global, VehicleInfo &vehicle_info) : PoseController(nh, nh_global, vehicle_info)
{
	setpoint_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	cur_pose_sub_ = nh_.subscribe("mavros/local_position/pose", 10, &LocalVelocityController::curPoseCB, this);
}

LocalVelocityController::~LocalVelocityController()
{
	setpoint_vel_pub_.shutdown();
}

void LocalVelocityController::goTo(){
	double kp_vel;
	nh_global_.getParam("pid/kp", kp_vel);
	geometry_msgs::Twist vel;

	vel.linear.x = (target_.pose.position.x - cur_pose_.pose.position.x) * kp_vel;
	vel.linear.y = (target_.pose.position.y - cur_pose_.pose.position.y) * kp_vel;
	vel.linear.z = (target_.pose.position.z - cur_pose_.pose.position.z) * kp_vel;

	setpoint_vel_pub_.publish(vel);
}

void LocalVelocityController::setTatget(const geometry_msgs::PoseStamped &target){
	setpoint_publish_flag_ = true;

	if ((target_.pose.position.x != target.pose.position.x) ||
		(target_.pose.position.y != target.pose.position.y) ||
		(target_.pose.position.z != target.pose.position.z))
		target_ = target;
}

double LocalPlanner::kp_attractive_;
double LocalPlanner::kp_repulsive_;

LocalPlanner::LocalPlanner(ros::NodeHandle &nh_global) : nh_global_(nh_global)
{}

tf2::Vector3 LocalPlanner::generate(){
	nh_global_.getParamCached("local_plan/kp_attractive", kp_attractive_);
	local_plan_ = err_ * kp_attractive_;
}

PFLocalPlanner::PFLocalPlanner(ros::NodeHandle &nh_global) : LocalPlanner(nh_global)
{}

tf2::Vector3 PFLocalPlanner::generate(){
	nh_global_.getParamCached("local_plan/kp_attractive", kp_attractive_);
	local_plan_ = sum_repulsive_ * kp_repulsive_ + err_ * kp_attractive_;
}

Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global)
	: vehicle_info_({1, "mavros"}),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(nh_mul),
	  nh_global_(nh_global),
	  scen_pos_(std::pair<int, int>(0, 0))
{
	vehicleInit();
}

Vehicle::Vehicle(ros::NodeHandle &nh_mul, ros::NodeHandle &nh_global, const VehicleInfo &vehicle_info)
	: vehicle_info_(vehicle_info),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(nh_mul),
	  nh_global_(nh_global),
	  scen_pos_(std::pair<int, int>(0, 0))
{
	vehicleInit();
}

Vehicle::Vehicle(const Vehicle &rhs)
	: vehicle_info_(rhs.vehicle_info_),
	  nh_(ros::NodeHandle(vehicle_info_.vehicle_name_)),
	  nh_mul_(rhs.nh_mul_),
	  nh_global_(rhs.nh_global_),
	  scen_pos_(std::pair<int, int>(0, 0))
{
	vehicleInit();
	controllers_.reserve(3);
	local_planners_.reserve(2);
	std::vector<PoseController*>::const_iterator it_cont;
	std::vector<LocalPlanner>::const_iterator it_planner;
	for(it_cont = rhs.controllers_.begin(); it_cont !=rhs.controllers_.end(); it_cont++)
		controllers_.push_back(*it_cont);
	
	for(it_planner = rhs.local_planners_.begin(); it_planner !=rhs.local_planners_.end();it_planner++)
		local_planners_.push_back(*it_planner);

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
	controllers_.reserve(3);
	local_planners_.reserve(2);
	std::vector<PoseController*>::const_iterator it_cont;
	std::vector<LocalPlanner>::const_iterator it_planner;
	for(it_cont = rhs.controllers_.begin(); it_cont !=rhs.controllers_.end(); it_cont++)
		controllers_.push_back(*it_cont);
	
	for(it_planner = rhs.local_planners_.begin(); it_planner !=rhs.local_planners_.end();it_planner++)
		local_planners_.push_back(*it_planner);
		
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
	takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

	multi_arming_sub_ = nh_mul_.subscribe("arming", 10, &Vehicle::multiArming, this);
	multi_set_mode_sub_ = nh_mul_.subscribe("set_mode", 10, &Vehicle::multiSetMode, this);
	multi_set_home_sub_ = nh_mul_.subscribe("set_home", 10, &Vehicle::multiSetHome, this);
	multi_takeoff_sub_ = nh_mul_.subscribe("takeoff", 10, &Vehicle::multiTakeoff, this);
	multi_land_sub_ = nh_mul_.subscribe("land", 10, &Vehicle::multiLand, this);

	controllers_.reserve(3);
	controllers_.push_back(new GeoPoseController(nh_, nh_global_, vehicle_info_));	
	controllers_.push_back(new LocalPoseController(nh_, nh_global_, vehicle_info_));	
	controllers_.push_back(new LocalVelocityController(nh_, nh_global_, vehicle_info_));	

	controller_ptr_ = controllers_[1];

	local_planners_.reserve(2);
	local_planners_.push_back(LocalPlanner(nh_global_));
	local_planners_.push_back(PFLocalPlanner(nh_global_));
	lp_ptr_ = &local_planners_[1];

	ROS_INFO_STREAM(vehicle_info_.vehicle_name_ << " instance generated");
}

void Vehicle::release()
{
	state_sub_.shutdown();
	battery_sub_.shutdown();

	arming_client_.shutdown();
	set_mode_client_.shutdown();
	takeoff_client_.shutdown();
	land_client_.shutdown();

	multi_arming_sub_.shutdown();
	multi_set_mode_sub_.shutdown();
	multi_set_home_sub_.shutdown();
	multi_takeoff_sub_.shutdown();
	multi_land_sub_.shutdown();

	std::vector<PoseController*>().swap(controllers_);
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
	Controllers controller = Controllers::GEO_POSE_CONTROLLER;
	setController(controller);
	GeoPoseController* GPcontroller_ptr = dynamic_cast<GeoPoseController*>(controller_ptr_);
	if(GPcontroller_ptr == nullptr){
		ROS_ERROR("Fail to change PoseController to GeoPoseController");
	}
	else{
		GPcontroller_ptr->setHome();
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

	release();

	nh_ = ros::NodeHandle(vehicle_info_.vehicle_name_);

	vehicleInit();
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
	if (((current_mode == "offboard") || (current_mode == "OFFBOARD")) && (!controller_ptr_->isPublished()))
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
	Controllers controller = Controllers::GEO_POSE_CONTROLLER;
	setController(controller);
	GeoPoseController* GPcontroller_ptr = dynamic_cast<GeoPoseController*>(controller_ptr_);
	if(GPcontroller_ptr == nullptr){
		ROS_ERROR("Fail to change PoseController to GeoPoseController");
		return false;
	}
	else{
		auto home = GPcontroller_ptr->getHome();
		auto cur_pose = GPcontroller_ptr->getCurPose();
		mavros_msgs::CommandTOL msg;
		msg.request.min_pitch = 0;
		msg.request.yaw = 0;
		msg.request.latitude = cur_pose.pose.position.latitude;
		msg.request.longitude = cur_pose.pose.position.longitude;
		msg.request.altitude = home.pose.position.altitude + takeoff_alt;
		if (takeoff_client_.call(msg) && msg.response.success)
			ROS_INFO_STREAM(msg.response.result);
		else
			ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call takeoff service. " << msg.response.result);
		return msg.response.success;
	}
}

bool Vehicle::land()
{
	Controllers controller = Controllers::GEO_POSE_CONTROLLER;
	setController(controller);
	GeoPoseController* GPcontroller_ptr = dynamic_cast<GeoPoseController*>(controller_ptr_);
	if(GPcontroller_ptr == nullptr){
		ROS_ERROR("Fail to change PoseController to GeoPoseController");
		return false;
	}
	else{
		auto cur_pose = GPcontroller_ptr->getCurPose();
		mavros_msgs::CommandTOL msg;
		msg.request.min_pitch = 0;
		msg.request.yaw = 0;
		msg.request.latitude = cur_pose.pose.position.latitude;
		msg.request.longitude = cur_pose.pose.position.longitude;
		msg.request.altitude = 0;

		if (land_client_.call(msg) && msg.response.success)
			ROS_INFO_STREAM(msg.response.result);
		else
			ROS_ERROR_STREAM(vehicle_info_.vehicle_name_ << " failed to call land service. " << msg.response.result);
		return msg.response.success;
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

geographic_msgs::GeoPoseStamped Vehicle::getHome(){
	Controllers controller = Controllers::GEO_POSE_CONTROLLER;
	setController(controller);
	GeoPoseController* controller_ptr = dynamic_cast<GeoPoseController*>(controller_ptr_);
	if(controller_ptr == nullptr){
		ROS_ERROR("Fail to change PoseController to GeoPoseController");
		geographic_msgs::GeoPoseStamped msg;
		return msg;
	}
	else{
		return controller_ptr->getHome();
	}
}

void Vehicle::setGlobalPose(const tf2::Vector3 &global_pose){
	for(auto lp : local_planners_)
		lp.setGlobalPose(global_pose);
}

void Vehicle::setErr(const tf2::Vector3 &err){
	for(auto lp : local_planners_)
		lp.setErr(err);
}

void Vehicle::setSumOfRepulsive(const tf2::Vector3 &sum_of_repulsive){
	setLocalPlanner(LocalPlanners::PF_LOCAL_PLANNER);
	PFLocalPlanner* planner = dynamic_cast<PFLocalPlanner*>(lp_ptr_);
	planner->setSumOfRepulsive(sum_of_repulsive);
}

tf2::Vector3 Vehicle::getSumOfRepulsive(){
	setLocalPlanner(LocalPlanners::PF_LOCAL_PLANNER);
	PFLocalPlanner* planner = dynamic_cast<PFLocalPlanner*>(lp_ptr_);
	return planner->getSumOfRepulsive();
}

void Vehicle::goTo(const Controllers &controller){
	tf2::Vector3 local_plan = lp_ptr_->generate();

	setController(controller);
	if (controller==Controllers::LOCAL_POSE_CONTROLLER){
		LocalPoseController* lp_controller_ptr = dynamic_cast<LocalPoseController*>(controller_ptr_);
		auto cur_pose = lp_controller_ptr->getCurPose();
		geometry_msgs::PoseStamped target;
		target.pose.position.x = cur_pose.pose.position.x + local_plan.getX();
		target.pose.position.y = cur_pose.pose.position.y + local_plan.getY();
		target.pose.position.z = cur_pose.pose.position.z + local_plan.getZ();
		lp_controller_ptr->setTatget(target);
	}
	else if(controller == Controllers::LOCAL_VELOCITY_CONTROLLER){
		LocalVelocityController* lv_controller_ptr = dynamic_cast<LocalVelocityController*>(controller_ptr_);
		auto cur_pose = lv_controller_ptr->getCurPose();
		geometry_msgs::PoseStamped target;
		target.pose.position.x = cur_pose.pose.position.x + local_plan.getX();
		target.pose.position.y = cur_pose.pose.position.y + local_plan.getY();
		target.pose.position.z = cur_pose.pose.position.z + local_plan.getZ();
		lv_controller_ptr->setTatget(target);
	}
}

double SwarmVehicle::repulsive_range_;
double SwarmVehicle::max_speed_;

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
	offset_.reserve(num_of_vehicle_);
	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1);
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}
	goto_vehicle_server_ = nh_global_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);
}

SwarmVehicle::SwarmVehicle(const SwarmVehicle &rhs)
	: swarm_name_(rhs.swarm_name_),
	  num_of_vehicle_(rhs.num_of_vehicle_),
	  nh_(rhs.nh_),
	  nh_mul_(rhs.nh_mul_),
	  nh_global_(rhs.nh_global_),
	  multi_setpoint_publish_flag_(rhs.multi_setpoint_publish_flag_)
{
	VehicleInfo vehicle_info_[num_of_vehicle_];
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);

	camila_ = rhs.camila_;
	
	nh_ = rhs.nh_;
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;
	goto_vehicle_server_ = nh_global_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);

	*this = rhs;
}

const SwarmVehicle &SwarmVehicle::operator=(const SwarmVehicle &rhs)
{
	if (this == &rhs)
		return *this;

	swarm_name_ = rhs.swarm_name_;
	num_of_vehicle_ = rhs.num_of_vehicle_;

	std::vector<Vehicle>().swap(camila_);
	std::vector<tf2::Vector3>().swap(offset_);
	camila_.reserve(num_of_vehicle_);
	offset_.reserve(num_of_vehicle_);

	camila_ = rhs.camila_;

	nh_ = rhs.nh_;
	nh_mul_ = rhs.nh_mul_;
	nh_global_ = rhs.nh_global_;
	goto_vehicle_server_ = nh_global_.advertiseService("goto_vehicle", &SwarmVehicle::gotoVehicle, this);

	return *this;
}

void SwarmVehicle::release()
{
	goto_vehicle_server_.shutdown();
	std::vector<Vehicle>().swap(camila_);
	std::vector<tf2::Vector3>().swap(offset_);
}

SwarmVehicle::~SwarmVehicle()
{
	release();
}

void SwarmVehicle::limit(tf2::Vector3 &v, const double &limit)
{
	if (v.length() > limit)
		v = v.normalize() * limit;
}

void SwarmVehicle::setVehicleGlobalPose()
{
	geographic_msgs::GeoPoseStamped origin = camila_.front().getHome();
	for (auto &vehicle : camila_)
	{
		auto GPcontroller_ptr = dynamic_cast<GeoPoseController*>(vehicle.getController());
		tf2::Vector3 vehicle_pos = convertGeoToENU(GPcontroller_ptr->getCurPose(), origin);
		vehicle.setGlobalPose(vehicle_pos);
	}
}

void SwarmVehicle::calRepulsive(Vehicle &vehicle)
{
	tf2::Vector3 sum;
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
		sum /= cnt;
	vehicle.setSumOfRepulsive(sum);
}

void SwarmVehicle::calAttractive(Vehicle &vehicle)
{
	tf2::Vector3 err(0, 0, 0);
	auto LPcontroller_ptr = dynamic_cast<LocalPoseController*>(vehicle.getController());
	geometry_msgs::PoseStamped target_pos = LPcontroller_ptr->getTatget();
	geometry_msgs::PoseStamped current_pos = LPcontroller_ptr->getCurPose();
	err.setX(target_pos.pose.position.x - current_pos.pose.position.x);
	err.setY(target_pos.pose.position.y - current_pos.pose.position.y);
	err.setZ(target_pos.pose.position.z - current_pos.pose.position.z);

	vehicle.setErr(err);
}

bool SwarmVehicle::gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request &req,
							   swarm_ctrl_pkg::srvGoToVehicle::Response &res)
{
	geometry_msgs::PoseStamped msg;
	auto vehicle = camila_[req.num_drone - 1];

	updateOffset();

	if (req.num_drone > 0 && req.num_drone <= num_of_vehicle_)
	{
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = req.x + offset_[req.num_drone - 1].getX();
		msg.pose.position.y = req.y + offset_[req.num_drone - 1].getY();
		msg.pose.position.z = req.z;
	}
	auto LPcontroller_ptr = dynamic_cast<LocalPoseController*>(vehicle.getController());
	LPcontroller_ptr->setTatget(msg);

	return true;
}

tf2::Vector3 SwarmVehicle::convertGeoToENU(const geographic_msgs::GeoPoseStamped &coord,
										   const geographic_msgs::GeoPoseStamped &home)
{
	static const float epsilon = std::numeric_limits<double>::epsilon();

	double lat_rad = coord.pose.position.latitude * M_DEG_TO_RAD;
	double lon_rad = coord.pose.position.longitude * M_DEG_TO_RAD;

	double ref_lon_rad = home.pose.position.longitude * M_DEG_TO_RAD;
	double ref_lat_rad = home.pose.position.latitude * M_DEG_TO_RAD;

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
	offset.setZ(coord.pose.position.altitude - home.pose.position.altitude);

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

bool SwarmVehicle::isPublish()
{
	multi_setpoint_publish_flag_ = true;
	for (auto &vehicle : camila_)
	{
		auto controller_ptr = vehicle.getController();
		if (controller_ptr->isPublished() != true)
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

	release(); // service 및 camila vector 초기화

	camila_.reserve(num_of_vehicle_);
	VehicleInfo vehicle_info_[num_of_vehicle_];

	for (int i = 0; i < num_of_vehicle_; i++)
	{
		vehicle_info_[i].vehicle_id_ = i + 1;
		vehicle_info_[i].vehicle_name_ = swarm_name_ + std::to_string(i + 1);
		camila_.push_back(Vehicle(nh_mul_, nh_global_, vehicle_info_[i]));
	}
}

std::string SwarmVehicle::getSwarmInfo() const
{
	return swarm_name_;
}

const std::vector<Vehicle> *SwarmVehicle::getSwarmVehicle() const
{
	return &camila_;
}

const std::vector<tf2::Vector3> *SwarmVehicle::getSwarmOffset() const
{
	return &offset_;
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

void SwarmVehicle::updateOffset()
{
	geographic_msgs::GeoPoseStamped leader = camila_.front().getHome();

	int i = 0;
	if (offset_.empty())
	{
		for (auto &vehicle : camila_)
		{
			vehicle.setController(Controllers::GEO_POSE_CONTROLLER);
			auto controller_ptr = dynamic_cast<GeoPoseController*>(vehicle.getController());
			geographic_msgs::GeoPoseStamped follower = controller_ptr->getCurPose();
			tf2::Vector3 offset = convertGeoToENU(leader, follower);
			offset_.push_back(offset);
			ROS_INFO_STREAM("offset_[" << i << "] = " << offset_[i].getX() << ", " << offset_[i].getY() << ", " << offset_[i].getZ());
			i++;
		}
	}
}

void SwarmVehicle::setScenario(const tf2::Vector3 &swarm_target, const std::vector<tf2::Vector3> &scens)
{
	geometry_msgs::PoseStamped msg;
	msg.header.stamp = ros::Time::now();
	int i = 0;
	ROS_INFO("die?");
	for (auto &vehicle : camila_)
	{
		// if (scens.empty())
		// {
		// 	msg.pose.position.x = swarm_target.getX() + offset_[i].getX();
		// 	msg.pose.position.y = swarm_target.getY() + offset_[i].getY();
		// 	msg.pose.position.z = swarm_target.getZ() + offset_[i].getZ();
		// }
		// else
		// {
		// 	msg.pose.position.x = swarm_target.getX() + offset_[i].getX() + scens[i].getX();
		// 	msg.pose.position.y = swarm_target.getY() + offset_[i].getY() + scens[i].getY();
		// 	msg.pose.position.z = swarm_target.getZ() + offset_[i].getZ() + scens[i].getZ();
		// }
		msg.pose.position.x = swarm_target.getX() + offset_[i].getX();
		msg.pose.position.y = swarm_target.getY() + offset_[i].getY();
		msg.pose.position.z = swarm_target.getZ() + offset_[i].getZ();
		auto controller_ptr = dynamic_cast<LocalPoseController*>(vehicle.getController());
		controller_ptr->setTatget(msg);
		i++;
	}
}

void SwarmVehicle::run()
{
	if (isPublish())
	{
		bool use_velocity_controller, use_repulsive_force;
		nh_global_.getParamCached("use_velocity_controller", use_velocity_controller);
		nh_global_.getParamCached("local_plan/repulsive_range", repulsive_range_);
		nh_global_.getParamCached("local_plan/use_repulsive_force", use_repulsive_force);
		nh_global_.getParamCached("local_plan/max_speed", max_speed_);
		setVehicleGlobalPose();
		for (auto &vehicle : camila_)
		{
			tf2::Vector3 local_plan;
			calAttractive(vehicle);
			if (use_repulsive_force)
			{
				vehicle.setLocalPlanner(LocalPlanners::PF_LOCAL_PLANNER);
				calRepulsive(vehicle);
			}
			else
				vehicle.setLocalPlanner(LocalPlanners::LOCAL_PLANNER);
			if (use_velocity_controller)
				vehicle.goTo(Controllers::LOCAL_VELOCITY_CONTROLLER);
			else
				vehicle.goTo(Controllers::LOCAL_POSE_CONTROLLER);
		}
	}
}