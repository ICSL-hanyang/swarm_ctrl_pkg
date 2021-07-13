#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/HomePosition.h>
#include <swarm_ctrl_pkg/srvGoToVehicle.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <swarm_ctrl_pkg/srvMultiSetpointGlobal.h>

#define CONSTANTS_RADIUS_OF_EARTH 6371000 /* meters (m)		*/
#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)

typedef struct vehicle_info
{
	int vehicle_id_;
	std::string vehicle_name_;
} VehicleInfo;

/* 	Drone modes : 
	MANUAL, STABILIZED,	OFFBOARD, ACRO, ALTCTL,	POSCTL,	RATTITUDE, 
	AUTO.TAKEOFF, AUTO.LAND, AUTO.MISSION, AUTO.LOITER,	AUTO.RTL,
	AUTO.RTGS, AUTO.READY,*/

class LocalPlanner;

class Plans
{
public:
	virtual tf2::Vector3 generate(LocalPlanner &){};
};

class AttractiveOnly : public Plans
{
private:
	static AttractiveOnly* instance_;
	AttractiveOnly(){};
public:
	static AttractiveOnly* getInstance(){
		if(instance_ == nullptr)
			instance_ = new AttractiveOnly();
		return instance_;
	}
	virtual tf2::Vector3 generate(LocalPlanner &);
};

class PotentialField : public Plans
{
private:
	static PotentialField* instance_;
	PotentialField(){};
public:
	static PotentialField* getInstance(){
		if(instance_ == nullptr)
			instance_ = new PotentialField();
		return instance_;
	}
	virtual tf2::Vector3 generate(LocalPlanner &);
};

class AdaptivePotentialField : public Plans
{
private:
	static AdaptivePotentialField* instance_;
	AdaptivePotentialField(){};
public:
	static AdaptivePotentialField* getInstance(){
		if(instance_ == nullptr)
			instance_ = new AdaptivePotentialField();
		return instance_;
	}
	virtual tf2::Vector3 generate(LocalPlanner &);
};

class LocalPlanner
{
private:
	Plans* plan_;
	tf2::Vector3 global_pose_;
protected:
	double kp_attractive_;
	double kp_repulsive_;
	double ki_repulsive_;
	double kd_repulsive_;
	double kp_repulsive_vel_;
	double mag_min_dist_;
	double t_safety_;
	tf2::Vector3 err_;
	tf2::Vector3 pre_repulsive_;
	tf2::Vector3 repulsive_;
	tf2::Vector3 repulsive_integral_;
	tf2::Vector3 repulsive_vel_;
	tf2::Vector3 local_plan_;
	tf2::Vector3 min_dist_;
public:
	LocalPlanner();
	LocalPlanner(const LocalPlanner &);
	const LocalPlanner &operator=(const LocalPlanner &);
	void setKpAtt(double &kp_a){kp_attractive_=kp_a;};
	double getKpAtt(){return kp_attractive_;};
	void setKpRep(double &kp_r){kp_repulsive_=kp_r;};
	void setKiRep(double &ki_r){ki_repulsive_=ki_r;};
	void setKdRep(double &kd_r){kd_repulsive_=kd_r;};
	double getKpRep(){return kp_repulsive_;};
	double getKiRep(){return ki_repulsive_;};
	double getKdRep(){return kd_repulsive_;};
	void setKpRepVel(double &kp_rep_vel){kp_repulsive_vel_=kp_rep_vel;};
	double getKpRepVel(){return kp_repulsive_vel_;};
	tf2::Vector3 generate(){return plan_->generate(*this);};
	void setGlobalPose(const tf2::Vector3 &pose){global_pose_ = pose;};
	tf2::Vector3 getGlobalPose(){return global_pose_;};
	void setErr(const tf2::Vector3 &err){err_ = err;};
	tf2::Vector3 getErr(){return err_;};
	void setRepulsive(const tf2::Vector3 &repulsive){repulsive_=repulsive;};
	tf2::Vector3 getRepulsive(){return repulsive_;};
	void setRepulsiveVel(const tf2::Vector3 &repulsive_vel){repulsive_vel_=repulsive_vel;};
	tf2::Vector3 getRepulsiveVel(){return repulsive_vel_;};
	void setPlanner(Plans* plan){plan_ = plan;};
	tf2::Vector3 getAttOut(){return err_*kp_attractive_;};
	tf2::Vector3 getRepOut();
	tf2::Vector3 getRepVelOut(){return repulsive_vel_*kp_repulsive_vel_;};
	void setMinDist(const tf2::Vector3 &min_dist){min_dist_ = min_dist;};
	tf2::Vector3 getMinDist(){return min_dist_;};
	void setMagMinDist(double &mag_min_dist){mag_min_dist_ = mag_min_dist;};
	double getMagMinDist(){return mag_min_dist_;};
	void setTSafety(double &t_safety){t_safety_ = t_safety;};
	double getTSafety(){return t_safety_;};
};

template <typename T>
class PoseController
{
protected:
	std::string &vehicle_name_;
	ros::NodeHandle &nh_;
	ros::NodeHandle &nh_global_;
	T pose_;
	T local_path_;
	T target_;
	tf2::Vector3 vel_;
	ros::Publisher setpoint_pub_;
	ros::Publisher setpoint_vel_pub_;
	ros::Subscriber pose_sub_;
	ros::Subscriber vel_sub_;
	bool setpoint_publish_flag_;
public:
	PoseController(ros::NodeHandle &, ros::NodeHandle &, std::string &);
	virtual void goTo(){;};
	virtual void goToVel(){;};
	std::string getName() const {return vehicle_name_;};
	T getPose() const {return pose_;};
	tf2::Vector3 getVel() const {return vel_;};
	void setLocalPath(const T &local_path){local_path_ = local_path;};
	T getLocalPath() const {return local_path_;};
	virtual void setTarget(const T &target){target_ = target;};
	T getTarget() const {return target_;};
	bool isPublished() const {return setpoint_publish_flag_;};
};

class GeoPoseController : public PoseController<sensor_msgs::NavSatFix>
{
private:
	sensor_msgs::NavSatFix home_;
	ros::Subscriber home_sub_;
	ros::ServiceClient set_home_client_;

public:
	GeoPoseController(ros::NodeHandle &, ros::NodeHandle &, std::string &);
	~GeoPoseController();
	void homeCB(const mavros_msgs::HomePosition::ConstPtr &);
	void poseCB(const sensor_msgs::NavSatFix::ConstPtr &msg){pose_ = *msg;};
	bool setHome();
	sensor_msgs::NavSatFix getHome() const {return home_;};
	virtual void goTo();
	virtual void goToVel();
	virtual void setTarget(const sensor_msgs::NavSatFix &);
};

class LocalPoseController : public PoseController<geometry_msgs::PoseStamped>
{
public:
	LocalPoseController(ros::NodeHandle &, ros::NodeHandle &, std::string &);
	~LocalPoseController();
	void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){pose_ = *msg;};
	void velCB(const geometry_msgs::TwistStamped::ConstPtr &msg);
	virtual void goTo();
	virtual void goToVel();
	virtual void setTarget(const geometry_msgs::PoseStamped &);
};

class Vehicle
{
  private:
	VehicleInfo vehicle_info_;

	ros::NodeHandle nh_;
	ros::NodeHandle &nh_mul_;
	ros::NodeHandle &nh_global_;

	/*drone state*/
	mavros_msgs::State state_;
	sensor_msgs::BatteryState cur_battery_;

	/* ros subscriber*/
	ros::Subscriber state_sub_;
	ros::Subscriber battery_sub_;

	/* ros service client*/
	ros::ServiceClient arming_client_;
	ros::ServiceClient set_mode_client_;
	ros::ServiceClient set_home_client_;
	ros::ServiceClient takeoff_client_;
	ros::ServiceClient land_client_;

	/* ros multi sub client */
	ros::Subscriber multi_arming_sub_;
	ros::Subscriber multi_set_mode_sub_;
	ros::Subscriber multi_set_home_sub_;
	ros::Subscriber multi_takeoff_sub_;
	ros::Subscriber multi_land_sub_;

	ros::Publisher g_pose_pub_;
	ros::Publisher att_pub_;
	ros::Publisher rep_pub_;
	ros::Publisher r_vel_pub_;

	std::pair<int, int> scen_pose_;
	static double max_speed_;

	/*fermware version=> diagnositic_msgs/DiagnosticStatus*/

	void vehicleInit();
	void release();
	void limit(tf2::Vector3 &, const double &);
	void stateCB(const mavros_msgs::State::ConstPtr &);
	void batteryCB(const sensor_msgs::BatteryState::ConstPtr &msg){cur_battery_ = *msg;};

	/* multi callback functions */
	void multiArming(const std_msgs::Bool::ConstPtr &);
	void multiSetMode(const std_msgs::String::ConstPtr &);
	void multiSetHome(const std_msgs::Empty::ConstPtr &);
	void multiTakeoff(const std_msgs::Empty::ConstPtr &);
	void multiLand(const std_msgs::Empty::ConstPtr &);

	LocalPlanner local_planner_;
	GeoPoseController gp_controller_;
	LocalPoseController lp_controller_;

  public:
	Vehicle() = delete;
	Vehicle(ros::NodeHandle &, ros::NodeHandle &);
	Vehicle(ros::NodeHandle &, ros::NodeHandle &, const VehicleInfo &);
	Vehicle(const Vehicle &);
	const Vehicle &operator=(const Vehicle &);
	~Vehicle();

	void setVehicleInfo(const VehicleInfo &);
	VehicleInfo getInfo() const {return vehicle_info_;};
	mavros_msgs::State getState() const {return state_;};
	sensor_msgs::BatteryState getBattery() const {return cur_battery_;};

	/*main drone function*/
	bool arming(const bool &);
	bool setMode(const std::string &);
	bool takeoff(const double &);
	bool land();
	void setGeoTarget(const sensor_msgs::NavSatFix &target){gp_controller_.setTarget(target);};
	void setLocalTarget(const geometry_msgs::PoseStamped &target){lp_controller_.setTarget(target);};
	void goTo();

	void setScenPose(const std::pair<int, int> &scen_pose){scen_pose_ = scen_pose;};
	std::pair<int,int> getScenPose() const {return scen_pose_;};

	//global position
	bool setHomeGlobal(){gp_controller_.setHome();};
	sensor_msgs::NavSatFix getHomeGeo() const {return gp_controller_.getHome();};
	sensor_msgs::NavSatFix getGeoPose() const {return gp_controller_.getPose();};
	sensor_msgs::NavSatFix getTargetGeo() const {return gp_controller_.getTarget();};

	geometry_msgs::PoseStamped getTargetLocal() const {return lp_controller_.getTarget();};
	geometry_msgs::PoseStamped getLocalPose() const {return lp_controller_.getPose();};

	void setLocalPlanner(Plans *plan){local_planner_.setPlanner(plan);};
	void setGlobalPose(const tf2::Vector3 &global_pose){
		geometry_msgs::Vector3 msg;
		msg.x = global_pose.getX();
		msg.y = global_pose.getY();
		msg.z = global_pose.getZ();
		local_planner_.setGlobalPose(global_pose);
		g_pose_pub_.publish(msg);
	};
	tf2::Vector3 getGlobalPose(){return local_planner_.getGlobalPose();};
	tf2::Vector3 getVel() const {return lp_controller_.getVel();};
	void setErr(const tf2::Vector3 &err){
		geometry_msgs::Vector3 msg;
		msg.x = err.getX();
		msg.y = err.getY();
		msg.z = err.getZ();
		local_planner_.setErr(err);
		att_pub_.publish(msg);
	};
	void setRepulsive(const tf2::Vector3 &repulsive){
		geometry_msgs::Vector3 msg;
		msg.x = repulsive.getX();
		msg.y = repulsive.getY();
		msg.z = repulsive.getZ();
		local_planner_.setRepulsive(repulsive);
		rep_pub_.publish(msg);
	};
	void setRepulsiveVel(const tf2::Vector3 &repulsive_vel){
		geometry_msgs::Vector3 msg;
		msg.x = repulsive_vel.getX();
		msg.y = repulsive_vel.getY();
		msg.z = repulsive_vel.getZ();
		local_planner_.setRepulsiveVel(repulsive_vel);
		r_vel_pub_.publish(msg);
	};
	void setMinDist(const tf2::Vector3 &min_dist){local_planner_.setMinDist(min_dist);};
	void setMagMinDist(double &mag_min_dist){local_planner_.setMagMinDist(mag_min_dist);};

	bool isPublish() const {return lp_controller_.isPublished();};
};

class SwarmVehicle
{
  private:
	/* swarm_info */
	std::string swarm_name_;
	int num_of_vehicle_;

	std::vector<Vehicle> camila_;
	std::vector<Vehicle>::iterator iter_;
	std::vector<tf2::Vector3> offset_;
	std::vector<uint8_t> scen_hex_;

	ros::NodeHandle nh_;
	ros::NodeHandle nh_mul_;
	ros::NodeHandle &nh_global_;
	ros::ServiceServer multi_setpoint_local_server_;
	ros::ServiceServer multi_setpoint_global_server_;
	ros::ServiceServer goto_vehicle_server_;

	tf2::Vector3 swarm_target_local_;

	std::string formation_;
	bool target_changed_flag_;
	double angle_;
	ros::Time prev_;

	static double sensing_range_;
	static int scen_num_;
	static std::string scen_str_;

	void swarmServiceInit();
	void release();
	void updateOffset();

	void setVehicleGlobalPose();
	void calRepulsive(Vehicle &);
	void calAttractive(Vehicle &);
	tf2::Vector3 calFv(const tf2::Vector3 &,const tf2::Vector3 &);
	void formationGenerator();
	void scenario2();
	void scenario3();
	void scenario4();
	void scenario5();
	void scenario6();
	void hexToCoord(std::vector<std::pair<int,int>> &, const uint8_t &, const int &, const bool &);

	bool multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request &req,
							swarm_ctrl_pkg::srvMultiSetpointLocal::Response &res);
	bool multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request &req,
							 swarm_ctrl_pkg::srvMultiSetpointGlobal::Response &res);
	bool gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request &req,
					 swarm_ctrl_pkg::srvGoToVehicle::Response &res);

	static tf2::Vector3 convertGeoToENU(const sensor_msgs::NavSatFix &,
										const sensor_msgs::NavSatFix &);
	static geographic_msgs::GeoPoint convertENUToGeo(const geometry_msgs::PoseStamped &,
													 const sensor_msgs::NavSatFix &);

  public:
	SwarmVehicle(ros::NodeHandle &, const std::string &swarm_name = "camila", const int &num_of_vehicle = 1);
	SwarmVehicle(const SwarmVehicle &);
	const SwarmVehicle &operator=(const SwarmVehicle &);
	~SwarmVehicle();

	void setSwarmInfo(const std::string &, const int &);
	std::string getSwarmInfo() const;

	void addVehicle(const VehicleInfo &);
	void deleteVehicle(const VehicleInfo &);
	void showVehicleList() const;

	void run();
};

#endif