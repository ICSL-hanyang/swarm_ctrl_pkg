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
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
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
	AttractiveOnly();
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
	PotentialField();
public:
	static PotentialField* getInstance(){
		if(instance_ == nullptr)
			instance_ = new PotentialField();
		return instance_;
	}
	virtual tf2::Vector3 generate(LocalPlanner &);
};

class LocalPlanner
{
private:
	Plans* plan_;
	tf2::Vector3 cur_global_pose_;
protected:
	double kp_attractive_;
	double kp_repulsive_;
	tf2::Vector3 err_;
	tf2::Vector3 sum_repulsive_;
	tf2::Vector3 local_plan_;
public:
	LocalPlanner();
	LocalPlanner(const LocalPlanner &);
	const LocalPlanner &operator=(const LocalPlanner &);
	void setKpAtt(double &kp_a){kp_attractive_=kp_a;};
	double getKpAtt(){return kp_attractive_;};
	void setKpRep(double &kp_r){kp_repulsive_=kp_r;};
	double getKpRep(){return kp_repulsive_;};
	tf2::Vector3 generate();
	void setGlobalPose(const tf2::Vector3 &pose){cur_global_pose_ = pose;};
	tf2::Vector3 getGlobalPose(){return cur_global_pose_;};
	void setErr(const tf2::Vector3 &err){err_ = err;};
	tf2::Vector3 getErr(){return err_;};
	void setSumOfRepulsive(const tf2::Vector3 &sum_repulsive){sum_repulsive_=sum_repulsive;};
	tf2::Vector3 getSumOfRepulsive(){return sum_repulsive_;};
	void setPlanner(Plans* plan){plan_ = plan;};
};

template <typename T>
class PoseController
{
protected:
	std::string &vehicle_name_;
	ros::NodeHandle &nh_;
	ros::NodeHandle &nh_global_;
	T cur_pose_;
	T target_;
	ros::Publisher setpoint_pub_;
	ros::Publisher setpoint_vel_pub_;
	ros::Subscriber cur_pose_sub_;
	bool setpoint_publish_flag_;
public:
	PoseController(ros::NodeHandle &, ros::NodeHandle &, std::string &);
	virtual void goTo(){;};
	virtual void goToVel(){;};
	std::string getName() const {return vehicle_name_;};
	T getCuPose() const {return cur_pose_;};
	virtual void setTarget(const T &target){target_ = target;};
	T getTarget() const {return target_;};
	bool isPublished() const {return setpoint_publish_flag_;};
};

class GeoPoseController : public PoseController<geographic_msgs::GeoPoseStamped>
{
private:
	geographic_msgs::GeoPoseStamped home_;
	ros::Subscriber home_sub_;
	ros::ServiceClient set_home_client_;

public:
	GeoPoseController(ros::NodeHandle &, ros::NodeHandle &, std::string &);
	~GeoPoseController();
	void homeCB(const mavros_msgs::HomePosition::ConstPtr &);
	void curPoseCB(const sensor_msgs::NavSatFix::ConstPtr &);
	virtual void goTo();
	virtual void goToVel();
	bool setHome();
	geographic_msgs::GeoPoseStamped getHome() const {return home_;};
};

class LocalPoseController : public PoseController<geometry_msgs::PoseStamped>
{
public:
	LocalPoseController(ros::NodeHandle &, ros::NodeHandle &, std::string &);
	~LocalPoseController();
	void curPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){cur_pose_ = *msg;};
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
	mavros_msgs::State cur_state_;
	sensor_msgs::BatteryState cur_battery_;

	/* ros subscriber*/
	ros::Subscriber state_sub_;
	ros::Subscriber battery_sub_;
	ros::Subscriber home_sub_;
	ros::Subscriber global_pos_sub_;

	/* ros publisher*/
	ros::Publisher setpoint_global_pub_;

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

	/* global coordinate*/
	sensor_msgs::NavSatFix home_global_;
	sensor_msgs::NavSatFix cur_global_;
	sensor_msgs::NavSatFix tar_global_;

	std::pair<int, int> scen_pos_;

	/*fermware version=> diagnositic_msgs/DiagnosticStatus*/

	void vehicleInit();
	void release();
	void stateCB(const mavros_msgs::State::ConstPtr &);
	void batteryCB(const sensor_msgs::BatteryState::ConstPtr &);
	void homeCB(const mavros_msgs::HomePosition::ConstPtr &);
	void globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &);

	/* multi callback functions */
	void multiArming(const std_msgs::Bool::ConstPtr &);
	void multiSetMode(const std_msgs::String::ConstPtr &);
	void multiSetHome(const std_msgs::Empty::ConstPtr &);
	void multiTakeoff(const std_msgs::Empty::ConstPtr &);
	void multiLand(const std_msgs::Empty::ConstPtr &);

	LocalPlanner local_planner_;
	LocalPoseController lp_controller_;

  public:
	Vehicle() = delete;
	Vehicle(ros::NodeHandle &, ros::NodeHandle &);
	Vehicle(ros::NodeHandle &, ros::NodeHandle &, const VehicleInfo &);
	Vehicle(const Vehicle &);
	const Vehicle &operator=(const Vehicle &);
	~Vehicle();

	void setVehicleInfo(const VehicleInfo &);
	VehicleInfo getInfo() const;
	mavros_msgs::State getState() const;
	sensor_msgs::BatteryState getBattery() const;

	/*main drone function*/
	bool arming(const bool &);
	bool setMode(const std::string &);
	bool takeoff(const double &);
	bool land();
	void gotoGlobal(const sensor_msgs::NavSatFix &);
	void setLocalTarget(const geometry_msgs::PoseStamped &);
	void goTo();
	void goToVel();

	void setScenPos(const std::pair<int, int> &);
	std::pair<int,int> getScenPos() const;

	//global position
	bool setHomeGlobal();
	sensor_msgs::NavSatFix getHomeGlobal() const;
	sensor_msgs::NavSatFix getGlobalPosition() const;
	sensor_msgs::NavSatFix getTargetGlobal() const;

	geometry_msgs::PoseStamped getTargetLocal() const {return lp_controller_.getTarget();};
	geometry_msgs::PoseStamped getLocalPose() const {return lp_controller_.getCuPose();};

	void setLocalPlanner(Plans *);
	void setGlobalPose(const tf2::Vector3 &);
	tf2::Vector3 getGlobalPose(){return local_planner_.getGlobalPose();};
	void setErr(const tf2::Vector3 &);
	void setSumOfRepulsive(const tf2::Vector3 &);

	bool isPublish() const;
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

	static double repulsive_range_;
	static double max_speed_;
	static int scen_num_;
	static std::string scen_str_;

	void swarmServiceInit();
	void release();
	void updateOffset();

	void limit(tf2::Vector3 &, const double &);
	void setVehicleGlobalPose();
	void calRepulsive(Vehicle &);
	void calAttractive(Vehicle &);
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
	bool isPublish();

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