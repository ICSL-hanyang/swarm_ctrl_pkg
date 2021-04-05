#ifndef VEHICLE_H
#define VEHICLE_H

/* cpp header */
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

/* ros header */
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

enum Controllers{
	GEO_POSE_CONTROLLER,
	LOCAL_POSE_CONTROLLER,
	LOCAL_VELOCITY_CONTROLLER
};

enum LocalPlanners{
	LOCAL_PLANNER,
	PF_LOCAL_PLANNER
};
class PoseController
{
private:
	vehicle_info &vehicle_info_;
protected:
	ros::NodeHandle &nh_;
	ros::NodeHandle &nh_global_;
	bool setpoint_publish_flag_;
public:
	PoseController(ros::NodeHandle &nh, ros::NodeHandle &nh_global, VehicleInfo &vehicle_info);
	virtual void goTo() {};
	std::string getName() const {return vehicle_info_.vehicle_name_;};
	bool isPublished() const {return setpoint_publish_flag_;};
};

class GeoPoseController : public PoseController
{
private:
	geographic_msgs::GeoPoseStamped home_;
	geographic_msgs::GeoPoseStamped cur_pose_;
	geographic_msgs::GeoPoseStamped target_;
	ros::Publisher setpoint_geo_pub_;
	ros::Subscriber home_sub_;
	ros::Subscriber cur_pose_sub_;
	ros::ServiceClient set_home_client_;

public:
	GeoPoseController(ros::NodeHandle &, ros::NodeHandle &, VehicleInfo &);
	~GeoPoseController();
	void homeCB(const mavros_msgs::HomePosition::ConstPtr &);
	void curPoseCB(const sensor_msgs::NavSatFix::ConstPtr &);
	virtual void goTo() override;
	bool setHome();
	void setTatget(const geographic_msgs::GeoPoseStamped &);
	geographic_msgs::GeoPoseStamped getHome() const {return home_;};
	geographic_msgs::GeoPoseStamped getCurPose() const {return cur_pose_;};
	geographic_msgs::GeoPoseStamped getTatget() const {return target_;};
};

class LocalPoseController : public PoseController
{
private:
	geometry_msgs::PoseStamped cur_pose_;	
	geometry_msgs::PoseStamped target_;	
	ros::Publisher setpoint_local_pub_;
	ros::Subscriber cur_pose_sub_;
public:
	LocalPoseController(ros::NodeHandle &, ros::NodeHandle &, VehicleInfo &);
	~LocalPoseController();
	void curPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){cur_pose_ = *msg;};
	virtual void goTo() override;
	geometry_msgs::PoseStamped getCurPose() const {return cur_pose_;};
	void setTatget(const geometry_msgs::PoseStamped &);
	geometry_msgs::PoseStamped getTatget() const {return target_;};
};

class LocalVelocityController : public PoseController
{
private:
	geometry_msgs::PoseStamped cur_pose_;
	geometry_msgs::PoseStamped target_;	
	ros::Publisher setpoint_vel_pub_;
	ros::Subscriber cur_pose_sub_;
public:
	LocalVelocityController(ros::NodeHandle &, ros::NodeHandle &, VehicleInfo &);
	~LocalVelocityController();
	void curPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg){cur_pose_=*msg;};
	virtual void goTo() override;
	void setTatget(const geometry_msgs::PoseStamped &);
	geometry_msgs::PoseStamped getCurPose() const {return cur_pose_;};
};

class LocalPlanner
{
private:
	tf2::Vector3 global_cur_pose_;
protected:
	ros::NodeHandle &nh_global_;
	tf2::Vector3 err_;
	tf2::Vector3 local_plan_;
	static double kp_attractive_;
	static double kp_repulsive_;
public:
	LocalPlanner(ros::NodeHandle &);
	virtual tf2::Vector3 generate();
	void setGlobalPose(const tf2::Vector3 &pose){global_cur_pose_ = pose;};
	tf2::Vector3 getGlobalPose(){return global_cur_pose_;};
	void setErr(const tf2::Vector3 &err){err_ = err;};
	tf2::Vector3 getErr(){return err_;};
};

class PFLocalPlanner : public LocalPlanner
{
private:
	tf2::Vector3 sum_repulsive_;
public:
	PFLocalPlanner(ros::NodeHandle &);
	tf2::Vector3 generate() override;
	void setSumOfRepulsive(const tf2::Vector3 &sum_repulsive){sum_repulsive_=sum_repulsive;};
	tf2::Vector3 getSumOfRepulsive(){return sum_repulsive_;};
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

	/* ros service client*/
	ros::ServiceClient arming_client_;
	ros::ServiceClient set_mode_client_;
	ros::ServiceClient takeoff_client_;
	ros::ServiceClient land_client_;

	/* ros multi sub client */
	ros::Subscriber multi_arming_sub_;
	ros::Subscriber multi_set_mode_sub_;
	ros::Subscriber multi_set_home_sub_;
	ros::Subscriber multi_takeoff_sub_;
	ros::Subscriber multi_land_sub_;

	std::pair<int, int> scen_pos_;

	/*fermware version=> diagnositic_msgs/DiagnosticStatus*/

	void vehicleInit();
	void release();
	void stateCB(const mavros_msgs::State::ConstPtr &);
	void batteryCB(const sensor_msgs::BatteryState::ConstPtr &msg){cur_battery_ = *msg;};

	/* multi callback functions */
	void multiArming(const std_msgs::Bool::ConstPtr &);
	void multiSetMode(const std_msgs::String::ConstPtr &);
	void multiSetHome(const std_msgs::Empty::ConstPtr &);
	void multiTakeoff(const std_msgs::Empty::ConstPtr &);
	void multiLand(const std_msgs::Empty::ConstPtr &);

	std::vector<PoseController*> controllers_;
	PoseController *controller_ptr_;
	std::vector<LocalPlanner> local_planners_;
	LocalPlanner* lp_ptr_;

  public:
	Vehicle() = delete;
	Vehicle(ros::NodeHandle &, ros::NodeHandle &);
	Vehicle(ros::NodeHandle &, ros::NodeHandle &, const VehicleInfo &);
	Vehicle(const Vehicle &);
	const Vehicle &operator=(const Vehicle &);
	~Vehicle();

	void setVehicleInfo(const VehicleInfo &);
	VehicleInfo getInfo() const {return vehicle_info_;};
	mavros_msgs::State getState() const {return cur_state_;};
	sensor_msgs::BatteryState getBattery() const {return cur_battery_;};

	/*main drone function*/
	bool arming(const bool &);
	bool setMode(const std::string &);
	bool takeoff(const double &);
	bool land();

	/* setpoint control method */
	void setScenPos(const std::pair<int, int> &);
	std::pair<int, int> getScenPos() const;

	void setController(const Controllers &controller){controller_ptr_ = controllers_[controller];};
	PoseController* getController() const {return controller_ptr_;};
	geographic_msgs::GeoPoseStamped getHome();

	void setLocalPlanner(const LocalPlanners &planner){lp_ptr_= &local_planners_[planner];};
	void setGlobalPose(const tf2::Vector3 &);
	tf2::Vector3 getGlobalPose(){return lp_ptr_->getGlobalPose();};
	void setErr(const tf2::Vector3 &);
	tf2::Vector3 getErr(){return lp_ptr_->getErr();};
	void setSumOfRepulsive(const tf2::Vector3 &);
	tf2::Vector3 getSumOfRepulsive();

	void goTo(const Controllers &);
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

	ros::NodeHandle nh_;
	ros::NodeHandle nh_mul_;
	ros::NodeHandle &nh_global_;
	ros::ServiceServer goto_vehicle_server_;

	bool multi_setpoint_publish_flag_;

	static double repulsive_range_;
	static double max_speed_;

	void release();

	void limit(tf2::Vector3 &, const double &);
	void setVehicleGlobalPose();
	void calRepulsive(Vehicle &);
	void calAttractive(Vehicle &);

	bool gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request &req,
					 swarm_ctrl_pkg::srvGoToVehicle::Response &res);

	static tf2::Vector3 convertGeoToENU(const geographic_msgs::GeoPoseStamped &,
										const geographic_msgs::GeoPoseStamped &);
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
	const std::vector<Vehicle> *getSwarmVehicle() const;
	const std::vector<tf2::Vector3> *getSwarmOffset() const;

	void addVehicle(const VehicleInfo &);
	void deleteVehicle(const VehicleInfo &);
	void showVehicleList() const;
	void updateOffset();
	void setScenario(const tf2::Vector3 &, const std::vector<tf2::Vector3> &);

	void run();
};

#endif