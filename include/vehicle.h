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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/HomePosition.h>
//#include <mavros_msgs/GlobalPositionTarget.h>
#include <swarm_ctrl_pkg/srvGoToVehicle.h>
#include <swarm_ctrl_pkg/srvSetSwarmTarget.h>

#define CONSTANTS_RADIUS_OF_EARTH 6371000 /* meters (m)		*/
#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)

typedef struct vehicle_info
{
	int system_id_;
	std::string vehicle_name_;
} VehicleInfo;

/* 	Drone modes : 
	MANUAL, STABILIZED,	OFFBOARD, ACRO, ALTCTL,	POSCTL,	RATTITUDE, 
	AUTO.TAKEOFF, AUTO.LAND, AUTO.MISSION, AUTO.LOITER,	AUTO.RTL,
	AUTO.RTGS, AUTO.READY,*/

class Vehicle
{
  private:
	VehicleInfo vehicle_info;

	ros::NodeHandle nh_;
	ros::NodeHandle &rNH_mul_;
	ros::NodeHandle &rNH_global_;

	/*drone state*/
	mavros_msgs::State cur_state_;
	sensor_msgs::BatteryState cur_battery_;

	/* ros subscriber*/
	ros::Subscriber state_sub_;
	ros::Subscriber battery_sub_;
	ros::Subscriber global_pos_sub_;

	/* ros publisher*/
	ros::Publisher setpoint_vel_pub_;

	/* ros service client*/
	ros::ServiceClient arming_client_;
	ros::ServiceClient set_mode_client_;
	ros::ServiceClient set_home_client_;
	ros::ServiceClient takeoff_client_;
	ros::ServiceClient land_client_;

	/* ros multi sub */
	ros::Subscriber multi_arming_sub_;
	ros::Subscriber multi_set_mode_sub_;
	ros::Subscriber multi_set_home_sub_;
	ros::Subscriber multi_takeoff_sub_;
	ros::Subscriber multi_land_sub_;

	/* global coordinate*/
	sensor_msgs::NavSatFix home_global_;
	sensor_msgs::NavSatFix cur_global_;

	bool setpoint_publish_flag_;

	/*firmware version=> diagnositic_msgs/DiagnosticStatus*/

	void vehicleInit();
	void stateCB(const mavros_msgs::State::ConstPtr &);
	void batteryCB(const sensor_msgs::BatteryState::ConstPtr &);
	void globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &);

  public:
	Vehicle() = delete;
	Vehicle(ros::NodeHandle &, ros::NodeHandle &);
	Vehicle(ros::NodeHandle &, ros::NodeHandle &, const VehicleInfo &);
	Vehicle(const Vehicle &rhs);
	const Vehicle &operator=(const Vehicle &rhs);
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

	/* multi callback functions */
	void multiArming(const std_msgs::Bool::ConstPtr &);
	void multiSetMode(const std_msgs::String::ConstPtr &);
	void multiSetHome(const std_msgs::Empty::ConstPtr &);
	void multiTakeoff(const std_msgs::Empty::ConstPtr &);
	void multiLand(const std_msgs::Empty::ConstPtr &);

	//global position
	sensor_msgs::NavSatFix getGlobalPosition() const;

	bool isReceivedGlobalPos() const;
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
	std::vector<geometry_msgs::Vector3> offset_;

	ros::NodeHandle nh_;
	ros::NodeHandle nh_mul_;
	ros::NodeHandle &rNH_global_;

	ros::ServiceServer swarm_target_server_;

	sensor_msgs::NavSatFix swarm_map_;

	tf2_ros::StaticTransformBroadcaster static_offset_bc_;
	geometry_msgs::TransformStamped swarm_target_TF_;
	tf2_ros::TransformBroadcaster swarm_target_bc_;

	std::string formation_;

	bool multi_setpoint_publish_flag_;

	/* private operation */
	void setSwarmMap();
	void offsetPublisher();
	void formationGenerater();
	static geometry_msgs::Vector3 convertGeoToENU(const sensor_msgs::NavSatFix &,
												  const sensor_msgs::NavSatFix &);
	static geographic_msgs::GeoPoint convertENUToGeo(const geometry_msgs::PoseStamped &,
													 const sensor_msgs::NavSatFix &);
	bool setSwarmTarget(swarm_ctrl_pkg::srvSetSwarmTarget::Request &,
						swarm_ctrl_pkg::srvSetSwarmTarget::Response &);
	bool isPublish();

	void scenario1() const;

  public:
	SwarmVehicle(ros::NodeHandle &, const std::string &swarm_name = "camila", const int &num_of_vehicle = 1);
	SwarmVehicle(const SwarmVehicle &);
	const SwarmVehicle &operator=(const SwarmVehicle &);
	~SwarmVehicle();

	void setSwarmInfo(const std::string &swarm_name, const int &num_of_vehicle);
	std::string getSwarmInfo() const;

	void addVehicle(const VehicleInfo&);
	void deleteVehicle(const VehicleInfo&);
	void showVehicleList() const;

	void init();
	void run();
};

#endif