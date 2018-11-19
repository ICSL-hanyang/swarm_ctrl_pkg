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
#include <mavros_msgs/GlobalPositionTarget.h>
#include <setpoint_server/SetPoint.h>
#include <swarm_ctrl_pkg/srvGoToVehicle.h>
#include <swarm_ctrl_pkg/srvSetSwarmTarget.h>

#define CONSTANTS_RADIUS_OF_EARTH 6371000 /* meters (m)		*/
#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)

typedef struct vehicle_info
{
	int system_id;
	std::string vehicle_name;
} VehicleInfo;

/* 	Drone modes : 
	MANUAL, STABILIZED,	OFFBOARD, ACRO, ALTCTL,	POSCTL,	RATTITUDE, 
	AUTO.TAKEOFF, AUTO.LAND, AUTO.MISSION, AUTO.LOITER,	AUTO.RTL,
	AUTO.RTGS, AUTO.READY,*/

class Vehicle
{
  private:
	VehicleInfo vehicle_info;

	ros::NodeHandle nh;
	ros::NodeHandle nh_mul;
	ros::NodeHandle nh_global;

	/*drone state*/
	mavros_msgs::State cur_state;
	sensor_msgs::BatteryState cur_battery;

	/* ros subscriber*/
	ros::Subscriber state_sub;
	ros::Subscriber battery_sub;
	//ros::Subscriber home_sub;
	//ros::Subscriber local_pos_sub;
	ros::Subscriber global_pos_sub;

	/* ros publisher*/
	ros::Publisher setpoint_vel_pub;
	// ros::Publisher setpoint_local_pub;
	// ros::Publisher setpoint_global_pub;

	/* ros service client*/
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;
	ros::ServiceClient set_home_client;
	ros::ServiceClient takeoff_client;
	ros::ServiceClient land_client;
	//ros::ServiceClient setpoint_client;

	/* ros multi sub */
	ros::Subscriber multi_arming_sub;
	ros::Subscriber multi_set_mode_sub;
	ros::Subscriber multi_set_home_sub;
	ros::Subscriber multi_takeoff_sub;
	ros::Subscriber multi_land_sub;

	/* local coordinate*/
	// geometry_msgs::PoseStamped home_local;
	// geometry_msgs::PoseStamped cur_local;
	// geometry_msgs::PoseStamped tar_local;

	/* global coordinate*/
	sensor_msgs::NavSatFix home_global;
	sensor_msgs::NavSatFix cur_global;
	//sensor_msgs::NavSatFix tar_global;

	bool setpoint_publish_flag;
	double kp;

	/*firmware version=> diagnositic_msgs/DiagnosticStatus*/

	void vehicleInit();

  public:
	Vehicle();
	Vehicle(VehicleInfo _vehicle_info);
	Vehicle(const Vehicle &rhs);
	const Vehicle &operator=(const Vehicle &rhs);

	void setVehicleInfo(VehicleInfo new_vehicle_info);
	VehicleInfo getInfo();
	void stateCB(const mavros_msgs::State::ConstPtr &msg);
	mavros_msgs::State getState();
	void batteryCB(const sensor_msgs::BatteryState::ConstPtr &msg);
	sensor_msgs::BatteryState getBattery();

	/*main drone function*/
	bool arming(bool _arm_state);
	bool setMode(std::string _mode);
	bool takeoff(double _takeoff_alt);
	bool land();
	
	/* multi callback functions */
	void multiArming(const std_msgs::Bool::ConstPtr &msg);
	void multiSetMode(const std_msgs::String::ConstPtr &msg);
	void multiSetHome(const std_msgs::Empty::ConstPtr &trigger);
	void multiTakeoff(const std_msgs::Empty::ConstPtr &trigger);
	void multiLand(const std_msgs::Empty::ConstPtr &trigger);

	//global position
	void globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr &msg);
	sensor_msgs::NavSatFix getGlobalPosition();

	bool isPublish();
};

class SwarmVehicle
{
  private:
	/* swarm_info */
	std::string swarm_name;
	int num_of_vehicle;

	std::vector<Vehicle> camila;
	std::vector<Vehicle>::iterator iter;
	std::vector<geometry_msgs::Vector3> offset;

	ros::NodeHandle nh;
	ros::NodeHandle nh_global;
	ros::ServiceServer swarm_target_server;

	//sensor_msgs::NavSatFix swarm_position_global;
	sensor_msgs::NavSatFix swarm_map;

	tf2_ros::StaticTransformBroadcaster static_offset_bc;
	tf2_ros::TransformBroadcaster swarm_target_bc;

	std::string formation = "POINT";

	bool multi_setpoint_publish_flag;
	//double angle;

  public:
	SwarmVehicle(std::string _swarm_name = "camila", int _num_of_vehicle = 1); //have to add default value
	SwarmVehicle(const SwarmVehicle &rhs);
	const SwarmVehicle &operator=(const SwarmVehicle &rhs);
	~SwarmVehicle();

	void setSwarmInfo(std::string _swarm_name, int _num_of_vehicle);
	std::string getSwarmInfo();

	void addVehicle(VehicleInfo _vehicle_info);
	void deleteVehicle(VehicleInfo _vehicle_info);
	void showVehicleList();

	void setSwarmMap(); 
	void offsetPublisher();
	void formationGenerater(); 
	bool setSwarmTarget(swarm_ctrl_pkg::srvSetSwarmTarget::Request &req,
							swarm_ctrl_pkg::srvSetSwarmTarget::Response &res);  

	geometry_msgs::Vector3 convertGeoToENU(sensor_msgs::NavSatFix _coord,
										   sensor_msgs::NavSatFix _home);
	geographic_msgs::GeoPoint convertENUToGeo(geometry_msgs::PoseStamped _local,
											  sensor_msgs::NavSatFix _home_global);

	bool isPublish();

	void run();
};

#endif