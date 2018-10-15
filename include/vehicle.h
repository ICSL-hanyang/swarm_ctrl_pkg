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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>  
#include <mavros_msgs/CommandBool.h>  
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <swarm_ctrl_pkg/srvGoToVehicle.h>
#include <swarm_ctrl_pkg/srvMultiSetpointLocal.h>
#include <swarm_ctrl_pkg/srvMultiSetpointGlobal.h>

#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/
#define M_DEG_TO_RAD (M_PI / 180.0)
#define M_RAD_TO_DEG (180.0 / M_PI)
#define SPACING 5

typedef struct vehicle_info{
	int system_id;
	std::string vehicle_name;
}VehicleInfo;

/* 	Drone modes : 
	MANUAL, STABILIZED,	OFFBOARD, ACRO, ALTCTL,	POSCTL,	RATTITUDE, 
	AUTO.TAKEOFF, AUTO.LAND, AUTO.MISSION, AUTO.LOITER,	AUTO.RTL,
	AUTO.RTGS, AUTO.READY,*/

class Vehicle{
	private:
		VehicleInfo vehicle_info;
		
		ros::NodeHandle nh;
		ros::NodeHandle nh_mul;

		/*drone state*/
		mavros_msgs::State cur_state;
		sensor_msgs::BatteryState cur_battery;

		/* ros subscriber*/
		ros::Subscriber state_sub;
		ros::Subscriber battery_sub;
		ros::Subscriber local_pos_sub;
		ros::Subscriber global_pos_sub;

		/* ros publisher*/
		ros::Publisher setpoint_vel_pub;
		ros::Publisher setpoint_local_pub;
		ros::Publisher setpoint_global_pub;

		/* ros service client*/
		ros::ServiceClient arming_client;
		ros::ServiceClient set_mode_client;
		ros::ServiceClient set_home_client;
		
		/* ros multi sub client */
		ros::Subscriber multi_arming_sub;
		ros::Subscriber multi_set_mode_sub;
		ros::Subscriber multi_set_home_sub;

		/* local coordinate*/
		geometry_msgs::PoseStamped home_local;
		geometry_msgs::PoseStamped cur_local;
		geometry_msgs::PoseStamped tar_local;
		
		/* global coordinate*/
		sensor_msgs::NavSatFix home_global;
		sensor_msgs::NavSatFix cur_global;
		sensor_msgs::NavSatFix tar_global;

		bool setpoint_publish_flag;
		double kp;

		/*fermware version=> diagnositic_msgs/DiagnosticStatus*/

		void vehicleInit();

	public:
		Vehicle();
		Vehicle(VehicleInfo _vehicle_info);
		Vehicle(const Vehicle &rhs);
		const Vehicle& operator=(const Vehicle &rhs);

		void setVehicleInfo(VehicleInfo new_vehicle_info);
		VehicleInfo getInfo();
		void stateCB(const mavros_msgs::State::ConstPtr& msg);
		mavros_msgs::State getState();
		void batteryCB(const sensor_msgs::BatteryState::ConstPtr& msg);
		sensor_msgs::BatteryState getBattery();
		
		/*main drone function*/
		bool arming(bool _arm_state);
		bool setMode(std::string _mode);
		void gotoGlobal(sensor_msgs::NavSatFix _tar_global);
		void setLocalTarget(geometry_msgs::PoseStamped _tar_local);
		void gotoLocal();
		void gotoVel();

		/* multi callback functions */
		void multiArming(const std_msgs::Bool::ConstPtr& msg);
		void multiSetMode(const std_msgs::String::ConstPtr& msg);
		void multiSetHome(const std_msgs::Empty::ConstPtr& trigger);
		
		//global position
		bool setHomeGlobal();
		sensor_msgs::NavSatFix getHomeGlobal();
		void globalPositionCB(const sensor_msgs::NavSatFix::ConstPtr& msg);
		sensor_msgs::NavSatFix getGlobalPosition();
		sensor_msgs::NavSatFix getTargetGlobal();

		//local position
		void setHomeLocal();
		geometry_msgs::PoseStamped getHomeLocal();
		void localPositionCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
		geometry_msgs::PoseStamped getLocalPosition();
		geometry_msgs::PoseStamped getTargetLocal();

		geometry_msgs::Vector3 convertGeoToENU(double coord_lat, double coord_long, 
			double coord_alt, double home_lat, double home_long, double home_alt);

		bool isPublish();
};

class SwarmVehicle{
	private:
		/* swarm_info */
		std::string swarm_name;
		int num_of_vehicle;

		std::vector<Vehicle> camila;
		std::vector<Vehicle>::iterator iter;

		ros::NodeHandle nh;
		ros::ServiceServer multi_setpoint_local_server;
		ros::ServiceServer multi_setpoint_global_server;
		ros::ServiceServer goto_vehicle_server;

		geometry_msgs::PoseStamped swarm_position_local;
		sensor_msgs::NavSatFix swarm_position_global;
		
		std::string formation;
		double min_length;
		bool control_method;
		bool multi_setpoint_publish_flag;
	public:
		SwarmVehicle(std::string _swarm_name = "camila", int _num_of_vehicle = 1); //have to add default value
		SwarmVehicle(const SwarmVehicle &rhs);
		~SwarmVehicle();
		const SwarmVehicle& operator=(const SwarmVehicle &rhs);
		
		void setSwarmInfo(std::string _swarm_name, int _num_of_vehicle);
		std::string getSwarmInfo();

		void addVehicle(VehicleInfo _vehicle_info);
		void deleteVehicle(VehicleInfo _vehicle_info);
		void showVehicleList();

		bool multiSetpointLocal(swarm_ctrl_pkg::srvMultiSetpointLocal::Request& req,
			swarm_ctrl_pkg::srvMultiSetpointLocal::Response& res);
		bool multiSetpointGlobal(swarm_ctrl_pkg::srvMultiSetpointGlobal::Request& req,
			swarm_ctrl_pkg::srvMultiSetpointGlobal::Response& res);
		bool gotoVehicle(swarm_ctrl_pkg::srvGoToVehicle::Request& req,
			swarm_ctrl_pkg::srvGoToVehicle::Response& res);	

		geometry_msgs::Vector3 convertGeoToENU(double coord_lat, double coord_long, 
			double coord_alt, double home_lat, double home_long, double home_alt);
		geometry_msgs::Vector3 convertENUToGeo(double x, double y, double z, 
			double home_lat, double home_long, double home_alt);

		bool isPublish();

		void run();
};

#endif