#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <mavros_msgs/CommandBool.h> //arm용
#include <mavros_msgs/SetMode.h> //OFFBOARD 모드 설정용
#include <mavros_msgs/State.h> //mavros 메세지 활용용
#include <mavros_msgs/CommandHome.h> //set_home 
#include <sensor_msgs/TimeReference.h> //tf_old data

//Parameter
int mode = 0;
int pre_mode = 0;

double coor[3] = {0.0,};
double com_x = 0.0;
double com_y = 0.0;
double com_z = 1.5;

mavros_msgs::State leader_current_state;
sensor_msgs::TimeReference leader_current_time;

void leader_state_cb(const mavros_msgs::State::ConstPtr& msg){
    leader_current_state = *msg;
}

void leader_time_ref_cb(const sensor_msgs::TimeReference::ConstPtr& msg){
	leader_current_time = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "control_node");

    ROS_INFO("Control node executed");

    ros::NodeHandle nh;
    ros::Subscriber leader_state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, leader_state_cb);
    ros::Subscriber leader_time_ref_sub = nh.subscribe<sensor_msgs::TimeReference> ("mavros/time_reference", 1, leader_time_ref_cb);
    ros::Publisher leader_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::ServiceClient leader_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient leader_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");
    ros::ServiceClient leader_set_home_client = nh.serviceClient<mavros_msgs::CommandHome> ("mavros/set_home");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0); //period 0.01 s
    
    // wait for FCU connection
    while(ros::ok() && leader_current_state.connected){
        ROS_INFO("Leader_drone connected");
		ros::spinOnce();
        rate.sleep();
    }
    nh.setParam("test_node/x", 0.0);
    nh.setParam("test_node/y", 0.0);
    nh.setParam("test_node/z", 1.5);
    
    //leader initial position
    geometry_msgs::PoseStamped leader_pose;
    leader_pose.pose.position.x = 0;
    leader_pose.pose.position.y = 0;
    leader_pose.pose.position.z = 1.5;
    leader_pose.header.stamp = leader_current_time.time_ref;

    //send a few setpoints before starting
    for(int i = 50 ; ros::ok() && i > 0 ; --i){
    	leader_local_pos_pub.publish(leader_pose);
        ros::spinOnce();
        rate.sleep();
    }

    //switch mode to OFFBOARD
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";

    //reset home position
    mavros_msgs::CommandHome set_home;
    set_home.request.current_gps = true;
    if(leader_set_home_client.call(set_home) && set_home.response.success){
    	ROS_INFO("Leader_drone reset home positon");
    }
    

    //leader_arming_client
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();	

    while(ros::ok()){

		if( leader_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)) ){
	        if( leader_set_mode_client.call(set_mode) && set_mode.response.success){
	            ROS_INFO("Leader_drone OFFBOARD enabled");
	        }
	        else{
	        	ROS_INFO("Leader_drone OFFBOARD disabled");	
	        }
	        last_request = ros::Time::now();
	    } 
		else{
	        if( !leader_current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)) ){
	            if( leader_arming_client.call(arm_cmd) && arm_cmd.response.success){
	                ROS_INFO("Leader_drone armed");
	            }
	            last_request = ros::Time::now();
	        
	    	}
	    }
	    
	    nh.getParam("test_node/x",com_x);
	    nh.getParam("test_node/y",com_y);
	    nh.getParam("test_node/z",com_z);

	    if(com_x != coor[0] | com_y != coor[1] | com_z != coor[2]){
	    	ROS_INFO("(%lf, %lf, %lf)", com_x, com_y, com_z);	
	    	leader_pose.pose.position.x = com_x;
	    	leader_pose.pose.position.y = com_y;
	    	leader_pose.pose.position.z = com_z;
	    	leader_pose.header.stamp = leader_current_time.time_ref;
	    }

	    leader_local_pos_pub.publish(leader_pose);

	    coor[0] = com_x;
	    coor[1] = com_y;
	    coor[2] = com_z;

        ros::spinOnce();
        rate.sleep();
        
    }
	
	

    return 0;
}

