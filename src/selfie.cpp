#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <mavros_msgs/CommandBool.h> //arm용
#include <mavros_msgs/SetMode.h> //offboard 모드 설정용
#include <mavros_msgs/State.h> //mavros 메세지 활용용

//Parameter
int mode = 0;
int pre_mode = 0;

double coor[3] = {0.0,};
double com_x = 0.0;
double com_y = 0.0;
double com_z = 1.5;

mavros_msgs::State selfie_current_state;

void selfie_state_cb(const mavros_msgs::State::ConstPtr& msg){
    selfie_current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");

    ROS_INFO("Control node executed");

    ros::NodeHandle nh;
    ros::Subscriber selfie_state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, selfie_state_cb);
    ros::Publisher selfie_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::ServiceClient selfie_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient selfie_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0); //period 0.05 s
    
    // wait for FCU connection
    while(ros::ok() && selfie_current_state.connected){
        ROS_INFO("Selfie_drone connected");
		ros::spinOnce();
        rate.sleep();
    }
    nh.setParam("test_node/x", 0.0);
    nh.setParam("test_node/y", 0.0);
    nh.setParam("test_node/z", 1.5);
    
    //selfie initial position
    geometry_msgs::PoseStamped selfie_pose;
    selfie_pose.pose.position.x = 0;
    selfie_pose.pose.position.y = 0;
    selfie_pose.pose.position.z = 1.5;

    //send a few setpoints before starting
    for(int i = 50 ; ros::ok() && i > 0 ; --i){
    	selfie_local_pos_pub.publish(selfie_pose);
        ros::spinOnce();
        rate.sleep();
    }

    //switch mode to OFFBOARD
    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";

    //selfie_arming_client
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();	

    while(ros::ok()){

		if( selfie_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)) ){
	        if( selfie_set_mode_client.call(set_mode) && set_mode.response.success){
	            ROS_INFO("Selfie_drone Offboard enabled");
	        }
	        else{
	        	ROS_INFO("Selfie_drone Offboard disabled");	
	        }
	        last_request = ros::Time::now();
	    } 
		else{
	        if( !selfie_current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)) ){
	            if( selfie_arming_client.call(arm_cmd) && arm_cmd.response.success){
	                ROS_INFO("Selfie_drone armed");
	            }
	            last_request = ros::Time::now();
	        
	    	}
	    }
	    
	    nh.getParam("test_node/x",com_x);
	    nh.getParam("test_node/y",com_y);
	    nh.getParam("test_node/z",com_z);

	    if(com_x != coor[0] | com_y != coor[1] | com_z != coor[2]){
	    	ROS_INFO("(%lf, %lf, %lf)", com_x, com_y, com_z);	
	    	selfie_pose.pose.position.x = com_x;
	    	selfie_pose.pose.position.y = com_y;
	    	selfie_pose.pose.position.z = com_z;
	    }

	    selfie_local_pos_pub.publish(selfie_pose);

	    coor[0] = com_x;
	    coor[1] = com_y;
	    coor[2] = com_z;

        ros::spinOnce();
        rate.sleep();
        
    }
	
	

    return 0;
}

