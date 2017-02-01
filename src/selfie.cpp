#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //set position 용
#include <mavros_msgs/CommandBool.h> //arm용
#include <mavros_msgs/SetMode.h> //offboard 모드 설정용
#include <mavros_msgs/State.h> //mavros 메세지 활용용
#include <mavros_msgs/OverrideRCIn.h> //mavros rc 용
#include "math.h" //수식 입력용

int roll = 1500;
int pitch= 1500;
int throttle = 950;
int yaw = 1500;
int mode = -1;
int pre_mode = 0;
double offset = 0.0;

mavros_msgs::State selfie_current_state;

void selfie_state_cb(const mavros_msgs::State::ConstPtr& msg){
    selfie_current_state = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");

    ROS_INFO("Control node executed");

    ros::NodeHandle nh;
    ros::Subscriber selfie_state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, selfie_state_cb);
    ros::Publisher selfie_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    ros::Publisher selfie_rc_pub = nh.advertise<mavros_msgs::OverrideRCIn> ("mavros/rc/override", 10);
    ros::ServiceClient selfie_arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    ros::ServiceClient selfie_set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0); //period 0.05 s

	ROS_INFO("Parameter set mode 0");

    // wait for FCU connection
    while(ros::ok() && selfie_current_state.connected){
        ROS_INFO("Selfie_drone connected");
		ros::spinOnce();
        rate.sleep();
    }
    
    //selfie initial position
    geometry_msgs::PoseStamped selfie_pose;
    selfie_pose.pose.position.x = 0;
    selfie_pose.pose.position.y = 0;
    selfie_pose.pose.position.z = 0;

    //selfie rc initial
   	//mavros_msgs::OverrideRCIn overRc;
   	//overRc.channels = {roll, pitch, throttle, yaw, 898, 898, 898, 898};

    //send a few setpoints before starting
    for(int i = 20 ; ros::ok() && i > 0 ; --i){
    	selfie_local_pos_pub.publish(selfie_pose);
        //selfie_rc_pub.publish(overRc);
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

		if( selfie_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)) ){
	        if( selfie_set_mode_client.call(set_mode) && set_mode.response.success){
	            ROS_INFO("Selfie_drone Offboard enabled");
	        }
	        else{
	        	ROS_INFO("Selfie_drone Offboard disabled");	
	        }
	        last_request = ros::Time::now();
	    } 
		else{
	        if( !selfie_current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)) ){
	            if( selfie_arming_client.call(arm_cmd) && arm_cmd.response.success){
	                ROS_INFO("Selfie_drone armed");
	            }
	            last_request = ros::Time::now();
	        }
	    }
	    nh.param("control_node/mode", mode, -1);
	    nh.param("control_node/mode", offset, 0.0);

	    if(pre_mode != mode){
	    	ROS_INFO("Parameter set mode : %d", mode);	
	    }
	    
	    if(mode == 0){		
	    	selfie_pose.pose.position.x = 0;
		    selfie_pose.pose.position.y = 0;
		    selfie_pose.pose.position.z = 2;
	    }
	    else if(mode == 1){
	    	selfie_pose.pose.position.x = offset;
		    selfie_pose.pose.position.y = 0;
		    selfie_pose.pose.position.z = 2;	
	    }
	    else if(mode == 2){
	    	selfie_pose.pose.position.x = 0;
		    selfie_pose.pose.position.y = offset;
		    selfie_pose.pose.position.z = 2;	
	    }
	    else if(mode == 3){
	    	selfie_pose.pose.position.x = 0;
		    selfie_pose.pose.position.y = 0;
		    selfie_pose.pose.position.z = 2 + offset;	
	    }
	    else{
	    	selfie_pose.pose.position.x = 0;
		    selfie_pose.pose.position.y = 0;
		    selfie_pose.pose.position.z = 0;	
	    }

	    selfie_local_pos_pub.publish(selfie_pose);
		/*pub rc data*/
        //selfie_rc_pub.publish(overRc);

	    pre_mode = mode; 

        ros::spinOnce();
        rate.sleep();
    }
	
	

    return 0;
}

