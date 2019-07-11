#include <mission.h>

Mission::Mission() : 
    nh_(ros::NodeHandle("")),
    wp_index_(-1),
    initial_yaw_(0),
    cur_waypoint_(tf2::Vector3(0,0,0))
{
	local_pos_sub_ = nh_.subscribe("/camila1/mavros/local_position/pose", 5, &Mission::localPositionCB, this);
	setpoint_client_ = nh_.serviceClient<swarm_ctrl_pkg::srvMultiSetpointLocal>("/multi_setpoint_local");
}

Mission::~Mission()
{
    local_pos_sub_.shutdown();
    setpoint_client_.shutdown();
}

void Mission::localPositionCB(const geometry_msgs::PoseStamped::ConstPtr &msg){
    cur_local_ = *msg;
}


void Mission::clear(){
    waypoints_.clear();
}

void Mission::pushWaypoint(const tf2::Vector3 &waypoint){
    waypoints_.push_back(waypoint);
}

bool Mission::checkReached()
{
    findYaw();
    double c_x, c_y, c_z, rotated_x, rotated_y;
    c_x = cur_waypoint_.getX();
    c_y = cur_waypoint_.getY();
    c_z = cur_waypoint_.getZ();
    rotated_x = c_x * cos(initial_yaw_) - c_y*sin(initial_yaw_);
    rotated_y = c_x * sin(initial_yaw_) + c_y*cos(initial_yaw_);
    if( (abs(rotated_x - cur_local_.pose.position.x) < 0.7) && 
        (abs(rotated_y - cur_local_.pose.position.y) < 0.7) &&
        (abs(c_z - cur_local_.pose.position.z) < 0.3) )
        return true;
    else
        return false;
}

bool Mission::checkTimeOut(){
    if((ros::Time::now() > prev_waypoint_start_ + ros::Duration(60)))
        return true;
    else
        return false;
}
 
void Mission::findYaw(){
    if(initial_yaw_ == 0)
    {
        tf::Quaternion q(
            cur_local_.pose.orientation.x,
            cur_local_.pose.orientation.y,
            cur_local_.pose.orientation.z,
            cur_local_.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        initial_yaw_ = y;
        ROS_INFO_STREAM(initial_yaw_);
    }
}

void Mission::run()
{
    if(checkReached() || checkTimeOut())
    {
        // ROS_INFO_STREAM(cur_waypoint_.getX() <<" "<<  cur_waypoint_.getY()<<" " << cur_waypoint_.getZ());
        wp_index_++;
        if(waypoints_.size() > wp_index_){
            cur_waypoint_ = waypoints_.at(wp_index_);
            prev_waypoint_start_ = ros::Time::now();
        }
        swarm_ctrl_pkg::srvMultiSetpointLocal msg;
        msg.request.formation = "POINT";
        msg.request.x = cur_waypoint_.getX();
        msg.request.y = cur_waypoint_.getY();
        msg.request.z = cur_waypoint_.getZ();

        if(setpoint_client_.call(msg) && msg.response.success);
        else
            ROS_INFO("Can not call next setpoint!");
    }
}
