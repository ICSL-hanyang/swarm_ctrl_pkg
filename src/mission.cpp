#include <mission.h>

Mission::Mission()
{
}

Mission::~Mission()
{
}

void Mission::clear(){
    waypoints_.clear();
}

void Mission::pushWaypoint(tf2::Vector3 &waypoint){
    waypoints_.push_back(waypoint);
}

bool Mission::checkReached() const
{
    if( (abs(cur_waypoint_.getX() - cur_local_.pose.position.x) < 0.5) && 
        (abs(cur_waypoint_.getY() - cur_local_.pose.position.y) < 0.5) &&
        (abs(cur_waypoint_.getZ() - cur_local_.pose.position.z) < 0.5) )
        return true;
    else
        return false;
}
