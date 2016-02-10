#include "speed_control/waypoints_follower.h"
#include <ros/init.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <mutex>
#include <speed_control/config_toolConfig.h>


void waypoints_follower::config_callback(speed_control::config_toolConfig &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f %f %f %f", config.kp1, config.kp2, config.ki1, config.ki2);
  if (!config.Apply) return;
  kp1 = config.kp1;
  kp2 = config.kp2;
  ki1 = config.ki1;
  ki2 = config.ki2;  
  max_linear_speed = config.Max_linear_speed;
  max_angular_speed = config.Max_angular_speed;
}

void waypoints_follower::target_manager(const geometry_msgs::Point & msg)
{
    ROS_INFO("target manager callback");
    if (targets.size()>0)
    {
        if (fabs(targets.back().x-msg.x)<0.01 && fabs(targets.back().y-msg.y)<0.01 && fabs(targets.back().z-msg.z)<0.01)
        {
            return; //same target as before, reject
        }
    }
    ROS_INFO("new target added");
    targets.push_back(msg);
    if (!active)
        activate();
}

//TODO why does polygon use Point32 instead of Point?!?!
void waypoints_follower::setTargetCallback(const geometry_msgs::PolygonConstPtr& targets)
{
    std::unique_lock<std::mutex>(targets_mtx);
    for (auto target:targets->points)
    {
        geometry_msgs::Point target_temp;
        target_temp.x=target.x;
        target_temp.y=target.y;
        target_temp.z=target.z;
        target_manager(target_temp);
    }
}


waypoints_follower::waypoints_follower(double max_speed, double reached_threshold):max_speed(max_speed),reached_threshold(reached_threshold)
{
    active=false;
    start_new_target=false;
    command_sub = n.subscribe<std_msgs::String>("command",1,&waypoints_follower::command_manager,this);
    deactivation_reason=deactivate_reason::NO_MORE_TARGETS;
    localized=false;
}

    
void waypoints_follower::init()
{
    abstract_controller::init();
    f = boost::bind(&waypoints_follower::config_callback, this, _1, _2);   
    server.setCallback(f);
    target_sub = this->n.subscribe<geometry_msgs::Polygon>("targets",10,&waypoints_follower::setTargetCallback,this);
}


void waypoints_follower::run()
{
    ros::Rate loop_rate(10);
    
    const double MAX_TWIST_LINEAR = 0.5;
    const double MAX_TWIST_ANGULAR = 0.5;
    const double TURNING_RADIUS=0.3;
    while (ros::ok())
    {
        loop_rate.sleep();        
        if (!active) continue;
        ROS_INFO("active");
        
        if (!localized)
        {
            ROS_WARN("waiting for localization");
            continue;
        }
        tf::Quaternion q(current_pos.pose.pose.orientation.x, current_pos.pose.pose.orientation.y, current_pos.pose.pose.orientation.z, current_pos.pose.pose.orientation.w);    
        double yaw=tf::getYaw(q);
        setPosition(current_pos.pose.pose.position.x,current_pos.pose.pose.position.y,yaw);
        if (distance(next_target)<TURNING_RADIUS && !turning && targets.size()>0 && !start_new_target) //Will not use circle if this is the last target or if we are starting now
        {
            //TODO start turning along a circle
            //Find next heading, circle radius, circle length, circle angle
            turning = true;
            //TODO
            //We kind of reached the target, we should switch to the next one, but only after the turning ended
            reached_next_target = true;
        }
        if (reached_next_target || start_new_target) //Change of target!
        {
            ROS_INFO("starting new target");
            start_new_target = false;
            if (targets.size()==0)
            {
                deactivate(deactivate_reason::NO_MORE_TARGETS);
                return;
            }
            std::unique_lock<std::mutex>(targets_mtx);
            
            next_target = targets.front();
            targets.pop_front();
            geometry_msgs::Pose2D current_pose;
            current_pose.x=x;
            current_pose.y=y;            
        
            xtarget = next_target.x;
            ytarget = next_target.y;
            reached_next_target=false;
        }
        if (turning)
        {
            //either keep publishing same rotation and speed or feedback on the circle information
            
        }
        else if (straight)
        {
            double length=sqrt(pow(ytarget-y,2)+pow(xtarget-x,2));
            twist.linear.x=kp1*length>max_speed?max_speed:kp1*length;
            twist.angular.z=kp2*sin(atan2(ytarget-y,xtarget-x)-theta);
            ROS_DEBUG_STREAM("controller run xt: "<<xtarget<<" yt: "<<ytarget<<" x: "<<x<<" y: "<<y);
        }
        comand_pub.publish(twist);

        ros::spinOnce();
        
    }
}

void waypoints_follower::command_manager(const std_msgs::StringConstPtr & msg)
{
    ROS_INFO("command manager callback");
    if (msg->data=="activate" && !active)
        activate();
    else if (msg->data=="deactivate" && active)
        deactivate(deactivate_reason::USER_DEACTIVATE);
}

void waypoints_follower::setPosition(double x, double y, double theta)
{
    this->x=x;
    this->y=y;
    this->theta=theta;
}

void waypoints_follower::activate()
{
    ROS_INFO("activation called");
    if (active) return;   
    ROS_INFO("trying to activate");
    if (deactivation_reason==deactivate_reason::USER_DEACTIVATE)
    {
        active=true;
        deactivation_reason=deactivate_reason::NONE;
        return;
    }
    else if (deactivation_reason==deactivate_reason::NO_MORE_TARGETS ||
        deactivation_reason==deactivate_reason::UNREACHABLE_TARGET ||
        deactivation_reason==deactivate_reason::GLOBAL_TARGET_NOT_REACHED
    )
    {
        if (targets.size()>0)
        {
            ROS_INFO("activation added new target");
            active=true;
            start_new_target=true;
            deactivation_reason=deactivate_reason::NONE;
            return;
        }
        else
        {
            ROS_WARN("cannot activate without targets");
            return;
        }
    }
    else
    {
        ROS_INFO("activation default to start new target"); 
        active=true;
        start_new_target=true;
    }
}


void waypoints_follower::deactivate(deactivate_reason r)
{
    active=false;
    this->deactivation_reason=r;
    twist.linear.x=0;
    twist.angular.z=0;
    comand_pub.publish(twist);
}

double waypoints_follower::distance(const geometry_msgs::Point& target) const
{
    double xtarget = target.x;
    double ytarget = target.y;
    
    double distance = sqrt(pow(ytarget-y,2)+pow(xtarget-x,2));
    return distance;
}

bool waypoints_follower::reached(const geometry_msgs::Point& target) const
{
    return distance(target) < reached_threshold;    
}
