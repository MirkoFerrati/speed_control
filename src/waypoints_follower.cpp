#include "speed_control/waypoints_follower.h"
#include <ros/init.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <mutex>
#include <speed_control/config_toolConfig.h>
#include <mrtstar/locking.h>


void waypoints_follower::config_callback(speed_control::config_toolConfig &config, uint32_t level) {
  //ROS_INFO("Reconfigure Request: %f %f %f %f", config.kp1, config.kp2, config.ki1, config.ki2);
  if (!config.Apply) return;
  kp1 = config.kp1;
  kp2 = config.kp2;
  ki1 = config.ki1;
  ki2 = config.ki2;  
  max_linear_speed = config.Max_linear_speed;
  max_angular_speed = config.Max_angular_speed;
  config.Apply=false;
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

void waypoints_follower::setTargetCallback(const mrtstar::locking::ConstPtr& targets)
{
    std::unique_lock<std::mutex>(targets_mtx);
    if (targets->command=="switch")
        this->targets.clear();
    for (auto target:targets->occupied_nodes)
    {
        geometry_msgs::Point target_temp;
        target_temp.x=target.x;
        target_temp.y=target.y;
        target_temp.z=(target.t_start.toSec()+target.t_end.toSec())*0.5;
        target_manager(target_temp);
    }
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
    kp1=0.5;
    kp2=1.5;
    ki1=0.5;
    ki2=1.5;
    
}

    
void waypoints_follower::init()
{
    f = boost::bind(&waypoints_follower::config_callback, this, _1, _2);   
    server.setCallback(f);
    target_sub = this->n.subscribe<geometry_msgs::Polygon>("targets",10,&waypoints_follower::setTargetCallback,this);
    comand_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);  
    straight=turning=false;
}


void waypoints_follower::run()
{
    
    const double MAX_TWIST_LINEAR = 1.0;
    const double MAX_TWIST_ANGULAR = 0.5;
    const double TURNING_RADIUS=1.0;

        if (!active) return;
//         ROS_INFO("active");
        
        if (!localized)
        {
            ROS_WARN("waiting for localization");
            return;
        }
        if (reached(next_target) || start_new_target) //Change of target!
        {
            start_new_target = false;
            if (targets.size()==0)
            {
                deactivate(deactivate_reason::NO_MORE_TARGETS);
                return;
            }
            ROS_INFO("starting new target");
            
            std::unique_lock<std::mutex>(targets_mtx);
            
            next_target = targets.front();
            targets.pop_front();
            geometry_msgs::Pose2D current_pose;
            current_pose.x=x;
            current_pose.y=y;            
            
            xtarget = next_target.x;
            ytarget = next_target.y;
            straight=true;
            turning=false;
        }
        /* More complex implementation, for the future, it chooses speed and angular radius
        if (distance(next_target)<TURNING_RADIUS && !turning && targets.size()>0 && !start_new_target) //Will not use circle if this is the last target or if we are starting now
        {
            //TODO start turning along a circle
            //Find next heading, circle radius, circle length, circle angle
            double current_heading=theta;
            double heading=atan2(ytarget-targets.front().y,xtarget-targets.front().x);
            double current_speed=twist.linear.x;
            double next_speed=distance(next_target,targets.front())/(targets.front().z-next_target.z);
            double delta_heading=heading-current_heading;
            bool rotate_left;
            if((0<delta_heading && delta_heading<M_PI) || (-2*M_PI<delta_heading && delta_heading<-M_PI ))
                rotate_left=true;
            else rotate_left=false;
            bool ok=false;
            double desired_speed=(current_speed+next_speed)/2.0;
            double radius=max_turning_radius/10.0;
            double alpha=2*radius/wheel_distance;
            while(!ok)
            {
                double right_wheel_speed=-desired_speed*(1/alpha+1);
                if (right_wheel_speed>max_speed) //Too fast, slow down
                {
                    right_wheel_speed=max_speed;
                }
                double left_wheel_speed=2*desired_speed-right_wheel_speed;
                if (left_wheel_speed>max_speed) //Also too fast, change speed and turning radius
                {
                    desired_speed=0.9*desired_speed;
                    radius=radius+max_turning_radius*0.1;
                }
                else
                {
                    ok=true;
                }
            }
            turning = true;
            twist.linear.x=desired_speed;
            twist.angular.z=(right_wheel_speed-left_wheel_speed)/2.0;
        }
        */
        if (distance(next_target)<TURNING_RADIUS && !turning && targets.size()>0 && !start_new_target) //Will not use circle if this is the last target or if we are starting now
        {
            turning=true;
            straight=false;
            desired_heading=atan2(targets.front().y-ytarget,targets.front().x-xtarget);
            double current_speed=twist.linear.x;
            double next_speed=distance(next_target,targets.front())/(targets.front().z-next_target.z);
            desired_speed=(current_speed+next_speed)/2.0;
            twist.linear.x=desired_speed;
            ROS_INFO("starting turning");
            
        }
        double length = distance(next_target);
        double theta_err;
        
        if (turning)
        {
            //either keep publishing same rotation and speed or feedback on the circle information
            twist.linear.x=desired_speed*(1+ki1*(TURNING_RADIUS-length));
            twist.angular.z=-ki2*sin(desired_heading-theta);
            theta_err=desired_heading-theta;
            if (fabs(desired_heading-theta)<0.1 )//|| distance())
            {
                //We kind of reached the desired heading, we should switch to the next target and stop turning
                
                ROS_INFO("starting new target");
                std::unique_lock<std::mutex>(targets_mtx);
                
                next_target = targets.front();
                targets.pop_front();
                geometry_msgs::Pose2D current_pose;
                current_pose.x=x;
                current_pose.y=y;            
                
                xtarget = next_target.x;
                ytarget = next_target.y;
                straight=true;
                ROS_INFO("starting straight");
                
                turning=false;
            }
        }
        if (straight)
        {
            
            double delta = next_target.z - ros::Time::now().toSec();
            if (delta<0)
            {
                twist.linear.x = 0;
                return;
            }
            twist.linear.x=length/delta;
            theta_err=atan2(ytarget-y,xtarget-x)-theta;
            if (fabs(fabs(theta_err)-M_PI)<0.1) theta_err=theta_err+0.1;
            twist.angular.z=-kp2*sin(theta_err);
        }
        twist.linear.x = std::min(twist.linear.x,MAX_TWIST_LINEAR);
        comand_pub.publish(twist);
        ROS_DEBUG_STREAM("controller run xt: "<<xtarget<<" yt: "<<ytarget<<" x: "<<x<<" y: "<<y<<" t "<<next_target.z);
        ROS_DEBUG_STREAM("controller run theta:"<<theta<<" error "<<theta_err<<" v: "<<twist.linear.x<<" w: "<<twist.angular.z);
        
        ros::spinOnce();
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
    localized=true;
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

double waypoints_follower::distance(const geometry_msgs::Point& p1,const geometry_msgs::Point& p2) const
{
    double distance = sqrt(pow(p1.y-p2.y,2)+pow(p1.x-p2.x,2));
    return distance;
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
