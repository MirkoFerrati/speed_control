#include "speed_control/abstract_controller.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

abstract_controller::abstract_controller()
{
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    
    error_ang = 0;
    error_lin = 0;
    error_ang_old = 0;
    error_lin_old = 0;
}



double abstract_controller::ErrorAngle(geometry_msgs::PoseWithCovarianceStamped cur, geometry_msgs::Pose2D ref)
{
    tf::Quaternion q(cur.pose.pose.orientation.x, cur.pose.pose.orientation.y, cur.pose.pose.orientation.z, cur.pose.pose.orientation.w);    
    double yaw=tf::getYaw(q);
    double Ex = ref.x - cur.pose.pose.position.x;   //x error
    double Ey = ref.y - cur.pose.pose.position.y;   //y error
    double ref_theta = atan2(Ey, Ex);
    double Et = ref_theta-yaw;   //theta error
    return Et;
}



double abstract_controller::ErrorLinear(geometry_msgs::PoseWithCovarianceStamped cur, geometry_msgs::Pose2D ref)
{
    double Ex = ref.x - cur.pose.pose.position.x;           //errore lungo x
    double Ey = ref.y - cur.pose.pose.position.y;           //errore lungo y
    double Etx = pow(pow(Ex,2)+pow(Ey,2),0.5);
    return Etx;
}



void abstract_controller::ReadCurrentPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    current_pos= *msg;
    localized=true;
}

void abstract_controller::init()
{
    cur_pose_sub = n.subscribe("odom", 1000, &abstract_controller::ReadCurrentPosition, this);        
    comand_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);  
}
