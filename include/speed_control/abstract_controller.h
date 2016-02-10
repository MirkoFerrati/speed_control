#ifndef ABSTRACT_CONTROLLER_H
#define ABSTRACT_CONTROLLER_H
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>




class abstract_controller
{
protected:
    geometry_msgs::PoseWithCovarianceStamped current_pos;
    geometry_msgs::Twist twist;
    double kp1 = 0;
    double kp2 = 0;
    double ki1 = 0;
    double ki2 = 0;
    ros::NodeHandle n;
    ros::Subscriber cur_pose_sub;
    ros::Publisher comand_pub;
    double error_ang;
    double error_lin;
    double error_ang_old;
    double error_lin_old;
    bool localized;
    
protected:
    void ReadCurrentPosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    double ErrorAngle(geometry_msgs::PoseWithCovarianceStamped cur, geometry_msgs::Pose2D ref);
    double ErrorLinear(geometry_msgs::PoseWithCovarianceStamped cur, geometry_msgs::Pose2D ref);
    
public:
    abstract_controller();
    void init();
};

#endif //ABSTRACT_CONTROLLER_H