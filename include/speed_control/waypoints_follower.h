#ifndef WAYPOINTS_FOLLOWERR_H
#define WAYPOINTS_FOLLOWERR_H
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "abstract_controller.h"

#include <geometry_msgs/Polygon.h>
#include <speed_control/config_toolConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mutex>

/**
 * @brief If an error occurs, the planner will deactivate and stop the motion,
 * the reasons are here
 * 
 */
enum class deactivate_reason{
    USER_DEACTIVATE, /**<User stopped the robot manually */
    NO_MORE_TARGETS, /**<The last target was reached, waiting for more targets*/
    UNREACHABLE_TARGET, /**<The target cannot be reached from current position */
    GLOBAL_TARGET_NOT_REACHED, /**< */
    WAIT_FOR_LOCALIZATION, /**< the agent has not received any position information*/
    NONE /**<deactivation was reset, reason is empty */
};


class waypoints_follower:public abstract_controller
{    
private:
    dynamic_reconfigure::Server<speed_control::config_toolConfig> server;
    dynamic_reconfigure::Server<speed_control::config_toolConfig>::CallbackType f;
    double max_linear_speed;
    double max_angular_speed;
    std::list<geometry_msgs::Point> targets;
    ros::Subscriber target_sub;
    std::mutex targets_mtx;
    bool start_new_target;
    bool active;
    ros::Subscriber command_sub;
    double x;
    double y;
    double theta;
    deactivate_reason deactivation_reason;
    geometry_msgs::Point next_target;
    double xtarget;
    double ytarget;
    double max_speed;
    double reached_threshold;
    bool turning;
    bool reached_next_target;
    bool straight;
public:
    waypoints_follower(double max_speed, double reached_threshold);
    void setTargetCallback(const geometry_msgs::PolygonConstPtr& targets);
    void config_callback(speed_control::config_toolConfig &config, uint32_t level);
    void command_manager(const std_msgs::StringConstPtr & msg);
    
    void init();
    void run();
    
private:
    void target_manager(const geometry_msgs::Point& msg);
    void setPosition(double x, double y, double theta);
    double distance(const geometry_msgs::Point& target) const;
    bool reached(const geometry_msgs::Point& target) const;
    void activate();
    void deactivate(deactivate_reason USER_DEACTIVATE);
    
};

#endif //WAYPOINTS_FOLLOWERR_H