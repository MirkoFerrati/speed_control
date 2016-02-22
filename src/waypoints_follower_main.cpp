#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <sstream>
#include <math.h>
#include "speed_control/waypoints_follower.h"
    
using namespace std;

int main(int argc, char **argv)
{
    //TODO get parameters from outside
    ros::init(argc, argv, "waypoints_follower");
    waypoints_follower controller(2,0.1);
    controller.init();
    tf::TransformListener listener;
    std::string prefix,agent_name;
    ros::NodeHandle nh("~");
    prefix = nh.getNamespace().substr(0,nh.getNamespace().length()-std::string("waypoints_follower").length());
    
    if(!nh.getParam("private_name", agent_name))
    {
        if (agent_name=="")
        {
            if (prefix!="") agent_name=prefix;
            else
            {
                agent_name="iRobot";
                ROS_WARN("missing agent name AND namespace, using default values");
            }
        }
        else prefix=agent_name;
    }
    else
        prefix=agent_name;
    ros::Rate loop_rate(3);
    
    while ( ros::ok() )
    {
        bool localized=false;
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("map",prefix+"base_link", 
                                     ros::Time(0), transform);
            localized=true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        
        tf::Vector3 o = transform.getOrigin();
        double roll, pitch, yaw;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
        
        if (localized)
        {
            controller.setPosition(o.getX(),o.getY(),yaw);
            controller.run();
        }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
   
    

    return 0;
}
