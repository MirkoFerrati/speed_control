#include "ros/ros.h"
#include <geometry_msgs/Polygon.h>
#include <sstream>
#include <math.h>


using namespace std;

double dist(double x,double y, double x1, double y1)
{
    return sqrt(pow(x-x1,2)+pow(y-y1,2));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_waypoints_follower");
    ros::NodeHandle nh;
    geometry_msgs::Polygon poly;
    ros::Publisher publish = nh.advertise<geometry_msgs::Polygon>("targets",1);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(1, 10);
    std::uniform_real_distribution<> dis1(0.2, 1);
    double max_speed=0.5;
    sleep(1);
    geometry_msgs::Point32 point;
    point.x=dis(gen);
    point.y=dis(gen);
    point.z=ros::Time::now().toSec()+20.0;
    poly.points.push_back(point);
    point.x=dis(gen);
    point.y=dis(gen);
    point.z=point.z+dist(poly.points.back().x,poly.points.back().y,point.x,point.y)/(max_speed*dis1(gen));
    poly.points.push_back(point);
    point.x=dis(gen);
    point.y=dis(gen);
    point.z=point.z+dist(poly.points.back().x,poly.points.back().y,point.x,point.y)/(max_speed*dis1(gen));
    poly.points.push_back(point);
    
    publish.publish(poly);
    for (auto p:poly.points)
    {
        std::cout<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
    }
    ros::spin();
    return 0;
}
