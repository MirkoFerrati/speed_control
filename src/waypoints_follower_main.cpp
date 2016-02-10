#include "ros/ros.h"
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
    controller.run();

    return 0;
}
