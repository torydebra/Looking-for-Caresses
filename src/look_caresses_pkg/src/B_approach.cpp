#include "ros/ros.h"
#include "std_msgs/String.h"
#include "look_caresses_pkg/platform_sensors.h" /**auto generated .h by gencpp. it is inside devel/include*/
#include "look_caresses_pkg/platform_control.h"
#include <iostream>
#include <string>
using namespace std;

ros::Publisher pub;
ros::Subscriber sub;

void chatterCallback(const sensor_msgs::Range &sensor_range)
{
    //geometry_msgs::Twist body_vel;
    ROS_INFO("CULOOOOOOOOOOO");
    look_caresses_pkg::platform_control msg;
    if (sensor_range.range > 0.35) { // in meters
        msg.body_vel.linear.x = 100;
        ROS_INFO("MERDAAAAAAAAAAAAAAAAAAAA");
    }
    else {
        msg.body_vel.linear.x = 0;
    }
    //pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "B_approach");
    ros::NodeHandle nh;

   // pub = nh.advertise<look_caresses_pkg::platform_control>("/miro/rob01/platform/control",100);
    sub = nh.subscribe("/miro/rob01/platform/sensors", 1000, chatterCallback);

    ros::spin();

    return 0;
}
