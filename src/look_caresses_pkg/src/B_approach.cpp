#include "ros/ros.h"
#include "std_msgs/String.h"
#include "look_caresses_pkg/platform_sensors.h" /**auto generated .h by gencpp. it is inside devel/include*/
#include "look_caresses_pkg/platform_control.h"
#include <iostream>
#include <string>
#include <numeric>
#include <boost/circular_buffer.hpp>

#define averageNum 10
ros::Publisher pub;
ros::Subscriber sub;
boost::circular_buffer<float> sonarMsgs(averageNum);


void sonarCallback(const sensor_msgs::Range &sensor_range)
{
    look_caresses_pkg::platform_control msg;
    sonarMsgs.push_back(sensor_range.range);
    ROS_INFO("EOH %f", sonarMsgs.back());

    if (sonarMsgs.full()){ //wait to have some values of range

      float sum = std::accumulate(sonarMsgs.front(), sonarMsgs.back(), 0); //0 init the sum to 0
      float average = sum/averageNum;
      ROS_INFO("Sonar Range: %f", sum);

      if (average > 0.15) { // in meters
        ROS_INFO("GAY");
        msg.body_vel.linear.x = 30;
        msg.body_vel.angular.z = 0.1; //because miro turns unwanted
      }
      else {
        msg.body_vel.linear.x = 0;
        msg.body_vel.angular.z = 0;
      }
    } else {
      msg.body_vel.linear.x = 0;
      msg.body_vel.angular.z = 0;
    }
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "B_approach");
    ros::NodeHandle nh;

    pub = nh.advertise<look_caresses_pkg::platform_control>("/miro/rob01/platform/control",100);
    sub = nh.subscribe("/miro/rob01/sensors/sonar_range", 1000, sonarCallback);

    ros::spin();

    return 0;
}
