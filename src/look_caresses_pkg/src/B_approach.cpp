#include "ros/ros.h"
#include "std_msgs/String.h"
#include "look_caresses_pkg/platform_sensors.h"
#include "look_caresses_pkg/platform_control.h"
#include <iostream>
#include <string>
#include <numeric>
#include <boost/circular_buffer.hpp>

#define averageNum 20 // number of values to consider in the average for the sonar range
ros::Publisher pub;
ros::Subscriber sub;
boost::circular_buffer<float> sonarMsgs(averageNum); // to have more than 1 measure from the sonar


void sonarCallback(const sensor_msgs::Range &sensor_range)
{
    look_caresses_pkg::platform_control msg;
    sonarMsgs.push_front(sensor_range.range);

    if (sonarMsgs.full()){ //wait to have "averageNum" values of range

      // average between more than 1 value to avoid the errors of the sonar (sometimes returns zeros)
      float sum = std::accumulate(sonarMsgs.begin(), sonarMsgs.end(), 0.0); //0 init the sum to 0
      float average = sum/averageNum;
      ROS_INFO("Sonar Range average: %f", average);

      if (average > 0.15) { // in meters
        msg.body_vel.linear.x = 30; // good choiche for the linear velocity
        msg.body_vel.angular.z = 0.05; // because miro turns unwanted
      }
      else { // miro must not move
        msg.body_vel.linear.x = 0;
        msg.body_vel.angular.z = 0;
      }
    } else { // miro must not move
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
