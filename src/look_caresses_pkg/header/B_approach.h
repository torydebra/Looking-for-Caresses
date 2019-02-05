#ifndef B_APPROACH_H
#define B_APPROACH_H

#include <boost/circular_buffer.hpp>
#include "ros/ros.h"
#include "look_caresses_pkg/platform_sensors.h"
#include "look_caresses_pkg/platform_control.h"

#define averageNum 20 // number of values to consider in the average for the sonar range

class B_approach {
private:
  int counter;
  ros::NodeHandle nh;
  ros::Publisher pubPlat;
  ros::Subscriber subRange;
  boost::circular_buffer<float> sonarMsgs; // to have more than 1 measure from the sonar

  void sonarCallback(const sensor_msgs::Range &sensor_range);
  void unsubTopics();
  void subTopics();

public:
  B_approach (ros::NodeHandle nh, ros::Publisher pubPlat);
  int main ();
};

#endif // B_APPROACH_H
