#ifndef B_APPROACH_H
#define B_APPROACH_H

#include <boost/circular_buffer.hpp>
#include "ros/ros.h"
#include "look_caresses_pkg/platform_sensors.h"

#define averageNum 20 // number of values to consider in the average for the sonar range

class B_approach {
private:
  int counter;
  ros::Publisher pubPlat;
  ros::Publisher pubVel;
  ros::Subscriber subRange;
  boost::circular_buffer<float> sonarMsgs(int num); // to have more than 1 measure from the sonar

  void sonarCallback(const sensor_msgs::Range &sensor_range);
public:
  B_approach::B_approach (int argc, char **argv);
  int main ();
};

#endif // B_APPROACH_H
